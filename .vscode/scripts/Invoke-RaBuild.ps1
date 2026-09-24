[CmdletBinding()]
param(
    [ValidateSet("CPU0", "CPU1", "All")]
    [string] $Target = "All",

    [switch] $Clean,
    [switch] $Regenerate,
    [switch] $Flash,

    [ValidateRange(1, 64)]
    [int] $Jobs = [Environment]::ProcessorCount
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$raRoot = Join-Path $repositoryRoot "firmware\ra8p1"
$projectNames = switch ($Target) {
    "CPU0" { @("SoundExplorationRover_CPU0") }
    "CPU1" { @("SoundExplorationRover_CPU1") }
    default { @("SoundExplorationRover_CPU0", "SoundExplorationRover_CPU1") }
}

$jlinkInterface = if ($env:JLINK_INTERFACE) { $env:JLINK_INTERFACE } else { "SWD" }
$jlinkSpeed = if ($env:JLINK_SPEED) { $env:JLINK_SPEED } else { "4000" }
$jlinkDeviceCpu0 = if ($env:JLINK_DEVICE_CPU0) { $env:JLINK_DEVICE_CPU0 } else { "R7KA8P1KF_CPU0" }
$jlinkDeviceCpu1 = if ($env:JLINK_DEVICE_CPU1) { $env:JLINK_DEVICE_CPU1 } else { "R7KA8P1KF_CPU1" }

function Find-E2StudioRoot {
    if ($env:E2STUDIO_HOME) {
        $configuredRoot = (Resolve-Path $env:E2STUDIO_HOME).Path
        if (Test-Path (Join-Path $configuredRoot "eclipse\e2studioc.exe")) {
            return $configuredRoot
        }

        throw "E2STUDIO_HOME does not point to an e2 studio installation: $configuredRoot"
    }

    $installations = Get-ChildItem -Path "C:\Renesas\RA" -Directory -Filter "e2studio_v*" -ErrorAction SilentlyContinue |
        Where-Object { Test-Path (Join-Path $_.FullName "eclipse\e2studioc.exe") } |
        Sort-Object Name -Descending

    if (-not $installations) {
        throw "e2 studio was not found. Set E2STUDIO_HOME to the installation root."
    }

    return $installations[0].FullName
}

function Find-ArmGccBin([string] $E2StudioRoot) {
    if ($env:ARM_GCC_TOOLCHAIN_PATH) {
        $configuredBin = (Resolve-Path $env:ARM_GCC_TOOLCHAIN_PATH).Path
        if (Test-Path (Join-Path $configuredBin "arm-none-eabi-gcc.exe")) {
            return $configuredBin
        }

        throw "ARM_GCC_TOOLCHAIN_PATH does not contain arm-none-eabi-gcc.exe: $configuredBin"
    }

    $gcc = Get-ChildItem -Path (Join-Path $E2StudioRoot "toolchains\gcc_arm") -Filter "arm-none-eabi-gcc.exe" -File -Recurse -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1

    if (-not $gcc) {
        throw "Arm GNU Toolchain was not found below $E2StudioRoot."
    }

    return $gcc.DirectoryName
}

function Find-GnuMake([string] $E2StudioRoot) {
    $renesasMake = Get-ChildItem -Path (Join-Path $E2StudioRoot "eclipse\plugins") -Filter "make.exe" -File -Recurse -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -match "gnumake" } |
        Sort-Object FullName -Descending |
        Select-Object -First 1

    if ($renesasMake) {
        return $renesasMake.FullName
    }

    $pathMake = Get-Command make.exe -ErrorAction SilentlyContinue
    if ($pathMake) {
        return $pathMake.Source
    }

    throw "GNU Make was not found in e2 studio or PATH."
}

function Find-JLinkExecutable {
    if ($env:JLINK_EXE) {
        $configuredJLink = $env:JLINK_EXE
        if (Test-Path -LiteralPath $configuredJLink -PathType Leaf) {
            return (Resolve-Path -LiteralPath $configuredJLink).Path
        }

        throw "JLINK_EXE does not point to J-Link Commander: $configuredJLink"
    }

    foreach ($commandName in @("JLink.exe", "JLinkExe.exe", "JLinkExe")) {
        $pathCommand = Get-Command $commandName -ErrorAction SilentlyContinue
        if ($pathCommand) {
            return $pathCommand.Source
        }
    }

    $searchRoots = @()
    if ($env:ProgramFiles) {
        $searchRoots += (Join-Path $env:ProgramFiles "SEGGER")
    }
    if (${env:ProgramFiles(x86)}) {
        $searchRoots += (Join-Path ${env:ProgramFiles(x86)} "SEGGER")
    }

    foreach ($searchRoot in $searchRoots) {
        if (-not (Test-Path -LiteralPath $searchRoot)) {
            continue
        }

        $jlink = Get-ChildItem -Path $searchRoot -Filter "JLink.exe" -File -Recurse -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending |
            Select-Object -First 1

        if ($jlink) {
            return $jlink.FullName
        }
    }

    throw "SEGGER J-Link Commander (JLink.exe) was not found. Install the J-Link Software and Documentation Pack, add JLink.exe to PATH, or set JLINK_EXE."
}

function Invoke-JLinkCommandFile(
    [string] $JLinkExecutable,
    [string] $Device,
    [string] $CommandFile
) {
    $arguments = @(
        "-Device", $Device,
        "-If", $jlinkInterface,
        "-Speed", $jlinkSpeed,
        "-AutoConnect", "1",
        "-ExitOnError", "1",
        "-NoGui", "1"
    )

    if ($env:JLINK_SERIAL) {
        $arguments += @("-USB", $env:JLINK_SERIAL)
    }

    $arguments += @("-CommandFile", $CommandFile)

    & $JLinkExecutable @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "J-Link Commander failed for device $Device with exit code $LASTEXITCODE."
    }
}

function Invoke-FlashProject(
    [string] $ProjectName,
    [string] $JLinkExecutable
) {
    $device = switch ($ProjectName) {
        "SoundExplorationRover_CPU0" { $jlinkDeviceCpu0 }
        "SoundExplorationRover_CPU1" { $jlinkDeviceCpu1 }
        default { throw "No J-Link device mapping for project: $ProjectName" }
    }

    $elf = Join-Path $raRoot "$ProjectName\Debug\$ProjectName.elf"
    if (-not (Test-Path -LiteralPath $elf -PathType Leaf)) {
        throw "ELF to flash was not found: $elf"
    }

    $commandFile = Join-Path ([System.IO.Path]::GetTempPath()) ("ra8p1-jlink-flash-{0}.jlink" -f [Guid]::NewGuid().ToString("N"))

    try {
        @(
            "r"
            "h"
            ('loadfile "{0}"' -f $elf)
            "h"
            "q"
        ) | Set-Content -LiteralPath $commandFile -Encoding ASCII

        Write-Host "Flashing $ProjectName with J-Link device $device..."
        Invoke-JLinkCommandFile -JLinkExecutable $JLinkExecutable -Device $device -CommandFile $commandFile
    }
    finally {
        Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
    }
}

function Restart-TargetFromCpu0([string] $JLinkExecutable) {
    $commandFile = Join-Path ([System.IO.Path]::GetTempPath()) ("ra8p1-jlink-run-{0}.jlink" -f [Guid]::NewGuid().ToString("N"))

    try {
        @(
            "r"
            "g"
            "q"
        ) | Set-Content -LiteralPath $commandFile -Encoding ASCII

        Write-Host "Resetting RA8P1 and starting from CPU0..."
        Invoke-JLinkCommandFile -JLinkExecutable $JLinkExecutable -Device $jlinkDeviceCpu0 -CommandFile $commandFile
    }
    finally {
        Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
    }
}

function Invoke-FastBuild([string] $ProjectName, [string] $MakeExecutable) {
    $buildDirectory = Join-Path $raRoot "$ProjectName\Debug"
    $makefile = Join-Path $buildDirectory "makefile"

    if (-not (Test-Path $makefile)) {
        throw "Generated makefile is missing for $ProjectName. Run 'RA8P1: Generate + Clean Build All' first."
    }

    Push-Location $buildDirectory
    try {
        if ($Clean) {
            & $MakeExecutable -r clean "SHELL=cmd.exe"
            if ($LASTEXITCODE -ne 0) {
                throw "$ProjectName clean failed with exit code $LASTEXITCODE."
            }
        }

        & $MakeExecutable -r "-j$Jobs" all "SHELL=cmd.exe"
        if ($LASTEXITCODE -ne 0) {
            throw "$ProjectName build failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }
}

function Invoke-ManagedBuild([string] $ProjectName, [string] $E2StudioRoot) {
    $headlessRoot = Join-Path $repositoryRoot ".vscode\.e2studio-headless"
    $workspace = Join-Path $headlessRoot "workspace"
    $configuration = Join-Path $headlessRoot "configuration"
    $stdoutLog = Join-Path $headlessRoot "$ProjectName.stdout.log"
    $stderrLog = Join-Path $headlessRoot "$ProjectName.stderr.log"
    New-Item -ItemType Directory -Force -Path $workspace, $configuration | Out-Null

    $operation = if ($Clean) { "-cleanBuild" } else { "-build" }
    $arguments = @(
        "-nosplash",
        "-consoleLog",
        "-configuration", ('"{0}"' -f $configuration),
        "-application", "org.eclipse.cdt.managedbuilder.core.headlessbuild",
        "-data", ('"{0}"' -f $workspace),
        "-importAll", ('"{0}"' -f $raRoot),
        $operation, "$ProjectName/Debug"
    )

    Write-Host "Generating and building $ProjectName with e2 studio..."
    $process = Start-Process -FilePath (Join-Path $E2StudioRoot "eclipse\e2studioc.exe") `
        -ArgumentList $arguments `
        -PassThru `
        -WindowStyle Hidden `
        -RedirectStandardOutput $stdoutLog `
        -RedirectStandardError $stderrLog

    $deadline = [DateTime]::UtcNow.AddMinutes(15)
    $buildFinished = $false
    while (-not $process.HasExited) {
        if (Test-Path $stdoutLog) {
            $recentOutput = Get-Content $stdoutLog -Tail 80 -ErrorAction SilentlyContinue
            if ($recentOutput -match "Build Finished\. 0 errors") {
                $buildFinished = $true
                break
            }
        }

        if ([DateTime]::UtcNow -ge $deadline) {
            Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
            throw "$ProjectName managed build timed out after 15 minutes. See $stdoutLog and $stderrLog."
        }

        Start-Sleep -Seconds 2
    }

    if ($buildFinished -and (-not $process.HasExited)) {
        for ($waitIndex = 0; $waitIndex -lt 5; $waitIndex++) {
            Start-Sleep -Seconds 2
            $process.Refresh()
            if ($process.HasExited) {
                break
            }
        }

        if (-not $process.HasExited) {
            Write-Warning "$ProjectName build finished, but e2 studio did not exit. Stopping only headless process $($process.Id)."
            Stop-Process -Id $process.Id -Force
            $process.WaitForExit()
        }
    }

    if (Test-Path $stdoutLog) {
        Get-Content $stdoutLog -Tail 120
    }
    if (Test-Path $stderrLog) {
        Get-Content $stderrLog -Tail 80
    }

    if ((-not $buildFinished) -and ($process.ExitCode -ne 0)) {
        throw "$ProjectName managed build failed with exit code $($process.ExitCode)."
    }

    $elf = Join-Path $raRoot "$ProjectName\Debug\$ProjectName.elf"
    if ((-not $buildFinished) -or (-not (Test-Path $elf))) {
        throw "$ProjectName managed build completed without producing $elf."
    }
}

$e2StudioRoot = Find-E2StudioRoot
$gccBin = Find-ArmGccBin $e2StudioRoot
$env:ARM_GCC_TOOLCHAIN_PATH = $gccBin
$env:Path = "$gccBin;$env:Path"

Write-Host "e2 studio: $e2StudioRoot"
Write-Host "Arm GCC:   $gccBin"

if ($Regenerate) {
    foreach ($projectName in $projectNames) {
        Invoke-ManagedBuild $projectName $e2StudioRoot
    }
}
else {
    $makeExecutable = Find-GnuMake $e2StudioRoot
    Write-Host "GNU Make:  $makeExecutable"
    foreach ($projectName in $projectNames) {
        Invoke-FastBuild $projectName $makeExecutable
    }
}

Write-Host "RA8P1 $Target build completed."

if ($Flash) {
    $jlinkExecutable = Find-JLinkExecutable
    Write-Host "J-Link:     $jlinkExecutable"
    Write-Host "Interface:  $jlinkInterface @ $jlinkSpeed kHz"

    # Program CPU1 first. CPU0 is the boot/master core, so CPU0 is programmed
    # last and the board is reset only after all selected images are in place.
    $flashProjectNames = switch ($Target) {
        "CPU0" { @("SoundExplorationRover_CPU0") }
        "CPU1" { @("SoundExplorationRover_CPU1") }
        default { @("SoundExplorationRover_CPU1", "SoundExplorationRover_CPU0") }
    }

    foreach ($projectName in $flashProjectNames) {
        Invoke-FlashProject -ProjectName $projectName -JLinkExecutable $jlinkExecutable
    }

    Restart-TargetFromCpu0 -JLinkExecutable $jlinkExecutable
    Write-Host "RA8P1 $Target flash completed."
}
