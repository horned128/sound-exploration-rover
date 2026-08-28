[CmdletBinding()]
param(
    [ValidateSet("Build", "Upload", "Monitor", "Clean")]
    [string] $Action = "Build"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$projectDirectory = Join-Path $repositoryRoot "firmware\esp32s3"

function Find-PlatformIo {
    $commands = @("platformio.exe", "pio.exe")
    foreach ($commandName in $commands) {
        $command = Get-Command $commandName -ErrorAction SilentlyContinue
        if ($command) {
            return $command.Source
        }
    }

    if ($env:PLATFORMIO_CORE_DIR) {
        foreach ($commandName in $commands) {
            $candidate = Join-Path $env:PLATFORMIO_CORE_DIR "penv\Scripts\$commandName"
            if (Test-Path $candidate) {
                return $candidate
            }
        }
    }

    foreach ($commandName in $commands) {
        $candidate = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\$commandName"
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    throw "PlatformIO Core was not found. Initialize the PlatformIO extension in VS Code first."
}

$platformIo = Find-PlatformIo

if ($Action -eq "Upload") {
    Write-Host "XIAO upload: disconnect EK-RA8P1 J7 and connect the XIAO-side USB-C directly to the PC."
    Write-Host "If the previous rover firmware is running, enter the XIAO ROM bootloader first:"
    Write-Host "  hold the XIAO BOOT button, tap XIAO RESET, release RESET, then release BOOT."
    Write-Host "  Do not use the ReSpeaker XVF3800 RESET button for this operation."
}

$platformIoArguments = switch ($Action) {
    "Build" { @("run", "--project-dir", $projectDirectory) }
    "Upload" { @("run", "--project-dir", $projectDirectory, "--target", "upload") }
    "Monitor" { @("device", "monitor", "--project-dir", $projectDirectory) }
    "Clean" { @("run", "--project-dir", $projectDirectory, "--target", "clean") }
}

Write-Host "PlatformIO: $platformIo"
Write-Host "Project:    $projectDirectory"
& $platformIo @platformIoArguments
if ($LASTEXITCODE -ne 0) {
    if ($Action -eq "Upload") {
        Write-Warning "If the log stopped at 'Connecting' with 'Write timeout', COM was detected but the XIAO was not in its ROM bootloader. Enter the bootloader as shown above and run ESP32: Upload again."
    }

    throw "PlatformIO $Action failed with exit code $LASTEXITCODE."
}
