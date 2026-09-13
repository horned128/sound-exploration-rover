#!/usr/bin/env bash
set -euo pipefail

TARGET="All"
CLEAN=0
REGENERATE=0
JOBS="$(sysctl -n hw.logicalcpu 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"

usage() {
    cat <<'USAGE'
Usage: Invoke-RaBuild.sh [options]

Options:
  --target CPU0|CPU1|All   Build target (default: All)
  --clean                  Clean before build
  --regenerate             Use e2 studio headless managed build
  --jobs N                 Parallel make jobs
  -h, --help               Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --target|-Target)
            [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
            TARGET="$2"
            shift 2
            ;;
        --clean|-Clean)
            CLEAN=1
            shift
            ;;
        --regenerate|-Regenerate)
            REGENERATE=1
            shift
            ;;
        --jobs|-Jobs)
            [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
            JOBS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

case "$TARGET" in
    CPU0|CPU1|All) ;;
    *)
        echo "Invalid target: $TARGET (expected CPU0, CPU1, or All)" >&2
        exit 2
        ;;
esac

if ! [[ "$JOBS" =~ ^[0-9]+$ ]] || [[ "$JOBS" -lt 1 ]] || [[ "$JOBS" -gt 64 ]]; then
    echo "Invalid jobs value: $JOBS (expected 1..64)" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RA_ROOT="$REPOSITORY_ROOT/firmware/ra8p1"
SOLUTION_NAME="SoundExplorationRover"
SOLUTION_DIR="$RA_ROOT/$SOLUTION_NAME"
SOLUTION_BUNDLE="$SOLUTION_DIR/build/$SOLUTION_NAME.sbd"

PROJECT_NAMES=()
case "$TARGET" in
    CPU0)
        PROJECT_NAMES=("SoundExplorationRover_CPU0")
        ;;
    CPU1)
        PROJECT_NAMES=("SoundExplorationRover_CPU1")
        ;;
    All)
        PROJECT_NAMES=("SoundExplorationRover_CPU0" "SoundExplorationRover_CPU1")
        ;;
esac

find_e2studio_executable() {
    local configured candidate

    if [[ -n "${E2STUDIO_HOME:-}" ]]; then
        configured="$E2STUDIO_HOME"
        for candidate in \
            "$configured" \
            "$configured/Contents/MacOS/e2studio" \
            "$configured/E2studio.app/Contents/MacOS/e2studio" \
            "$configured/e2studio"; do
            if [[ -f "$candidate" && -x "$candidate" ]]; then
                printf '%s\n' "$candidate"
                return 0
            fi
        done
        echo "E2STUDIO_HOME does not point to a usable e2 studio executable or app bundle: $configured" >&2
        return 1
    fi

    candidate="/Applications/E2studio.app/Contents/MacOS/e2studio"
    if [[ -x "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi

    candidate="$(find /Applications -maxdepth 6 -type f -path '*/Contents/MacOS/e2studio' -perm -111 2>/dev/null | sort -r | head -n 1 || true)"
    if [[ -n "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi

    if command -v e2studio >/dev/null 2>&1; then
        command -v e2studio
        return 0
    fi

    echo "e2 studio was not found." >&2
    echo "Install the macOS RA build, or set E2STUDIO_HOME to E2studio.app (or its executable)." >&2
    return 1
}

find_arm_gcc_bin() {
    local e2_exec="$1"
    local candidate app_bundle search_root gcc

    if [[ -n "${ARM_GCC_TOOLCHAIN_PATH:-}" ]]; then
        candidate="$ARM_GCC_TOOLCHAIN_PATH"
        if [[ -x "$candidate/arm-none-eabi-gcc" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
        echo "ARM_GCC_TOOLCHAIN_PATH does not contain arm-none-eabi-gcc: $candidate" >&2
        return 1
    fi

    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        dirname "$(command -v arm-none-eabi-gcc)"
        return 0
    fi

    app_bundle="${e2_exec%/Contents/MacOS/e2studio}"
    for search_root in "$app_bundle" "$(dirname "$app_bundle")"; do
        [[ -d "$search_root" ]] || continue
        gcc="$(find "$search_root" -maxdepth 9 -type f -name arm-none-eabi-gcc -perm -111 2>/dev/null | sort -r | head -n 1 || true)"
        if [[ -n "$gcc" ]]; then
            dirname "$gcc"
            return 0
        fi
    done

    echo "Arm GNU Toolchain was not found." >&2
    echo "Set ARM_GCC_TOOLCHAIN_PATH to the bin directory containing arm-none-eabi-gcc." >&2
    return 1
}

find_e2studio_java() {
    local e2_exec="$1"
    local app_bundle eclipse_root candidate

    app_bundle="${e2_exec%/Contents/MacOS/e2studio}"
    eclipse_root="$app_bundle/Contents/eclipse"
    candidate="$(find "$eclipse_root/plugins" -maxdepth 4 -type f -path '*/jre/bin/java' -perm -111 2>/dev/null | sort -r | head -n 1 || true)"
    if [[ -n "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi

    echo "Bundled e2 studio Java runtime was not found below $eclipse_root/plugins." >&2
    return 1
}

find_equinox_launcher() {
    local e2_exec="$1"
    local app_bundle eclipse_root candidate

    app_bundle="${e2_exec%/Contents/MacOS/e2studio}"
    eclipse_root="$app_bundle/Contents/eclipse"
    candidate="$(find "$eclipse_root/plugins" -maxdepth 1 -type f -name 'org.eclipse.equinox.launcher_*.jar' | sort -r | head -n 1 || true)"
    if [[ -n "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi

    echo "Equinox launcher JAR was not found below $eclipse_root/plugins." >&2
    return 1
}

find_gnu_make() {
    local e2_exec="$1"
    local app_bundle candidate

    if command -v gmake >/dev/null 2>&1; then
        command -v gmake
        return 0
    fi
    if command -v make >/dev/null 2>&1; then
        command -v make
        return 0
    fi

    app_bundle="${e2_exec%/Contents/MacOS/e2studio}"
    candidate="$(find "$app_bundle" -maxdepth 9 -type f -name make -path '*gnumake*' -perm -111 2>/dev/null | sort -r | head -n 1 || true)"
    if [[ -n "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi

    echo "GNU Make was not found. Install make/gmake or add it to PATH." >&2
    return 1
}

invoke_fast_build() {
    local project_name="$1"
    local make_executable="$2"
    local build_directory="$RA_ROOT/$project_name/Debug"
    local makefile="$build_directory/makefile"

    if [[ ! -f "$makefile" ]]; then
        echo "Generated makefile is missing for $project_name." >&2
        echo "Run 'RA8P1: Generate + Clean Build $TARGET' on this Mac first." >&2
        return 1
    fi

    pushd "$build_directory" >/dev/null
    if [[ $CLEAN -eq 1 ]]; then
        "$make_executable" -r clean
    fi

    if ! "$make_executable" -r -j"$JOBS" all; then
        echo "$project_name fast build failed." >&2
        echo "If Debug/ was generated on Windows, regenerate the project once on this Mac before using Fast Build." >&2
        popd >/dev/null
        return 1
    fi
    popd >/dev/null
}

# Clean-only passes also print "Build Finished. 0 errors". Only accept the
# requested project's actual build; otherwise a large build is killed mid-compile.
managed_build_succeeded() {
    awk -v project="$2" '
        /\*\*\*\* .*([Bb]uild) of configuration/ {
            current = index($0, "for project " project " ****") && $0 !~ /Clean-only/
            if (current) success = 0
        }
        current && /Build Finished\. 0 errors/ { success = 1 }
        current && /Build Failed/ { success = 0 }
        END { exit !success }
    ' "$1" 2>/dev/null
}

invoke_managed_build() {
    local project_name="$1"
    local e2_exec="$2"
    local project_dir="$RA_ROOT/$project_name"
    local project_file="$project_dir/.project"
    local headless_root="$REPOSITORY_ROOT/.vscode/.e2studio-headless"
    local workspace="$headless_root/workspace-$project_name"
    local configuration="$headless_root/configuration-$project_name"
    local stdout_log="$headless_root/$project_name.stdout.log"
    local stderr_log="$headless_root/$project_name.stderr.log"
    local operation="-build"
    local java_executable launcher_jar
    local pid exit_code=0 build_finished=0
    local deadline wait_index

    if [[ ! -f "$project_file" ]]; then
        echo "Eclipse project descriptor is missing: $project_file" >&2
        echo "Restore the .project file for $project_name before running the managed build." >&2
        return 1
    fi

    if [[ ! -f "$SOLUTION_DIR/.project" ]]; then
        echo "RA multicore solution project is missing: $SOLUTION_DIR/.project" >&2
        return 1
    fi

    # RA multicore solution projects generate a solution Smart Bundle (.sbd) in
    # the e2 studio GUI. CDT headless build cannot generate the solution project
    # itself, but CPU0/CPU1 DDSC generation depends on this bundle.
    if [[ ! -f "$SOLUTION_BUNDLE" ]]; then
        echo "RA multicore solution Smart Bundle is missing:" >&2
        echo "  $SOLUTION_BUNDLE" >&2
        echo >&2
        echo "Open the SoundExplorationRover solution in e2 studio GUI once so that" >&2
        echo "build/SoundExplorationRover.sbd is generated, then run this task again." >&2
        echo "For clean-clone/headless builds, keep that solution .sbd under version control." >&2
        return 1
    fi

    # A failed Eclipse import can leave stale workspace or OSGi metadata that
    # points at a project whose descriptor can no longer be resolved.
    # Regenerate always starts from clean, project-specific headless state.
    rm -rf "$workspace" "$configuration"
    mkdir -p "$headless_root" "$workspace" "$configuration"
    : > "$stdout_log"
    : > "$stderr_log"

    if [[ $CLEAN -eq 1 ]]; then
        operation="-cleanBuild"
    fi

    java_executable="$(find_e2studio_java "$e2_exec")"
    launcher_jar="$(find_equinox_launcher "$e2_exec")"

    # The native macOS Eclipse launcher initializes AppKit even for a headless
    # application. It aborts in sandboxed/CI sessions at RegisterApplication.
    # Direct Equinox startup bypasses that Cocoa launcher and remains headless.
    echo "Generating and building $project_name with e2 studio Equinox..."
    "$java_executable" \
        -Xms512m \
        -Xmx3g \
        --add-modules=ALL-SYSTEM \
        -Djava.security.manager=allow \
        -jar "$launcher_jar" \
        -nosplash \
        -consoleLog \
        -configuration "$configuration" \
        -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
        -data "$workspace" \
        -importAll "$RA_ROOT" \
        "$operation" "$project_name/Debug" \
        >"$stdout_log" 2>"$stderr_log" &
    pid=$!

    deadline=$((SECONDS + 900))
    while kill -0 "$pid" 2>/dev/null; do
        if managed_build_succeeded "$stdout_log" "$project_name"; then
            build_finished=1
            break
        fi

        if [[ $SECONDS -ge $deadline ]]; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
            echo "$project_name managed build timed out after 15 minutes." >&2
            echo "See $stdout_log and $stderr_log" >&2
            return 1
        fi
        sleep 2
    done

    if [[ $build_finished -eq 1 ]] && kill -0 "$pid" 2>/dev/null; then
        for wait_index in 1 2 3 4 5; do
            sleep 2
            if ! kill -0 "$pid" 2>/dev/null; then
                break
            fi
        done
        if kill -0 "$pid" 2>/dev/null; then
            echo "$project_name build finished; stopping the lingering headless Equinox process $pid."
            kill "$pid" 2>/dev/null || true
        fi
    fi

    set +e
    wait "$pid" 2>/dev/null
    exit_code=$?
    set -e

    if managed_build_succeeded "$stdout_log" "$project_name"; then
        build_finished=1
    fi

    if [[ -f "$stdout_log" ]]; then
        tail -n 120 "$stdout_log" || true
    fi
    if [[ -s "$stderr_log" ]]; then
        tail -n 80 "$stderr_log" || true
    fi

    if [[ $build_finished -eq 0 && $exit_code -ne 0 ]]; then
        echo "$project_name managed build failed with exit code $exit_code." >&2
        local eclipse_log="$workspace/.metadata/.log"
        if [[ -s "$eclipse_log" ]]; then
            echo "--- e2 studio workspace log (tail) ---" >&2
            tail -n 120 "$eclipse_log" >&2 || true
        fi
        return 1
    fi

    local elf="$RA_ROOT/$project_name/Debug/$project_name.elf"
    if [[ $build_finished -eq 0 || ! -f "$elf" ]]; then
        echo "$project_name managed build completed without producing $elf." >&2
        return 1
    fi
}

if [[ ! -d "$RA_ROOT" ]]; then
    echo "RA8P1 project root was not found: $RA_ROOT" >&2
    exit 1
fi

E2STUDIO_EXECUTABLE="$(find_e2studio_executable)"
GCC_BIN="$(find_arm_gcc_bin "$E2STUDIO_EXECUTABLE")"
export ARM_GCC_TOOLCHAIN_PATH="$GCC_BIN"
export PATH="$GCC_BIN:$PATH"

echo "e2 studio: $E2STUDIO_EXECUTABLE"
echo "Arm GCC:   $GCC_BIN"

if [[ $REGENERATE -eq 1 ]]; then
    for project_name in "${PROJECT_NAMES[@]}"; do
        invoke_managed_build "$project_name" "$E2STUDIO_EXECUTABLE"
    done
else
    MAKE_EXECUTABLE="$(find_gnu_make "$E2STUDIO_EXECUTABLE")"
    echo "GNU Make:  $MAKE_EXECUTABLE"
    for project_name in "${PROJECT_NAMES[@]}"; do
        invoke_fast_build "$project_name" "$MAKE_EXECUTABLE"
    done
fi

echo "RA8P1 $TARGET build completed."
