#!/usr/bin/env bash
set -euo pipefail

ACTION="Build"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --action|-Action)
            [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
            ACTION="$2"
            shift 2
            ;;
        Build|Upload|Monitor|Clean)
            ACTION="$1"
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

case "$ACTION" in
    Build|Upload|Monitor|Clean) ;;
    *)
        echo "Invalid action: $ACTION (expected Build, Upload, Monitor, or Clean)" >&2
        exit 2
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PROJECT_DIRECTORY="$REPOSITORY_ROOT/firmware/esp32s3"

find_platformio() {
    local candidate

    if command -v platformio >/dev/null 2>&1; then
        command -v platformio
        return 0
    fi
    if command -v pio >/dev/null 2>&1; then
        command -v pio
        return 0
    fi

    if [[ -n "${PLATFORMIO_CORE_DIR:-}" ]]; then
        for candidate in \
            "$PLATFORMIO_CORE_DIR/penv/bin/platformio" \
            "$PLATFORMIO_CORE_DIR/penv/bin/pio"; do
            if [[ -x "$candidate" ]]; then
                printf '%s\n' "$candidate"
                return 0
            fi
        done
    fi

    for candidate in \
        "$HOME/.platformio/penv/bin/platformio" \
        "$HOME/.platformio/penv/bin/pio"; do
        if [[ -x "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done

    echo "PlatformIO Core was not found." >&2
    echo "Initialize the PlatformIO VS Code extension, or add platformio/pio to PATH." >&2
    return 1
}

PLATFORMIO="$(find_platformio)"

if [[ ! -d "$PROJECT_DIRECTORY" ]]; then
    echo "PlatformIO project directory was not found: $PROJECT_DIRECTORY" >&2
    exit 1
fi

if [[ "$ACTION" == "Upload" ]]; then
    echo "XIAO upload: disconnect EK-RA8P1 J7 and connect the XIAO-side USB-C directly to the Mac."
    echo "If the previous rover firmware is running, enter the XIAO ROM bootloader first:"
    echo "  hold the XIAO BOOT button, tap XIAO RESET, release RESET, then release BOOT."
    echo "  Do not use the ReSpeaker XVF3800 RESET button for this operation."
fi

ARGS=()
case "$ACTION" in
    Build)
        ARGS=(run --project-dir "$PROJECT_DIRECTORY")
        ;;
    Upload)
        ARGS=(run --project-dir "$PROJECT_DIRECTORY" --target upload)
        ;;
    Monitor)
        ARGS=(device monitor --project-dir "$PROJECT_DIRECTORY")
        ;;
    Clean)
        ARGS=(run --project-dir "$PROJECT_DIRECTORY" --target clean)
        ;;
esac

echo "PlatformIO: $PLATFORMIO"
echo "Project:    $PROJECT_DIRECTORY"

set +e
"$PLATFORMIO" "${ARGS[@]}"
EXIT_CODE=$?
set -e

if [[ $EXIT_CODE -ne 0 ]]; then
    if [[ "$ACTION" == "Upload" ]]; then
        echo "Warning: if the log stopped at 'Connecting' with a write timeout, the serial port may be visible while the XIAO is not in its ROM bootloader." >&2
        echo "Enter the bootloader as shown above and run ESP32: Upload again." >&2
    fi
    echo "PlatformIO $ACTION failed with exit code $EXIT_CODE." >&2
    exit "$EXIT_CODE"
fi
