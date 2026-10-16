#!/usr/bin/env bash
# tools/test_fast.sh - Fast targeted unit / integration test runner
# Usage:
#   ./tools/test_fast.sh                        # Run native_test_runner with compact summary
#   ./tools/test_fast.sh -f test_peugeot_407    # Filtered test execution
#   ./tools/test_fast.sh -e integration_test    # Run integration suite
set -eo pipefail

PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"
if [ ! -x "$PIO" ]; then
    echo "Error: PlatformIO CLI not found at $PIO" >&2
    exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

ENV="native_test_runner"
FILTER_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -e|--environment)
            ENV="$2"
            shift 2
            ;;
        -f|--filter)
            FILTER_ARGS+=("-f" "$2")
            shift 2
            ;;
        *)
            FILTER_ARGS+=("$1")
            shift
            ;;
    esac
done

OUT=$("$PIO" test -e "$ENV" "${FILTER_ARGS[@]}" 2>&1) || {
    echo "FAILED!"
    echo "$OUT" | grep -E "FAILED|ERROR|test_.*\[" | head -n 30
    exit 1
}

# Print concise summary and timing only
echo "$OUT" | grep -E "PASSED.*Took|FAILED|ERROR|SUMMARY.*|.*PASSED.*[0-9]{2}:[0-9]{2}:[0-9]{2}" | tail -n 10

