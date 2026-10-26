#!/usr/bin/env bash
# tools/verify_all.sh - Token-efficient parallel full verification gate
# Runs all 4 verification targets in parallel: native unit tests, integration tests, STM32 build, ESP32 build.
# Emits compact summary on success to avoid token context inflation.
set -eo pipefail

PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"
if [ ! -x "$PIO" ]; then
    echo "Error: PlatformIO CLI not found at $PIO" >&2
    exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

WITH_ESP32="${WITH_ESP32:-0}"
for arg in "$@"; do
    case "$arg" in
        --with-esp32|--esp32|-a|--all)
            WITH_ESP32=1
            ;;
    esac
done

START_TIME=$(date +%s)
echo "=== OpenCanbox Verification Gate (Parallel) ==="

TMP_DIR="$(mktemp -d /tmp/opencanbox_verify.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT

# Launch core verification targets concurrently
"$PIO" test -e native_test_runner > "$TMP_DIR/unit.log" 2>&1 &
PID_UNIT=$!

"$PIO" test -e integration_test > "$TMP_DIR/int.log" 2>&1 &
PID_INT=$!

"$PIO" run -s -e stm32_cbox > "$TMP_DIR/stm32.log" 2>&1 &
PID_STM=$!

"$PIO" run -s -e raise_nuc131 > "$TMP_DIR/nuc131.log" 2>&1 &
PID_NUC=$!

PID_ESP=""
if [ "$WITH_ESP32" = "1" ]; then
    "$PIO" run -s -e esp32_cbox > "$TMP_DIR/esp32.log" 2>&1 &
    PID_ESP=$!
fi

# Await completion and record exit statuses
STATUS_UNIT=0; wait $PID_UNIT || STATUS_UNIT=$?
STATUS_INT=0; wait $PID_INT || STATUS_INT=$?
STATUS_STM=0; wait $PID_STM || STATUS_STM=$?
STATUS_NUC=0; wait $PID_NUC || STATUS_NUC=$?

STATUS_ESP=0
if [ -n "$PID_ESP" ]; then
    wait $PID_ESP || STATUS_ESP=$?
fi

FAILED=0
TOTAL_TARGETS=4

report_test_result() {
    local target_num="$1"
    local name="$2"
    local status="$3"
    local log="$4"

    echo -n "[$target_num] $name... "
    if [ "$status" -eq 0 ]; then
        local summary
        summary=$(grep -E "[0-9]+ test cases:.*succeeded" "$log" | tail -n 1 | tr -d '=' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || true)
        [ -z "$summary" ] && summary="PASSED"
        echo "PASSED ($summary)"
    else
        echo "FAILED!"
        local test_fails
        test_fails=$(grep -E "(\[FAILED\]|Expected .* Was )" "$log" || true)
        if [ -n "$test_fails" ]; then
            echo "$test_fails" | head -n 25
            local fail_sum
            fail_sum=$(grep -E "[0-9]+ test cases:.*failed" "$log" | tr -d '=' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || true)
            [ -n "$fail_sum" ] && echo "  -> $fail_sum"
        else
            local comp_errors
            comp_errors=$(grep -E "(error:|fatal error:|\*\*\* \[)" "$log" || true)
            if [ -n "$comp_errors" ]; then
                echo "$comp_errors" | head -n 25
            else
                tail -n 25 "$log"
            fi
        fi
        FAILED=1
    fi
}

report_build_result() {
    local target_num="$1"
    local name="$2"
    local status="$3"
    local log="$4"

    echo -n "[$target_num] $name... "
    if [ "$status" -eq 0 ]; then
        echo "PASSED"
    else
        echo "FAILED!"
        local comp_errors
        comp_errors=$(grep -E "(error:|fatal error:|\*\*\* \[)" "$log" || true)
        if [ -n "$comp_errors" ]; then
            echo "$comp_errors" | head -n 25
        else
            tail -n 25 "$log"
        fi
        FAILED=1
    fi
}

# 1. Native Unit Tests
report_test_result "1/4" "Native Unit Tests (native_test_runner)" "$STATUS_UNIT" "$TMP_DIR/unit.log"

# 2. Integration Tests
report_test_result "2/4" "Integration Tests (integration_test)" "$STATUS_INT" "$TMP_DIR/int.log"

# 3. STM32 Firmware Build (Hiworld Hardware)
report_build_result "3/4" "STM32 Firmware Target (stm32_cbox)" "$STATUS_STM" "$TMP_DIR/stm32.log"

# 4. NUC131 Firmware Build (Raise Hardware)
report_build_result "4/4" "NUC131 Firmware Target (raise_nuc131)" "$STATUS_NUC" "$TMP_DIR/nuc131.log"

# Optional: ESP32 Firmware Build
if [ "$WITH_ESP32" = "1" ]; then
    TOTAL_TARGETS=5
    report_build_result "5/5" "ESP32 Firmware Target (esp32_cbox)" "$STATUS_ESP" "$TMP_DIR/esp32.log"
fi

if [ $FAILED -ne 0 ]; then
    exit 1
fi

ELAPSED=$(($(date +%s) - START_TIME))
echo "=== All ${TOTAL_TARGETS} verification targets passed in ${ELAPSED}s ==="
