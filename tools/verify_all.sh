#!/usr/bin/env bash
# tools/verify_all.sh - Token-efficient full verification gate
# Runs all 4 verification targets: native unit tests, integration tests, STM32 build, ESP32 build.
# Emits compact summary on success to avoid token context inflation.
set -eo pipefail

PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"
if [ ! -x "$PIO" ]; then
    echo "Error: PlatformIO CLI not found at $PIO" >&2
    exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

START_TIME=$(date +%s)
echo "=== OpenCanbox Verification Gate ==="

# 1. Native Unit Tests
echo -n "[1/4] Native Unit Tests (native_test_runner)... "
UNIT_OUT=$("$PIO" test -e native_test_runner 2>&1) || {
    echo "FAILED!"
    echo "$UNIT_OUT" | grep -E "FAILED|ERROR|test_.*\[" | head -n 30
    exit 1
}
SUMMARY_LINE=$(echo "$UNIT_OUT" | grep -E "SUMMARY" -A 1 | tail -n 1 || true)
echo "PASSED ($SUMMARY_LINE)"

# 2. Integration Tests
echo -n "[2/4] Integration Tests (integration_test)... "
INT_OUT=$("$PIO" test -e integration_test 2>&1) || {
    echo "FAILED!"
    echo "$INT_OUT" | grep -E "FAILED|ERROR|test_.*\[" | head -n 30
    exit 1
}
INT_SUMMARY=$(echo "$INT_OUT" | grep -E "SUMMARY" -A 1 | tail -n 1 || true)
echo "PASSED ($INT_SUMMARY)"

# 3. STM32 Firmware Build
echo -n "[3/4] STM32 Firmware Target (stm32_cbox)... "
STM_OUT=$("$PIO" run -s -e stm32_cbox 2>&1) || {
    echo "FAILED!"
    echo "$STM_OUT" | head -n 30
    exit 1
}
echo "PASSED"

# 4. ESP32 Firmware Build
echo -n "[4/4] ESP32 Firmware Target (esp32_cbox)... "
ESP_OUT=$("$PIO" run -s -e esp32_cbox 2>&1) || {
    echo "FAILED!"
    echo "$ESP_OUT" | head -n 30
    exit 1
}
echo "PASSED"

ELAPSED=$(($(date +%s) - START_TIME))
echo "=== All 4 verification targets passed in ${ELAPSED}s ==="

