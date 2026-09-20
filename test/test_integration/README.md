# Scenario-Based CAN Log Integration Testing

This directory contains end-to-end integration tests that replay real-world automotive CAN bus captures against the **OpenCanbox Core** firmware pipeline and verify decoded vehicle state transitions and serial Head Unit (Raise / Hiworld) outputs.

---

## 1. Overview & Architecture

Scenario tests replay recorded `.csv` CAN logs chronologically through the router and vehicle profile decoders using a **virtual time engine** (`can_log_player`).

```
+-------------------------------------------------------------------------------------------------+
|                                     CAN LOG PLAYER ENGINE                                       |
|                                                                                                 |
|   +------------------------------------+             +--------------------------------------+   |
|   |         CSV Log Streamer           |             |         Virtual Clock Engine         |   |
|   |  - Reads Time Stamp (µs), CAN ID,  |             |  - Calculates simulated delta-t      |   |
|   |    DLC, and Data Bytes D1..D8      |             |  - Fires can_router_periodic_100ms() |   |
|   +-----------------+------------------+             +------------------+-------------------+   |
|                     |                                                   |                       |
|                     | can_router_process_can(&frame)                    | 100ms timer ticks     |
|                     v                                                   v                       |
|   +-----------------------------------------------------------------------------------------+   |
|   |                                     CANBOX CORE                                         |   |
|   |  - vehicle_profile (PSA, VAG, etc.) -> Decodes raw signals into vehicle_state_t         |   |
|   |  - can_router -> Dispatches events to Head Unit protocol driver (Raise / Hiworld)       |   |
|   +-----------------------------------------------------------------------------------------+   |
|                                             |                                                   |
|                                             v                                                   |
|   +-----------------------------------------------------------------------------------------+   |
|   |                               SCENARIO STEP ASSERTIONS                                  |   |
|   |  - Step 1: Pre-ignition state verification                                              |   |
|   |  - Step 2: Ignition / ACC event transition assertions                                   |   |
|   |  - Step 3: Steady-state periodic telemetry & virtual clock progression assertions        |   |
|   |  - Step 4: Shutdown / settled-off state assertions                                      |   |
|   +-----------------------------------------------------------------------------------------+   |
+-------------------------------------------------------------------------------------------------+
```

### Key Highlights:
* **Zero Real-Time Delays:** Log playback uses virtual time. An 86-second log with 10,000+ frames executes in **under 10 milliseconds**.
* **Precise Scenario Milestones:** Inspect intermediate state by advancing to specific timestamps or CAN IDs.
* **Pure C99:** Runs natively on Linux host under PlatformIO with Unity test runner.

---

## 2. CAN Log File Format

Place all raw CAN capture files in `test/test_integration/data/` using standard 14-column CSV formatting:

```csv
Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
48977493,00000190,false,Rx,0,8,FF,FF,02,77,FF,FF,FF,FF,
48978640,00000110,false,Rx,0,8,FF,FF,FF,FF,00,00,00,00,
48978652,000001D0,false,Rx,0,7,08,00,00,00,00,0B,0B,00,
49363170,00000036,false,Rx,0,8,0E,00,00,0F,02,00,00,50,
```

### Column Specification:
| Column | Type | Example | Description |
|:---|:---|:---|:---|
| **Time Stamp** | Integer (µs) | `48977493` | Microseconds timestamp from CAN logger |
| **ID** | Hex (8 chars) | `00000190` | CAN Arbitration ID (`0x190`) |
| **Extended** | Boolean | `false` | `true` for 29-bit CAN 2.0B, `false` for 11-bit CAN 2.0A |
| **Dir** | String | `Rx` / `Tx` | Direction of frame |
| **Bus** | Integer | `0` | CAN channel index |
| **LEN** | Integer (0..8) | `8` | Data Length Code (DLC) |
| **D1..D8** | Hex (2 chars) | `0E, 00, ...` | Payload data bytes (1-indexed) |

---

## 3. The `can_log_player` API Reference

Include the player header in your test file:
```c
#include "can_log_player.h"
```

### Lifecycle Functions
```c
// Opens the CSV log file and initializes playback state
bool can_log_player_open(can_log_player_t *player, const char *csv_path);

// Closes the file handle
void can_log_player_close(can_log_player_t *player);
```

### Step & Replay Functions
```c
// Replays a single CAN frame from the log
bool can_log_player_step(can_log_player_t *player);

// Replays frames sequentially until target timestamp (in microseconds) is reached
uint32_t can_log_player_replay_until_timestamp(can_log_player_t *player, uint64_t target_timestamp_us);

// Replays frames until a frame matching the specified CAN ID is processed
uint32_t can_log_player_replay_until_can_id(can_log_player_t *player, uint32_t can_id);

// Replays all remaining frames in the log
uint32_t can_log_player_replay_all(can_log_player_t *player);
```

### Player State Attributes
Inspect progress during test assertions:
* `player.frames_processed` — Total number of CAN frames dispatched so far.
* `player.simulated_elapsed_us` — Total simulated log time elapsed ($\mu s$).
* `player.periodic_ticks_fired` — Total number of $100\text{ms}$ periodic cycles executed.

---

## 4. How to Write a New Scenario Test: Step-by-Step

### Step 1: Add your CAN log capture
Copy your `.csv` log into `test/test_integration/data/`:
```bash
cp /path/to/my_doors_scenario.csv test/test_integration/data/doors_open_close.csv
```

### Step 2: Create a scenario test file
Create `test/test_integration/test_scenario_<name>.c` using this template:

```c
#include "unity.h"
#include "can_log_player.h"
#include "core/can_router.h"

static const char *LOG_PATH = "test/test_integration/data/doors_open_close.csv";

void test_scenario_doors_open_close(void) {
    can_log_player_t player;
    bool opened = can_log_player_open(&player, LOG_PATH);
    TEST_ASSERT_TRUE_MESSAGE(opened, "Failed to open CAN log CSV file");

    can_router_init();

    // -------------------------------------------------------------
    // Step 1: Baseline / Initial State
    // Replay initial frames before the action begins
    // -------------------------------------------------------------
    can_log_player_replay_until_timestamp(&player, 1000000ULL); // 1.0s mark
    const vehicle_state_t *state = can_router_get_state();

    TEST_ASSERT_FALSE(state->doors.door_driver);
    TEST_ASSERT_FALSE(state->doors.trunk);

    // -------------------------------------------------------------
    // Step 2: Action Trigger (Driver Door Opened)
    // Replay until door frame (e.g., 0x036 or 0x220) arrives
    // -------------------------------------------------------------
    can_log_player_replay_until_can_id(&player, 0x036);
    TEST_ASSERT_TRUE(state->doors.door_driver);

    // -------------------------------------------------------------
    // Step 3: Sustained Activity & Time Progression
    // Replay through the remainder of the scenario
    // -------------------------------------------------------------
    can_log_player_replay_all(&player);

    // Assert final settled state
    TEST_ASSERT_FALSE(state->doors.door_driver);
    TEST_ASSERT_GREATER_THAN(0, player.frames_processed);

    can_log_player_close(&player);
}
```

### Step 3: Register the test in `test_integration.c`
In `test/test_integration/test_integration.c`:

1. Declare the test function prototype:
```c
void test_scenario_doors_open_close(void);
```

2. Add `RUN_TEST` inside `main()`:
```c
int main(void) {
    UNITY_BEGIN();
    // Existing tests...
    RUN_TEST(test_scenario_ignition_off_after_power_on);
    RUN_TEST(test_scenario_doors_open_close);
    return UNITY_END();
}
```

### Step 4: Run the test suite
Execute via PlatformIO:
```bash
~/.platformio/penv/bin/pio test -e integration_test
```

---

## 5. Best Practices & Guidelines

1. **Keep Tests Non-Destructive and Non-Blocking:**
   Never use `sleep()` or `usleep()` inside tests. Let `can_log_player` manage time simulation.
2. **Step by Logical Milestones:**
   Divide the test into distinct phases (Boot $\to$ User Action $\to$ Steady State $\to$ Power Down). This makes failure diagnostics immediate and clear.
3. **Verify Both Core State and Serial Protocol:**
   Assert decoded state via `can_router_get_state()` and verify that the virtual clock drives periodic broadcasts (`periodic_ticks_fired`).
4. **Boundary Checks:**
   Always verify frame count and elapsed microseconds at the conclusion of a scenario test using `TEST_ASSERT_EQUAL_UINT32(expected_frames, player.frames_processed)`.

