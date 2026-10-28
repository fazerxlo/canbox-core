---
name: debug-canbox-firmware-live
description: >-
  Debug, flash, verify, and trace OpenCanbox firmware (NUC131/STM32 via ST-Link), PSA Comfort CAN bus
  (can0 connected to Physical BSI or Simulator, including switching between them), and Android Head Unit
  (com.qf.vehicle over ADB). Enforces non-destructive live diagnosis before any flash or restart.
---

# Live Hardware Debugging: HU (ADB) $\leftrightarrow$ Canbox (ST-Link) $\leftrightarrow$ BSI / Simulator (`can0`)

End-to-end runbook for live debugging, multi-stream capture, firmware/APK deployment, and verification across:
1. **CAN Bus (`can0` @ 125 kbps):** Connected to either a **Physical Peugeot 407 BSI** or the **CAN2004 Simulator** (`../simulator`), supporting live switching (`BSI <-> Simulator`) to isolate root causes and verify fixes.
2. **Canbox MCU (NUC131 / STM32):** Connected via **ST-Link V2** (`openocd_nuc131.cfg` + `tools/nuc131_flasher.py`) and UART (38,400 8N1).
3. **Android Head Unit (`com.qf.vehicle` in `../QF_Canbus_system`):** Monitored/controlled over wireless ADB (`192.168.1.33:9876`).

---

## 0. CRITICAL RULE: Non-Destructive Diagnosis First (NEVER Flash/Restart Without User Approval)

> [!CAUTION]
> **Live State Preservation Gate:** Many vehicle/UI bugs only occur in a specific runtime state and **disappear if the Canbox MCU or Head Unit app restarts**.
> Until you have captured the live bug state AND the user **explicitly agrees** to flash, deploy, or restart:
> 1. **DO NOT start OpenOCD or run `nuc131_flasher.py`** (connecting OpenOCD with `reset-init` or flashing immediately halts/resets the NUC131 MCU and wipes RAM state).
> 2. **DO NOT run `build_and_deploy.sh`, `adb_upload.sh`, `adb install`, or `am force-stop com.qf.vehicle`** (restarts the Android app and clears UI/service state; use `--no-deploy` if pre-building).
> 3. **DO NOT inject CAN frames (`cansend`), wakeup pulses, or start/stop Simulator** while diagnosing a live bug unless asked.
> 4. **Start with Passive Capture Only:** Use `candump`, `adb logcat -d`, `adb shell uiautomator dump`, `adb shell screencap`, or `tools/record_repro_session.py`.

---

## 1. Sibling Repositories & Fast Tooling Map

Use relative paths from `canbox-core` workspace root:

| Component | Relative Path | Key Tools & Mandatory Token-Saving Rules |
| :--- | :--- | :--- |
| **Canbox Firmware** | `.` (`canbox-core`) | Build: `~/.platformio/penv/bin/pio run -s -e raise_nuc131`<br>Flash App (`0x2000`, **only after user approval**): `python3 tools/nuc131_flasher.py .pio/build/raise_nuc131/program.bin 0x2000` |
| **HU Android APK** (`com.qf.vehicle`) | `../QF_Canbus_system` | **Read `../QF_Canbus_system/AGENTS.md` before modifying APK files.**<br>- Extract 1 Smali method: `python3 ../QF_Canbus_system/tools/extract_smali_method.py <file.smali> <method>` (never read whole Smali files)<br>- Lookup/add resource IDs: `python3 ../QF_Canbus_system/tools/add_res_id.py --lookup <name>` (never read `public.xml`/`ids.xml`)<br>- Update changelog: `python3 ../QF_Canbus_system/tools/add_changelog.py` (never read `CHANGELOG.md`)<br>- Build only (safe): `../QF_Canbus_system/tools/build_and_deploy.sh --no-deploy`<br>- Build + ADB Deploy (**only after user approval**): `../QF_Canbus_system/tools/build_and_deploy.sh` |
| **CAN2004 Simulator** | `../simulator` | Virtualenv: `../simulator/.venv/bin/python`<br>- Canonical signal DB: `../simulator/signal-db/*.yaml` (never edit `generated/*.py` manually)<br>- Regenerate encoders: `../simulator/.venv/bin/python ../simulator/signal-db/codegen/gen_py_encoder.py` |
| **PSA Lab Scripts** | `../psa-can-lab` | Signal audits & bench verification helpers (`../psa-can-lab/tools/`). |

---

## 2. Passive 1-Turn Pre-Flight & Bus Source Auto-Detection

Run this **100% passive, non-destructive** check in a single turn to verify **ADB**, **`can0`**, and detect whether **`can0` is on Physical BSI or Simulator**:

```bash
# 1. Check / auto-connect ADB (non-destructive, does not restart app)
adb devices | grep -q "192.168.1.33:9876.*device" || adb connect 192.168.1.33:9876
adb devices

# 2. Check can0 interface state
ip link show can0

# 3. Passive auto-detect: SIMULATOR vs PHYSICAL BSI vs IDLE/SLEEP
pgrep -fa "simulator.*(app|can_runner)\.py" && echo "MODE: SIMULATOR_PROCESS_ACTIVE" || echo "MODE: NO_LOCAL_SIMULATOR"
timeout 1.5s candump -tz can0 -n 20
```

### Connection Failure Stop-and-Ask Rules
1. **ADB Offline / Unreachable:** If `adb devices` does not list `192.168.1.33:9876	device` after one `adb connect 192.168.1.33:9876`, **STOP immediately and ask the user to connect/enable ADB on the Head Unit**. Never scan IPs or loop.
2. **`can0` Down / Missing:** If `ip link show can0` is `DOWN` or missing, try `../simulator/slcan.sh` (or `slcand -o -c -s4 /dev/ttyUSB0 can0 && ip link set up can0 txqueuelen 1000`). If `/dev/ttyUSB*` is absent or requires root (`sudo`), **STOP and ask the user** to bring up `can0`.
3. **ST-Link Disconnected:** When authorized to flash, if OpenOCD fails with `Error: open failed` / `libusb_open failed`, **STOP and ask the user to check the ST-Link USB/SWD connection**.

---

## 3. Detecting & Switching `Physical BSI <-> Simulator`

| Pre-Flight Observation | Detected Mode | Rules & Collision Protection |
| :--- | :--- | :--- |
| `pgrep` shows `simulator` (`app.py` / `can_runner.py`) | **SIMULATOR MODE** | Safe to adjust simulator signals or inject `cansend can0 <ID>#<HEX>` (once live diagnosis is complete). |
| No simulator process + `candump` shows periodic `0x036`, `0x0F6`, `0x128`, `0x260`, `0x361` | **PHYSICAL BSI MODE (Awake)** | **COLLISION HAZARD:** Never start Simulator or transmit periodic frames on BSI-owned IDs (`0x036`, `0x0F6`, `0x128`, `0x15B`, `0x260`, `0x361`, `0x165`). Use passive `candump`, HU UI taps, or one-shot `cansend` only on absent ECUs (when approved). |
| No simulator process + `candump` returns 0 frames | **BSI ASLEEP (Economy) OR IDLE BUS** | Do **not** inject wakeup frames if diagnosing a sleep/standby bug. Otherwise (with user consent), send one wakeup pulse (`cansend can0 036#0E00000F010000A0 && cansend can0 0F6#8800000000500020`) and sniff 1s: if `0x260`/`0x361` appear $\rightarrow$ **PHYSICAL BSI**; if silent $\rightarrow$ **IDLE BUS**. |

### Switching Workflow (`BSI -> Simulator -> BSI`)
1. **Observe on Physical BSI:** Passively capture live CAN + ADB logcat + UI state (`uiautomator dump`) without resetting MCU or HU.
2. **Switch to Simulator (Isolate Root Cause / Fuzz States):** Ask user to disconnect/switch off Physical BSI (or confirm `timeout 1.0s candump can0,036:7FF,260:7FF` is silent), then use Simulator or batched `cansend` vectors to reproduce and test fixes.
3. **Switch Back to Physical BSI (Final Verification):** Stop simulator (`pkill -f "simulator.*(app|can_runner)\.py" 2>/dev/null || true`), ask user to reconnect/power on Physical BSI, verify clean BSI traffic (`candump -tz can0,036:7FF,260:7FF -n 6`), and confirm the fix on real hardware.

### 3.1 Simulator Parity, Gating & Configuration Gotchas (Avoid False Firmware RCA)
When a feature works on Physical BSI but fails in Simulator Mode (e.g., missing trip telemetry, HVAC status, or stalk keys):
1. **Always verify `can0` first:** Run `timeout 1.5s candump can0,<CAN_ID>:7FF`. Never assume the Canbox firmware or Android Head Unit is broken until you prove the CAN frame is actually on the wire.
2. **One-shot CAN injection verification (`cansend`):** Inject a synthetic frame (e.g. `cansend can0 221#000044028004B0`). If Canbox MCU immediately decodes it and sends the UART uplink (`Cmd 0x13`/`0x1A`), the Canbox firmware and HU app are functioning correctly; the bug is 100% inside the Simulator.
3. **Common Simulator failure modes:**
   - **Disabled module in `../simulator/config.yml`:** The module providing or gating the frame may be commented out (e.g. `#- swc`, `#- clim`).
   - **Overly restrictive `required_modules` in `../simulator/signal-db/*.yaml`:** When messages span multiple ECUs/domains (e.g., `Msg221` carries both BSI trip calculations and SWC wiper tip buttons), ensure `required_modules` includes all transmitting modules (`bsi-trip`, `swc`).
   - **Unregenerated encoders:** If `signal-db/*.yaml` is modified, run `../simulator/.venv/bin/python ../simulator/signal-db/codegen/gen_py_encoder.py` to update `generated/*_messages.py`.
   - **UI tab state:** Some simulator messages require an explicit UI tab to be loaded or active before scheduling.

---

## 4. Non-Destructive Live Capture & UI Inspection

### 4.1 Fast UI Inspection via `uiautomator dump` (Preferred Over OCR)
Returns exact Android `resource-id` and `text` strings in 1 command without OCR errors:
```bash
adb -s 192.168.1.33:9876 shell uiautomator dump /sdcard/window_dump.xml >/dev/null && \
adb -s 192.168.1.33:9876 shell cat /sdcard/window_dump.xml | python3 -c "
import sys, xml.etree.ElementTree as ET
for n in ET.fromstring(sys.stdin.read()).iter('node'):
    r, t = n.attrib.get('resource-id', ''), n.attrib.get('text', '')
    if r or t: print(f'{r:45s} | {t}')
"
```

### 4.2 Fast Screenshot (< 3s Timed Popups)
Direct `exec-out screencap` takes ~2.6s over WiFi and misses 2.5s popups. Always capture to `/sdcard/` first (~100 ms):
```bash
adb -s 192.168.1.33:9876 shell screencap -p /sdcard/_cap.png && adb -s 192.168.1.33:9876 pull /sdcard/_cap.png /tmp/screen_verify.png
```

### 4.3 Non-Destructive Logcat & CAN Sniffing
```bash
# Dump recent UART uplink/downlink & parser logs without clearing buffer:
adb -s 192.168.1.33:9876 logcat -d -t 200 | grep -E "CAN_DEBUG|CANBOX_UART_TX|HU_UART_TX|PeugeotData|handleVehicleData|writeData" | tail -n 40

# Targeted CAN sniffing & baseline/action delta diffing:
candump -tz can0,15B:7FF,260:7FF,361:7FF -n 15
python3 tools/diff_canbox_frames.py --can-interface can0 --filter 15B,361

# Correlated multi-stream recorder (microsecond CAN + UART + Android audio/UI events):
./tools/record_repro_session.py --can can0 --adb 192.168.1.33:9876
```

### 4.4 Fast-Path Root Cause Tracing: Local 4-Layer Hierarchy (Avoid Smali Traps)
When an unexpected popup, UI glitch, or periodic state update appears on the HU:
1. **Extract wire frame from logcat:** Identify the Hiworld command (`0x5A 0xA5 <len> <cmd> <payload...>`).
2. **Consult local specs first:** Match `<cmd>` in the Subsystem Lookup Table (`AGENTS.md`) and read `doc/CANBOX_SPEC_*.md`.
3. **Trace wire builder:** Search `src/protocols/hiworld_psa_builders.c` to see which field in `vehicle_state_t` drives the frame or popup bit.
4. **Trace router & periodic dispatch:** Check `src/core/can_router.c` (`dirty.*` or periodic sync / heartbeat) to see why the packet is emitted periodically.
5. **Trace decoder:** Check `src/profiles/psa_407_*.c` to see how the incoming CAN frame sets the state field.
> [!IMPORTANT]
> **Do NOT search or decompile Android Smali (`../QF_Canbus_system`)** when the UART command ID and byte meanings are already defined in `canbox-core` specs and builders. Reserve smali audits strictly for completely unhandled or unknown vendor commands.

---

## 5. Flashing Canbox MCU & Deploying HU APK (ONLY After User Approval)

### 5.1 NUC131 Build & Flash via ST-Link (`openocd_nuc131.cfg`)
* **Never** use stock `interface/stlink.cfg` or OpenOCD `flash write_image` (NUC131 chip ID `0x11413102` requires `openocd_nuc131.cfg` + `tools/nuc131_flasher.py`, which automatically clears the boot token and runs `reset run`).
* **One-Shot Build, Flash Application (`0x2000`) & Stop OpenOCD:**
  ```bash
  ~/.platformio/penv/bin/pio run -s -e raise_nuc131 && (pgrep -x openocd >/dev/null || (openocd -f openocd_nuc131.cfg -d0 >/tmp/openocd.log 2>&1 & sleep 0.5)) && python3 tools/nuc131_flasher.py .pio/build/raise_nuc131/program.bin 0x2000 && pkill -x openocd
  ```
* **Flash Bootloader (`0x0000`, only if requested):**
  ```bash
  ~/.platformio/penv/bin/pio run -s -e nuc131_bootloader && (pgrep -x openocd >/dev/null || (openocd -f openocd_nuc131.cfg -d0 >/tmp/openocd.log 2>&1 & sleep 0.5)) && python3 tools/nuc131_flasher.py .pio/build/nuc131_bootloader/program.bin 0x0000 && pkill -x openocd
  ```

### 5.2 HU APK Build, Sign & Deploy (`../QF_Canbus_system`)
```bash
# Pre-verify Smali (<1s) + Build + Sign + ADB Install (restarts com.qf.vehicle):
../QF_Canbus_system/tools/build_and_deploy.sh
```

---

## 6. Interactive Stimulation Recipes (When Approved)

* **Inject Synthetic Hiworld Frame into HU (`CanbusAlertDebugReceiver`):**
  ```bash
  adb -s 192.168.1.33:9876 shell am broadcast -a com.canbus.action.INJECT_CAN_PACKET -p com.qf.vehicle --es data "5AA50276060280"
  ```
* **Open HU Fragments / Activities:**
  - Settings: `adb -s 192.168.1.33:9876 shell am start -n com.qf.vehicle/.fragments.FragmentContainerActivity --es fragmentclass com.qf.vehicle.fragments.CentralSettingFragment`
  - TPMS (index `2`): `adb -s 192.168.1.33:9876 shell am start -n com.qf.vehicle/.fragments.FragmentContainerActivity --ei fragment_index 2`
  - Radio / CD: `adb -s 192.168.1.33:9876 shell am start -n com.qf.vehicle/com.qf.vehicle.activities.OriginalTuner` (or `OriginalCD`)
  - Clear Settings Visibility Cache: `adb -s 192.168.1.33:9876 shell "am force-stop com.qf.vehicle && rm -f /data/data/com.qf.vehicle/shared_prefs/centralstate_enable_db.xml"`

---

## 7. Hardware Troubleshooting Quick Table

| Symptom | Cause | Fast Fix |
| :--- | :--- | :--- |
| `auto_probe failed` / `Device ID: 0x11413102` | Stock OpenOCD flash driver used | Always use `openocd -f openocd_nuc131.cfg` + `python3 tools/nuc131_flasher.py`. |
| `libusb_open failed` / Target not halting | Hung OpenOCD or ST-Link USB state | `pkill -9 openocd`; if still failing, ask user to replug ST-Link USB. |
| `can0: No such device` | SLCAN unplugged or `slcand` exited | Run `../simulator/slcan.sh` or ask user to reconnect CAN adapter. |
| `RD4 resets on CD spinup (5E0#20 1E)` | Bench 12V transient current drop | Expected bench power behavior; `canbox-core` absorbs `00 00 00 00` reboot frames. |
| Telemetry missing in Simulator mode | Module disabled in `config.yml` or gated in `signal-db` | Check `candump can0,<id>:7FF`; verify `config.yml` and `required_modules` in `signal-db/*.yaml`. |
