#!/usr/bin/env python3
"""
triage_svt_failures.py — Ultra-Compact, Token-Optimized Failure Triage for OEM CAN Box SVT Runs.

Eliminates exploratory LLM tool calls (zero grep, zero python3 -c log parsing, zero multi-span file reads)
by automatically resolving:
  1. Exact source file & line numbers (`file:StartLine-EndLine`) for every failed test and protocol decoder.
  2. Exact scenario-scoped CAN stimuli & UART RX frames (`raw_hex` + decoded dict) within `[t_test - 1.5s .. t_test + 0.5s]`.
  3. Root-cause classification across Raise, Hiworld, SimpleSoft, and OpenCanbox protocols.

Usage:
  python3 tools/triage_svt_failures.py [--latest | RUN_DIR]
  python3 tools/triage_svt_failures.py --latest --test RADAR-03
  python3 tools/triage_svt_failures.py --latest --no-color
"""

import os
import re
import sys
import glob
import json
import argparse
import datetime
from typing import Dict, List, Any, Optional, Tuple


def find_latest_run(base_dirs: Tuple[str, ...] = ("svt/runs", "runs")) -> Optional[str]:
    """Find the most recent directory containing report.json."""
    candidates = []
    for b in base_dirs:
        for rpath in glob.glob(os.path.join(b, "*", "report.json")):
            candidates.append(os.path.dirname(rpath))
    if not candidates:
        return None
    candidates.sort(key=lambda d: os.path.getmtime(os.path.join(d, "report.json")), reverse=True)
    return candidates[0]


def detect_dut_proto(dut_name: str) -> str:
    """Resolve protocol family from DUT profile name."""
    d = dut_name.lower()
    if "raise" in d or "rzc" in d:
        return "raise"
    if "simplesoft" in d or "xp" in d:
        return "simplesoft"
    if "hiworld" in d or "opencanbox" in d:
        return "hiworld"
    return "generic"


def locate_test_source_span(test_id: str, suite_name: str, repo_root: str = ".") -> str:
    """
    Find the exact file and line range (`StartLine-EndLine`) for a test_id in `svt/`
    so the LLM agent can call `read_file` directly in Turn 1 without grepping.
    """
    search_files = []
    if suite_name:
        cand = os.path.join(repo_root, "svt", f"subsystem-{suite_name}.py")
        if os.path.exists(cand):
            search_files.append(cand)
        cand2 = os.path.join(repo_root, "svt", f"base-bsi-{suite_name}.py")
        if os.path.exists(cand2):
            search_files.append(cand2)

    for p in sorted(glob.glob(os.path.join(repo_root, "svt", "*.py"))):
        if p not in search_files:
            search_files.append(p)

    pat = re.compile(re.escape(test_id), re.IGNORECASE)
    for fpath in search_files:
        try:
            with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()
            for idx, line in enumerate(lines):
                if pat.search(line):
                    # Walk backward to enclosing `def ` if nearby
                    start = max(1, idx + 1 - 8)
                    for b in range(idx, max(-1, idx - 35), -1):
                        if lines[b].lstrip().startswith("def "):
                            start = b + 1
                            break
                    end = min(len(lines), start + 65)
                    rel = os.path.relpath(fpath, repo_root)
                    return f"{rel}:{start}-{end}"
        except Exception:
            continue
    return f"svt/subsystem-{suite_name}.py:1-90"


def locate_proto_decoder(dut_proto: str, repo_root: str = ".") -> str:
    """Return relative path to protocol decoder file."""
    p = os.path.join(repo_root, "svt", "lib", "protocols", f"{dut_proto}.py")
    if os.path.exists(p):
        return os.path.relpath(p, repo_root)
    return "svt/lib/protocols/"


def parse_execution_log(log_path: str) -> Dict[str, Dict[str, float]]:
    """Extract suite start and end epoch timestamps from svt_execution.log."""
    suite_windows: Dict[str, Dict[str, float]] = {}
    if not os.path.exists(log_path):
        return suite_windows

    entries = []
    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "svt.harness: [" in line and "Reusing shared active CAN bus" in line:
                parts = line.split()
                if len(parts) >= 2:
                    dt_str = f"{parts[0]} {parts[1].replace(',', '.')}"
                    try:
                        t_epoch = datetime.datetime.fromisoformat(dt_str).timestamp()
                        s_name = line.split("svt.harness: [")[1].split("]")[0]
                        entries.append((t_epoch, s_name))
                    except Exception:
                        pass
            elif "Tearing down shared baseline" in line or "Test execution interrupted" in line:
                parts = line.split()
                if len(parts) >= 2:
                    dt_str = f"{parts[0]} {parts[1].replace(',', '.')}"
                    try:
                        t_epoch = datetime.datetime.fromisoformat(dt_str).timestamp()
                        entries.append((t_epoch, "_END_"))
                    except Exception:
                        pass

    for i in range(len(entries)):
        t_start, s_name = entries[i]
        if s_name == "_END_":
            continue
        t_end = entries[i + 1][0] if i + 1 < len(entries) else t_start + 15.0
        suite_windows[s_name] = {"start": t_start, "end": t_end, "duration": t_end - t_start}

    return suite_windows


def index_uart_log(uart_path: str) -> Dict[str, Any]:
    """Index all UART RX and DUT CAN TX events with timestamps for fast time-window slicing."""
    summary: Dict[str, Any] = {
        "total_lines": 0,
        "dut_events_count": 0,
        "t_first": None,
        "t_last": None,
        "cmd_freq": {},
        "all_events": [],
        "dut_can_tx": [],
        "max_silence_sec": 0.0,
    }
    if not os.path.exists(uart_path):
        return summary

    prev_t = None
    with open(uart_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            summary["total_lines"] += 1
            try:
                ev = json.loads(line)
            except Exception:
                continue

            t = ev.get("timestamp")
            if t is None:
                continue

            name = ev.get("name", "")
            raw = ev.get("raw_hex", "")
            val = ev.get("value", {})

            if name == "dut_can_tx":
                summary["dut_can_tx"].append({"t": t, "raw": raw, "val": val})
                continue

            summary["dut_events_count"] += 1
            if summary["t_first"] is None:
                summary["t_first"] = t
            summary["t_last"] = t

            cmd = raw[:6] if raw.startswith("5AA5") and len(raw) >= 8 else (raw[:4] if len(raw) >= 4 else "UNKN")
            summary["cmd_freq"][cmd] = summary["cmd_freq"].get(cmd, 0) + 1

            if prev_t is not None:
                gap = t - prev_t
                if gap > summary["max_silence_sec"]:
                    summary["max_silence_sec"] = gap
            prev_t = t

            summary["all_events"].append({
                "t": t,
                "name": name,
                "raw": raw,
                "val": val,
            })

    return summary


def index_can_log(can_path: str) -> Dict[str, Any]:
    """Index CAN frames and payload transitions for fast scenario window slicing."""
    summary: Dict[str, Any] = {
        "total_frames": 0,
        "t_first": None,
        "t_last": None,
        "can_ids": set(),
        "corrupted_220": False,
        "transitions": [],  # Only records when a CAN ID's payload changes (eliminates cyclic noise)
    }
    if not os.path.exists(can_path):
        return summary

    last_payload_by_id: Dict[str, str] = {}
    with open(can_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            summary["total_frames"] += 1
            if "(" in line and ")" in line and "#" in line:
                try:
                    t_str = line.split("(")[1].split(")")[0]
                    t = float(t_str)
                    if summary["t_first"] is None:
                        summary["t_first"] = t
                    summary["t_last"] = t

                    left, payload = line.split("#", 1)
                    payload = payload.strip()
                    can_id = left.split()[-1].upper().zfill(3)
                    summary["can_ids"].add(can_id)

                    if can_id == "220" and len(payload) >= 2:
                        b0 = int(payload[:2], 16)
                        if (b0 & 0x04) == 0:
                            summary["corrupted_220"] = True

                    if last_payload_by_id.get(can_id) != payload:
                        last_payload_by_id[can_id] = payload
                        summary["transitions"].append((t, can_id, payload))
                except Exception:
                    continue

    return summary


def slice_window_evidence(
    t_test: Optional[float],
    suite_win: Optional[Dict[str, float]],
    uart_info: Dict[str, Any],
    can_info: Dict[str, Any],
    pre_sec: float = 2.0,
    post_sec: float = 0.5,
) -> Dict[str, List[str]]:
    """
    Extract a deduplicated, token-capped snapshot of CAN transitions and UART packets
    around the exact failure timestamp `t_test` (or suite window fallback).
    """
    if t_test is not None:
        w_start, w_end = t_test - pre_sec, t_test + post_sec
    elif suite_win:
        w_start, w_end = suite_win["start"], suite_win["end"]
    else:
        return {"can_deltas": [], "uart_rx": [], "dut_can_tx": []}

    can_deltas = []
    seen_can = set()
    for t, cid, pay in can_info.get("transitions", []):
        if w_start <= t <= w_end:
            key = f"{cid}#{pay}"
            if key not in seen_can:
                seen_can.add(key)
                can_deltas.append(f"0x{cid}#{pay}")

    uart_rx = []
    seen_uart = set()
    for ev in uart_info.get("all_events", []):
        if w_start <= ev["t"] <= w_end:
            # Skip repetitive keepalive ACKs unless nothing else arrived
            if ev["name"] in ("heartbeat_ack", "ack") and len(uart_rx) > 0:
                continue
            sig = f"{ev['name']}:{ev['raw']}"
            if sig not in seen_uart:
                seen_uart.add(sig)
                val_compact = json.dumps(ev["val"], separators=(",", ":")) if ev["val"] else "{}"
                if len(val_compact) > 64:
                    val_compact = val_compact[:61] + "...}"
                uart_rx.append(f"{ev['raw']} ({ev['name']}={val_compact})")

    dut_tx = []
    seen_tx = set()
    for tx in uart_info.get("dut_can_tx", []):
        if w_start <= tx["t"] <= w_end:
            raw = tx["raw"]
            if raw not in seen_tx:
                seen_tx.add(raw)
                dut_tx.append(raw)

    return {
        "can_deltas": can_deltas[-6:],   # Cap to last 6 unique CAN transitions
        "uart_rx": uart_rx[-5:],         # Cap to last 5 unique UART packets
        "dut_can_tx": dut_tx[-4:],       # Cap to last 4 unique DUT CAN TX frames
    }


def diagnose_failure(
    test: Dict[str, Any],
    suite_name: str,
    suite_win: Optional[Dict[str, float]],
    uart_info: Dict[str, Any],
    can_info: Dict[str, Any],
    dut_proto: str = "raise",
) -> Tuple[str, str, str]:
    """Classify failure into 1 of 5 root-cause categories with concise, actionable fix guidance."""
    t_id = test.get("test_id", "")
    msg = test.get("message", "")

    if uart_info.get("dut_events_count", 0) == 0:
        return (
            "DUT_SLEEP_OR_SILENCE",
            "0 UART packets received across entire run (DUT asleep or unpowered).",
            "Power-cycle CAN box (+12V) and run `tools/svt_oem_preflight.py`.",
        )

    t_last_uart = uart_info.get("t_last")
    if suite_win and t_last_uart and t_last_uart < suite_win["start"]:
        return (
            "DUT_SLEEP_OR_SILENCE",
            f"UART stopped at t={t_last_uart:.1f}s before suite '{suite_name}' (t={suite_win['start']:.1f}s); ~17.5s watchdog expired.",
            "Power-cycle CAN box (+12V) and pass `--persist-baseline` to `run_svt.py`.",
        )

    # --- CLIMATE ---
    if "CLIM" in t_id:
        if t_id == "CLIM-02" and "dual_mode" in msg:
            return (
                "INVALID_TEST_DESIGN",
                "407 has native dual temps; OEM box outputs L/R temps independently without setting synthetic `dual_mode` bit.",
                "Assert independent `temp_driver` / `temp_passenger` values instead of `dual_mode is True`.",
            )
        if t_id == "CLIM-08":
            return (
                "INVALID_TEST_DESIGN",
                "Raise/OEM boxes broadcast outdoor temp in Cmd 0x36 (`outdoor_temp`), not inside climate Cmd 0x21/0x31.",
                "Update CLIM-08 assertion to accept `outdoor_temp` event (Cmd 0x36).",
            )
        if t_id == "CLIM-07" and "1D0" not in can_info.get("can_ids", set()):
            return (
                "INVALID_CAN_STIMULUS",
                "Rear defrost requires BSI companion frame 0x1D0 (B0 Bit 6 = 0x40) alongside 0x1E3.",
                "Inject 0x1D0 companion payload in CLIM-07.",
            )
        return (
            "INVALID_TEST_DESIGN",
            f"HVAC bitfield/scaling mismatch on `{dut_proto}`: {msg}",
            f"Align predicate in `svt/subsystem-climate.py` or `svt/lib/protocols/{dut_proto}.py` with captured UART hex.",
        )

    # --- RADAR & SAS ---
    if "RADAR" in t_id or "SAS" in t_id:
        if "SAS" in t_id:
            return (
                "INVALID_CAN_STIMULUS",
                f"SAS failure ({msg}): Comfort CAN uses 0x10B (signed BE16, 0.1 deg) while 0x305 is HS-CAN; also requires Reverse Gear in 0x0F6.",
                "Ensure both 0x10B and 0x305 are injected with Reverse Gear active (`0x0F6` B7=0x80), or check `sas_supported` in DUT YAML.",
            )
        return (
            "INVALID_TEST_DESIGN",
            f"Radar mismatch on `{dut_proto}` ({msg}): check Reverse Gear gate (`0x0F6`), debounce hold (>=300ms), or distance scale (Raise Cmd 0x32 0-7 vs SimpleSoft Cmd 0x22/0x23 vs Hiworld Cmd 0x41).",
            f"Inspect Window Evidence UART hex below and align `svt/lib/protocols/{dut_proto}.py` / `svt/subsystem-radar_sas.py`.",
        )

    # --- KEYS & SWC ---
    if "KEYS" in t_id:
        if t_id in ("KEYS-06", "KEYS-07", "KEYS-08", "KEYS-09", "KEYS-10"):
            return (
                "INVALID_TEST_DESIGN",
                f"Key/stalk mismatch on `{dut_proto}` ({msg}): check pulse-release (`legacy_keys: true`), Virtual EMF `0x167#82...` on wiper tip (`0x221` B0=0x08), or RD4 shortcut vs nav key split (`0x3E5`).",
                f"Check Window Evidence UART/CAN TX below and update `svt/subsystem-keys.py` or `svt/lib/protocols/{dut_proto}.py`.",
            )
        return (
            "INVALID_TEST_DESIGN",
            f"Key timing or code mapping mismatch: {msg}",
            f"Compare Window Evidence UART hex against `svt/lib/protocols/{dut_proto}.py`.",
        )

    # --- DOORS ---
    if "DOOR" in t_id:
        if can_info.get("corrupted_220"):
            return (
                "INVALID_CAN_STIMULUS",
                "CAN 0x220 injected without bit 2 (`0x04` body network awake flag), signaling bus sleep.",
                "Retain `| 0x04` in all 0x220 door payloads (`84 00`, `44 00`, `24 00`, `14 00`, `0C 00`).",
            )
        if dut_proto == "raise":
            return (
                "INVALID_CAN_STIMULUS",
                "Raise decodes doors from Cluster 0x14C / requires HU query `2E 8F 01 38 37`.",
                "Patch both 0x220 and 0x14C in `_patch_doors()` and ensure HU poll `0x8F 0x38` is active.",
            )

    # --- TRIP COMPUTER ---
    if "TRIP" in t_id:
        return (
            "DUT_REAL_FAILURE",
            f"Trip telemetry discrepancy on `{dut_proto}`: {msg}",
            "Check 0x221/0x261/0x2A1 stimulus or mark unsupported telemetry in DUT YAML.",
        )

    # --- UNSUPPORTED OEM SUITES ---
    if any(k in t_id for k in ("TPMS", "DSP", "CORE", "AUDIO")):
        return (
            "DUT_UNSUPPORTED_FEAT",
            f"OEM `{dut_proto}` firmware does not implement {t_id} on Peugeot 407: {msg}",
            f"Mark scenario/suite as `SKIP` or `XFAIL` in `svt/duts/<dut>.yaml`.",
        )

    return (
        "DUT_REAL_FAILURE",
        f"Unexpected DUT output: {msg}",
        f"Compare Window Evidence UART hex against `svt/lib/protocols/{dut_proto}.py`.",
    )


def render_compact_output(
    run_dir: str,
    dut_name: str,
    dut_proto: str,
    total_passed: int,
    triage_results: List[Dict[str, Any]],
    uart_info: Dict[str, Any],
    can_info: Dict[str, Any],
    max_detailed: int = 8,
) -> str:
    """
    Build a single, high-density plain-text/Markdown report (< 1,200 tokens)
    containing exact file:line targets and window-sliced CAN/UART hex evidence.
    """
    proto_file = locate_proto_decoder(dut_proto)
    dut_yaml = f"svt/duts/{dut_name}.yaml"

    lines = [
        f"# SVT TRIAGE SUMMARY: {dut_name} (proto={dut_proto})",
        f"Run: {os.path.relpath(run_dir)} | Passed: {total_passed} | Failed/XFailed: {len(triage_results)} | "
        f"CAN IDs: {len(can_info.get('can_ids', []))} | UART Pkts: {uart_info.get('dut_events_count', 0)}",
        "",
        "## 1. TURN-1 BATCH READ TARGETS (Do NOT grep or run python3 -c; read these exact spans):",
    ]

    # Collect unique file spans to read
    read_targets = []
    for r in triage_results[:max_detailed]:
        if r["src_span"] not in read_targets:
            read_targets.append(r["src_span"])
    lines.append(f"- Test File(s) : {', '.join(f'`{t}`' for t in read_targets)}")
    lines.append(f"- Proto Decoder: `{proto_file}`")
    lines.append(f"- DUT Profile  : `{dut_yaml}`")
    lines.append("")

    lines.append("## 2. FAILED SCENARIOS & WINDOW EVIDENCE (t_fail - 2.0s .. +0.5s)")
    lines.append("| Test ID | Status | Category | Source Location | Error Message |")
    lines.append("| :--- | :--- | :--- | :--- | :--- |")
    for r in triage_results:
        err_short = (r["message"] or "").replace("\n", " ")
        if len(err_short) > 75:
            err_short = err_short[:72] + "..."
        lines.append(
            f"| `{r['test_id']}` | {r['status']} | `{r['category']}` | `{r['src_span']}` | {err_short} |"
        )
    lines.append("")

    # Per-failure inline evidence block (eliminates need for `--test` drill-down or `uart.jsonl` reads)
    shown = triage_results[:max_detailed]
    for r in shown:
        ev = r["evidence"]
        can_str = ", ".join(ev["can_deltas"]) if ev["can_deltas"] else "(no CAN delta in window)"
        uart_str = " ; ".join(ev["uart_rx"]) if ev["uart_rx"] else "(SILENT — 0 UART packets in window)"
        tx_str = f" | DUT_CAN_TX: {', '.join(ev['dut_can_tx'])}" if ev["dut_can_tx"] else ""

        lines.append(f"### [{r['test_id']}] {r['title']} (`{r['src_span']}`)")
        lines.append(f"- **Assert Fail** : {r['message']}")
        lines.append(f"- **CAN Injected**: {can_str}{tx_str}")
        lines.append(f"- **UART Rx Hex** : {uart_str}")
        lines.append(f"- **Diagnosis**   : [{r['category']}] {r['root_cause']}")
        lines.append(f"- **Action**      : {r['recommendation']}")
        lines.append("")

    if len(triage_results) > max_detailed:
        omitted = len(triage_results) - max_detailed
        lines.append(f"*({omitted} additional failures omitted for token cap; pass `--test <ID>` to inspect)*")

    return "\n".join(lines)


def drill_down_single_test(
    run_dir: str,
    test_id: str,
    triage_results: List[Dict[str, Any]],
    suite_windows: Dict[str, Any],
    uart_info: Dict[str, Any],
    can_info: Dict[str, Any],
):
    """Surgical trace for a single scenario (capped to <= 40 lines)."""
    target = next((r for r in triage_results if r["test_id"].upper() == test_id.upper()), None)
    if not target:
        print(f"Test ID '{test_id}' not found among failures in {run_dir}.")
        return

    t_fail = target.get("timestamp")
    s_win = suite_windows.get(target["suite"])
    w_start = (t_fail - 3.0) if t_fail else (s_win["start"] if s_win else 0.0)
    w_end = (t_fail + 1.0) if t_fail else (s_win["end"] if s_win else float("inf"))

    print(f"# DRILL-DOWN: {target['test_id']} ({target['title']})")
    print(f"- **Source Span**: `{target['src_span']}`")
    print(f"- **Status/Cat** : {target['status']} | `{target['category']}`")
    print(f"- **Failure Msg**: {target['message']}")
    print(f"- **Root Cause** : {target['root_cause']}")
    print(f"- **Action**     : {target['recommendation']}")
    print(f"\n## CAN Payload Transitions ({w_start:.2f}s .. {w_end:.2f}s):")
    c_cnt = 0
    for t, cid, pay in can_info.get("transitions", []):
        if w_start <= t <= w_end:
            print(f"  [{t:.3f}] 0x{cid}#{pay}")
            c_cnt += 1
            if c_cnt >= 18:
                print("  ... (capped at 18 transitions)")
                break

    print(f"\n## UART RX Packets ({w_start:.2f}s .. {w_end:.2f}s):")
    u_cnt = 0
    for ev in uart_info.get("all_events", []):
        if w_start <= ev["t"] <= w_end:
            val_s = json.dumps(ev["val"], separators=(",", ":"))
            print(f"  [{ev['t']:.3f}] {ev['raw']:<24s} -> {ev['name']} {val_s}")
            u_cnt += 1
            if u_cnt >= 18:
                print("  ... (capped at 18 packets)")
                break


def main():
    parser = argparse.ArgumentParser(description="Token-Optimized SVT Failure Triage & Evidence Extractor.")
    parser.add_argument("run_dir", nargs="?", help="Path to SVT run directory (default: latest run)")
    parser.add_argument("--latest", action="store_true", help="Automatically pick latest run")
    parser.add_argument("--test", help="Drill down into a single failed test ID")
    parser.add_argument("--max-detailed", type=int, default=8, help="Max failures to show full hex evidence for (default: 8)")
    parser.add_argument("-o", "--output", help="Write markdown audit report to file")
    args = parser.parse_args()

    run_dir = args.run_dir
    if not run_dir or args.latest:
        run_dir = find_latest_run()
        if not run_dir:
            print("Error: No SVT run directory found.", file=sys.stderr)
            sys.exit(1)

    run_dir = os.path.abspath(run_dir)
    report_path = os.path.join(run_dir, "report.json")
    if not os.path.exists(report_path):
        print(f"Error: report.json not found in {run_dir}", file=sys.stderr)
        sys.exit(1)

    with open(report_path, "r", encoding="utf-8") as f:
        report_data = json.load(f)

    dut_name = report_data.get("dut_name", "unknown")
    dut_proto = detect_dut_proto(dut_name)

    suite_windows = parse_execution_log(os.path.join(run_dir, "svt_execution.log"))
    uart_info = index_uart_log(os.path.join(run_dir, "uart.jsonl"))
    can_info = index_can_log(os.path.join(run_dir, "can.log"))

    total_passed = 0
    failed_tests = []
    for s in report_data.get("suites", []):
        s_name = s.get("suite", "")
        for res in s.get("results", []):
            if res.get("status") == "PASS":
                total_passed += 1
            elif res.get("status") in ("FAIL", "XFAIL"):
                res["suite"] = s_name
                failed_tests.append(res)

    if not failed_tests:
        print(f"ALL {total_passed} TESTS PASSED in {os.path.relpath(run_dir)}. Zero failures to triage.")
        return

    triage_results = []
    for t in failed_tests:
        s_name = t.get("suite", "")
        s_win = suite_windows.get(s_name)
        t_id = t.get("test_id", "")
        t_ts = t.get("timestamp")

        cat, cause, rec = diagnose_failure(t, s_name, s_win, uart_info, can_info, dut_proto=dut_proto)
        src_span = locate_test_source_span(t_id, s_name)
        evidence = slice_window_evidence(t_ts, s_win, uart_info, can_info)

        triage_results.append({
            "test_id": t_id,
            "title": t.get("title", ""),
            "status": t.get("status", "FAIL"),
            "suite": s_name,
            "timestamp": t_ts,
            "message": t.get("message", ""),
            "category": cat,
            "root_cause": cause,
            "recommendation": rec,
            "src_span": src_span,
            "evidence": evidence,
        })

    if args.test:
        drill_down_single_test(run_dir, args.test, triage_results, suite_windows, uart_info, can_info)
        return

    report_text = render_compact_output(
        run_dir,
        dut_name,
        dut_proto,
        total_passed,
        triage_results,
        uart_info,
        can_info,
        max_detailed=args.max_detailed,
    )

    print(report_text)

    out_path = args.output or os.path.join(run_dir, "svt_failure_triage.md")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(report_text + "\n")


if __name__ == "__main__":
    main()
