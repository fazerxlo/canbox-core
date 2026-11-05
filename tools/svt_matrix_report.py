#!/usr/bin/env python3
"""
svt_matrix_report.py — Token-Optimized Multi-CANbox SVT Benchmark Matrix & Diff Generator.

Consolidates multiple SVT report.json artifacts into a unified side-by-side
subsystem scorecard and scenario matrix without consuming agent prompt tokens.

Key Token & Bug Fixes:
  1. Searches both `svt/runs` and `runs` by default (matching `triage_svt_failures.py`).
  2. Extracts scenarios from both nested `suites[*].results` AND top-level `results`.
  3. Computes pass/fail/skip totals reliably even if top-level keys are omitted in `report.json`.
  4. Terminal stdout defaults to compact mode (Scorecard + Divergent/Failing rows only, <800 tokens),
     while `--output <file.md>` or `--full` writes the complete 100+ row matrix.

Usage:
  python3 tools/svt_matrix_report.py --latest
  python3 tools/svt_matrix_report.py --latest --full
  python3 tools/svt_matrix_report.py --output doc/MULTI_CANBOX_SVT_COMPARISON_REPORT.md --latest
"""

import os
import sys
import glob
import json
import argparse
from typing import Dict, List, Any, Tuple


def find_latest_reports(base_dirs: Tuple[str, ...] = ("svt/runs", "runs")) -> Dict[str, str]:
    """Find the latest report.json for each unique DUT across svt/runs and runs."""
    dut_reports: Dict[str, str] = {}
    candidates: List[str] = []
    for b in base_dirs:
        candidates.extend(glob.glob(os.path.join(b, "*", "report.json")))

    for rpath in sorted(candidates, key=os.path.getmtime, reverse=True):
        try:
            with open(rpath, "r", encoding="utf-8") as f:
                data = json.load(f)
            dut = data.get("dut_name", "unknown")
            if dut not in dut_reports:
                dut_reports[dut] = rpath
        except Exception:
            continue
    return dut_reports


def load_reports(report_paths: List[str]) -> List[Dict[str, Any]]:
    reports = []
    for p in report_paths:
        if not os.path.exists(p):
            print(f"Warning: report file not found: {p}", file=sys.stderr)
            continue
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
                data["_filepath"] = p
                reports.append(data)
        except Exception as e:
            print(f"Warning: failed to load {p}: {e}", file=sys.stderr)
    return reports


def iter_report_results(report: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Extract scenario results regardless of whether `report.json` stores them
    inside `suites[*].results` or top-level `results`.
    """
    results: List[Dict[str, Any]] = []
    seen_ids = set()

    for s in report.get("suites", []):
        s_name = s.get("suite", "")
        for res in s.get("results", []):
            sc_id = res.get("test_id")
            if sc_id and sc_id not in seen_ids:
                seen_ids.add(sc_id)
                r_copy = dict(res)
                r_copy.setdefault("suite", s_name)
                results.append(r_copy)

    for res in report.get("results", []):
        sc_id = res.get("test_id")
        if sc_id and sc_id not in seen_ids:
            seen_ids.add(sc_id)
            results.append(dict(res))

    return results


def summarize_suite_counts(suite_data: Dict[str, Any]) -> Tuple[int, int, int, int]:
    """Return (pass, fail, skip, total) from suite dict or its nested results list."""
    res_list = suite_data.get("results", [])
    if res_list and "pass" not in suite_data:
        p = sum(1 for r in res_list if r.get("status") == "PASS")
        f = sum(1 for r in res_list if r.get("status") in ("FAIL", "XFAIL"))
        s = sum(1 for r in res_list if r.get("status") == "SKIP")
        return p, f, s, len(res_list)

    p = suite_data.get("pass", 0)
    f = suite_data.get("fail", 0) + suite_data.get("xfail", 0)
    s = suite_data.get("skip", 0)
    tot = suite_data.get("total", p + f + s)
    return p, f, s, tot


def summarize_report_totals(report: Dict[str, Any]) -> Tuple[int, int, int, int]:
    """Return overall (pass, fail, skip, total) for a report."""
    all_res = iter_report_results(report)
    if all_res and "total_tests" not in report:
        p = sum(1 for r in all_res if r.get("status") == "PASS")
        f = sum(1 for r in all_res if r.get("status") in ("FAIL", "XFAIL"))
        s = sum(1 for r in all_res if r.get("status") == "SKIP")
        return p, f, s, len(all_res)

    p = report.get("pass", 0)
    f = report.get("fail", 0) + report.get("xfail", 0)
    s = report.get("skip", 0)
    tot = report.get("total_tests", p + f + s)
    return p, f, s, tot


def generate_matrix_markdown(reports: List[Dict[str, Any]], include_all_passing: bool = True) -> str:
    if not reports:
        return "# Multi-CANbox SVT Comparison Matrix\n\nNo report data provided."

    dut_names = [r.get("dut_name", "DUT") for r in reports]
    duts_str = ", ".join(f"`{d}`" for d in dut_names)

    lines = [
        "# Multi-CANbox SVT Comparison Matrix",
        f"**Compared DUTs ({len(reports)}):** {duts_str}",
        "",
        "## 1. Subsystem Scorecard (Pass/Fail/Skip)",
        "",
    ]

    all_suites: List[str] = []
    for r in reports:
        for s in r.get("suites", []):
            s_name = s.get("suite")
            if s_name and s_name not in all_suites:
                all_suites.append(s_name)

    cols = ["Subsystem Suite"] + [f"{d} (P/F/S)" for d in dut_names]
    lines.append("| " + " | ".join(cols) + " |")
    lines.append("| " + " | ".join(["---"] * len(cols)) + " |")

    for s_name in all_suites:
        row = [s_name]
        for r in reports:
            suite_data = next((s for s in r.get("suites", []) if s.get("suite") == s_name), None)
            if suite_data:
                p, f, s, total = summarize_suite_counts(suite_data)
                badge = f"**{p}**/{f}/{s}" if f > 0 else f"{p}/{f}/{s}"
                icon = "FAIL" if f > 0 else "OK"
                row.append(f"{icon} {badge} ({total})")
            else:
                row.append("—")
        lines.append("| " + " | ".join(row) + " |")

    total_row = ["**TOTAL**"]
    for r in reports:
        p, f, s, tot = summarize_report_totals(r)
        pct = (p / tot * 100.0) if tot > 0 else 0.0
        total_row.append(f"**{p}/{tot}** ({pct:.1f}%) F:{f} S:{s}")
    lines.append("| " + " | ".join(total_row) + " |")

    # Collect all scenarios across all reports (handling both nested and flat schemas)
    scenarios_dict: Dict[str, Dict[str, Any]] = {}
    for r in reports:
        d_name = r.get("dut_name", "DUT")
        for res in iter_report_results(r):
            sc_id = res.get("test_id")
            if not sc_id:
                continue
            if sc_id not in scenarios_dict:
                scenarios_dict[sc_id] = {
                    "id": sc_id,
                    "title": res.get("title", ""),
                    "dut_status": {},
                }
            scenarios_dict[sc_id]["dut_status"][d_name] = {
                "status": res.get("status", "UNKNOWN"),
                "message": res.get("message", ""),
            }

    universal_pass = 0
    divergent_tests = 0
    scenario_rows: List[str] = []

    for sc_id in sorted(scenarios_dict.keys()):
        item = scenarios_dict[sc_id]
        statuses = [item["dut_status"].get(d, {}).get("status", "—") for d in dut_names]
        is_universal_pass = (len(set(statuses)) == 1 and statuses[0] == "PASS")

        if is_universal_pass:
            universal_pass += 1
            if not include_all_passing:
                continue
        else:
            divergent_tests += 1

        title_short = item["title"]
        if not include_all_passing and len(title_short) > 48:
            title_short = title_short[:45] + "..."

        row = [f"`{sc_id}`", title_short]
        for d in dut_names:
            st_info = item["dut_status"].get(d)
            if not st_info:
                row.append("—")
                continue
            st = st_info["status"]
            msg = (st_info.get("message") or "").replace("\n", " ").strip()
            if st == "FAIL" and not include_all_passing and msg:
                msg_short = msg[:32] + "..." if len(msg) > 35 else msg
                row.append(f"FAIL ({msg_short})")
            else:
                row.append(st)

        scenario_rows.append("| " + " | ".join(row) + " |")

    section_title = (
        "## 2. Full Scenario Compatibility Matrix"
        if include_all_passing
        else f"## 2. Divergent / Failing Scenarios Only ({divergent_tests} shown, {universal_pass} universal PASS omitted)"
    )
    lines.extend(["", section_title, ""])

    if scenario_rows:
        cols = ["Scenario ID", "Description"] + dut_names
        lines.append("| " + " | ".join(cols) + " |")
        lines.append("| " + " | ".join(["---"] * len(cols)) + " |")
        lines.extend(scenario_rows)
    else:
        lines.append("*All scenarios passed universally across all compared DUTs.*")

    lines.extend([
        "",
        "## 3. Compatibility Summary",
        f"- **Universal PASS across all DUTs:** `{universal_pass}` scenarios",
        f"- **Divergent / Feature-Gated / Failing:** `{divergent_tests}` scenarios",
    ])

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Multi-CANbox SVT Comparison Matrix Generator")
    parser.add_argument("reports", nargs="*", help="Paths to report.json files")
    parser.add_argument("--latest", action="store_true", help="Automatically load latest report for each unique DUT")
    parser.add_argument("--runs-dir", default=None, help="Specific runs directory (default: searches svt/runs and runs)")
    parser.add_argument("--full", action="store_true", help="Print all passing rows to stdout (default stdout shows only divergent/failing rows to save tokens)")
    parser.add_argument("--output", "-o", default=None, help="Output markdown file path (writes full matrix to file)")

    args = parser.parse_args()

    base_dirs = (args.runs_dir,) if args.runs_dir else ("svt/runs", "runs")

    if args.reports:
        report_paths = args.reports
    else:
        latest_map = find_latest_reports(base_dirs)
        print(f"Discovered latest runs for {len(latest_map)} DUTs: {list(latest_map.keys())}", file=sys.stderr)
        report_paths = list(latest_map.values())

    if not report_paths:
        print("Error: No reports found in svt/runs/ or runs/.", file=sys.stderr)
        return 1

    reports_data = load_reports(report_paths)

    if args.output:
        full_md = generate_matrix_markdown(reports_data, include_all_passing=True)
        out_abs = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(out_abs), exist_ok=True)
        with open(out_abs, "w", encoding="utf-8") as f:
            f.write(full_md)
        # Print compact summary to stdout so the agent doesn't have to read_file the generated markdown!
        compact_md = generate_matrix_markdown(reports_data, include_all_passing=args.full)
        print(compact_md)
        print(f"Full matrix report saved to: {args.output}")
    else:
        print(generate_matrix_markdown(reports_data, include_all_passing=args.full))

    return 0


if __name__ == "__main__":
    sys.exit(main())

