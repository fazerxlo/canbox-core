#!/usr/bin/env python3
"""
Fast-Path Symbol, CAN ID, and UART Command Locator for OpenCanbox Core.

Usage:
  ./tools/code_map.py                     # List all C functions with exact [StartLine-EndLine] spans
  ./tools/code_map.py 0x167 0x1B          # Locate functions, references, tests & spec sections for patterns
  ./tools/code_map.py --funcs psa_stalk   # List only matching function [StartLine-EndLine] spans
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEARCH_DIRS = ["include", "src", "test", "doc"]

FUNC_START_RE = re.compile(
    r"^(?:static\s+)?(?:inline\s+)?(?:const\s+)?(?:struct\s+\w+\s+|enum\s+\w+\s+|\w+[\w\s\*]*?)\s+"
    r"([a-zA-Z_]\w*)\s*\([^;]*$"
)


def iter_files(dirs, exts):
    for d in dirs:
        base = os.path.join(ROOT, d)
        if not os.path.isdir(base):
            continue
        for root, _, files in os.walk(base):
            for f in sorted(files):
                if any(f.endswith(ext) for ext in exts):
                    yield os.path.relpath(os.path.join(root, f), ROOT)


def extract_c_functions(rel_path):
    """Return list of (func_name, start_line, end_line, sig_line) for a C/H file."""
    full_path = os.path.join(ROOT, rel_path)
    try:
        with open(full_path, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except OSError:
        return []

    funcs = []
    n = len(lines)
    i = 0
    while i < n:
        line = lines[i].rstrip()
        if line and not line[0].isspace() and not line.startswith(("#", "/", "*")):
            m = FUNC_START_RE.match(line)
            if m and m.group(1) not in ("if", "for", "while", "switch", "return", "sizeof"):
                fname = m.group(1)
                start_idx = i
                # Scan for opening brace within 6 lines
                j = i
                brace_depth = 0
                found_open = False
                while j < min(n, i + 6):
                    if ";" in lines[j] and "{" not in lines[j]:
                        break
                    if "{" in lines[j]:
                        found_open = True
                        break
                    j += 1
                if found_open:
                    k = start_idx
                    while k < n:
                        for ch in lines[k]:
                            if ch == "{":
                                brace_depth += 1
                            elif ch == "}":
                                brace_depth -= 1
                        if brace_depth == 0 and k >= j:
                            funcs.append((fname, start_idx + 1, k + 1, line.strip()))
                            i = k
                            break
                        k += 1
        i += 1
    return funcs


def main():
    args = sys.argv[1:]
    funcs_only = False
    if "--funcs" in args:
        funcs_only = True
        args.remove("--funcs")

    patterns = [re.compile(p, re.IGNORECASE) for p in args]

    c_files = list(iter_files(["include", "src", "test"], (".c", ".h")))
    doc_files = list(iter_files(["doc"], (".md",)))

    if not patterns:
        print("=== C Function Spans [StartLine-EndLine] ===")
        for rel in c_files:
            funcs = extract_c_functions(rel)
            if funcs:
                for fname, sl, el, _ in funcs:
                    print(f"{rel}:{sl}-{el}\t{fname}()  ({el - sl + 1} lines)")
        return

    print("=== 1. Matching Function Spans (Use directly in read_file StartLine/EndLine) ===")
    matched_spans = 0
    for rel in c_files:
        full_path = os.path.join(ROOT, rel)
        with open(full_path, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
        funcs = extract_c_functions(rel)
        for fname, sl, el, _ in funcs:
            body = "".join(lines[sl - 1 : el])
            if any(p.search(fname) or p.search(body) for p in patterns):
                print(f"  {rel}:{sl}-{el}\t{fname}()  [{el - sl + 1} lines]")
                matched_spans += 1
                if matched_spans >= 40:
                    print("  ... (capped at 40 function spans)")
                    break
        if matched_spans >= 40:
            break

    if funcs_only:
        return

    print("\n=== 2. Code & Header Matches (include/, src/, test/) ===")
    code_hits = 0
    for rel in c_files:
        full_path = os.path.join(ROOT, rel)
        with open(full_path, "r", encoding="utf-8", errors="replace") as fh:
            for idx, line in enumerate(fh, 1):
                if any(p.search(line) for p in patterns):
                    print(f"  {rel}:{idx}: {line.strip()[:100]}")
                    code_hits += 1
                    if code_hits >= 45:
                        print("  ... (capped at 45 code hits)")
                        break
        if code_hits >= 45:
            break

    print("\n=== 3. Spec & Doc Headings / Matches (doc/) ===")
    doc_hits = 0
    for rel in doc_files:
        full_path = os.path.join(ROOT, rel)
        with open(full_path, "r", encoding="utf-8", errors="replace") as fh:
            for idx, line in enumerate(fh, 1):
                if any(p.search(line) for p in patterns):
                    print(f"  {rel}:{idx}: {line.strip()[:100]}")
                    doc_hits += 1
                    if doc_hits >= 25:
                        print("  ... (capped at 25 doc hits)")
                        break
        if doc_hits >= 25:
            break


if __name__ == "__main__":
    main()

