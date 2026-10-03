#!/usr/bin/env python3
"""Summarise nextpnr critical paths from a build log: critpath.py build.log [clock-substring]"""
import re, sys
log = open(sys.argv[1]).read().splitlines()
want = sys.argv[2] if len(sys.argv) > 2 else ""
i = 0
while i < len(log):
    m = re.match(r"Info: Critical path report for clock '(.*?)'", log[i])
    if m and want in m.group(1):
        print("==", m.group(1))
        srcs, total = [], None
        j = i + 1
        while j < len(log) and not log[j].startswith("Info: Critical path report"):
            t = re.match(r"Info:\s+(clk-to-q|logic|routing|setup)\s+([\d.]+)\s+([\d.]+)\s+(Source|Net)\s+(\S+)", log[j])
            d = re.match(r"Info:\s+(/\S+?\.v):(\d+)", log[j])
            if t and t.group(4) == "Source":
                total = t.group(3)
                if t.group(1) == "clk-to-q": srcs.append(("START", t.group(5)[:70], total))
            if d and "oss-cad-suite" not in d.group(1):
                loc = d.group(1).split("/")[-1] + ":" + d.group(2)
                if not srcs or srcs[-1][0] != loc: srcs.append((loc, "", total))
            if re.match(r"Info: [\d.]+ ns logic", log[j]): break
            j += 1
        for s in srcs: print("  %-28s %6s  %s" % s)
        i = j
    i += 1
