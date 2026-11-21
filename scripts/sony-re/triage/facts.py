"""Compact fact sheets for functions, to speed up writing docs.

usage: facts.py VA [VA ...] [-d N]
       facts.py --sub ctrl-boot [--verdict core] [--undoc] [--min L] [--max L]
                [--from VA] [--to VA] [-d N] [--brief]
-d N   print up to N body lines of the decompile (declarations stripped)
--brief one line per function (no decompile)
Callers/callees named in a doc get '*'. Library/CRT callees are dropped.
"""
import argparse, glob, json, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
ADAPTER = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter"

ap = argparse.ArgumentParser()
ap.add_argument("vas", nargs="*")
ap.add_argument("-d", type=int, default=0)
ap.add_argument("--sub"); ap.add_argument("--verdict", default="core")
ap.add_argument("--undoc", action="store_true")
ap.add_argument("--min", type=int, default=0); ap.add_argument("--max", type=int, default=10**9)
ap.add_argument("--from", dest="lo"); ap.add_argument("--to", dest="hi")
ap.add_argument("--brief", action="store_true")
a = ap.parse_args()

feat = {}
with open(os.path.join(OUT, "features.jsonl"), encoding="utf-8") as f:
    for l in f:
        r = json.loads(l); feat[int(r["va"], 16)] = r
tri = {}
with open(os.path.join(OUT, "triage.tsv"), encoding="utf-8") as f:
    hdr = f.readline().rstrip("\n").split("\t")
    for l in f:
        r = dict(zip(hdr, l.rstrip("\n").split("\t"))); tri[int(r["va"], 16)] = r
named = {}
rx = re.compile(r"(?:0x|FUN_|sub_)(18[0-9a-fA-F]{7})\b")
for d in glob.glob(os.path.join(ADAPTER, "docs", "**", "*.md"), recursive=True):
    base = os.path.basename(d)[:-3]
    with open(d, encoding="utf-8", errors="replace") as f:
        for m in rx.finditer(f.read()):
            named.setdefault(int(m.group(1), 16), base)

def lib(v):
    return v >= 0x180238000  # CRT / STL / imports tail of .text

def tag(v):
    s = f"{v:x}"[3:]
    if v in named: return s + "*"
    r = feat.get(v)
    return s + (f"({r['lines']})" if r else "")

vas = [int(v, 16) for v in a.vas]
if not vas:
    lo = int(a.lo, 16) if a.lo else 0; hi = int(a.hi, 16) if a.hi else 1 << 40
    for v, r in sorted(tri.items()):
        if a.sub and r["subsystem"] != a.sub: continue
        if a.verdict and r["verdict"] != a.verdict: continue
        if a.undoc and v in named: continue
        if not (a.min <= int(r["lines"]) <= a.max and lo <= v < hi): continue
        vas.append(v)

body = {}
if a.d and not a.brief:
    want = {f"0x{v:x}" for v in vas}
    DECL = re.compile(r"^\s+[A-Za-z_][\w\s\*:<>,]*?[\s\*]+\(?\*?[\w\[\]]+\)?(\[[^\]]*\])*;\s*//\s*\S+")
    cur = None
    with open(os.path.join(OUT, "decomp_all.c"), encoding="utf-8") as f:
        for l in f:
            if l.startswith("// ==== "):
                k = l.split()[2]; cur = k if k in want else None
                if cur: body[cur] = []
                continue
            if cur and not DECL.match(l) and l.strip() not in ("", "{", "}"):
                body[cur].append(l.rstrip())

for v in vas:
    r = feat.get(v, {}); t = tri.get(v, {})
    cs = [int(x, 16) for x in r.get("callers", [])]
    ce = [int(x, 16) for x in r.get("callees", []) if not lib(int(x, 16))]
    nlib = len(r.get("callees", [])) - len(ce)
    line = (f"## {v:x} L{t.get('lines', r.get('lines'))} {t.get('area','')}"
            f" <- {','.join(tag(c) for c in cs[:6])}{'+' if len(cs) > 6 else ''}"
            f" -> {','.join(tag(c) for c in ce[:10])}{'+' if len(ce) > 10 else ''}"
            f"{f' lib{nlib}' if nlib else ''}")
    if r.get("vt"): line += f" vt={r['vt'][:2]}"
    if r.get("strs"): line += " s=" + "|".join(s[:40] for s in r["strs"][:3])
    if v in named: line += f" [doc:{named[v]}]"
    print(line)
    if a.d and not a.brief:
        b = body.get(f"0x{v:x}", [])
        for l in b[1:a.d + 1]:
            print("  " + l.strip()[:160])
        if len(b) > a.d + 1: print(f"  ... +{len(b) - a.d - 1}")
