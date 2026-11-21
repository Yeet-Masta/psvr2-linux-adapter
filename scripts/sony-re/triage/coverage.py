"""Doc coverage of triaged functions.

Usage: coverage.py [--min LINES] [--verdict core] [--list]
Scans the adapter repo docs for each VA (0x... or FUN_...) and reports
how many triaged functions no doc names yet.
"""
import argparse, glob, os, re, collections

def _txt(path, t):
    """Doc text to scan for VAs; generated tables count their first column only."""
    if path.replace("\\", "/").endswith("catalogue/trivial.md"):
        return chr(10).join(l.split("|")[1] for l in t.splitlines() if l.startswith("| `0x"))
    return t

ADAPTER = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter"
TRIAGE = os.path.join(os.path.dirname(__file__), "..", "out", "triage.tsv")

ap = argparse.ArgumentParser()
ap.add_argument("--min", type=int, default=0)
ap.add_argument("--verdict", default=None)
ap.add_argument("--list", action="store_true")
ap.add_argument("--sub", default=None)
a = ap.parse_args()

named = set()
docs = glob.glob(os.path.join(ADAPTER, "docs", "**", "*.md"), recursive=True)
docs += glob.glob(os.path.join(ADAPTER, "**", "RESEARCH*.md"), recursive=True)
rx = re.compile(r"(?:0x|FUN_|sub_)(18[0-9a-fA-F]{7})\b")
for d in set(docs):
    with open(d, encoding="utf-8", errors="replace") as f:
        for m in rx.finditer(_txt(d, f.read())):
            named.add(int(m.group(1), 16))

rows = []
with open(TRIAGE, encoding="utf-8") as f:
    hdr = f.readline().rstrip("\n").split("\t")
    for line in f:
        r = dict(zip(hdr, line.rstrip("\n").split("\t")))
        rows.append(r)

tot = collections.Counter(); und = collections.Counter()
todo = []
for r in rows:
    if a.verdict and r["verdict"] != a.verdict:
        continue
    if a.sub and r["subsystem"] != a.sub:
        continue
    if int(r["lines"]) < a.min:
        continue
    k = (r["verdict"], r["subsystem"])
    tot[k] += 1
    if int(r["va"], 16) not in named:
        und[k] += 1
        todo.append(r)
for k in sorted(tot):
    print(f"{k[0]:6} {k[1]:12} {tot[k]:5} undoc {und[k]:5}")
print("total", sum(tot.values()), "undoc", sum(und.values()))
if a.list:
    for r in sorted(todo, key=lambda r: (r["subsystem"], r["va"])):
        print(r["subsystem"], r["va"], r["lines"], r["area"], r["reason"][:80], sep="\t")
