"""depth.py [--min N] [--verdict V]: how well each function is described.

own    = has its own table row or prose line naming it as the subject
guess  = its own descriptions are all tagged [guess]
mention= only appears inside other functions' rows (caller/callee cells)
none   = not in the docs
Prints functions that are not 'own', largest first."""
import argparse, csv, os, re
from docscan import read_docs

ap = argparse.ArgumentParser()
ap.add_argument("--min", type=int, default=100)
ap.add_argument("--verdict", default="core")
ap.add_argument("--tsv", default=os.path.join(os.path.dirname(__file__), "..", "out", "triage.tsv"))
a = ap.parse_args()

lines = []
for path, text in read_docs():
    lines.extend(text.splitlines())

rows = [r for r in csv.DictReader(open(a.tsv, encoding="utf-8"), delimiter="\t")
        if r["verdict"] == a.verdict and int(r["lines"]) >= a.min]
idx = {}
WEAK = re.compile(r"\[guess\]|not yet written up|named only|not traced|skipped|stages only|meaning unknown|same family|not described", re.I)
pat = re.compile(r"(?:0x|FUN_)(18[0-9a-f]{7})", re.I)
for i, l in enumerate(lines):
    for m in pat.finditer(l):
        idx.setdefault(m.group(1).lower(), []).append(i)

out = []
for r in rows:
    h = r["va"].lower().replace("0x", "")
    hits = idx.get(h, [])
    if not hits:
        out.append(("none", r)); continue
    own, guess = 0, 0
    for i in hits:
        l = lines[i]
        if l.startswith("|"):
            first = l.split("|")[1]
            if h in first.lower():
                own += 1
                if WEAK.search(l):
                    guess += 1
        else:
            own += 1
            if WEAK.search(l):
                guess += 1
    if own == 0:
        out.append(("mention", r))
    elif own == guess:
        out.append(("guess", r))
out.sort(key=lambda x: -int(x[1]["lines"]))
tot = {}
for k, r in out:
    tot[k] = tot.get(k, 0) + int(r["lines"])
    print(f'{k:8} {r["va"]} {r["lines"]:>5} {r["subsystem"]:14} {r["reason"][:70]}')
print("total", len(out), tot)
