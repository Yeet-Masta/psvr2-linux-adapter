"""desc.py VA... : first documented description of each function (catalogue row or prose line)."""
import re, sys
from docscan import read_docs

def main():
    items = list(read_docs())
    for a in sys.argv[1:]:
        h = a.lower().replace("0x", "")
        h = h[3:] if len(h) == 9 else h.zfill(6)
        if len(h) == 6:
            h = "180" + h
        pat = re.compile(r"(?:0x|FUN_)" + h + r"\b", re.I)
        best = None
        for name, text in items:
            for line in text.splitlines():
                if pat.search(line):
                    row = line.startswith("| `0x" + h) or line.startswith("| `FUN_" + h)
                    if row:
                        best = line
                        break
                    if best is None:
                        best = line
            if best and (best.startswith("| `0x" + h) or best.startswith("| `FUN_" + h)):
                break
        print(h[3:], ":", (best or "-").strip()[:230])

main()
