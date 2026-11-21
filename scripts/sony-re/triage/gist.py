"""Compact gist of functions: signature + first N meaningful statements.
usage: gist.py [-n N] VA..."""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
args = sys.argv[1:]; n = 8
if args and args[0] == "-n":
    n = int(args[1]); args = args[2:]
want = {int(a, 16) for a in args}
DECL = re.compile(r"^\s+[A-Za-z_][\w\s\*:<>,]*?[\s\*]+\(?\*?[\w\[\]]+\)?(\[[^\]]*\])*;\s*//")
NOISE = re.compile(r"^(v\d+ = v\d+;|v\d+ = a\d+;|\{|\}|else|do|LABEL_\d+:|// .*)$")
cur = None; body = {}
with open(os.path.join(HERE, "..", "out", "decomp_all.c"), encoding="utf-8") as f:
    for l in f:
        if l.startswith("// ==== "):
            v = int(l.split()[2], 16); cur = v if v in want else None
            if cur: body[cur] = []
            continue
        if cur and not DECL.match(l):
            s = l.strip()
            if s and not NOISE.match(s):
                body[cur].append(s)
def sh(s):
    s = re.sub(r"__fastcall |__int64", lambda m: "i64" if m.group(0) == "__int64" else "", s)
    s = re.sub(r"\(_QWORD \*\)|\(_DWORD \*\)|\(float \*\)|\(double \*\)|\(_BYTE \*\)|\(__m128 \*\)|\(__m128i \*\)", "", s)
    return s
for v in sorted(body):
    b = body[v]
    print(f"## {v:x} [{len(b)}] " + " ; ".join(sh(x) for x in b[:n])[:600])
