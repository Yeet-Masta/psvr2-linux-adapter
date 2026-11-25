"""Label functions by body shape (boilerplate detection).

usage: shapes.py [--sub S] [--verdict V] [--undoc] [--show SHAPE] [--out FILE]
Shapes: xordec, vec_grow, vec_free, vec_ctor, eh_ctor, thunk, getter, setter,
        zero_init, copy, dtor, nullret, other.
Writes out/shapes.tsv (va, lines, shape) for all functions when --out given.
"""
import sys, argparse, collections, glob, json, os, re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from docscan import doc_text, splice, CATALOGUE  # noqa: E402

def _txt(path, t):
    """Doc text to scan for VAs; generated tables count their first column only."""
    return doc_text(t)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
ADAPTER = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter"

ap = argparse.ArgumentParser()
ap.add_argument("--sub"); ap.add_argument("--verdict")
ap.add_argument("--undoc", action="store_true")
ap.add_argument("--show"); ap.add_argument("--out")
a = ap.parse_args()

DECL = re.compile(r"^\s+[A-Za-z_][\w\s\*:<>,]*?[\s\*]+\(?\*?[\w\[\]]+\)?(\[[^\]]*\])*;\s*//\s*\S+")
bodies = {}; cur = None
with open(os.path.join(OUT, "decomp_all.c"), encoding="utf-8") as f:
    for l in f:
        if l.startswith("// ==== "):
            cur = int(l.split()[2], 16); bodies[cur] = []; continue
        if cur is not None and not DECL.match(l):
            s = l.strip()
            if s and s not in ("{", "}"):
                bodies[cur].append(s)

R = {
    "assign_const": re.compile(r"^\*?\(?.*=\s*(-?0x[0-9A-Fa-f]+u?i?6?4?L*|-?\d+(\.\d+)?f?|nullptr|[a-z_]+\w*)\s*;$"),
    "xor": re.compile(r"\^="),
    "allocv": re.compile(r"\+ 8LL\)\)\(\*a1, "),
    "freev": re.compile(r"\+ 24LL\)\)\(\*a1"),
    "eh": re.compile(r"eh vector (constructor|destructor) iterator"),
    "call": re.compile(r"\bsub_[0-9A-F]+\("),
    "ret_field": re.compile(r"^return \*?\(?[\w\s\*]*\)?\(?\(?\w+\s*\+\s*\d+\)?\)?;$|^return \*?a1(\[\d+\])?;$|^return \w+->\w+;$"),
    "copy": re.compile(r"^\*\(.*\)\(?\w+ \+ \d+\)? = \*\(.*\)\(?\w+ \+ \d+\)?;$|^\w+\[\d+\] = \w+\[\d+\];$"),
    "zero": re.compile(r"= (0|0LL|0\.0|nullptr|0i64);$|memset\("),
    "free": re.compile(r"\bfree\(|operator delete|j_free"),
    "vtbl": re.compile(r"`vftable'"),
}

def shape(b):
    sig, body = (b[0], b[1:]) if b else ("", [])
    n = len(body)
    if n == 0:
        return "nullret"
    calls = sum(1 for l in body if R["call"].search(l))
    if n <= 16 and any(R["xor"].search(l) for l in body) and calls == 0 and \
            all(R["xor"].search(l) or "0x" in l or l.startswith("return") or "=" in l for l in body):
        return "xordec"
    if any(R["allocv"].search(l) for l in body) and n <= 70:
        return "vec_grow"
    if n <= 8 and any(R["freev"].search(l) for l in body):
        return "vec_free"
    if n <= 6 and any(R["eh"].search(l) for l in body):
        return "eh_ctor"
    if n <= 3 and calls == 1 and all(l.startswith("return") or R["call"].search(l) for l in body):
        return "thunk"
    if n == 1 and R["ret_field"].match(body[0]):
        return "getter"
    if n <= 2 and body[-1].startswith("return") and calls == 0 and "=" in body[0] and len(body) == 2:
        return "setter"
    if n >= 3 and sum(1 for l in body if R["copy"].match(l)) >= 0.7 * n:
        return "copy"
    if n >= 3 and sum(1 for l in body if R["zero"].search(l) or R["vtbl"].search(l)) >= 0.7 * n:
        return "zero_init"
    if n <= 25 and any(R["free"].search(l) for l in body) and any(R["vtbl"].search(l) for l in body):
        return "dtor"
    if n <= 2 and body[-1].startswith("return") and calls == 0:
        return "const_ret"
    return "other"

tri = {}
with open(os.path.join(OUT, "triage.tsv"), encoding="utf-8") as f:
    hdr = f.readline().rstrip("\n").split("\t")
    for l in f:
        r = dict(zip(hdr, l.rstrip("\n").split("\t"))); tri[int(r["va"], 16)] = r
named = set()
rx = re.compile(r"(?:0x|FUN_|sub_)(18[0-9a-fA-F]{7})\b")
for d in glob.glob(os.path.join(ADAPTER, "docs", "**", "*.md"), recursive=True):
    with open(d, encoding="utf-8", errors="replace") as f:
        named.update(int(m.group(1), 16) for m in rx.finditer(_txt(d, f.read())))

cnt = collections.Counter(); lines = collections.Counter(); rows = []
for v, b in sorted(bodies.items()):
    t = tri.get(v)
    s = shape(b)
    rows.append((v, len(b), s))
    if not t: continue
    if a.sub and t["subsystem"] != a.sub: continue
    if a.verdict and t["verdict"] != a.verdict: continue
    if a.undoc and v in named: continue
    cnt[s] += 1; lines[s] += len(b)
    if a.show == s:
        print(f"{v:x}\t{len(b)}\t{t['subsystem']}\t{b[1][:100] if len(b) > 1 else ''}")
if not a.show:
    for s, c in cnt.most_common():
        print(f"{s:10} {c:5} fns {lines[s]:6} lines")
if a.out:
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("va\tlines\tshape\n")
        for v, n, s in rows:
            f.write(f"0x{v:x}\t{n}\t{s}\n")
