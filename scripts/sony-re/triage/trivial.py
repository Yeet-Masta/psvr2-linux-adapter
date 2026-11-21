"""Generate docs/sony-re/catalogue/trivial.md: one row per boilerplate
function (shape detected by shapes.py rules) that no doc names yet.

usage: trivial.py [--all]   (--all: include already-documented functions)
Text is generated in our own words from the shape; no decompiled code is
copied into the output.
"""
import collections, glob, json, os, re, sys

def _txt(path, t):
    """Doc text to scan for VAs; generated tables count their first column only."""
    if path.replace("\\", "/").endswith(("catalogue/trivial.md", "catalogue/library.md")):
        return chr(10).join(l.split("|")[1] for l in t.splitlines() if l.startswith("| `0x"))
    return t

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
ADAPTER = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter"
DST = os.path.join(ADAPTER, "docs", "sony-re", "catalogue", "trivial.md")
sys.argv = [sys.argv[0]] + [x for x in sys.argv[1:] if x != "--all"]
ALL = "--all" in sys.argv

# reuse the shape rules
src = open(os.path.join(HERE, "shapes.py"), encoding="utf-8").read()
ns = {"__file__": os.path.join(HERE, "shapes.py")}
exec(src.split("tri = {}")[0].replace("a = ap.parse_args()", "a = ap.parse_args([])"), ns)
bodies, shape = ns["bodies"], ns["shape"]

tri = {}
with open(os.path.join(OUT, "triage.tsv"), encoding="utf-8") as f:
    hdr = f.readline().rstrip("\n").split("\t")
    for l in f:
        r = dict(zip(hdr, l.rstrip("\n").split("\t"))); tri[int(r["va"], 16)] = r
feat = {}
with open(os.path.join(OUT, "features.jsonl"), encoding="utf-8") as f:
    for l in f:
        j = json.loads(l); feat[int(j["va"], 16)] = j
xs = {}
with open(os.path.join(OUT, "xorstr.txt"), encoding="utf-8") as f:
    for l in f:
        p = l.rstrip("\n").split("\t", 1)
        if len(p) == 2:
            xs.setdefault(int(p[0], 16), p[1])
named = set()
rx = re.compile(r"(?:0x|FUN_|sub_)(18[0-9a-fA-F]{7})\b")
for d in glob.glob(os.path.join(ADAPTER, "docs", "**", "*.md"), recursive=True):
    if os.path.abspath(d) == os.path.abspath(DST):
        continue
    with open(d, encoding="utf-8", errors="replace") as f:
        named.update(int(m.group(1), 16) for m in rx.finditer(_txt(d, f.read())))

GLOB = re.compile(r"\b(?:sub|off|unk|qword|dword|byte|word|xmmword|stru|flt|dbl)_(18[0-9A-Fa-f]{7})\b")
def gva(s):
    m = GLOB.search(s); return f"`0x{m.group(1).lower()}`" if m else None
def num(s):
    s = re.sub(r"(u?i64|uLL|LL|u|i)$", "", s.strip())
    try: return int(s, 0)
    except ValueError: return None
TYPES = {"_QWORD": "8-byte", "_DWORD": "4-byte", "_WORD": "2-byte", "_BYTE": "byte",
         "unsigned __int8": "byte", "char": "byte", "float": "float", "double": "double",
         "__int64": "8-byte", "int": "4-byte", "__m128i": "16-byte", "__m128": "16-byte"}

SKIP = {"if", "while", "for", "return", "sizeof", "LODWORD", "HIDWORD", "LOBYTE", "HIBYTE", "LOWORD", "BYTE1", "BYTE2"}
def demangle(n):
    m = re.match(r"\?(\w+)@\?\$(\w+)@", n)
    if m: return f"std::{m.group(2)}::{m.group(1)}"
    m = re.match(r"\?(\w+)@(\w+)@@", n)
    if m: return f"{m.group(2)}::{m.group(1)}"
    return n.lstrip("?")
def named_call(body):
    for l in body:
        for m in re.finditer(r"(__imp_)?(\?[\w@?$]+|[A-Za-z_]\w*)\(", l):
            n = m.group(2)
            if n in SKIP or re.fullmatch(r"v\d+", n) or n.startswith("sub_") or n.startswith("_m") or "vftable" in l and n.startswith("std"):
                continue
            return ("import" if m.group(1) else "call"), demangle(n)
    return None

def small(b):
    sig, body = b[0], b[1:]
    j = " ".join(body)
    if re.search(r"m(128|256)\w*\[", j):
        return "reads one lane of a SIMD vector selected by argument 2"
    if "^ 0x8000000080000000" in j:
        return "copies two packed floats with both signs flipped"
    if re.search(r"~\*a3 & \*a1 \| \*a3 & \*a2", j):
        return "bitwise select: bits of argument 2 where the mask (argument 3) is set, else argument 1"
    if re.search(r"\(\*\(__int64 \(__fastcall \*\*\)\(_QWORD\)\)\(\*\*\(_QWORD \*\*\)\(a1 \+ (\d+)\) \+ (\d+)LL\)\)", j):
        m = re.search(r"\(a1 \+ (\d+)\) \+ (\d+)LL", j)
        return f"calls virtual slot {int(m.group(2)) // 8} of the member object at +0x{int(m.group(1)):x}"
    if sig.startswith("bool") and len(body) == 1:
        return "predicate: tests its argument or one field against a constant range or bit mask (no side effects)"
    if re.fullmatch(r"\*\(_DWORD \*\)\(a1 \+ (\d+)\) = 0;", body[0]) and body[-1] == "return 1;":
        return f"clears the 4-byte field at +0x{int(re.search(r'(\d+)', body[0][16:]).group(1)):x} and returns true"
    if "`vftable'" in j and "_Func_impl" in j:
        return "std::function plumbing: installs a lambda's call table into the target"
    if "RTTI Type Descriptor" in j:
        return "std::function plumbing: returns the lambda's RTTI type descriptor"
    if "onexit(" in j:
        return "registers an exit handler (CRT onexit)"
    if "setstate(" in j:
        return "sets the fail state of a std::ios stream"
    if re.search(r"memset\(a1, \*a3, 0x100u\)", j):
        return "fills a 256-byte table with one byte value"
    if re.search(r"memset\(a1, 0, a2 - ", j):
        return "zeroes the byte range [first, last)"
    if re.search(r"\(\*\(\w+ \(__fastcall \*\*\)\(\w+\)\)\(\*\(_QWORD \*\)a1 \+ (\d+)LL\)\)\(a1\)", j):
        k = int(re.search(r"\+ (\d+)LL\)\)\(a1\)", j).group(1)) // 8
        return f"calls virtual slot {k} of its object"
    if m := re.match(r"(?:qword|dword|byte|word|off)_(18[0-9A-F]{7}) = a(\d);", body[0]):
        return f"stores argument {m.group(2)} into global `0x{m.group(1).lower()}`"
    if m := re.search(r"\b(?:dword|qword)_(18[0-9A-F]{7})\+\+", j):
        return f"returns and increments the counter `0x{m.group(1).lower()}`"
    if m := re.search(r"return \*\((\w+) \*\)\(a1 \+ (\d+)\) > 0;", j):
        return f"tests whether the field at +0x{int(m.group(2)):x} is positive"
    if m := re.search(r"return (-?\d+\.\d+);", j):
        return f"returns the constant {m.group(1)}"
    if m := re.fullmatch(r"return a1 \+ (\d+);", body[-1]) if len(body) == 1 else None:
        return f"returns the address of the sub-object at +0x{int(m.group(1)):x}"
    if m := re.search(r"return \(\w[\w ]*\)(?:byte|dword|word|qword)_(18[0-9A-F]{7});", j):
        return f"reads global `0x{m.group(1).lower()}`"
    if re.search(r"8LL \* a2 \+ 8\)", j):
        return "reads a field of the entry selected by argument 2 in a pointer table"
    nc = named_call(body)
    if nc:
        kind, n = nc
        if kind == "import":
            return f"import stub for `{n}`"
        if re.match(r"(clock|_Xtime_get_ticks)$", n) and "*a1 =" in j:
            return f"stores `{n}()` into its argument"
        return f"wrapper around `{n}`"
    return None

def describe(va, b, s):
    r = small(b) if s in ("const_ret", "getter", "setter", "thunk") and len(b) <= 4 else None
    return r or describe0(va, b, s)

def describe0(va, b, s):
    sig, body = b[0], b[1:]
    j = feat.get(va, {})
    if s == "nullret":
        return "empty body (no-op)"
    if s == "const_ret":
        m = re.match(r"return (.*);$", body[-1]); x = m.group(1) if m else ""
        if (n := num(x)) is not None:
            return f"returns the constant {n}"
        if x.startswith('"'):
            return f"returns a constant string {x[:48]}"
        if g := gva(x):
            return f"returns the address of global {g}"
        if re.fullmatch(r"a\d", x):
            return f"returns argument {x[1:]} unchanged"
        return "returns a constant"
    if s == "getter":
        x = body[0]
        if re.fullmatch(r"return a\d;", x):
            return f"identity: returns argument {x[8]}"
        m = re.search(r"\*\((.+?) \*\)\(?a\d \+ (\d+)\)?", x)
        if m:
            t = TYPES.get(m.group(1).strip(), m.group(1).strip())
            return f"reads the {t} field at +0x{int(m.group(2)):x} of its object"
        if re.fullmatch(r"return \*a1;", x):
            return "reads the first field of its object"
        if g := gva(x):
            return f"reads global {g}"
        return "returns one field of its object"
    if s == "thunk":
        cs = [c for c in j.get("callees", [])]
        tgt = f"`{cs[0]}`" if cs else "an import"
        call = next(l for l in body if ns["R"]["call"].search(l) or "(" in l)
        extra = " with extra constant arguments" if re.search(r", -?(\d+|0x[0-9A-F]+)u?\)", call) else ""
        idx = " on an entry of a global table" if re.search(r"\+ \d+ \* a\d", call) else ""
        return f"forwards to {tgt}{idx}{extra}"
    if s == "setter":
        m = re.match(r"\*\((.+?) \*\)\(a\d \+ (\d+)\) = a(\d);", body[0])
        if m:
            return f"stores argument {m.group(3)} into the {TYPES.get(m.group(1), m.group(1))} field at +0x{int(m.group(2)):x}"
        if "_Xtime_get_ticks" in body[0]:
            return "stores the current time (100 ns ticks) into its argument"
        if "__m128" in body[0]:
            return "reads one lane of a 4-wide SIMD vector"
        if "printf_options" in body[0] or "scanf_options" in body[0]:
            return "CRT stdio wrapper (fetches the locale option block, forwards)"
        return "two-statement leaf (one store or load, then return)"
    if s == "eh_ctor":
        m = re.search(r"iterator'\(([^,]+), (\w+), (\w+), (.*)\);", " ".join(body))
        kind = "destroys" if "destructor" in " ".join(body) else "constructs"
        if m:
            sz, n = num(m.group(2)), num(m.group(3)); fn = m.group(4)
            fe = "no-op element function" if "nullsub" in fn else f"element function {gva(fn) or 'inline'}"
            off = re.search(r"\+ (\d+)", m.group(1))
            at = f" at +0x{int(off.group(1)):x}" if off else ""
            return f"{kind} an array{at} of {n} × 0x{sz:x}-byte elements ({fe})"
        return f"{kind} an array of elements (compiler array helper)"
    if s == "vec_free":
        return "RnD::Vector release: frees the storage through its allocator and zeroes begin/end/capacity"
    if s == "vec_grow":
        sh = re.search(r">> (\d)\b", " ".join(body)); dv = re.search(r"/ (\d+)\b", " ".join(body))
        el = f"{1 << int(sh.group(1))}-byte" if sh else (f"{dv.group(1)}-byte" if dv else "fixed-size")
        return f"RnD::Vector grow/insert for {el} elements (allocates through the allocator, moves the old items)"
    if s == "dtor":
        m = re.search(r"&([\w:]+)::`vftable'", " ".join(body))
        cls = f" of `{m.group(1)}`" if m else ""
        frees = " (frees itself when the flag argument is set)" if re.search(r"free|delete", " ".join(body)) else ""
        return f"destructor{cls}{frees}"
    if s == "zero_init":
        m = re.search(r"&([\w:]+)::`vftable'", " ".join(body))
        cls = f" of `{m.group(1)}`" if m else ""
        return f"constructor/reset{cls}: sets about {len(body)} fields to zero or defaults"
    if s == "copy":
        return f"copies a record field by field (about {len(body)} fields)"
    if s == "xordec":
        if va in xs:
            return f"decodes an obfuscated log string: {xs[va][:90]}"
        return "decodes an obfuscated log string (XOR with fixed 64-bit keys)"
    return None

rows = collections.defaultdict(list)
for va, b in sorted(bodies.items()):
    t = tri.get(va)
    if not t or (va in named and not ALL):
        continue
    s = shape(b)
    if s == "other":
        continue
    d = describe(va, b, s)
    if not d:
        continue
    cl = feat.get(va, {}).get("callers", [])
    cs = ", ".join(f"`{c}`" for c in cl[:2]) + (f" +{len(cl) - 2}" if len(cl) > 2 else "")
    rows[t["subsystem"]].append((va, t["verdict"], s, d, cs or "none (vtable/callback)"))

n = sum(len(v) for v in rows.values())
o = ["# Function catalogue: trivial functions", "",
     "Generated by `scripts/sony-re/triage/trivial.py` from the body shape of",
     "each function (rules in `shapes.py`). Every row is a leaf or near-leaf",
     "helper: a constant return, a field read, a forwarder, a compiler array",
     "helper, an `RnD::Vector` grow/free, a destructor, a constructor that",
     "only zeroes fields, a record copy or an obfuscated-string decoder. The",
     "role column is in our own words and is `[sure]` for the shape; what the",
     "caller does with the value is in the caller's own entry.", "",
     f"{n} functions.", ""]
for sub in sorted(rows):
    o += [f"## {sub} ({len(rows[sub])})", "",
          "| Function | Verdict | Shape | Role | Callers |",
          "|----------|---------|-------|------|---------|"]
    for va, v, s, d, cs in rows[sub]:
        o.append(f"| `0x{va:x}` | {v} | {s} | {d.replace('|', '/')} | {cs} |")
    o.append("")
with open(DST, "w", encoding="utf-8", newline="\n") as f:
    f.write("\n".join(o))
print(n, "rows ->", DST)
