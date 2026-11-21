# Triage every function in inventory.tsv into core / ref / fluff.
#   core  = behaviour the Linux driver needs (already ported or still to port)
#   ref   = worth reading for knowledge, or an optional feature; no direct port needed now
#   fluff = library, platform glue, debug tooling, Windows-only app plumbing
# Method: seed labels (status TSVs, RTTI vtable class, library ranges, manual overrides, doc-named roots),
# then propagate down the call graph: a helper gets the "best" label of its callers
# (core > ref > fluff). Library code never inherits.
# Inputs: repo inventory.tsv + status/*.tsv, out/features.jsonl (q_features.py), out/docrefs.json,
#         tools/classes.tsv, tools/overrides.tsv.
# Output: TSV on stdout.
import csv, json, re, sys, os, collections

REPO = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter\scripts\sony-re"
OUT = r"C:\Users\Yeet Masta\Claude\psvr2-re\out"
TOOLS = os.path.dirname(os.path.abspath(__file__))
RANK = {"core": 3, "ref": 2, "fluff": 1}

def tsv(p):
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f, delimiter="\t"))

def rows(p):
    for l in open(p, encoding="utf-8"):
        l = l.rstrip("\n")
        if l and not l.startswith("#"):
            yield l.split("\t")

inv = tsv(os.path.join(REPO, "inventory.tsv"))
invset = {r["va"] for r in inv}
status = {}
for fn in os.listdir(os.path.join(REPO, "status")):
    for r in tsv(os.path.join(REPO, "status", fn)):
        status[r["va"]] = (fn[:-4], r["area"], r["status"], r.get("note") or "")
feat = {}
with open(os.path.join(OUT, "features.jsonl"), encoding="utf-8") as f:
    for l in f:
        r = json.loads(l)
        feat[r["va"]] = r
docrefs = json.load(open(os.path.join(OUT, "docrefs.json")))
classes = [(c, v, a, why) for c, v, a, why in rows(os.path.join(TOOLS, "classes.tsv"))]
ovr = []
for lo, hi, v, a, why in rows(os.path.join(TOOLS, "overrides.tsv")):
    lo = int(lo, 16); hi = lo if hi == "-" else int(hi, 16)
    ovr.append((lo, hi, v, a, why))

LIBSUB = {"import/thunk", "lib:crt", "lib:linalg", "lib:misc", "lib:json+config"}
IO = re.compile(r"^(WinUsb_|ReadFile$|WriteFile$|DeviceIoControl|HidD_|CreateFileW|GetOverlappedResult|CancelIo)")
UI = re.compile(r"^(MessageBox|CreateProcess|ShellExecute|CreateWindow|DialogBox)")
LIBSTR = re.compile(r"Json::|in Json::Value|too long$|escape sequence|is not a number|Unable to parse token|Missing '[,}\]]'|bad allocation|invalid string position|string too long|map/set|vector<T>")
sub_of = {r["va"]: r["subsystem"] for r in inv}

label = {}   # va -> (verdict, area, reason, source)  source: seed|prop
fixed = set()  # seeds that propagation may not raise

def seed(va, v, area, why, fix=True):
    label[va] = (v, area, why, "seed")
    if fix:
        fixed.add(va)

for va, f in feat.items():
    ea = int(va, 16)
    sub = sub_of.get(va, "")
    st = status.get(va)
    known = st and st[2] != "todo"
    hit = next(((v, a, why) for lo, hi, v, a, why in ovr if lo <= ea <= hi and (lo == hi or not known)), None)
    if hit:
        seed(va, *hit); continue
    if st and st[2].startswith("skip"):
        n = st[3]
        if "other-subsystem" in st[2]:
            pass  # trace.tsv says it belongs elsewhere: let class/propagation decide
        elif st[2] == "skip:dup":
            seed(va, "core", "ctrlboot", f"{st[2]}: {n}", fix=False); continue
        else:
            seed(va, "fluff", st[0], f"{st[2]}: {n}"); continue
    vt = " ".join(x for x in f["vt"] if "<lambda_" not in x)  # std::function lambda bodies: label by use
    c = next(((v, a, why) for cn, v, a, why in classes if cn in vt), None) if vt else None
    if c:
        seed(va, c[0], c[1], f"method of {vt.replace('const ','').split('::`')[0]}: {c[2]}"); continue
    if st and st[2].split(":")[0] in ("bit-exact", "spec-only", "tolerance"):
        if st[0] == "trace":  # trace.tsv: mostly VrTrace itself; only the room/chaperone glue matters
            if re.search(r"map state|universe|chaperone|sceBoundaryMeta|pose7", st[3]):
                seed(va, "core", "room", f"{st[2]}: {st[3]}")
            elif re.search(r"image header|ST |receiveImage|receiveSlam|image format|view id|record validate", st[3]):
                seed(va, "ref", "trace-format", f"VrTrace, but documents a wire format: {st[3]}")
            else:
                seed(va, "fluff", "debug", f"VrTrace ({st[2]}): {st[3]}")
            continue
        seed(va, "core", st[0], f"{st[2]}: {st[3]}", fix=False); continue
    if any(LIBSTR.search(x) for x in f["strs"]):
        seed(va, "fluff", "lib", "library (jsoncpp / STL, by its error strings)"); continue
    if sub == "lib:json+config" and ea >= 0x180154870:
        pass  # ConfigManager / ShareManager glue, not jsoncpp: label by class and callers
    elif sub in LIBSUB or f["name"].startswith(("j_", "nullsub", "??", "?", "_")) and sub not in ("ledsync",):
        seed(va, "fluff", "lib", f"library ({sub or 'CRT/STL'})"); continue
    if any(IO.match(i) for i in f["imps"]):
        seed(va, "core", "io", "device I/O: " + ",".join(i for i in f["imps"] if IO.match(i)), fix=False); continue
    if any(UI.match(i) for i in f["imps"]):
        seed(va, "fluff", "app", "Windows UI/process: " + ",".join(i for i in f["imps"] if UI.match(i))); continue

NOPROP = {"lib", "init", "platform"}  # library / initialiser / OS wrappers call everything; do not inherit from them
# propagate down the call graph (callers -> callees)
changed = True
it = 0
while changed and it < 50:
    changed = False; it += 1
    for va, f in feat.items():
        if va not in label:
            continue
        v, area, why, src = label[va]
        if area in NOPROP:
            continue
        for c in f["callees"]:
            if c in fixed or c not in feat:
                continue
            cur = label.get(c)
            if cur is None or RANK[v] > RANK[cur[0]]:
                label[c] = (v, area, f"helper of {va} ({area})", "prop")
                changed = True

# leftovers, round 1: a wrapper of core/ref code inherits upward (iterate)
for _ in range(10):
    for va, f in feat.items():
        if va in label:
            continue
        best = None
        for c in f["callees"]:
            l = label.get(c)
            if l and l[0] in ("core", "ref") and (best is None or RANK[l[0]] > RANK[best[0]]):
                best = l
        if best:
            label[va] = (best[0], best[1], f"calls {best[1]} code", "up")
# round 2: MSVC lays out functions per object file, so an unlabelled function between two
# neighbours of one verdict (same inventory subsystem) gets that verdict.
order = sorted(feat, key=lambda v: int(v, 16))
def near(i, step):
    j = i + step
    while 0 <= j < len(order):
        l = label.get(order[j])
        if l and l[3] != "nb" and l[1] not in NOPROP:
            return order[j], l
        j += step
    return None, None
for i, va in enumerate(order):
    if va in label:
        continue
    a, la = near(i, -1); b, lb = near(i, 1)
    sub = sub_of.get(va)
    if la and lb and la[0] == lb[0] and sub_of.get(a) == sub == sub_of.get(b):
        label[va] = (la[0], la[1], f"same object file as neighbours {a}..{b} ({la[1]})", "nb")
lines_of = {r["va"]: int(r["lines"] or 0) for r in inv}
for va in invset - set(feat):  # Ghidra-only boundaries (IDA merged them into a neighbour)
    feat[va] = {"va": va, "name": "", "size": 0, "strs": [], "imps": [], "callees": [], "callers": [], "drefs": 0, "vt": []}
# round 3: what is still unlabelled has no direct caller (virtual method, callback, table entry).
CORESUB = {"ctrl-boot": "ctrlboot", "ctrl-tracker": "ctrltracker", "ins+irtracker": "tracking", "ledsync": "ledsync", "libpad": "libpad"}
for va, f in feat.items():
    if va in label:
        continue
    sub = sub_of.get(va, ""); st = status.get(va)
    stl = "_invalid_parameter_noinfo_noreturn" in f["imps"] and not f["strs"]
    if sub in CORESUB or (st and "other-subsystem" in st[2]):
        label[va] = ("core", CORESUB.get(sub, "ctrlboot"), f"virtual method / callback in {sub} code (no direct caller)", "sub")
    elif sub == "reloc":
        label[va] = ("ref", "reloc", "virtual method / callback in relocaliser code (no direct caller)", "sub")
    elif stl:
        label[va] = ("fluff", "lib", "MSVC STL instantiation (container insert/erase/free)", "sub")
    elif lines_of.get(va, 99) <= 12 and not f["strs"]:
        label[va] = ("fluff", "lib", "trivial stub (destructor, thunk, empty vtable slot)", "sub")
    elif any(i.startswith(("Cert", "Crypt", "Image")) for i in f["imps"]):
        label[va] = ("fluff", "app", "Authenticode check of Sony's companion executables", "sub")
    elif 0x180154870 <= int(va, 16) < 0x180160000:
        label[va] = ("ref", "config", "ConfigManager / ShareManager glue (vtable-only slot)", "sub")
    else:
        label[va] = ("unknown", "", "no labelled caller, callee or neighbour", "none")

if __name__ == "__main__":
    w = csv.writer(sys.stdout, delimiter="\t", lineterminator="\n")
    w.writerow(["va", "lines", "subsystem", "verdict", "area", "state", "docs", "reason"])
    cnt = collections.Counter()
    for r in inv:
        va = r["va"]
        v, area, why, src = label.get(va, ("unknown", "", "not in IDA function list", "none"))
        st = status.get(va)
        state = st[2] if st else ""
        cnt[v] += 1
        w.writerow([va, r["lines"], r["subsystem"], v, area, state, ",".join(docrefs.get(va, [])), why])
    print(dict(cnt), file=sys.stderr)
