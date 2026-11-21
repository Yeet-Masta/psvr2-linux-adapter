# Per-function feature dump for triage: one JSON object per function (JSONL).
# Fields: va, name, size, strs (referenced string literals), imps (imports called/referenced),
# callees, callers, drefs (data refs to the function start: vtables, tables), vt (vtable class names),
# consts (notable immediates: report ids etc. not collected; too noisy).
# Env IDA_DECOMP=path: also write all pseudocode to that file with "// ==== va name" headers.
import os, json, idautils, idc, ida_funcs, ida_name, ida_bytes, ida_nalt, ida_segment, ida_hexrays, ida_ua, ida_xref

# string literal map
strmap = {}
for s in idautils.Strings():
    try:
        strmap[s.ea] = str(s)[:160]
    except Exception:
        pass

imports = {}
def cb(ea, name, ordn):
    imports[ea] = name or f"ord{ordn}"
    return True
for i in range(ida_nalt.get_import_module_qty()):
    ida_nalt.enum_import_names(i, cb)

def vt_owner(ea):
    # walk back from a data slot to a ??_7 vtable name (stop after 80 slots)
    p = ea
    for _ in range(80):
        n = ida_name.get_name(p)
        if n and n.startswith("??_7"):
            return ida_name.demangle_name(n, 0) or n
        if n and not n.startswith(("off_", "unk_", "qword_")) and p != ea:
            return None
        p -= 8
        if not ida_funcs.get_func(ida_bytes.get_qword(p)):
            n = ida_name.get_name(p + 8)
            return (ida_name.demangle_name(n, 0) or n) if n and n.startswith("??_7") else None
    return None

dpath = os.environ.get("IDA_DECOMP")
dfile = open(dpath, "w", encoding="utf-8") if dpath else None
if dfile:
    ida_hexrays.init_hexrays_plugin()

for fea in idautils.Functions():
    f = ida_funcs.get_func(fea)
    rec = {"va": f"{fea:#x}", "name": ida_funcs.get_func_name(fea), "size": f.end_ea - f.start_ea}
    strs, imps, callees = [], set(), set()
    for item in idautils.FuncItems(fea):
        for x in idautils.XrefsFrom(item, 0):
            t = x.to
            if t in strmap:
                if strmap[t] not in strs:
                    strs.append(strmap[t])
            elif t in imports:
                imps.add(imports[t])
            else:
                g = ida_funcs.get_func(t)
                if g and g.start_ea != fea and x.iscode:
                    callees.add(g.start_ea)
                elif not x.iscode:
                    # pointer to string through one level (e.g. lea of a const char* table)
                    q = ida_bytes.get_qword(t) if ida_bytes.is_loaded(t) else 0
                    if q in strmap and strmap[q] not in strs:
                        strs.append(strmap[q])
                    elif q in imports:
                        imps.add(imports[q])
                    g2 = ida_funcs.get_func(t)
                    if g2 and g2.start_ea != fea:
                        callees.add(g2.start_ea)  # function pointer taken (thread proc, callback)
    callers, drefs, vts = set(), 0, set()
    for x in idautils.XrefsTo(fea):
        g = ida_funcs.get_func(x.frm)
        if g:
            callers.add(g.start_ea)
        else:
            drefs += 1
            o = vt_owner(x.frm)
            if o:
                vts.add(o)
    rec.update(strs=strs[:40], imps=sorted(imps), callees=[f"{c:#x}" for c in sorted(callees)],
               callers=[f"{c:#x}" for c in sorted(callers)], drefs=drefs, vt=sorted(vts))
    if dfile:
        try:
            txt = str(ida_hexrays.decompile(fea))
        except Exception as e:
            txt = f"// decompile failed: {e}"
        rec["lines"] = txt.count("\n") + 1
        dfile.write(f"// ==== {fea:#x} {rec['name']}\n{txt}\n")
    out(json.dumps(rec))

if dfile:
    dfile.close()
