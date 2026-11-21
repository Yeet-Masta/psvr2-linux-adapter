# Apply MSVC FLIRT signatures in memory (no save) and list functions they rename.
import ida_funcs, ida_auto, ida_name, idautils
def snap():
    d = {}
    for ea in idautils.Functions():
        f = ida_funcs.get_func(ea)
        d[ea] = (ida_name.get_name(ea), bool(f.flags & ida_funcs.FUNC_LIB))
    return d
before = snap()
for i in range(ida_funcs.get_idasgn_qty()):
    out("#pre", *ida_funcs.get_idasgn_desc_with_matches(i))
SIGS = ["vc64_14", "vc64_14.v1", "vc64ucrt", "vc64ucrt.v1", "vc64rtf", "vc64seh", "vc64seh.v1",
        "vc64extra", "vc64md", "vc64atl", "vc64atl.v1", "vc64mfc", "vc64mfc.v1"]
for s in SIGS:
    out("#plan", s, ida_funcs.plan_to_apply_idasgn(s))
ida_auto.auto_wait()
for i in range(ida_funcs.get_idasgn_qty()):
    out("#post", *ida_funcs.get_idasgn_desc_with_matches(i))
after = snap()
for ea, (n, lib) in sorted(after.items()):
    b = before.get(ea, ("", False))
    if lib or n != b[0]:
        dm = ida_name.demangle_name(n, ida_name.MNG_SHORT_FORM) or n
        out(f"0x{ea:x}\t{b[0]}\t{n}\t{int(b[1])}{int(lib)}\t{dm}")
