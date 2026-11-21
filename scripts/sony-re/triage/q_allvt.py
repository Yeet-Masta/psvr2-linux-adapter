# Dump every vtable that has an RTTI complete-object locator: name, slots.
# Output lines: "VT <ea> <demangled name>" then "  <slot> <target>".
import ida_bytes, ida_funcs, ida_name, idautils, ida_segment

def demangle(n):
    d = ida_name.demangle_name(n, 0)
    return d or n

text = ida_segment.get_segm_by_name(".text")
for ea, name in idautils.Names():
    if not (name.startswith("??_7") or "vftable" in name):
        continue
    slots = []
    i = 0
    while i < 200:
        q = ida_bytes.get_qword(ea + i * 8)
        f = ida_funcs.get_func(q)
        if not f or f.start_ea != q:
            break
        if i and ida_name.get_name(ea + i * 8):
            break
        slots.append(q)
        i += 1
    out(f"VT {ea:#x} {demangle(name)}")
    for k, q in enumerate(slots):
        out(f"  {k} {q:#x}")
