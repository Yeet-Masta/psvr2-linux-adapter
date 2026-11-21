# All xrefs (code or data) to each address in IDA_ARGS, with the referencing segment.
import os, idautils, ida_funcs, ida_segment, ida_name
for a in os.environ["IDA_ARGS"].split():
    ea = int(a, 16); refs = []
    for x in idautils.XrefsTo(ea):
        f = ida_funcs.get_func(x.frm)
        seg = ida_segment.get_segm_name(ida_segment.getseg(x.frm)) if ida_segment.getseg(x.frm) else "?"
        refs.append(f"{x.frm:#x}[{seg}{':fn ' + hex(f.start_ea) if f else ''}]")
    out(f"{ea:#x}: " + " ".join(refs[:6]))
