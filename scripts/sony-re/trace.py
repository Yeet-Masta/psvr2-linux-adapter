#!/usr/bin/env python3
"""Navigate the Ghidra export of Sony's PSVR2 driver by call graph.

Reads decomp.c, strings_xref.tsv (from decompile.sh) and inventory.tsv.
Library code (CRT/STL, jsoncpp, linear algebra, thunks, UI dialogs) is
pruned from every walk unless --all is given, so a trace stays on the code
that decides device behaviour.

  trace.py tree    FUN_x [-d N]      call tree from an entry point
  trace.py callers FUN_x [-d N]      who calls it (reverse tree)
  trace.py find    REGEX             functions whose strings match
  trace.py info    FUN_x             size, subsystem, strings, callers, callees
  trace.py bundle  FUN_x [-d N] [-o FILE]
                                     write the entry and every reachable
                                     non-library function, depth-first, to
                                     one file for reading top-down

Output of `bundle` is Sony's decompiled code: it goes to the work directory
outside the repo (default $PSVR2_RE_WORK/bundles) and must never be
committed or sent off this machine.
"""
import argparse
import os
import re
import sys
from collections import defaultdict

WORK = os.environ.get("PSVR2_RE_WORK", "/home/yeetmasta/psvr2-re-work")
HERE = os.path.dirname(os.path.abspath(__file__))
LIB = ("lib:", "import/thunk", "ui")
HDR = re.compile(r"^// ==== (FUN_[0-9a-f]+) @ ([0-9a-f]+)")
CALL = re.compile(r"\bFUN_[0-9a-f]{9}\b")


def load():
	body, order = {}, []
	cur = None
	with open(os.path.join(WORK, "decomp.c"), errors="replace") as f:
		for line in f:
			m = HDR.match(line)
			if m:
				cur = m.group(1)
				order.append(cur)
				body[cur] = []
			if cur:
				body[cur].append(line)
	callees = {}
	callers = defaultdict(set)
	for fn, lines in body.items():
		seen = []
		for c in CALL.findall("".join(lines[1:])):
			if c != fn and c in body and c not in seen:
				seen.append(c)
				callers[c].add(fn)
		callees[fn] = seen
	strings = defaultdict(list)
	with open(os.path.join(WORK, "strings_xref.tsv"), errors="replace") as f:
		for line in f:
			parts = line.rstrip("\n").split("\t", 2)
			if len(parts) < 3:
				continue
			for fn in set(filter(None, parts[1].split(","))):
				strings[fn].append(parts[2])
	sub = {}
	with open(os.path.join(HERE, "inventory.tsv")) as f:
		next(f)
		for line in f:
			va, _, s, _ = line.rstrip("\n").split("\t")
			sub["FUN_" + va[2:].lower()] = s
	# Per-subsystem status files correct inventory labels (for example the
	# controller-tracker frame code inside the range labelled lib:crt).
	sdir = os.path.join(HERE, "status")
	for name in sorted(os.listdir(sdir)) if os.path.isdir(sdir) else []:
		with open(os.path.join(sdir, name)) as f:
			for line in f:
				cols = line.rstrip("\n").split("\t")
				if len(cols) > 3 and cols[0].startswith("0x") and not cols[3].startswith("skip"):
					sub["FUN_" + cols[0][2:].lower()] = name.rsplit(".", 1)[0]
	return body, order, callees, callers, strings, sub


def norm(name):
	name = name.lower()
	if name.startswith("0x"):
		name = name[2:]
	if not name.startswith("fun_"):
		name = "fun_" + name
	return "FUN_" + name[4:]


def main():
	ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("cmd", choices=["tree", "callers", "find", "info", "bundle"])
	ap.add_argument("arg")
	ap.add_argument("-d", "--depth", type=int, default=4)
	ap.add_argument("-o", "--out")
	ap.add_argument("--all", action="store_true", help="do not prune library code")
	ap.add_argument("--min-lines", type=int, default=0, help="hide leaves smaller than this")
	a = ap.parse_args()

	body, order, callees, callers, strings, sub = load()

	def is_lib(fn):
		return not a.all and sub.get(fn, "").startswith(LIB)

	def label(fn):
		s = strings.get(fn, [])
		hint = (" | " + " / ".join(x[:60] for x in s[:2])) if s else ""
		return f"{fn} [{len(body[fn])}L {sub.get(fn, '?')}]{hint}"

	if a.cmd == "find":
		rx = re.compile(a.arg, re.I)
		for fn in order:
			hits = [s for s in strings.get(fn, []) if rx.search(s)]
			if hits:
				print(f"{label(fn)}\n    " + "\n    ".join(h[:120] for h in hits[:4]))
		return

	fn = norm(a.arg)
	if fn not in body:
		sys.exit(f"{fn}: not in decomp.c")

	if a.cmd == "info":
		print(label(fn))
		for s in strings.get(fn, []):
			print("  str:", s[:150])
		print("  callers:", " ".join(sorted(callers[fn])) or "-")
		print("  callees:", " ".join(c for c in callees[fn] if not is_lib(c)) or "-")
		libs = [c for c in callees[fn] if is_lib(c)]
		if libs:
			print("  library callees (pruned):", len(libs))
		return

	if a.cmd in ("tree", "callers"):
		edges = callees if a.cmd == "tree" else {k: sorted(v) for k, v in callers.items()}
		seen = set()

		def walk(f, d):
			kids = [c for c in edges.get(f, []) if not is_lib(c)]
			mark = " (seen)" if f in seen and kids else ""
			print("  " * d + label(f) + mark)
			if f in seen:
				return
			seen.add(f)
			if d >= a.depth:
				if kids:
					print("  " * (d + 1) + f"... {len(kids)} more")
				return
			for c in kids:
				if a.min_lines and len(body[c]) < a.min_lines and not edges.get(c):
					continue
				walk(c, d + 1)

		walk(fn, 0)
		return

	# bundle
	out = a.out or os.path.join(WORK, "bundles", f"{fn}_d{a.depth}.c")
	os.makedirs(os.path.dirname(out), exist_ok=True)
	seen, ordered = set(), []

	def collect(f, d):
		if f in seen or is_lib(f):
			return
		seen.add(f)
		ordered.append((f, d))
		if d < a.depth:
			for c in callees[f]:
				collect(c, d + 1)

	collect(fn, 0)
	total = 0
	with open(out, "w") as o:
		o.write(f"// bundle of {fn}, depth {a.depth}: {len(ordered)} functions\n")
		for f, d in ordered:
			o.write(f"\n// ---- depth {d} ---- {label(f)}\n")
			o.writelines(body[f])
			total += len(body[f])
	print(f"{out}: {len(ordered)} functions, {total} lines")


if __name__ == "__main__":
	main()
