"""Reverse-engineering progress for the Sony PSVR2 SteamVR driver DLL.

Usage: python scripts/sony-re/triage/progress.py [--check]

Reads scripts/sony-re/triage.tsv (every function in the DLL with its
verdict) and scans docs/ for function addresses. A function counts as
"documented" once any doc names its address (0x18..., FUN_18..., sub_18...).

Writes:
  docs/sony-re/progress.svg   stacked progress bar (core / ref / fluff)
  README.md                   table between the re-progress markers

--check exits 1 if either file is out of date (no writes).
"""
import collections, glob, os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from docscan import doc_text, splice, CATALOGUE  # noqa: E402

def _txt(path, t):
    """Doc text to scan for VAs; generated tables count their first column only."""
    return doc_text(t)

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
TRIAGE = os.path.join(ROOT, "scripts", "sony-re", "triage.tsv")
SVG = os.path.join(ROOT, "docs", "sony-re", "progress.svg")
README = os.path.join(ROOT, "README.md")
START, END = "<!-- re-progress:start -->", "<!-- re-progress:end -->"
VERDICTS = ("core", "ref", "fluff")


def documented_vas():
    rx = re.compile(r"(?:0x|FUN_|sub_)(18[0-9a-fA-F]{7})\b")
    named = set()
    for d in glob.glob(os.path.join(ROOT, "docs", "**", "*.md"), recursive=True):
        with open(d, encoding="utf-8", errors="replace") as f:
            named.update(int(m.group(1), 16) for m in rx.finditer(_txt(d, f.read())))
    return named


def load():
    with open(TRIAGE, encoding="utf-8") as f:
        hdr = f.readline().rstrip("\n").split("\t")
        return [dict(zip(hdr, l.rstrip("\n").split("\t"))) for l in f if l.strip()]


def bar(frac, width=20):
    n = round(frac * width)
    return "█" * n + "░" * (width - n)


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def make_svg(tot, doc, ntot, ndoc, lines_frac):
    w, h, bx, by, bw, bh = 640, 96, 16, 40, 608, 22
    colors = {"core": "#2f81f7", "ref": "#a371f7", "fluff": "#8b949e"}
    segs, x = [], bx
    for v in VERDICTS:
        sw = bw * doc[v] / ntot
        if sw > 0:
            segs.append(f'<rect x="{x:.1f}" y="{by}" width="{sw:.1f}" height="{bh}" fill="{colors[v]}"/>')
        x += sw
    legend, lx = [], bx
    for v in VERDICTS:
        legend.append(
            f'<rect x="{lx}" y="76" width="10" height="10" rx="2" fill="{colors[v]}"/>'
            f'<text x="{lx + 14}" y="85" class="s">{v} {doc[v]}/{tot[v]}</text>')
        lx += 150
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" role="img" aria-label="PSVR2 DLL: {ndoc} of {ntot} functions documented">
<style>
text{{font:600 14px -apple-system,Segoe UI,Helvetica,Arial,sans-serif;fill:#1f2328}}
.s{{font-weight:400;font-size:12px;fill:#59636e}}
.track{{fill:#d1d9e0}}
@media (prefers-color-scheme:dark){{text{{fill:#f0f6fc}}.s{{fill:#9198a1}}.track{{fill:#3d444d}}}}
</style>
<text x="{bx}" y="26">PSVR2 driver DLL: {ndoc} / {ntot} functions documented ({pct(ndoc, ntot):.1f}%)</text>
<text x="{bx + bw}" y="26" text-anchor="end" class="s">{100 * lines_frac:.1f}% of code by lines</text>
<rect class="track" x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="5"/>
<clipPath id="c"><rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="5"/></clipPath>
<g clip-path="url(#c)">{"".join(segs)}</g>
{"".join(legend)}
</svg>
"""


def make_table(rows, named):
    tot = collections.Counter(); doc = collections.Counter()
    for r in rows:
        if r["verdict"] != "core":
            continue
        tot[r["subsystem"]] += 1
        doc[r["subsystem"]] += int(r["va"], 16) in named
    out = ["| Subsystem (core functions) | Progress | Documented |", "|---|---|---|"]
    for s in sorted(tot, key=lambda s: -tot[s]):
        f = doc[s] / tot[s]
        out.append(f"| `{s}` | `{bar(f)}` {100 * f:.0f}% | {doc[s]} / {tot[s]} |")
    return out


def main():
    check = "--check" in sys.argv
    rows = load()
    named = documented_vas()
    tot = collections.Counter(r["verdict"] for r in rows)
    doc = collections.Counter(r["verdict"] for r in rows if int(r["va"], 16) in named)
    ntot, ndoc = len(rows), sum(doc.values())
    lines_all = sum(int(r["lines"]) for r in rows)
    lines_doc = sum(int(r["lines"]) for r in rows if int(r["va"], 16) in named)
    triaged = sum(1 for r in rows if r["verdict"] in VERDICTS)

    svg = make_svg(tot, doc, ntot, ndoc, lines_doc / lines_all)
    block = [
        START,
        "",
        "![PSVR2 DLL reverse-engineering progress](docs/sony-re/progress.svg)",
        "",
        "Progress of understanding Sony's PSVR2 SteamVR driver DLL, function by",
        "function. *Triaged* means the function has a verdict (core: the Linux",
        "driver needs it; ref: reference only; fluff: library or debug code).",
        "*Documented* means a doc under [docs/sony-re](docs/sony-re) explains it.",
        "Regenerate with `python scripts/sony-re/triage/progress.py`.",
        "",
        "| Stage | Progress | Functions |",
        "|---|---|---|",
        f"| Triaged | `{bar(triaged / ntot)}` {pct(triaged, ntot):.0f}% | {triaged} / {ntot} |",
        f"| Documented (all) | `{bar(ndoc / ntot)}` {pct(ndoc, ntot):.0f}% | {ndoc} / {ntot} |",
    ] + [
        f"| Documented ({v}) | `{bar(doc[v] / tot[v])}` {pct(doc[v], tot[v]):.0f}% | {doc[v]} / {tot[v]} |"
        for v in VERDICTS
    ] + ["", "<details><summary>Core functions by subsystem</summary>", ""] \
      + make_table(rows, named) + ["", "</details>", "", END]
    block = "\n".join(block)

    with open(README, encoding="utf-8") as f:
        readme = f.read()
    if START in readme:
        new = re.sub(re.escape(START) + ".*?" + re.escape(END), lambda m: block, readme, flags=re.S)
    else:
        new = readme.replace("## Contributing", "## Reverse-engineering progress\n\n" + block + "\n\n## Contributing", 1)

    old_svg = open(SVG, encoding="utf-8").read() if os.path.exists(SVG) else ""
    stale = new != readme or svg != old_svg
    if check:
        print("out of date" if stale else "up to date")
        sys.exit(1 if stale else 0)
    with open(README, "w", encoding="utf-8", newline="\n") as f:
        f.write(new)
    with open(SVG, "w", encoding="utf-8", newline="\n") as f:
        f.write(svg)
    print(f"documented {ndoc}/{ntot} ({pct(ndoc, ntot):.1f}%), lines {100 * lines_doc / lines_all:.1f}%")


if __name__ == "__main__":
    main()
