"""Shared helpers for the scripts that read or write the Sony RE docs.

Generated tables live inside docs/sony-re/function-catalogue.md between
<!-- gen:NAME:start --> and <!-- gen:NAME:end --> markers (NAME is
`trivial` or `library`). Only their first column counts as documentation.
"""
import glob, os, re

ADAPTER = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
if not os.path.isdir(os.path.join(ADAPTER, "docs", "sony-re")):   # copy run from the scratch repo's tools/
    ADAPTER = os.environ.get("PSVR2_ADAPTER", r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter")
CATALOGUE = os.path.join(ADAPTER, "docs", "sony-re", "function-catalogue.md")
GEN = re.compile(r"(<!-- gen:(\w+):start -->\n)(.*?)(<!-- gen:\2:end -->)", re.S)


def doc_paths():
    return glob.glob(os.path.join(ADAPTER, "docs", "**", "*.md"), recursive=True)


def doc_text(t, skip=()):
    """Text to scan for VAs: generated blocks count their first column only;
    blocks named in `skip` are dropped entirely."""
    def sub(m):
        if m.group(2) in skip:
            return ""
        return "\n".join(l.split("|")[1] for l in m.group(3).splitlines() if l.startswith("| `0x"))
    return GEN.sub(sub, t)


def read_docs(skip=()):
    """Yield (path, scan text) for every doc."""
    for d in doc_paths():
        with open(d, encoding="utf-8", errors="replace") as f:
            yield d, doc_text(f.read(), skip)


def splice(name, body):
    """Replace the generated block `name` in the catalogue with `body`."""
    with open(CATALOGUE, encoding="utf-8") as f:
        t = f.read()
    n = [0]
    def sub(m):
        if m.group(2) != name:
            return m.group(0)
        n[0] += 1
        return m.group(1) + body.strip("\n") + "\n" + m.group(4)
    t = GEN.sub(sub, t)
    if n[0] != 1:
        raise SystemExit(f"gen block {name!r} not found in {CATALOGUE}")
    with open(CATALOGUE, "w", encoding="utf-8", newline="\n") as f:
        f.write(t)
