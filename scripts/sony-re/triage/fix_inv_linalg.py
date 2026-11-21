# lib:linalg (0x180070950..0x18009ffe0) is mostly Sony code. Only the f2c CLAPACK
# block 0x180083700..0x18008d3ff stays a library. UKF + its double-precision
# matrix code -> ctrl-tracker; RnD math helpers from 0x18009d100 -> ctrl-boot.
import csv
p = r"C:\Users\Yeet Masta\Claude\psvr2-linux-adapter\scripts\sony-re\inventory.tsv"
rows = list(csv.reader(open(p, encoding="utf-8", newline=""), delimiter="\t"))
n = 0
for r in rows[1:]:
    ea = int(r[0], 16)
    if r[2] != "lib:linalg" or 0x180083700 <= ea <= 0x18008ee2f:
        continue
    r[2] = "ctrl-boot" if ea >= 0x18009d100 else "ctrl-tracker"
    r[3] = "todo"; n += 1
with open(p, "w", encoding="utf-8", newline="") as f:
    csv.writer(f, delimiter="\t", lineterminator="\n").writerows(rows)
print("relabelled", n)
