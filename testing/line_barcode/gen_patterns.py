#!/usr/bin/env python3
"""Code 39 helper for the line and barcode subsystem (Buddy 3).

Two jobs:

  python3 gen_patterns.py check
      Cross-checks the element patterns used by the firmware decoder
      (app_program/line_barcode/bc_decode.c) and by the PC simulation
      (lb_sim_bc.c) against the table below.  Exit status 0 = all agree.
      The table below is the standard Code 39 definition: every character is
      nine elements (bar, space, bar, ...), exactly three of them wide.

  python3 gen_patterns.py sheet [OUT.svg] [NARROW_MM]
      Writes a test sheet with *A*, *B*, *C* and *D* at true size (default
      narrow bar 3 mm, wide bar 9 mm, 20 mm tall).  Print it at 100 percent
      ("actual size", no "fit to page") and measure one narrow bar with a
      ruler to confirm the scale before using it on the track.

Only the Python standard library is used.
"""

import re
import sys
from pathlib import Path

# N = narrow, W = wide.  Elements alternate bar, space, bar, ... (nine per
# character, starting and ending with a bar).
CODE39 = {
    "A": "WNNNNWNNW", "B": "NNWNNWNNW", "C": "WNWNNWNNN", "D": "NNNNWWNNW",
    "E": "WNNNWWNNN", "F": "NNWNWWNNN", "G": "NNNNNWWNW", "H": "WNNNNWWNN",
    "I": "NNWNNWWNN", "J": "NNNNWWWNN", "K": "WNNNNNNWW", "L": "NNWNNNNWW",
    "M": "WNWNNNNWN", "N": "NNNNWNNWW", "O": "WNNNWNNWN", "P": "NNWNWNNWN",
    "Q": "NNNNNNWWW", "R": "WNNNNNWWN", "S": "NNWNNNWWN", "T": "NNNNWNWWN",
    "U": "WWNNNNNNW", "V": "NWWNNNNNW", "W": "WWWNNNNNN", "X": "NWNNWNNNW",
    "Y": "WWNNWNNNN", "Z": "NWWNWNNNN", "*": "NWNNWNWNN",
}

ROOT = Path(__file__).resolve().parent
DECODER = ROOT.parent.parent / "app_program" / "line_barcode" / "bc_decode.c"


def mask_of(pattern: str) -> int:
    """Bit k is set when element k is wide (the encoding used in bc_decode.c)."""
    return sum(1 << k for k, e in enumerate(pattern) if e == "W")


def check() -> int:
    errors = 0

    for char, pat in CODE39.items():
        if len(pat) != 9 or pat.count("W") != 3:
            print(f"table error: {char} {pat}")
            errors += 1

    # 1. firmware decoder table:  { 0x052u, '*' },
    text = DECODER.read_text()
    found = {
        chr_: int(mask, 16)
        for mask, chr_ in re.findall(r"\{\s*0x([0-9A-Fa-f]+)u?,\s*'(.)'\s*\}", text)
    }
    for char, pat in CODE39.items():
        want = mask_of(pat)
        got = found.get(char)
        status = "ok" if got == want else "MISMATCH"
        if got != want:
            errors += 1
        print(f"decoder  {char}  want 0x{want:03X}  have "
              f"{'none' if got is None else f'0x{got:03X}'}  {status}")
    extra = set(found) - set(CODE39)
    if extra:
        print("decoder has characters not in the table:", sorted(extra))
        errors += 1

    # 2. simulation table: "WNNNNWNNW", ... in lb_sim_bc.c
    sim = (ROOT / "lb_sim_bc.c").read_text()
    sim_pats = re.findall(r'"([NW]{9})"', sim)
    table_pats = [CODE39[c] for c in "ABCDEFGHIJKLMNOPQRSTUVWXYZ"]
    for want in table_pats:
        if want not in sim_pats:
            print(f"simulation is missing pattern {want}")
            errors += 1
    print("simulation patterns checked")

    print("RESULT:", "all agree" if errors == 0 else f"{errors} problem(s)")
    return 0 if errors == 0 else 1


def sheet(out: str, narrow: float) -> int:
    wide = 3.0 * narrow
    height = 20.0
    margin = 10.0
    gap_between = 25.0
    parts = []
    y = margin
    max_w = 0.0

    for cmd in "ABCD":
        x = margin
        pats = [CODE39["*"], CODE39[cmd], CODE39["*"]]
        for p, pat in enumerate(pats):
            for k, e in enumerate(pat):
                w = wide if e == "W" else narrow
                if k % 2 == 0:  # bars only; spaces stay white
                    parts.append(
                        f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" '
                        f'height="{height:.2f}" fill="#000"/>')
                x += w
            if p < 2:
                x += narrow  # inter-character gap
        max_w = max(max_w, x - margin)
        parts.append(
            f'<text x="{margin:.2f}" y="{y + height + 5:.2f}" '
            f'font-family="sans-serif" font-size="4">'
            f'*{cmd}*  length {x - margin:.1f} mm  narrow {narrow} mm  '
            f'wide {wide} mm</text>')
        y += height + gap_between

    total_w = max_w + 2 * margin
    total_h = y
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{total_w:.1f}mm" '
        f'height="{total_h:.1f}mm" viewBox="0 0 {total_w:.1f} {total_h:.1f}">'
        f'<rect width="100%" height="100%" fill="#fff"/>' + "".join(parts)
        + "</svg>")
    Path(out).write_text(svg)
    print(f"wrote {out}: code length {max_w:.1f} mm "
          f"(47 narrow units x {narrow} mm = {47 * narrow:.1f} mm)")
    return 0


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "check"
    if cmd == "check":
        return check()
    if cmd == "sheet":
        out = argv[2] if len(argv) > 2 else "barcode_sheet.svg"
        narrow = float(argv[3]) if len(argv) > 3 else 3.0
        return sheet(out, narrow)
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
