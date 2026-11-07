#!/usr/bin/env python3
"""Printable A4 track tiles for line-following tests (no extra packages).

Writes track_tiles.pdf: one page each for a straight, a 90-degree curve and
a cross junction.  Each tile is a 200 mm square centred on the A4 page; cut
along the corner marks and the tiles butt together edge to edge, because the
18 mm line always meets the tile edge at its centre.  Print at 100 %
("Actual size"); the 100 mm scale bar on every page checks that.
"""

from pathlib import Path

MM = 72.0 / 25.4                     # PDF points per millimetre
PAGE_W, PAGE_H = 210.0, 297.0        # A4, mm
TILE = 200.0                         # tile side, mm
LINE = 18.0                          # line width, mm
X0 = (PAGE_W - TILE) / 2.0           # tile origin on the page (bottom left)
Y0 = (PAGE_H - TILE) / 2.0
KAPPA = 0.5522847498                 # cubic Bezier for a quarter circle


def p(x, y):
    """Tile coordinates (mm, origin bottom left) to page points."""
    return f"{(X0 + x) * MM:.3f} {(Y0 + y) * MM:.3f}"


def rect(x, y, w, h):
    return f"{(X0 + x) * MM:.3f} {(Y0 + y) * MM:.3f} {w * MM:.3f} {h * MM:.3f} re f\n"


def straight():
    return rect(TILE / 2 - LINE / 2, 0, LINE, TILE)


def cross():
    return straight() + rect(0, TILE / 2 - LINE / 2, TILE, LINE)


def curve():
    """Quarter circle from the bottom edge centre to the right edge centre,
    centred on the bottom-right corner: radius 100 mm to the line centre."""
    cx, cy, r = TILE, 0.0, TILE / 2
    k = KAPPA * r
    path = (f"{LINE * MM:.3f} w 0 J\n"
            f"{p(cx - r, cy)} m\n"
            f"{p(cx - r, cy + k)} {p(cx - k, cy + r)} {p(cx, cy + r)} c S\n")
    return path


def frame():
    """Light tile outline and corner cut marks outside the tile."""
    s = "0.6 G 0.3 w [3 3] 0 d\n"
    s += f"{X0 * MM:.3f} {Y0 * MM:.3f} {TILE * MM:.3f} {TILE * MM:.3f} re S\n[] 0 d 0 G 0.5 w\n"
    for x, dx in ((0, -1), (TILE, 1)):
        for y, dy in ((0, -1), (TILE, 1)):
            s += f"{p(x, y + dy * 2)} m {p(x, y + dy * 10)} l S\n"
            s += f"{p(x + dx * 2, y)} m {p(x + dx * 10, y)} l S\n"
    return s


def text(x, y, size, msg):
    msg = msg.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    return f"BT /F1 {size} Tf {x * MM:.3f} {y * MM:.3f} Td ({msg}) Tj ET\n"


def labels(name):
    top = Y0 + TILE + 8
    s = "0 g\n" + text(X0, top + 10, 14, f"Line-follow track: {name}")
    s += text(X0, top + 3, 9, "Line 18 mm wide. Tile 200 x 200 mm. Print at 100 % (Actual size), "
              "then cut along the corner marks.")
    # 100 mm scale bar below the tile
    yb = Y0 - 14
    s += f"0 g {X0 * MM:.3f} {yb * MM:.3f} {100 * MM:.3f} {1.2 * MM:.3f} re f\n"
    for i in range(0, 101, 10):
        s += f"{(X0 + i) * MM:.3f} {(yb - 2) * MM:.3f} {0.3 * MM:.3f} {2 * MM:.3f} re f\n"
    s += text(X0 + 104, yb - 1, 8, "100 mm: measure this to check the print scale")
    return s


def page(name, track):
    return labels(name) + frame() + "0 g 0 G\n" + track


def build(pages, out):
    objs = []

    def add(body):
        objs.append(body)
        return len(objs)

    font = add("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")
    pages_id = len(objs) + 1 + 2 * len(pages)   # filled in after the page objects
    kids = []
    for content in pages:
        data = content.encode("latin-1")
        stream = add(f"<< /Length {len(data)} >>\nstream\n".encode("latin-1") + data + b"\nendstream")
        kids.append(add(f"<< /Type /Page /Parent {pages_id} 0 R "
                        f"/MediaBox [0 0 {PAGE_W * MM:.3f} {PAGE_H * MM:.3f}] "
                        f"/Resources << /Font << /F1 {font} 0 R >> >> /Contents {stream} 0 R >>"))
    add("<< /Type /Pages /Kids [" + " ".join(f"{k} 0 R" for k in kids) + f"] /Count {len(kids)} >>")
    catalog = add(f"<< /Type /Catalog /Pages {pages_id} 0 R >>")

    out_bytes = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out_bytes))
        body = body if isinstance(body, bytes) else body.encode("latin-1")
        out_bytes += f"{i} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(out_bytes)
    out_bytes += f"xref\n0 {len(objs) + 1}\n0000000000 65535 f \n".encode()
    for off in offsets:
        out_bytes += f"{off:010d} 00000 n \n".encode()
    out_bytes += (f"trailer\n<< /Size {len(objs) + 1} /Root {catalog} 0 R >>\n"
                  f"startxref\n{xref}\n%%EOF\n").encode()
    Path(out).write_bytes(out_bytes)


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    build([page("STRAIGHT", straight()),
           page("90-DEGREE CURVE (radius 100 mm)", curve()),
           page("CROSS JUNCTION", cross())],
          here / "track_tiles.pdf")
    print("wrote", here / "track_tiles.pdf")
