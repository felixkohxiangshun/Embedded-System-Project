#!/usr/bin/env python3
"""Printable A4 track tiles for line-following tests (no extra packages).

Writes track_tiles.pdf: one page each for a straight, a 90-degree curve and
a cross junction.  Each tile is the whole A4 page (210 x 297 mm) with no text
or marks, only the 18 mm line.  The line always meets the page edges at
their centres, so pages butt together edge to edge.  Print at 100 %
("Actual size", borderless if the printer can).
"""

from pathlib import Path

MM = 72.0 / 25.4                     # PDF points per millimetre
PAGE_W, PAGE_H = 210.0, 297.0        # A4, mm
LINE = 18.0                          # line width, mm
X0, Y0 = 0.0, 0.0                    # the tile is the whole page
KAPPA = 0.5522847498                 # cubic Bezier for a quarter circle


def p(x, y):
    """Tile coordinates (mm, origin bottom left) to page points."""
    return f"{(X0 + x) * MM:.3f} {(Y0 + y) * MM:.3f}"


def rect(x, y, w, h):
    return f"{(X0 + x) * MM:.3f} {(Y0 + y) * MM:.3f} {w * MM:.3f} {h * MM:.3f} re f\n"


def straight():
    return rect(PAGE_W / 2 - LINE / 2, 0, LINE, PAGE_H)


def cross():
    return straight() + rect(0, PAGE_H / 2 - LINE / 2, PAGE_W, LINE)


def curve():
    """From the bottom edge centre to the right edge centre.  The page is
    not square, so the line runs straight up first and then turns through
    a quarter circle of radius 105 mm (to the line centre) into the right
    edge.  Butt line caps end it flush with both page edges."""
    r = PAGE_W / 2                    # 105 mm
    cx, cy = PAGE_W, PAGE_H / 2 - r   # centre of the arc: (210, 43.5)
    k = KAPPA * r
    path = (f"{LINE * MM:.3f} w 0 J\n"
            f"{p(cx - r, 0)} m\n"
            f"{p(cx - r, cy)} l\n"
            f"{p(cx - r, cy + k)} {p(cx - k, cy + r)} {p(cx, cy + r)} c S\n")
    return path


def page(track):
    return "0 g 0 G\n" + track


def build(pages, out):
    objs = []

    def add(body):
        objs.append(body)
        return len(objs)

    pages_id = len(objs) + 1 + 2 * len(pages)   # filled in after the page objects
    kids = []
    for content in pages:
        data = content.encode("latin-1")
        stream = add(f"<< /Length {len(data)} >>\nstream\n".encode("latin-1") + data + b"\nendstream")
        kids.append(add(f"<< /Type /Page /Parent {pages_id} 0 R "
                        f"/MediaBox [0 0 {PAGE_W * MM:.3f} {PAGE_H * MM:.3f}] "
                        f"/Resources << >> /Contents {stream} 0 R >>"))
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
    build([page(straight()), page(curve()), page(cross())],
          here / "track_tiles.pdf")
    print("wrote", here / "track_tiles.pdf")
