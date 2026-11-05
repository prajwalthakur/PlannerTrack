#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""Renders every page of architecture.drawio to architecture_<page id>.png for the top-level README.md.

The .drawio file is the source of the figures: edit it with the draw.io VS Code extension, then either
export each page from draw.io (File > Export > PNG, same file name) or rerun this script:

    python3 docs/architecture/render_figures.py                     # every page
    python3 docs/architecture/render_figures.py raceline overview   # only these page ids

Like a draw.io PNG export with "include a copy of my diagram", every PNG carries the whole .drawio file
(its own page first) in an `mxfile` text chunk, so draw.io opens the PNG itself as an editable diagram.

This is a small renderer for the subset of draw.io the file uses: rectangles (rounded, dashed),
text cells, and straight polyline edges with fixed exit/entry points and arrow heads. Labels are
HTML with <b>, <font color/size> and <br>; there is no automatic word wrap.
"""

import base64
import re
import sys
import urllib.parse
import xml.etree.ElementTree as ET
import zlib
from copy import deepcopy
from html import unescape
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent
PX_TO_PT = 0.75  # draw.io works in CSS px (96 per inch); matplotlib fonts and lines in pt (72 per inch)
LINE_HEIGHT = 1.25


def parse_style(style):
    out = {}
    for item in (style or "").split(";"):
        if not item:
            continue
        key, _, value = item.partition("=")
        out[key] = value if _ else "1"
    return out


def graph_model(diagram):
    model = diagram.find("mxGraphModel")
    if model is not None:
        return model
    raw = base64.b64decode(diagram.text.strip())  # compressed page: base64(deflate(urlencode(xml)))
    xml = urllib.parse.unquote(zlib.decompress(raw, -15).decode())
    return ET.fromstring(xml)


def label_lines(value, size, color):
    """HTML label -> [(text, size, color, bold)], one entry per line."""
    value = re.sub(r"<br\s*/?>", "\n", value or "", flags=re.I)
    value = re.sub(r"</?(div|span|p)[^>]*>", "", value, flags=re.I)
    lines, current, stack = [], None, [(size, color, False)]
    for token in re.split(r"(<[^>]+>|\n)", value):
        if not token:
            continue
        if token == "\n":
            lines.append(current or ("", *stack[-1]))
            current = None
        elif token.startswith("</"):
            if len(stack) > 1:
                stack.pop()
        elif token.startswith("<"):
            s, c, b = stack[-1]
            tag = token[1:].split()[0].lower().rstrip(">")
            if tag == "b":
                b = True
            m = re.search(r"font-size:\s*([\d.]+)px", token)
            s = float(m.group(1)) if m else s
            m = re.search(r'color="(#[0-9a-fA-F]{6})"', token)
            c = m.group(1) if m else c
            stack.append((s, c, b))
        else:
            text = unescape(token)
            if current is None:
                current = (text, *stack[-1])
            else:
                current = (current[0] + text, *current[1:])
    if current is not None:
        lines.append(current)
    return lines


def draw_label(ax, cell, geo, style):
    lines = label_lines(cell.get("value"), float(style.get("fontSize", 12)), style.get("fontColor", "#000000"))
    if not lines:
        return
    x, y, w, h = geo
    align, valign = style.get("align", "center"), style.get("verticalAlign", "middle")
    spacing = float(style.get("spacing", 2))
    left = spacing + float(style.get("spacingLeft", 0))
    top = spacing + float(style.get("spacingTop", 0))
    bottom = spacing + float(style.get("spacingBottom", 0))
    heights = [s * LINE_HEIGHT for _, s, _, _ in lines]
    total = sum(heights)
    cy = {"top": y + top, "bottom": y + h - bottom - total}.get(valign, y + (h - total) / 2)
    cx = {"left": x + left, "right": x + w - left}.get(align, x + w / 2)
    bg = style.get("labelBackgroundColor")
    for (text, size, color, bold), lh in zip(lines, heights):
        ax.text(cx, cy + lh / 2, text, ha=align if align in ("left", "right") else "center", va="center",
                fontsize=size * PX_TO_PT, color=color, fontweight="bold" if bold else "normal", zorder=6,
                bbox=dict(fc=bg, ec="none", pad=1.5) if bg and bg != "none" else None)
        cy += lh


def draw_vertex(ax, cell, geo, style):
    x, y, w, h = geo
    if "text" not in style:
        fill = style.get("fillColor", "#ffffff")
        stroke = style.get("strokeColor", "#000000")
        lw = float(style.get("strokeWidth", 1)) * PX_TO_PT
        ls = (0, (5, 3)) if style.get("dashed") == "1" else "solid"
        if style.get("rounded") == "1":
            arc = float(style.get("arcSize", 20))
            r = arc / 2 if style.get("absoluteArcSize") == "1" else min(w, h) * arc / 100
            patch = FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}")
        else:
            patch = Rectangle((x, y), w, h)
        patch.set(fc="none" if fill == "none" else fill, ec="none" if stroke == "none" else stroke, lw=lw, ls=ls,
                  zorder=2)
        ax.add_patch(patch)
    draw_label(ax, cell, geo, style)


def anchor(geo, fx, fy):
    x, y, w, h = geo
    return x + float(fx) * w, y + float(fy) * h


def draw_edge(ax, cell, geos, style):
    g = cell.find("mxGeometry")
    pts = []
    src, dst = cell.get("source"), cell.get("target")
    if src in geos and "exitX" in style:
        pts.append(anchor(geos[src], style["exitX"], style["exitY"]))
    else:
        sp = g.find("mxPoint[@as='sourcePoint']")
        pts.append((float(sp.get("x")), float(sp.get("y"))) if sp is not None else None)
    arr = g.find("Array[@as='points']")
    if arr is not None:
        pts += [(float(p.get("x", 0)), float(p.get("y", 0))) for p in arr.findall("mxPoint")]
    if dst in geos and "entryX" in style:
        pts.append(anchor(geos[dst], style["entryX"], style["entryY"]))
    else:
        tp = g.find("mxPoint[@as='targetPoint']")
        pts.append((float(tp.get("x")), float(tp.get("y"))) if tp is not None else None)
    if None in pts:
        print(f"  skipped edge {cell.get('id')}: needs fixed exit/entry points", file=sys.stderr)
        return
    color = style.get("strokeColor", "#000000")
    lw = float(style.get("strokeWidth", 1)) * PX_TO_PT
    ls = (0, (5, 3)) if style.get("dashed") == "1" else "solid"
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, ls=ls, zorder=3, solid_capstyle="projecting")
    head = "-|>" if style.get("endArrow", "classic") != "none" else "-"
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle=head, mutation_scale=14, color=color, lw=lw, ls=ls,
                                 shrinkA=0, shrinkB=0, zorder=3))


def render(diagram, out, mxfile):
    model = graph_model(diagram)
    cells = model.find("root").findall("mxCell")
    geos = {}
    for c in cells:
        g = c.find("mxGeometry")
        if c.get("vertex") == "1" and g is not None:
            geos[c.get("id")] = tuple(float(g.get(k, 0)) for k in ("x", "y", "width", "height"))
    width = max(x + w for x, y, w, h in geos.values()) + 30
    height = max(y + h for x, y, w, h in geos.values()) + 30
    fig = plt.figure(figsize=(width / 96, height / 96))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.axis("off")
    for c in cells:
        style = parse_style(c.get("style"))
        if c.get("vertex") == "1" and c.get("id") in geos:
            draw_vertex(ax, c, geos[c.get("id")], style)
        elif c.get("edge") == "1":
            draw_edge(ax, c, geos, style)
    fig.savefig(out, dpi=192, facecolor=model.get("background", "#ffffff"),
                metadata={"Software": None, "mxfile": mxfile})
    plt.close(fig)
    print("wrote", out.relative_to(HERE.parent))


def embedded_file(root, page):
    """The whole .drawio file with ``page`` first, URL-encoded as draw.io writes it into a PNG."""
    copy = deepcopy(root)
    for diagram in copy.findall("diagram"):
        if diagram.get("id") == page:
            copy.remove(diagram)
            copy.insert(0, diagram)
    xml = ET.tostring(copy, encoding="unicode")
    return urllib.parse.quote(xml, safe="~()*!.'")  # same as JavaScript encodeURIComponent


def main():
    root = ET.parse(HERE / "architecture.drawio").getroot()
    pages = {d.get("id"): d for d in root.findall("diagram")}
    wanted = sys.argv[1:] or list(pages)
    unknown = [p for p in wanted if p not in pages]
    if unknown:
        sys.exit(f"unknown page id(s) {unknown}; pages: {list(pages)}")
    for page in wanted:
        render(pages[page], HERE / f"architecture_{page}.png", embedded_file(root, page))


if __name__ == "__main__":
    main()
