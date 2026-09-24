#!/usr/bin/env python3
"""Generate the PWA icon set in docs/icons/.

Re-run after changing the artwork below:
    pip install pillow
    python make_pwa_icons.py

Outputs (referenced by docs/manifest.webmanifest or docs/index.html):
    icon-192.png  icon-512.png            normal icons, art fills the canvas
    icon-192-maskable.png  icon-512-maskable.png
                                          art inside the safe zone, for Android
                                          adaptive-icon cropping
    apple-touch-icon.png (180)            iOS home screen, must be opaque
    favicon-32.png                        browser tab

The picture is the thing the page is for: a fixed board, rings of range around
it, and a second board out at the edge of what still gets through.
"""
import os
from PIL import Image, ImageDraw

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docs", "icons")

# Straight from the page's :root custom properties.
BG   = (18, 20, 26)     # --bg   #12141a
LINE = (44, 49, 61)     # --line #2c313d
ACC  = (96, 165, 250)   # --acc  #60a5fa
OK   = (74, 222, 128)   # --ok   #4ade80
DIM  = (139, 147, 167)  # --dim  #8b93a7

SS = 8  # supersample factor; drawn at SS*size then box-filtered down


def draw_icon(size, scale=1.0, rounded=True):
    """Render one icon. `scale` shrinks the artwork for the maskable safe zone."""
    S = size * SS
    img = Image.new("RGB", (S, S), BG)
    d = ImageDraw.Draw(img)

    if rounded:
        r = int(S * 0.22)
        d.rounded_rectangle([0, 0, S - 1, S - 1], radius=r, fill=BG,
                            outline=LINE, width=max(1, int(S * 0.012)))

    cx, cy = S / 2, S / 2
    u = S * scale / 100.0          # 1 unit = 1% of the (scaled) canvas

    bx, by = cx - 16 * u, cy + 14 * u    # base board, where you left it
    fx, fy = cx + 25 * u, cy - 22 * u    # the board you walked away with

    # --- range rings, fading outward: what still gets through, and how far ---
    for rad, col in ((26, (58, 99, 150)), (41, (44, 71, 106)), (56, (33, 50, 72))):
        d.ellipse([bx - rad * u, by - rad * u, bx + rad * u, by + rad * u],
                  outline=col, width=max(1, int(u * 3.0)))

    # --- the link itself ---
    d.line([(bx, by), (fx, fy)], fill=DIM, width=max(1, int(u * 2.2)))

    # --- the two boards ---
    r = 5.0 * u
    d.ellipse([fx - r, fy - r, fx + r, fy + r], fill=OK)       # roaming end
    r = 7.5 * u
    d.ellipse([bx - r, by - r, bx + r, by + r], fill=ACC)      # base

    return img.resize((size, size), Image.LANCZOS)


def save(img, name):
    path = os.path.join(OUT, name)
    img.save(path, "PNG", optimize=True)
    print("%-28s %5d bytes" % (name, os.path.getsize(path)))


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    save(draw_icon(192, scale=0.9), "icon-192.png")
    save(draw_icon(512, scale=0.9), "icon-512.png")
    # Maskable icons get cropped to a circle/squircle by the launcher: keep the
    # art well inside, and let the plate bleed to the edges.
    save(draw_icon(192, scale=0.72, rounded=False), "icon-192-maskable.png")
    save(draw_icon(512, scale=0.72, rounded=False), "icon-512-maskable.png")
    save(draw_icon(180, scale=0.9), "apple-touch-icon.png")
    save(draw_icon(32, scale=0.94), "favicon-32.png")
