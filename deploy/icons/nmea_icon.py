#!/usr/bin/env python3
"""NMEA sniffer icon: an NMEA2000 backbone with device drops — one node
amber and unidentified (the thing being sniffed) — same family as the raven."""
from PIL import Image, ImageDraw, ImageFilter
import sys

S = 1024
img = Image.new("RGBA", (S, S), (0, 0, 0, 0))

R = 232
grad = Image.new("RGBA", (S, S))
gd = ImageDraw.Draw(grad)
top, bot = (14, 20, 24), (24, 40, 46)          # teal-leaning deep sea
for y in range(S):
    t = y / S
    c = tuple(int(a + (b - a) * t) for a, b in zip(top, bot))
    gd.line([(0, y), (S, y)], fill=c + (255,))
mask = Image.new("L", (S, S), 0)
ImageDraw.Draw(mask).rounded_rectangle([0, 0, S-1, S-1], radius=R, fill=255)
img.paste(grad, (0, 0), mask)
d = ImageDraw.Draw(img)

def sc(p):
    x, y = p
    m = 74
    return (m + x * (S - 2*m) / 100.0, m + y * (S - 2*m) / 100.0)

# faint hex-dump rain in the background (the raw payloads)
hexes = ["7f 3a 90 c2", "b4 01 ff e0", "12 d6 55", "00 e8 71 33"]
for i, hx in enumerate(hexes):
    x, y = sc((5, 4 + i * 9.5))
    d.text((x, y), hx, fill=(120, 190, 210, 26), font_size=30)

# ---- the backbone: thick horizontal trunk with terminators -----------------
ty = 46
x0, x1 = sc((6, ty))[0], sc((94, ty))[0]
ymid = sc((0, ty))[1]
d.line([(x0, ymid), (x1, ymid)], fill=(90, 169, 230, 235), width=16)
for xr in (x0, x1):   # terminators
    d.rounded_rectangle([xr-16, ymid-30, xr+16, ymid+30], radius=8,
                        fill=(35, 48, 66, 255), outline=(90, 169, 230, 200), width=5)

# ---- device drops -----------------------------------------------------------
drops = [   # (x, below?, kind)  kind: known / gw / mystery
    (20, True,  "known"), (33, False, "known"), (47, True, "gw"),
    (63, False, "known"), (79, True,  "mystery"),
]
AMBER = (230, 169, 75)
for x, below, kind in drops:
    px, _ = sc((x, 0))
    dy = 22 if below else -22
    ny = ymid + (150 if below else -150)
    col = AMBER if kind == "mystery" else (90, 169, 230)
    d.line([(px, ymid + (8 if below else -8)), (px, ny)],
           fill=col + (200,), width=10)
    # tee on the trunk
    d.ellipse([px-14, ymid-14, px+14, ymid+14], fill=(35, 48, 66, 255),
              outline=col + (255,), width=5)
    if kind == "gw":
        # our gateway: small rounded box with a tail (USB) going down-off
        d.rounded_rectangle([px-52, ny-40, px+52, ny+40], radius=14,
                            fill=(30, 42, 58, 255), outline=(90,169,230,255), width=6)
        d.text((px-34, ny-22), "GW", fill=(200, 230, 250, 255), font_size=48)
        d.line([(px, ny+40), (px, ny+110)], fill=(140, 200, 235, 160), width=8)
    elif kind == "mystery":
        # the sniffed device: amber, glowing, with a ? — sits deeper
        glow = Image.new("RGBA", (S, S), (0,0,0,0))
        ImageDraw.Draw(glow).ellipse([px-95, ny-95, px+95, ny+95],
                                     fill=AMBER + (90,))
        glow = glow.filter(ImageFilter.GaussianBlur(30))
        img.alpha_composite(glow)
        d = ImageDraw.Draw(img)
        d.ellipse([px-62, ny-62, px+62, ny+62], fill=(46, 34, 17, 255),
                  outline=AMBER + (255,), width=8)
        d.text((px-19, ny-38), "?", fill=AMBER + (255,), font_size=76)
        # sonar pings fanning DOWN from it (it is a sonar we are hunting)
        for rr in (110, 160, 210):
            d.arc([px-rr, ny-rr, px+rr, ny+rr], start=35, end=145,
                  fill=AMBER + (120 - (rr - 110)//2,), width=7)
    else:
        d.ellipse([px-34, ny-34, px+34, ny+34], fill=(30, 42, 58, 255),
                  outline=(90, 169, 230, 255), width=6)

# ---- magnifier over the mystery node ---------------------------------------
mx, my = sc((76.5, 71))
r = 135
d.ellipse([mx-r, my-r, mx+r, my+r], outline=(220, 235, 250, 235), width=16)
d.ellipse([mx-r+16, my-r+16, mx+r-16, my+r-16], outline=(120, 190, 220, 60), width=6)
hx0, hy0 = mx + r*0.72, my + r*0.72
d.line([(hx0, hy0), (hx0+110, hy0+110)], fill=(220, 235, 250, 235), width=34)

img.save(sys.argv[1] if len(sys.argv) > 1 else "nmea.png")
print("wrote icon")
