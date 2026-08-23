#!/usr/bin/env python3
"""Wild Sync launcher icon: a low-poly (triangulated) raven — the
photogrammetry-mesh look — on a deep squircle, macOS Big Sur style."""
from PIL import Image, ImageDraw, ImageFilter
import math, sys

S = 1024
img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
d = ImageDraw.Draw(img)

# ---------- squircle background with vertical gradient ----------------------
R = 232                       # macOS-style corner radius at 1024
grad = Image.new("RGBA", (S, S))
gd = ImageDraw.Draw(grad)
top, bot = (16, 19, 28), (28, 36, 56)
for y in range(S):
    t = y / S
    c = tuple(int(a + (b - a) * t) for a, b in zip(top, bot))
    gd.line([(0, y), (S, y)], fill=c + (255,))
mask = Image.new("L", (S, S), 0)
ImageDraw.Draw(mask).rounded_rectangle([0, 0, S - 1, S - 1], radius=R, fill=255)
img.paste(grad, (0, 0), mask)
d = ImageDraw.Draw(img)

def sc(p):     # 0..100 design space -> pixels (with margin)
    x, y = p
    m = 74
    return (m + x * (S - 2 * m) / 100.0, m + y * (S - 2 * m) / 100.0)

# ---------- faint background triangulation ---------------------------------
bgpts = [(4,16),(20,4),(44,7),(6,44),(96,20),(80,6),(94,50),(97,84),(78,96),(52,95),(8,90),(24,97)]
bgedges = [(0,1),(1,2),(0,3),(1,3),(4,5),(4,6),(5,2),(6,7),(7,8),(8,9),(10,11),(9,11),(10,3)]
ov = Image.new("RGBA", (S, S), (0,0,0,0))
od = ImageDraw.Draw(ov)
for a, b in bgedges:
    od.line([sc(bgpts[a]), sc(bgpts[b])], fill=(120, 160, 220, 26), width=2)
for p in bgpts:
    x, y = sc(p)
    od.ellipse([x-4, y-4, x+4, y+4], fill=(140, 180, 235, 40))
ov.putalpha(ov.split()[3].point(lambda a: a))
img.alpha_composite(Image.composite(ov, Image.new("RGBA",(S,S),(0,0,0,0)), mask))
d = ImageDraw.Draw(img)

# ---------- raven geometry (side profile, facing right) --------------------
B = [(87,47),(66,36.5),(57,27),(45,22.5),(33,26),(25.5,35),(23,47),(25,59),
     (30,70),(38,78),(46,74.5),(52,79),(57,71),(63,73),(66,63),(64,54.5)]
sil = [sc(p) for p in B]

# soft drop shadow under the bird
sh = Image.new("RGBA", (S, S), (0,0,0,0))
ImageDraw.Draw(sh).polygon([(x+10, y+16) for x, y in sil], fill=(0, 0, 0, 110))
sh = sh.filter(ImageFilter.GaussianBlur(18))
img.alpha_composite(sh)
d = ImageDraw.Draw(img)

d.polygon(sil, fill=(15, 18, 26, 255))     # solid silhouette under the facets

# interior vertices
P1,P2,P3   = (45,32),(56,34),(37,34)
P4,P5,P6   = (29,44),(40,44),(52,44)
P7,P15     = (63,45.5),(74,44.5)
P8,P9,P10  = (33,55),(45,55),(57,57)
P11,P12,P13= (36,65),(49,65),(60,65)
P14        = (42,72)
b = B
TRIS = [
 # beak
 (b[0], b[1], P15), (b[1], P7, P15), (P15, P7, b[15]), (b[0], P15, b[15]),
 # crown / forehead
 (b[1], b[2], P2), (b[2], b[3], P1), (b[2], P1, P2), (b[3], b[4], P3),
 (b[3], P3, P1), (b[4], b[5], P3),
 # mid head
 (P1, P3, P5), (P1, P5, P6), (P1, P6, P2), (P2, P6, P7), (b[1], P2, P7),
 (b[5], P4, P3), (b[5], b[6], P4), (P3, P4, P5),
 # neck / body
 (b[6], b[7], P8), (b[6], P8, P4), (P4, P8, P5), (P5, P8, P9), (P5, P9, P6),
 (P6, P9, P10), (P6, P10, P7), (P7, P10, b[14]), (P7, b[14], b[15]),
 (b[7], b[8], P11), (b[7], P11, P8), (P8, P11, P9), (P9, P11, P12),
 (P9, P12, P10), (P10, P12, P13), (P10, P13, b[14]),
 # shaggy hackles / chest
 (b[8], b[9], P14), (b[8], P14, P11), (P11, P14, P12),
 (b[9], b[10], P14), (b[10], b[11], P12), (b[10], P12, P14),
 (b[11], b[12], P12), (b[12], b[13], P13), (b[12], P13, P12),
 (b[13], b[14], P13),
]
# iridescent facet palette (raven sheen: slate / indigo / teal / plum)
PAL = [(38,46,66),(46,40,72),(30,56,64),(52,36,60),(34,40,58),(26,32,46)]
def shade(tri, i):
    cx = sum(p[0] for p in tri) / 3.0
    cy = sum(p[1] for p in tri) / 3.0
    base = PAL[(i * 7) % len(PAL)]
    # light from upper-left: brighter when up-left, darker low-right
    lum = 1.25 - 0.006 * (cx + cy * 0.9)
    lum = max(0.55, min(1.5, lum))
    return tuple(min(255, int(c * lum)) for c in base) + (235,)
for i, t in enumerate(TRIS):
    d.polygon([sc(p) for p in t], fill=shade(t, i))
# facet edges — the "mesh"
seen = set()
for t in TRIS:
    for e in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        k = tuple(sorted(e))
        if k in seen:
            continue
        seen.add(k)
        d.line([sc(e[0]), sc(e[1])], fill=(150, 190, 240, 46), width=3)
# silhouette outline, crisp
d.line(sil + [sil[0]], fill=(8, 10, 15, 255), width=7, joint="curve")
# beak gape
d.line([sc((64.5, 50.5)), sc((81, 47.6))], fill=(8, 10, 15, 230), width=6)
# vertex dots on a few mesh points
for p in [b[2], b[5], b[9], b[13], P5, P10, P15, b[0]]:
    x, y = sc(p)
    d.ellipse([x-7, y-7, x+7, y+7], fill=(120, 200, 235, 150))
    d.ellipse([x-3, y-3, x+3, y+3], fill=(220, 245, 255, 220))

# ---------- eye -------------------------------------------------------------
ex, ey = sc((55.5, 37.5))
r = 26
d.ellipse([ex-r-6, ey-r-6, ex+r+6, ey+r+6], fill=(8, 10, 15, 255))
d.ellipse([ex-r, ey-r, ex+r, ey+r], fill=(230, 169, 75, 255))
d.ellipse([ex-r*0.45, ey-r*0.45, ex+r*0.45, ey+r*0.45], fill=(20, 16, 8, 255))
d.ellipse([ex+r*0.15, ey-r*0.55, ex+r*0.5, ey-r*0.2], fill=(255, 244, 214, 235))


img.save(sys.argv[1] if len(sys.argv) > 1 else "raven.png")
print("wrote icon")
