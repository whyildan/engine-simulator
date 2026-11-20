"""Draws the application icon (a rev gauge in the app's colours) and writes assets/enginesim.ico (16-256 px, each
size drawn on its own so the small ones stay crisp) plus assets/enginesim_256.png.
usage: python tools/make_icon.py [--preview]   (--preview: also a sheet of every size, icon_sizes_preview.png)"""
import math
import os
import sys
from PIL import Image, ImageDraw

BG_TOP, BG_BOT = (30, 32, 37), (17, 17, 19)     # app window background #111113, a little lighter at the top
BORDER = (52, 55, 62)
TRACK = (58, 61, 68)
ORANGE = (242, 128, 51)                         # slider grab (0.95, 0.5, 0.2)
ORANGE_HI = (255, 153, 64)                      # check mark (1.0, 0.6, 0.25)
RED = (229, 72, 77)
TICK = (150, 154, 162)

START, SWEEP = 135.0, 270.0                     # gauge opens at the bottom (PIL angles: clockwise from 3 o'clock)
VALUE = 0.72                                    # needle / orange sweep position
REDLINE = 0.86                                  # red zone from here to the end


def lerp(a, b, t):
    return tuple(int(round(x + (y - x) * t)) for x, y in zip(a, b))


def draw(size):
    k = 4                                       # supersampling
    n = size * k
    im = Image.new("RGBA", (n, n), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    small = size <= 32
    # rounded square with a vertical gradient
    mask = Image.new("L", (n, n), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, n - 1, n - 1], radius=int(0.22 * n), fill=255)
    grad = Image.new("RGBA", (n, n))
    gd = ImageDraw.Draw(grad)
    for y in range(n):
        gd.line([(0, y), (n, y)], fill=lerp(BG_TOP, BG_BOT, y / (n - 1)) + (255,))
    im.paste(grad, (0, 0), mask)
    if size >= 32:
        d.rounded_rectangle([k // 2, k // 2, n - 1 - k // 2, n - 1 - k // 2], radius=int(0.22 * n), outline=BORDER, width=max(1, k))
    # gauge
    cx, cy = 0.5 * n, (0.56 if small else 0.54) * n
    r = (0.40 if small else 0.34) * n
    w = int((0.15 if small else 0.085) * n)
    box = [cx - r, cy - r, cx + r, cy + r]
    d.arc(box, START, START + SWEEP, fill=TRACK, width=w)
    d.arc(box, START + REDLINE * SWEEP, START + SWEEP, fill=RED, width=w)
    d.arc(box, START, START + VALUE * SWEEP, fill=ORANGE, width=w)
    # ticks (larger sizes)
    if size >= 48:
        for i in range(10):
            a = math.radians(START + SWEEP * i / 9)
            r0, r1 = r - w * 0.9, r - w * (1.9 if i % 3 == 0 else 1.45)
            d.line([(cx + r0 * math.cos(a), cy + r0 * math.sin(a)), (cx + r1 * math.cos(a), cy + r1 * math.sin(a))],
                   fill=TICK, width=max(k, int(0.018 * n)))
    # needle: a tapered blade from the hub to near the arc
    a = math.radians(START + VALUE * SWEEP)
    ln = r - (0.55 if small else 0.9) * w
    tip = (cx + ln * math.cos(a), cy + ln * math.sin(a))
    hw = (0.075 if small else 0.045) * n
    px, py = -math.sin(a), math.cos(a)
    tail = (cx - 0.12 * n * math.cos(a), cy - 0.12 * n * math.sin(a))
    d.polygon([(tail[0] + px * hw * 0.7, tail[1] + py * hw * 0.7), tip, (tail[0] - px * hw * 0.7, tail[1] - py * hw * 0.7)],
              fill=ORANGE_HI)
    # hub
    hr = (0.115 if small else 0.075) * n
    d.ellipse([cx - hr, cy - hr, cx + hr, cy + hr], fill=BG_BOT, outline=ORANGE_HI, width=max(k, int(0.03 * n)))
    return im.resize((size, size), Image.LANCZOS)


def main():
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
    os.makedirs(root, exist_ok=True)
    sizes = [256, 128, 64, 48, 32, 24, 16]
    imgs = [draw(s) for s in sizes]
    imgs[0].save(os.path.join(root, "enginesim.ico"), format="ICO", sizes=[(s, s) for s in sizes], append_images=imgs[1:])
    imgs[0].save(os.path.join(root, "enginesim_256.png"))
    if "--preview" not in sys.argv:
        print("wrote", os.path.normpath(root))
        return
    # preview sheet of every size on the app background (for checking by eye)
    sheet = Image.new("RGBA", (sum(sizes) + 20 * (len(sizes) + 1), 296), (17, 17, 19, 255))
    x = 20
    for s, im in zip(sizes, imgs):
        sheet.paste(im, (x, 20), im)
        x += s + 20
    sheet.save(os.path.join(root, "icon_sizes_preview.png"))
    print("wrote", os.path.normpath(root))


if __name__ == "__main__":
    main()
