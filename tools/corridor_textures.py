"""Sign / poster textures for the college-floor world (own simple designs, no logos)."""
import os

from PIL import Image, ImageDraw, ImageFont

BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
REG = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def f(px, bold=True):
    return ImageFont.truetype(BOLD if bold else REG, px)


def centered(d, y, text, font, fill, w):
    tw = d.textlength(text, font=font)
    d.text(((w - tw) / 2, y), text, fill=fill, font=font)


def bars(d, x0, y0, x1, n, gap, col, h=10):
    for i in range(n):
        y = y0 + i * gap
        d.rectangle((x0, y, x1 - (i % 3) * 30, y + h), fill=col)


def make_all(out):
    os.makedirs(out, exist_ok=True)

    def save(img, name):
        img.save(os.path.join(out, name))

    # department banner
    im = Image.new('RGB', (2000, 300), (246, 246, 246)); d = ImageDraw.Draw(im)
    centered(d, 30, 'DEPARTMENT OF', f(105), (25, 30, 45), 2000)
    centered(d, 160, 'MECHATRONICS ENGINEERING', f(96), (25, 30, 45), 2000)
    save(im, 'corridor_banner.png')

    def sign(name, lines, size=(300, 300), bg=(205, 205, 210), fg=(40, 40, 40), px=52):
        im = Image.new('RGB', size, bg); d = ImageDraw.Draw(im)
        y = (size[1] - len(lines) * px * 1.25) / 2
        for ln in lines:
            centered(d, y, ln, f(px), fg, size[0]); y += px * 1.25
        save(im, name)

    sign('corridor_faculty_sign.png', ['FACULTY', 'ROOM'])
    sign('corridor_room_217.png', ['217'], (200, 300), (250, 250, 250), (20, 20, 20), 110)
    sign('corridor_nb208.png', ['NB-208', 'ROBOTICS &', 'MECHATRONICS', 'RESEARCH LAB'], (400, 400), (210, 210, 215), (30, 30, 30), 46)
    sign('corridor_clock.png', ['15:29'], (640, 260), (25, 18, 18), (255, 30, 30), 190)
    sign('corridor_lab_strip.png', ['MECHATRONICS'], (900, 140), (60, 60, 62), (235, 235, 235), 92)
    sign('corridor_exit.png', ['FIRE EXIT'], (500, 180), (20, 150, 60), (255, 255, 255), 80)
    sign('corridor_cctv.png', ['YOU ARE UNDER', 'CAMERA SURVEILLANCE'], (600, 180), (245, 200, 40), (20, 20, 20), 46)
    sign('corridor_water.png', ['WATER', 'COOLER'], (300, 300), (205, 205, 210), (40, 40, 40), 60)
    sign('corridor_lift.png', ['LIFT'], (400, 200), (60, 60, 64), (255, 255, 255), 110)
    sign('corridor_tinkerers.png', ["TINKERERS'", 'LAB'], (500, 600), (95, 98, 105), (240, 240, 240), 72)
    sign('corridor_room_tag.png', ['LAB'], (200, 300), (250, 250, 250), (20, 20, 20), 90)

    # cork notice board
    im = Image.new('RGB', (500, 380), (176, 138, 92)); d = ImageDraw.Draw(im)
    for i, (x, y, c) in enumerate([(40, 40, (250, 250, 250)), (220, 60, (255, 240, 150)), (340, 200, (250, 250, 250)),
                                   (90, 210, (200, 230, 255))]):
        d.rectangle((x, y, x + 120, y + 140), fill=c); bars(d, x + 12, y + 20, x + 108, 6, 18, (120, 120, 120), 6)
    save(im, 'corridor_cork.png')

    # mechatronics circles poster
    im = Image.new('RGB', (700, 1000), (238, 242, 248)); d = ImageDraw.Draw(im)
    for (cx, cy, c, t) in [(170, 250, (70, 150, 220), 'Electronics'), (530, 250, (240, 150, 90), 'Mechanical'),
                           (170, 760, (60, 170, 190), 'Computers'), (530, 760, (240, 120, 80), 'Control')]:
        d.ellipse((cx - 150, cy - 150, cx + 150, cy + 150), fill=c)
        centered(d, cy - 25, t, f(44), (255, 255, 255), cx * 2)
    d.ellipse((190, 340, 510, 660), fill=(255, 255, 255), outline=(80, 110, 160), width=12)
    centered(d, 450, 'Mechatronics', f(44), (40, 60, 110), 700)
    centered(d, 505, 'Engineering', f(44), (40, 60, 110), 700)
    save(im, 'corridor_mech_poster.png')

    # recruiters board (coloured tiles, no company logos)
    im = Image.new('RGB', (700, 1000), (250, 250, 250)); d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 699, 999), outline=(30, 30, 30), width=14)
    centered(d, 30, 'OUR RECRUITERS', f(56), (30, 30, 30), 700)
    cols = [(30, 80, 160), (200, 40, 40), (40, 140, 70), (230, 150, 30), (90, 90, 90), (20, 120, 180), (150, 60, 140)]
    k = 0
    for r in range(9):
        for c in range(4):
            x, y = 50 + c * 155, 120 + r * 95
            d.rectangle((x, y, x + 120, y + 55), fill=cols[k % len(cols)]); k += 3
    save(im, 'corridor_recruiters.png')

    # navy posters
    def navy(name, title1, title2=None):
        im = Image.new('RGB', (500, 800), (28, 40, 82)); d = ImageDraw.Draw(im)
        d.ellipse((170, 40, 330, 200), fill=(255, 255, 255))
        d.rectangle((225, 95, 275, 145), fill=(200, 40, 40))
        centered(d, 240, title1, f(60), (255, 255, 255), 500)
        bars(d, 50, 330, 450, 5, 26, (150, 160, 190), 8)
        if title2:
            centered(d, 490, title2, f(60), (255, 255, 255), 500)
            bars(d, 50, 580, 450, 6, 26, (150, 160, 190), 8)
        save(im, name)

    navy('corridor_vision.png', 'VISION', 'MISSION')
    navy('corridor_quality.png', 'QUALITY', 'POLICY')

    # code of conduct
    im = Image.new('RGB', (500, 800), (245, 245, 245)); d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 500, 150), fill=(245, 190, 40)); d.text((30, 40), 'CODE OF CONDUCT', fill=(40, 40, 40), font=f(44))
    d.text((30, 180), "DO'S", fill=(30, 130, 60), font=f(40)); bars(d, 40, 240, 440, 5, 28, (170, 170, 170), 8)
    d.text((30, 400), "DON'TS", fill=(200, 40, 40), font=f(40)); bars(d, 40, 460, 440, 10, 28, (170, 170, 170), 8)
    save(im, 'corridor_conduct.png')

    # fire action
    im = Image.new('RGB', (500, 800), (245, 215, 40)); d = ImageDraw.Draw(im)
    d.rounded_rectangle((40, 50, 460, 170), 20, fill=(30, 30, 30)); centered(d, 75, 'Fire Action!', f(60), (255, 255, 255), 500)
    for i, c in enumerate([(200, 40, 40), (40, 140, 60), (40, 140, 60), (200, 40, 40)]):
        d.rectangle((40, 230 + i * 140, 460, 320 + i * 140), fill=c)
    save(im, 'corridor_fire_action.png')

    # direction sign "2" (both sides)
    for name, arrows in (('corridor_dir_a.png', ['←', '→', '↱', '→']),
                         ('corridor_dir_b.png', ['→', '→', '↲', '←'])):
        im = Image.new('RGB', (1400, 400), (25, 25, 28)); d = ImageDraw.Draw(im)
        d.pieslice((-200, 0, 200, 800), 270, 360, fill=(200, 30, 40)); d.text((40, 120), '2', fill=(255, 255, 255), font=f(170))
        labels = [['FACULTY', 'STUDENT'], ['FIRE', 'EXIT'], ['LIFT'], ['FACULTY', 'STUDENT']]
        for i, (a, t) in enumerate(zip(arrows, labels)):
            xc = 330 + i * 270 + 120
            tw = d.textlength(a, font=f(110))
            d.text((xc - tw / 2, 30), a, fill=(255, 255, 255), font=f(110))
            for j, ln in enumerate(t):
                lw = d.textlength(ln, font=f(36))
                d.text((xc - lw / 2, 250 + j * 44), ln, fill=(255, 255, 255), font=f(36))
        save(im, name)

    # quote-style banners (own wording)
    def banner(name, text, bg, fg, border=None, size=(2000, 360), px=96):
        im = Image.new('RGB', size, bg); d = ImageDraw.Draw(im)
        if border:
            d.rectangle((0, 0, size[0] - 1, size[1] - 1), outline=border, width=24)
        lines = text.split('\n')
        y = (size[1] - len(lines) * px * 1.25) / 2
        for ln in lines:
            centered(d, y, ln, f(px), fg, size[0]); y += px * 1.25
        save(im, name)

    banner('corridor_q_blue.png', '“ Think like an engineer:\nmeasure, build, improve. ”', (232, 236, 245), (30, 35, 60), (40, 60, 140))
    banner('corridor_q_purple.png', '“ Learn. Build. Innovate.\nFor a better tomorrow. ”', (110, 40, 160), (255, 255, 255))
    banner('corridor_q_red.png', '“ Every failure is a step\ncloser to success. ”', (250, 250, 250), (25, 25, 25), (200, 30, 40))
    banner('corridor_q_green.png', '“ Find out why it failed,\nthen make it work. ”', (250, 250, 250), (25, 25, 25), (40, 150, 90))
    banner('corridor_q_orange.png', '“ Engineers turn ideas\ninto reality. ”', (240, 130, 50), (255, 255, 255))


if __name__ == '__main__':
    make_all(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'textures'))
