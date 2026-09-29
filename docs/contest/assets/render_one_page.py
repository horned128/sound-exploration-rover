"""Render the contest's editable, one-page SEROV system overview.

Run: python3 docs/contest/assets/render_one_page.py
Uses only Pillow and the two photographs kept alongside this script.
"""

from pathlib import Path
import math

from PIL import Image, ImageDraw, ImageFont, ImageOps


HERE = Path(__file__).resolve().parent
ASSETS = HERE
SIZE = (2560, 1440)
FONT_PATH = "/System/Library/Fonts/Hiragino Sans GB.ttc"

INK = "#172C3C"
MUTED = "#566977"
TEAL = "#007D8D"
TEAL_PALE = "#E6F4F5"
ORANGE = "#DF623E"
ORANGE_PALE = "#FFF0E9"
REVIEW_RED = "#C7464C"
SKY_PALE = "#ECF3F9"
LINE = "#D8E2E8"
BG = "#F5F8FA"
WHITE = "#FFFFFF"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT_PATH, size, index=2 if bold else 0)


def text(draw, xy, value, size, fill=INK, bold=False, anchor=None):
    draw.text(xy, value, font=font(size, bold), fill=fill, anchor=anchor)


def fitted_text(draw, xy, value, size, max_width, fill=INK, bold=False):
    while draw.textbbox((0, 0), value, font=font(size, bold))[2] > max_width:
        size -= 1
    text(draw, xy, value, size, fill, bold)


def rounded(draw, box, radius, fill, outline=None, width=1):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def dotted_rounded_border(draw, box, radius, color, dot_radius=4, step=18):
    """Draw a visible rounded dotted frame around the two judged kernels."""
    x1, y1, x2, y2 = box
    points = []
    for x in range(x1 + radius, x2 - radius + 1, step):
        points.extend(((x, y1), (x, y2)))
    for y in range(y1 + radius, y2 - radius + 1, step):
        points.extend(((x1, y), (x2, y)))
    for cx, cy, start in (
        (x2 - radius, y1 + radius, -90),
        (x2 - radius, y2 - radius, 0),
        (x1 + radius, y2 - radius, 90),
        (x1 + radius, y1 + radius, 180),
    ):
        for deg in range(start, start + 91, 24):
            rad = math.radians(deg)
            points.append((cx + radius * math.cos(rad),
                           cy + radius * math.sin(rad)))
    for x, y in points:
        draw.ellipse((x - dot_radius, y - dot_radius,
                      x + dot_radius, y + dot_radius), fill=color)


def arrow(draw, start, end, color=TEAL, width=7, head=17, dash=False):
    x1, y1 = start
    x2, y2 = end
    if dash:
        distance = math.dist(start, end)
        for i in range(0, int(distance), 22):
            a = i / distance
            b = min(i + 12, distance) / distance
            draw.line(
                [(x1 + (x2 - x1) * a, y1 + (y2 - y1) * a),
                 (x1 + (x2 - x1) * b, y1 + (y2 - y1) * b)],
                fill=color, width=width,
            )
    else:
        draw.line([start, end], fill=color, width=width)
    angle = math.atan2(y2 - y1, x2 - x1)
    side = head * 0.55
    p1 = (x2 - head * math.cos(angle) + side * math.sin(angle),
          y2 - head * math.sin(angle) - side * math.cos(angle))
    p2 = (x2 - head * math.cos(angle) - side * math.sin(angle),
          y2 - head * math.sin(angle) + side * math.cos(angle))
    draw.polygon([end, p1, p2], fill=color)


def node(draw, box, number, action, title, detail, foot, color, pale):
    rounded(draw, box, 24, pale, outline=color, width=3)
    x1, y1, x2, y2 = box
    rounded(draw, (x1 + 23, y1 + 22, x1 + 86, y1 + 67), 16, color)
    text(draw, (x1 + 54, y1 + 45), number, 23, WHITE, True, "mm")
    text(draw, (x1 + 100, y1 + 44), action, 27, color, True, "lm")
    available = x2 - x1 - 50
    fitted_text(draw, (x1 + 25, y1 + 86), title, 33,
                available, INK, True)
    fitted_text(draw, (x1 + 25, y1 + 132), detail, 24,
                available, MUTED)
    draw.line((x1 + 25, y2 - 54, x2 - 25, y2 - 54), fill=LINE, width=2)
    fitted_text(draw, (x1 + 25, y2 - 44), foot, 23,
                available, color, True)


def photo_contain(canvas, path, box, background=WHITE):
    x1, y1, x2, y2 = box
    canvas.paste(background, box)
    photo = Image.open(path).convert("RGB")
    fit = ImageOps.contain(photo, (x2 - x1, y2 - y1), Image.Resampling.LANCZOS)
    px = x1 + (x2 - x1 - fit.width) // 2
    py = y1 + (y2 - y1 - fit.height) // 2
    canvas.paste(fit, (px, py))
    return px, py, fit.width, fit.height


def label(draw, box, title, color, target, side="left"):
    x1, y1, x2, y2 = box
    source = (x1 if side == "left" else x2, (y1 + y2) // 2)
    draw.line([source, target], fill=color, width=4)
    draw.ellipse((target[0] - 8, target[1] - 8,
                  target[0] + 8, target[1] + 8), fill=color)
    rounded(draw, box, 12, WHITE, outline=color, width=3)
    text(draw, ((x1 + x2) // 2, (y1 + y2) // 2), title, 25,
         color, True, "mm")


def render():
    im = Image.new("RGB", SIZE, BG)
    d = ImageDraw.Draw(im)

    # Masthead
    d.rectangle((0, 0, 2560, 221), fill=WHITE)
    d.rectangle((0, 212, 2560, 221), fill=ORANGE)
    rounded(d, (72, 55, 87, 171), 7, ORANGE)
    text(d, (115, 47), "TRON PROGRAMMING CONTEST 2026", 30,
         TEAL, True)
    text(d, (113, 94), "Sound Exploration ROVer", 71, INK, True)
    text(d, (1635, 91), "音を覚え、探し、避けて進む", 45, INK, True)
    text(d, (1640, 154), "自律走行ローバー  SEROV", 27, MUTED)

    # Two presentation panels.
    rounded(d, (70, 253, 1566, 1175), 30, WHITE, outline=LINE, width=2)
    rounded(d, (1594, 253, 2490, 1175), 30, WHITE, outline=LINE, width=2)
    d.rectangle((114, 297, 135, 318), fill=INK)
    text(d, (154, 278), "システム構成図", 44, INK, True)
    rounded(d, (1128, 283, 1513, 332), 18, "#FFF1F1")
    text(d, (1320, 307), "審査対象：μT-Kernel ×2", 24,
         REVIEW_RED, True, "mm")
    text(d, (115, 349), "音の入力から進路判断、駆動までを機体内で完結", 28, MUTED)

    # The red dotted frame identifies precisely the two judged kernel instances.
    kernel_region = (819, 419, 1536, 727)
    rounded(d, kernel_region, 27, "#FFFAFA")
    dotted_rounded_border(d, kernel_region, 27, REVIEW_RED)
    text(d, (844, 430), "RA8P1  /  各コアで μT-Kernel 3.0", 24,
         REVIEW_RED, True)
    node(d, (113, 488, 365, 708), "01", "音を聞く", "4マイク",
         "ReSpeaker XVF3800", "音源方位・音の強さ", TEAL, TEAL_PALE)
    node(d, (482, 488, 734, 708), "02", "特徴を作る", "XIAO ESP32-S3",
         "音響フロントエンド", "USB CDCでCPU0へ", TEAL, TEAL_PALE)
    node(d, (848, 488, 1128, 708), "03", "進路を決める", "RA8P1 CPU0",
         "Cortex-M85 / 50 ms", "音の照合・TFLM・回避", "#23728E", SKY_PALE)
    node(d, (1242, 488, 1518, 708), "04", "安全に駆動", "RA8P1 CPU1",
         "Cortex-M33 / 1 ms", "4サーボ・左右モーター", ORANGE, ORANGE_PALE)
    for a, b, title in [
        ((370, 597), (477, 597), "I2S / I2C"),
        ((739, 597), (843, 597), "USB CDC"),
        ((1133, 597), (1237, 597), "IPC"),
    ]:
        arrow(d, a, b, TEAL if title != "IPC" else ORANGE, 6, 13)
        center_x = (a[0] + b[0]) // 2
        label_width = d.textbbox((0, 0), title, font=font(19, True))[2]
        rounded(d, (center_x - label_width // 2 - 4, 553,
                    center_x + label_width // 2 + 4, 585), 7, WHITE)
        text(d, (center_x, 571), title, 19, MUTED, True, "mm")

    # Inputs, optional diagnostics, and the physical output.
    rounded(d, (114, 807, 470, 943), 20, WHITE, outline=LINE, width=2)
    text(d, (137, 831), "任意の診断", 25, TEAL, True)
    text(d, (137, 874), "Wi-Fi UDP → PC", 27, INK, True)
    text(d, (137, 913), "走行にPC接続は不要", 22, MUTED)
    arrow(d, (606, 711), (435, 808), TEAL, 4, 12, dash=True)

    rounded(d, (634, 807, 1036, 943), 20, SKY_PALE,
            outline="#A8CDD9", width=2)
    text(d, (661, 829), "センサー入力", 25, "#23728E", True)
    text(d, (661, 875), "距離×3 ＋ 姿勢", 29, INK, True)
    text(d, (661, 912), "VL53L1X / BMI270", 22, MUTED)
    arrow(d, (977, 804), (977, 713), "#23728E", 5, 15)
    text(d, (862, 754), "GPIO I2C", 21, "#23728E", True)

    rounded(d, (1091, 807, 1518, 943), 20, ORANGE_PALE,
            outline="#E8B7A6", width=2)
    text(d, (1118, 829), "機体の動き", 25, ORANGE, True)
    text(d, (1118, 875), "4サーボ ＋ 6輪", 29, INK, True)
    text(d, (1118, 912), "ロッカーボギーで走行", 22, MUTED)
    arrow(d, (1379, 711), (1379, 804), ORANGE, 5, 15)
    text(d, (1394, 754), "PWM / GPIO", 21, ORANGE, True)

    rounded(d, (114, 1003, 1518, 1130), 20, "#FFF8E7",
            outline="#F0D58D", width=2)
    rounded(d, (140, 1031, 234, 1099), 22, "#E7B54C")
    text(d, (187, 1065), "SW1", 26, WHITE, True, "mm")
    text(d, (263, 1019), "その場で音を登録", 31, INK, True)
    text(d, (263, 1069), "長押し → 有効な見本を5件収集 → Code MRAMへ保存", 27,
         MUTED)

    # Real hardware: photographs from this submission, with only visually
    # verifiable callouts. The architecture details are in the left panel.
    d.rectangle((1637, 297, 1658, 318), fill=INK)
    text(d, (1678, 278), "実機配置図", 44, INK, True)
    text(d, (1638, 349), "提出した6輪ローバー", 28, MUTED)
    photo_contain(im, ASSETS / "rover-angle.jpg", (1632, 406, 2450, 925))
    d = ImageDraw.Draw(im)
    label(d, (2186, 416, 2426, 474), "4マイク基板", TEAL,
          (2052, 459), "left")
    label(d, (1640, 578, 1878, 636), "制御基板", "#23728E",
          (1995, 550), "right")
    label(d, (2163, 815, 2426, 873), "6輪の駆動機構", ORANGE,
          (2245, 780), "left")

    d.line((1638, 950, 2446, 950), fill=LINE, width=3)
    rounded(d, (1637, 978, 1980, 1131), 17, "#FAFCFD",
            outline=LINE, width=2)
    photo_contain(im, ASSETS / "rover-top.jpg", (1654, 986, 1961, 1122))
    d = ImageDraw.Draw(im)
    text(d, (2014, 991), "上面から見た機体", 31, INK, True)
    text(d, (2014, 1041), "中央：4マイク基板", 26, MUTED)
    text(d, (2014, 1082), "周囲：操舵・走行用の車輪", 25, MUTED)

    # Takeaway band across the bottom of the single-page slide.
    d.rectangle((0, 1202, 2560, 1440), fill=INK)
    text(d, (83, 1240), "作品の要点", 27, "#94CBD2", True)
    text(d, (82, 1283), "現場で音を覚え、音源へ進み、障害物を避ける。", 50,
         WHITE, True)
    d.line((1400, 1245, 1400, 1393), fill="#526675", width=2)
    for y, title, sub in [
        (1237, "デュアルコアOS", "CPU0が判断 / CPU1が駆動"),
        (1301, "デュアルAI", "音の見本照合 / TFLM走行制御"),
        (1365, "現場学習", "SW1操作 / MRAMに保存"),
    ]:
        d.ellipse((1462, y + 13, 1474, y + 25), fill="#E7B54C")
        text(d, (1504, y), title, 29, WHITE, True)
        text(d, (1796, y + 3), sub, 27, "#D6E1E7")

    im.save(HERE / "serov-one-page.png", optimize=True)
    im.save(HERE / "serov-one-page.pdf", "PDF", resolution=180.0)


if __name__ == "__main__":
    render()
