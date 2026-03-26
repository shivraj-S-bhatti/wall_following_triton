#!/usr/bin/env python3
"""Build a compact qualitative composite from the final afterimage screenshots."""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


QUAL_DIR = Path(__file__).resolve().parents[1] / "qualitative"
OUTPUT_PATH = QUAL_DIR / "qualitative_afterimages_composite.png"

# Label text is kept only for debugging; LaTeX subcaptions provide IEEE-style (a)--(c).
PANELS = [
    (
        "ibeam_afterimages.png",
        (90, 150, 970, 640),
        "A. I-beam / interior wall end",
        "qual_panel_a_ibeam.png",
    ),
    (
        "uturn_deadend_afterimages.png",
        (85, 250, 950, 760),
        "B. Dead-end / U-turn recovery",
        "qual_panel_b_uturn.png",
    ),
    (
        "final_3_turns_afterimages.png",
        (25, 150, 1070, 835),
        "C. Right turn, right U-turn, left turn",
        "qual_panel_c_three_turns.png",
    ),
]

TARGET_HEIGHT = 300
GUTTER = 24
PADDING = 10
# Set to 0 so panel PNGs are frame-only; composite can still stack labels if needed.
LABEL_HEIGHT = 0
BACKGROUND = (255, 255, 255)
FRAME = (217, 226, 236)
INK = (31, 41, 51)


def _load_font(size: int):
    try:
        return ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", size)
    except Exception:
        return ImageFont.load_default()


def _panel_image(
    filename: str,
    crop_box: tuple[int, int, int, int],
    label: str,
    *,
    draw_label: bool,
) -> Image.Image:
    source = Image.open(QUAL_DIR / filename).convert("RGB")
    panel = source.crop(crop_box)
    scale = TARGET_HEIGHT / panel.height
    target_width = int(panel.width * scale)
    panel = panel.resize((target_width, TARGET_HEIGHT), Image.LANCZOS)

    label_h = LABEL_HEIGHT if draw_label else 0
    framed = Image.new(
        "RGB",
        (target_width + 2 * PADDING, TARGET_HEIGHT + label_h + 2 * PADDING),
        BACKGROUND,
    )
    draw = ImageDraw.Draw(framed)
    framed.paste(panel, (PADDING, PADDING))
    draw.rectangle(
        (PADDING, PADDING, PADDING + target_width - 1, PADDING + TARGET_HEIGHT - 1),
        outline=FRAME,
        width=2,
    )
    if draw_label and label_h > 0:
        font = _load_font(19)
        bbox = draw.textbbox((0, 0), label, font=font)
        text_w = bbox[2] - bbox[0]
        draw.text(
            ((framed.width - text_w) / 2, TARGET_HEIGHT + PADDING + 6),
            label,
            fill=INK,
            font=font,
        )
    return framed


def main() -> None:
    # IEEE-style document uses LaTeX \subcaption for (a)--(c); export standalone frames.
    for filename, crop_box, _text, out_name in PANELS:
        solo = _panel_image(filename, crop_box, "", draw_label=False)
        out_path = QUAL_DIR / out_name
        solo.save(out_path)
        print(f"saved {out_path}")

    panels = [
        _panel_image(filename, crop_box, label, draw_label=False)
        for filename, crop_box, label, _out in PANELS
    ]
    total_width = sum(panel.width for panel in panels) + GUTTER * (len(panels) - 1)
    total_height = max(panel.height for panel in panels)
    composite = Image.new("RGB", (total_width, total_height), BACKGROUND)

    x = 0
    for panel in panels:
        composite.paste(panel, (x, 0))
        x += panel.width + GUTTER

    composite.save(OUTPUT_PATH)
    print(f"saved {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
