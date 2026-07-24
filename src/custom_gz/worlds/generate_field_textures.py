#!/usr/bin/env python3
"""Generate procedural farmland textures for the swarm worlds.

Each crop field in the swarm worlds is a flat 30x30 m box; instead of a solid
colour it gets a seeded 512x512 albedo texture that reads as row crops (or a
closed canopy with tramlines) from drone altitude. Textures are deliberately
cheap: one small PNG per crop, no extra geometry, no physics impact — the
drone cameras re-render the whole scene, so geometry would multiply across
every camera while a texture swap is effectively free.

None of the textures contain magenta, so the OpenCV HSV weed detector
(camera_processor.py) keeps an unambiguous target.

Output: src/custom_gz/models/farm_textures/textures/*.png, referenced from the
world SDFs as model://farm_textures/textures/<crop>.png (resolved via
GZ_SIM_RESOURCE_PATH, which start_gazebo.sh points at src/custom_gz/models).

Regenerate (deterministic for a given seed):
    python3 src/custom_gz/worlds/generate_field_textures.py --seed 42
"""
import argparse
import os
import random

from PIL import Image, ImageDraw, ImageFilter

SIZE = 512          # px per texture; one texture spans one 30 m field face
FIELD_M = 30.0      # field edge length the texture is stretched over
PX_PER_M = SIZE / FIELD_M


def _noise_layer(rng, base, amplitude):
    """Low-frequency blotchy noise around a base colour (soil/canopy variation)."""
    small = Image.new("RGB", (32, 32))
    px = small.load()
    for y in range(32):
        for x in range(32):
            px[x, y] = tuple(
                max(0, min(255, c + rng.randint(-amplitude, amplitude)))
                for c in base
            )
    return small.resize((SIZE, SIZE), Image.BILINEAR)


def _speckle(img, rng, color, count, r_px):
    """Scatter small dots (blooms, bolls, flower heads) over the canopy."""
    draw = ImageDraw.Draw(img)
    for _ in range(count):
        x = rng.uniform(0, SIZE)
        y = rng.uniform(0, SIZE)
        r = rng.uniform(0.5, r_px)
        draw.ellipse([x - r, y - r, x + r, y + r], fill=color)


def row_crop(rng, soil, canopy, row_spacing_m, row_width_m,
             gap_chance=0.06, speckle=None):
    """Parallel crop rows over bare soil (corn, soybean, cotton, sunflower)."""
    img = _noise_layer(rng, soil, 14)
    draw = ImageDraw.Draw(img)
    spacing = row_spacing_m * PX_PER_M
    width = max(2, int(row_width_m * PX_PER_M))
    y = rng.uniform(0, spacing)
    while y < SIZE:
        # draw each row as short jittered segments so it isn't a ruler line
        x = 0.0
        while x < SIZE:
            seg = rng.uniform(10, 30)
            if rng.random() > gap_chance:  # occasional emergence gap
                jitter = rng.uniform(-1.2, 1.2)
                shade = rng.randint(-18, 18)
                color = tuple(max(0, min(255, c + shade)) for c in canopy)
                draw.line([(x, y + jitter), (x + seg, y + jitter)],
                          fill=color, width=width)
            x += seg
        y += spacing
    img = img.filter(ImageFilter.GaussianBlur(0.6))
    if speckle:
        _speckle(img, rng, *speckle)
    return img


def canopy_crop(rng, canopy, tramline_spacing_m=None, soil=(96, 74, 50),
                amplitude=20, speckle=None):
    """Closed canopy (wheat, barley, rice, canola, alfalfa) with optional
    sprayer tramlines — the paired wheel tracks visible in real cereal fields."""
    img = _noise_layer(rng, canopy, amplitude)
    img = img.filter(ImageFilter.GaussianBlur(1.2))
    if speckle:
        _speckle(img, rng, *speckle)
    if tramline_spacing_m:
        draw = ImageDraw.Draw(img)
        spacing = tramline_spacing_m * PX_PER_M
        y = spacing / 2
        while y < SIZE:
            for offset in (-2, 2):  # paired wheel tracks
                draw.line([(0, y + offset), (SIZE, y + offset)],
                          fill=soil, width=2)
            y += spacing
    return img


def grass(rng):
    """Rough pasture/verge for the ground plane — stretched over 2000 m it
    reads as large-scale terrain variation, so keep it low-frequency and
    low-contrast (high amplitude turns into a giant checkerboard)."""
    img = _noise_layer(rng, (98, 122, 66), 10)
    return img.filter(ImageFilter.GaussianBlur(4.0))


def build_textures(rng):
    # Colours are deliberately ~30% brighter than real aerial photography:
    # ogre2's PBR pipeline (sRGB albedo + neutral scene ambient) renders
    # noticeably darker than the source PNG.
    soil = (130, 100, 70)
    return {
        # row crops: (soil, canopy, row spacing m, row width m)
        "corn": row_crop(rng, soil, (66, 132, 52), 0.76, 0.45),
        "soybean": row_crop(rng, soil, (82, 142, 74), 0.50, 0.34),
        "cotton": row_crop(rng, soil, (92, 134, 82), 0.96, 0.55,
                           speckle=((232, 232, 228), 500, 1.6)),
        "sunflower": row_crop(rng, soil, (76, 126, 60), 0.75, 0.55,
                              speckle=((224, 178, 40), 700, 2.2)),
        # closed canopies
        "wheat": canopy_crop(rng, (198, 174, 84), tramline_spacing_m=6.0),
        "barley": canopy_crop(rng, (188, 164, 96), tramline_spacing_m=6.0),
        "rice": canopy_crop(rng, (96, 146, 86), tramline_spacing_m=None,
                            amplitude=26),
        "canola": canopy_crop(rng, (222, 206, 60), tramline_spacing_m=7.5,
                              speckle=((236, 222, 80), 900, 2.0)),
        "alfalfa": canopy_crop(rng, (78, 148, 80), tramline_spacing_m=None),
        # ground plane
        "grass": grass(rng),
    }


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>farm_textures</name>
  <version>1.0</version>
  <sdf version="1.9"></sdf>
  <description>
    Procedural farmland albedo textures for the swarm worlds - generated by
    src/custom_gz/worlds/generate_field_textures.py, referenced from the world
    SDFs as model://farm_textures/textures/(crop).png. Texture pack only, not
    an includable model.
  </description>
</model>
"""


def main():
    default_out = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "models", "farm_textures"))
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--out", default=default_out)
    args = p.parse_args()

    rng = random.Random(args.seed)
    tex_dir = os.path.join(args.out, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    with open(os.path.join(args.out, "model.config"), "w") as f:
        f.write(MODEL_CONFIG)
    for name, img in build_textures(rng).items():
        path = os.path.join(tex_dir, f"{name}.png")
        img.save(path, optimize=True)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
