#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 Dmytro Varich
# SPDX-License-Identifier: MIT

"""Convert a grayscale/color map to a binary Nav2 occupancy map.

Output convention:
    white / 254 = free space
    black / 0   = occupied space

The script treats only sufficiently bright pixels as free space.
Everything else becomes occupied.
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def convert_map_to_nav2_binary(
    input_path: str,
    output_path: str,
    free_threshold: int = 220,
) -> None:
    """Convert map to binary PGM for Nav2.

    Args:
        input_path: Path to input map image.
        output_path: Path to output PGM map.
        free_threshold: Pixels brighter than this value become free.
                        Everything else becomes occupied.
    """
    img = Image.open(input_path).convert("L")
    arr = np.array(img)

    # Default: everything is occupied.
    out = np.zeros_like(arr, dtype=np.uint8)

    # Only clearly white areas are free.
    free_mask = arr >= free_threshold
    out[free_mask] = 254

    # Save as raw binary PGM (P5).
    Image.fromarray(out, mode="L").save(output_path, format="PPM")

    print(f"Saved Nav2 map: {output_path}")
    print(f"Free pixels: {np.count_nonzero(out == 254)}")
    print(f"Occupied pixels: {np.count_nonzero(out == 0)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert map to binary Nav2 occupancy PGM.",
    )
    parser.add_argument("input", help="Input map file")
    parser.add_argument("output", help="Output PGM file")
    parser.add_argument(
        "--free-threshold",
        type=int,
        default=220,
        help="Pixels >= this value are treated as free space. Default: 220.",
    )

    args = parser.parse_args()

    input_file = Path(args.input)
    if not input_file.exists():
        raise FileNotFoundError(f"Input file not found: {args.input}")

    convert_map_to_nav2_binary(
        input_path=args.input,
        output_path=args.output,
        free_threshold=args.free_threshold,
    )


if __name__ == "__main__":
    main()