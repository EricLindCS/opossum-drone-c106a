"""
Makes a checkerboard you can print.

    python3 make_checkerboard.py                              # 9 x 7 squares, 25 mm each
    python3 make_checkerboard.py --squares 10 8 --size-mm 20

Print it at 100% ("actual size", not "fit to page") and measure a square
afterwards. A 9 x 7 board has 8 x 6 inner corners, so --cols 8 --rows 6.
"""
import argparse

import cv2
import numpy as np

DPI = 300


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--squares', type=int, nargs=2, default=(9, 7), metavar=('ACROSS', 'DOWN'))
    ap.add_argument('--size-mm', type=float, default=25.0)
    ap.add_argument('--out', default='checkerboard.png')
    args = ap.parse_args()
    nx, ny = args.squares
    s = int(round(args.size_mm / 25.4 * DPI))
    margin = s // 2  # needs a white border or the outer corners don't get detected
    img = np.full((ny * s + 2 * margin, nx * s + 2 * margin), 255, np.uint8)
    for r in range(ny):
        for c in range(nx):
            if (r + c) % 2 == 0:
                img[margin + r * s: margin + (r + 1) * s, margin + c * s: margin + (c + 1) * s] = 0
    cv2.imwrite(args.out, img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    w_mm, h_mm = img.shape[1] / DPI * 25.4, img.shape[0] / DPI * 25.4
    print(f'saved {args.out} ({w_mm:.0f} x {h_mm:.0f} mm). '
          f'for calibrate.py use --cols {nx - 1} --rows {ny - 1} --square {args.size_mm / 1000:g}')


if __name__ == '__main__':
    main()
