"""
Camera calibration from checkerboard pictures (Section 1).

    python3 calibrate.py ../captures/<calib folder> --cols 8 --rows 6 --square 0.025

--cols and --rows are the number of INNER corners, not squares (a 9 x 7 board
has 8 x 6). --square is the side of one square in meters.

Saves calibration/calibration.json plus some pictures in calibration/ to check
the result with (corners/, undistorted/, coverage.png, distortion.png).


"""
import argparse
import glob
import json
import os

import cv2
import numpy as np

from common import CALIBRATION_PATH

OUT_DIR = os.path.dirname(CALIBRATION_PATH)
SUBPIX_WINDOW = (11, 11)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def board_points(cols, rows, square):
    # (cols*rows, 3) float32 corner positions on the z = 0 plane, row by row:
    # (0,0,0), (square,0,0), ..., then (0,square,0), ...
    # TODO 1.1: build the grid. np.mgrid[0:cols, 0:rows] gives you every
    # (col, row) pair, you just need to get it into the right order.
    blank = np.zeros((cols * rows, 3), np.float32)
    
    grid = np.mgrid[0:cols, 0:rows]
    blank[:, :2] = grid.T.reshape(-1,2)

    #lst = np.array([np.array([item[0], item[1], 0]) for item in x], np.float32)

    #grid = blank * square
    #print(grid.shape)

    return blank

def find_corners(gray, cols, rows):
    # returns the corners as a (cols*rows, 2) array, or None if the board isn't found

    # these help when the lighting is uneven
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    # TODO 1.2: cv2.findChessboardCorners with patternSize (cols, rows) and
    # flags=flags. if it finds the board, refine with cv2.cornerSubPix using
    # SUBPIX_WINDOW, a zero zone of (-1, -1), and SUBPIX_CRITERIA.
    # the OpenCV calibration tutorial has an example of both.
    retval, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)

    if not retval:
        return None
    else:
        corners_refined = cv2.cornerSubPix(gray, corners, SUBPIX_WINDOW, (-1, -1), SUBPIX_CRITERIA)
    return corners_refined.reshape(-1,2)


def calibrate(object_points, image_points, image_size):
    # object_points: one board_points() array per picture
    # image_points: the corners you found in each picture
    # image_size: (width, height)
    # should return rms, K, dist (flattened), rvecs, tvecs
    # TODO 1.3: cv2.calibrateCamera. pass None for the starting K and dist.
    # careful, it wants the image points shaped (N, 1, 2)

    image_points = [point.reshape(-1, 1, 2) for point in image_points]

    retval, cameraMatrix, disctCoeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, cameraMatrix = None, distCoeffs = None)

    return retval, cameraMatrix, disctCoeffs.flatten(), rvecs, tvecs



# Everything below here just draws pictures of the result.

def grid_lines(corners, cols, rows):
    grid = corners.reshape(rows, cols, 2)
    return list(grid) + list(grid.transpose(1, 0, 2))


def bend_px(corners, cols, rows):
    # furthest any corner is from the straight line between the two ends of its row/column
    worst = 0.0
    for line in grid_lines(corners, cols, rows):
        d = line[-1] - line[0]
        normal = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        worst = max(worst, float(np.abs((line - line[0]) @ normal).max()))
    return worst


def draw_grid(image, corners, cols, rows):
    def pt(p):
        return tuple(int(v) for v in np.round(p * 16))  # shift=4 -> 1/16 px
    for line in grid_lines(corners, cols, rows):
        cv2.line(image, pt(line[0]), pt(line[-1]), (0, 0, 255), 1, cv2.LINE_AA, shift=4)
    for p in corners:
        cv2.circle(image, pt(p), 3 * 16, (0, 200, 0), -1, cv2.LINE_AA, shift=4)


def label(image, text):
    cv2.rectangle(image, (0, 0), (image.shape[1], 30), (40, 40, 40), -1)
    cv2.putText(image, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def undistortion_figure(image, corners, K, dist, cols, rows):
    # before/after like the OpenCV tutorial. alpha=1 keeps every pixel so the
    # undistorted side gets curved black edges; the blue box is what you'd crop to
    h, w = image.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, dist, None, new_K)
    flat = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist, P=new_K).reshape(-1, 2)

    before = image.copy()
    draw_grid(before, corners, cols, rows)
    draw_grid(undistorted, flat, cols, rows)
    x, y, rw, rh = roi
    cv2.rectangle(undistorted, (x, y), (x + rw - 1, y + rh - 1), (255, 120, 0), 1)
    b0, b1 = bend_px(corners, cols, rows), bend_px(flat, cols, rows)
    label(before, f'original: corners up to {b0:.1f} px off straight')
    label(undistorted, f'undistorted: {b1:.1f} px')
    gap = np.full((h, 10, 3), 255, np.uint8)
    return np.hstack([before, gap, undistorted]), b0, b1


def coverage_figure(image_points, image_size, cols, rows):
    w, h = image_size
    canvas = np.full((h, w, 3), 255, np.uint8)
    colors = cv2.applyColorMap(np.linspace(0, 255, len(image_points)).astype(np.uint8), cv2.COLORMAP_TURBO)
    for pts, color in zip(image_points, colors.reshape(-1, 3)):
        grid = pts.reshape(rows, cols, 2)
        outline = np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
        cv2.polylines(canvas, [np.round(outline).astype(np.int32)], True, tuple(int(c) for c in color), 2, cv2.LINE_AA)
    label(canvas, f'{len(image_points)} pictures - do they reach every edge and corner?')
    return canvas


def distortion_figure(K, dist, image_size, step=40):
    # color = how many pixels the lens moves each point, arrows = which way (scaled up)
    w, h = image_size
    ys, xs = np.mgrid[0:h, 0:w]
    pixels = np.stack([xs.ravel(), ys.ravel()], 1).astype(np.float32)
    ideal = cv2.undistortPoints(pixels.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
    shift = (ideal - pixels).reshape(h, w, 2)
    size = np.linalg.norm(shift, axis=2)
    most = max(float(size.max()), 1e-6)
    canvas = cv2.applyColorMap((size / most * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    gain = 0.9 * step / most
    for y in range(step // 2, h, step):
        for x in range(step // 2, w, step):
            dx, dy = shift[y, x] * gain
            cv2.arrowedLine(canvas, (x, y), (int(round(x + dx)), int(round(y + dy))),
                            (255, 255, 255), 1, cv2.LINE_AA, tipLength=0.3)
    cv2.drawMarker(canvas, (int(round(K[0, 2])), int(round(K[1, 2]))), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    label(canvas, f'distortion moves pixels up to {most:.1f} px (dark = 0, yellow = {most:.1f}), '
                  f'arrows x{gain:.0f}, red + = principal point')
    return canvas, most


def main():
    
    ap = argparse.ArgumentParser()
    ap.add_argument('image_dir')
    ap.add_argument('--cols', type=int, required=True, help='inner corners across')
    ap.add_argument('--rows', type=int, required=True, help='inner corners down')
    ap.add_argument('--square', type=float, required=True, help='square size in meters')
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.image_dir, '*.png')) +
                   glob.glob(os.path.join(args.image_dir, '*.jpg')))
    if not paths:
        raise SystemExit(f'no images in {args.image_dir}')
    os.makedirs(os.path.join(OUT_DIR, 'corners'), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, 'undistorted'), exist_ok=True)

    grid = board_points(args.cols, args.rows, args.square)
    object_points, image_points, used = [], [], []
    for path in paths:
        image = cv2.imread(path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        corners = find_corners(gray, args.cols, args.rows)
        name = os.path.basename(path)
        if corners is None:
            print(f'{name}: no board found, skipping')
            continue
        cv2.drawChessboardCorners(image, (args.cols, args.rows), corners.reshape(-1, 1, 2), True)
        cv2.imwrite(os.path.join(OUT_DIR, 'corners', name), image)
        object_points.append(grid)
        image_points.append(corners.astype(np.float32))
        used.append(name)

    print(f'found the board in {len(used)}/{len(paths)} images')
    if len(used) < 5:
        raise SystemExit('need at least 5 good pictures (15-25 is better)')

    rms, K, dist, rvecs, tvecs = calibrate(object_points, image_points, image_size)

    # error for each picture so you can spot bad ones
    per_image = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        per_image.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1)))))
    print('\nRMS error per image (px):')
    for name, e in zip(used, per_image):
        print(f'  {name}: {e:.3f}' + ('  <-- check this one' if e > 1.0 else ''))

    np.set_printoptions(precision=3, suppress=True)
    print(f'\nRMS reprojection error: {rms:.3f} px')
    print(f'K =\n{K}')
    print(f'dist (k1, k2, p1, p2, k3) = {dist}')
    print('(camera_info says fx = fy = 700, cx = 320, cy = 240)')

    bends = []
    for name, corners in zip(used, image_points):
        image = cv2.imread(os.path.join(args.image_dir, name))
        figure, b0, b1 = undistortion_figure(image, corners, K, dist, args.cols, args.rows)
        cv2.imwrite(os.path.join(OUT_DIR, 'undistorted', name), figure)
        bends.append((b0, b1))
    bends = np.array(bends)
    print(f'\ncorners off straight lines: {np.median(bends[:, 0]):.2f} px before undistorting, '
          f'{np.median(bends[:, 1]):.2f} px after (median)')
    cv2.imwrite(os.path.join(OUT_DIR, 'coverage.png'), coverage_figure(image_points, image_size, args.cols, args.rows))
    figure, most = distortion_figure(K, dist, image_size)
    cv2.imwrite(os.path.join(OUT_DIR, 'distortion.png'), figure)
    print(f'distortion moves pixels up to {most:.1f} px')

    with open(CALIBRATION_PATH, 'w') as f:
        json.dump(dict(K=K.tolist(), dist=dist.tolist(), rms_px=rms, image_size=list(image_size),
                       board=dict(cols=args.cols, rows=args.rows, square_m=args.square),
                       images=used, per_image_rms_px=per_image), f, indent=2)
    print(f'saved {CALIBRATION_PATH} and pictures in {OUT_DIR}/')


if __name__ == '__main__':
    main()
