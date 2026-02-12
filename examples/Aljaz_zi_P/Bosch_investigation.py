import os
import argparse
import fnmatch
from typing import Tuple, Optional, List

import numpy as np
import cv2
import matplotlib
import depthai as dai

print(cv2.__version__)
print(np.__version__)
PRINTS = True
DEBUG_SHOW = True


def log(message: str) -> None:
    if PRINTS:
        print(message)


def show_debug(image_bgr, title: str, folder: str, filename: str) -> None:
    if image_bgr is None:
        return
    if DEBUG_SHOW and os.environ.get("DISPLAY"):
        cv2.imshow(title, image_bgr)
        cv2.waitKey(0)
        cv2.destroyWindow(title)
    else:
        path = os.path.join(folder, filename)
        cv2.imwrite(path, image_bgr)
        log(f"[debug] saved -> {path}")


def draw_points(image_bgr, points, color=(0, 255, 0)) -> None:
    if points is None:
        return
    pts = np.asarray(points).reshape(-1, 2)
    for x, y in pts:
        cv2.circle(image_bgr, (int(round(x)), int(round(y))), 3, color, -1)


NUM_SQUARES_X = 16
NUM_SQUARES_Y = 26
SQUARE_LENGTH = 50.0
MARKER_LENGTH = 40.0
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
ARUCO_PARAMS.minMarkerDistanceRate = 0.01
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B


def get_intrinsics(calib: dai.CalibrationHandler, socket, size: Tuple[int, int]):
    width, height = size
    K = np.array(calib.getCameraIntrinsics(socket, width, height))
    d = np.array(calib.getDistortionCoefficients(socket))
    return K, d


def detect_charuco(image_bgr, board):
    if len(image_bgr.shape) == 3:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    else:
        gray = image_bgr
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
    if ids is None or len(ids) == 0:
        return 0, None, None

    refined_corners, refined_ids, _, _ = cv2.aruco.refineDetectedMarkers(
        gray, board, corners, ids, rejectedCorners=rejected
    )
    if refined_ids is None or len(refined_ids) == 0:
        refined_corners, refined_ids = corners, ids

    num_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        refined_corners, refined_ids, gray, board, minMarkers=1
    )
    if charuco_corners is None or charuco_ids is None or len(charuco_ids) == 0:
        return 0, None, None

    return int(num_corners), charuco_corners.reshape(-1, 2), charuco_ids.flatten()


def build_charuco_object_points(board, ids):
    return board.chessboardCorners[ids].astype(np.float32)


def solve_pnp(object_points, image_points, K, d, label: str = ""):
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        K,
        d,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=10.0,
        iterationsCount=200,
        confidence=0.999,
    )
    if not success:
        raise RuntimeError(f"solvePnPRansac failed for Charuco detections {label}".strip())
    if inliers is not None and len(inliers) >= 6:
        inliers = inliers.flatten()
        success_refine, rvec, tvec = cv2.solvePnP(
            object_points[inliers],
            image_points[inliers],
            K,
            d,
            rvec,
            tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success_refine:
            raise RuntimeError("solvePnP refinement failed for Charuco inliers")
    return rvec, tvec


def project_points(points_3d, K, d, rvec, tvec):
    proj, _ = cv2.projectPoints(points_3d, rvec, tvec, K, d)
    return proj.reshape(-1, 2)


def plot_overlay_multi(
    image_bgr,
    points_and_styles,
    title,
    save_path: str,
    stats_label: str,
    point_errors: Optional[np.ndarray] = None,
) -> None:
    img = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    if point_errors is not None:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        ax_img, ax_hist = axes
        ax_img.imshow(img)
        for pts, color, label in points_and_styles:
            ax_img.scatter(pts[:, 0], pts[:, 1], s=12, c=color, label=label, alpha=0.7)
        pts = points_and_styles[0][0]
        for (x, y), err in zip(pts, point_errors):
            ax_img.text(
                x + 4,
                y + 4,
                f"{err:.2f}",
                color="yellow",
                fontsize=7,
                bbox=dict(facecolor="black", alpha=0.5, edgecolor="none", pad=1.5),
            )
        ax_img.set_title(title)
        ax_img.legend()
        if stats_label:
            ax_img.text(
                0.01,
                0.02,
                stats_label,
                transform=ax_img.transAxes,
                fontsize=9,
                color="white",
                bbox=dict(facecolor="black", alpha=0.6, edgecolor="none", pad=3),
            )
        ax_img.axis("off")

        ax_hist.hist(point_errors, bins=30, range=(0, 20), color="#4C78A8", alpha=0.85)
        mean_err = float(np.mean(point_errors)) if len(point_errors) else 0.0
        median_err = float(np.median(point_errors)) if len(point_errors) else 0.0
        ax_hist.set_title(f"Error histogram (0-20px)\nmean={mean_err:.2f}, median={median_err:.2f}")
        ax_hist.set_xlabel("pixel error")
        ax_hist.set_ylabel("count")
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)
    else:
        plt.figure(figsize=(7, 5))
        plt.imshow(img)
        for pts, color, label in points_and_styles:
            plt.scatter(pts[:, 0], pts[:, 1], s=12, c=color, label=label, alpha=0.7)
        plt.title(title)
        plt.legend()
        if stats_label:
            plt.text(
                0.01,
                0.02,
                stats_label,
                transform=plt.gca().transAxes,
                fontsize=9,
                color="white",
                bbox=dict(facecolor="black", alpha=0.6, edgecolor="none", pad=3),
            )
        plt.axis("off")
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        plt.close()


def match_charuco(ids_a, corners_a, ids_b, corners_b):
    ids_a = ids_a.astype(int)
    ids_b = ids_b.astype(int)
    common = np.intersect1d(ids_a, ids_b)
    if len(common) == 0:
        return None, None, None
    idx_a = np.array([np.where(ids_a == cid)[0][0] for cid in common])
    idx_b = np.array([np.where(ids_b == cid)[0][0] for cid in common])
    return common, corners_a[idx_a], corners_b[idx_b]


def triangulate_pair(corners_a, corners_b, K_a, d_a, K_b, d_b, R_ab, t_ab):
    x_a = corners_a.reshape(-1, 1, 2)
    x_b = corners_b.reshape(-1, 1, 2)
    x_a_u = cv2.undistortPoints(x_a, K_a, d_a)
    x_b_u = cv2.undistortPoints(x_b, K_b, d_b)
    P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
    P2 = np.hstack((R_ab, t_ab))
    pts4d = cv2.triangulatePoints(P1, P2, x_a_u.reshape(-1, 2).T, x_b_u.reshape(-1, 2).T)
    pts3d = (pts4d[0:3, :] / pts4d[3, :]).T
    return pts3d


def get_extrinsics(calib: dai.CalibrationHandler, src_socket, dst_socket):
    extrinsics = np.array(calib.getCameraExtrinsics(src_socket, dst_socket))
    R = extrinsics[0:3, 0:3]
    t = extrinsics[0:3, 3].reshape(3, 1)
    return R, t


def find_first_match(folder: str, patterns: List[str]) -> Optional[str]:
    for pattern in patterns:
        matches = [
            os.path.join(folder, name)
            for name in os.listdir(folder)
            if fnmatch.fnmatch(name, pattern)
        ]
        if matches:
            matches.sort()
            return matches[0]
    return None


def resolve_inputs(folder: str):
    if not os.path.isdir(folder):
        raise ValueError(f"Folder not found: {folder}")

    calib_path = find_first_match(folder, ["*calibration*.json", "*calib*.json"])
    if not calib_path:
        raise ValueError("Calibration json not found in folder")

    rgb_path = find_first_match(folder, ["*left*.png", "*left*.png", "*left*.jpg", "*left*.jpg"])
    right_path = find_first_match(folder, ["*right*.png", "*mono_right*.png", "*right*.jpg"])

    if not rgb_path or not os.path.exists(rgb_path):
        raise ValueError("RGB image not found in folder")
    if not right_path or not os.path.exists(right_path):
        raise ValueError("Right image not found in folder")

    return right_path, rgb_path, calib_path


def run(folder: str, right_path: str, rgb_path: str, calib_path: str) -> None:
    rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    if rgb_img is None:
        raise ValueError(f"Failed to read RGB image: {rgb_path}")
    right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)
    if right_img is None:
        raise ValueError(f"Failed to read right image: {right_path}")

    calib = dai.CalibrationHandler(str(calib_path))
    board = cv2.aruco.CharucoBoard_create(
        NUM_SQUARES_X, NUM_SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT
    )

    rgb_h, rgb_w = rgb_img.shape[:2]
    K_rgb, d_rgb = get_intrinsics(calib, LEFT_SOCKET, (rgb_w, rgb_h))
    fx_val = float(K_rgb[0, 0])
    fy_val = float(K_rgb[1, 1])
    cx_val = float(K_rgb[0, 2])
    cy_val = float(K_rgb[1, 2])

    """ HOW TO DO PNP METHOD OF YOU HAVE ONLY ONE CAMER (INTRINSICS ONLY, NO USAGE OF THE EXTRINSICS )"""
    num_rgb, corners_rgb, ids_rgb = detect_charuco(rgb_img, board)
    if num_rgb == 0:
        dbg = rgb_img.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(dbg, cv2.COLOR_BGR2GRAY), ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        show_debug(dbg, "rgb_markers", folder, "debug_rgb_markers.png")
        raise RuntimeError("No RGB charuco corners detected")
    obj_rgb = build_charuco_object_points(board, ids_rgb)
    try:
        rvec_rgb, tvec_rgb = solve_pnp(obj_rgb, corners_rgb, K_rgb, d_rgb, label="(RGB)")
    except Exception as exc:
        dbg = rgb_img.copy()
        draw_points(dbg, corners_rgb, color=(0, 255, 0))
        show_debug(dbg, "rgb_charuco", folder, "debug_rgb_charuco.png")
        raise
    proj_rgb = project_points(obj_rgb, K_rgb, d_rgb, rvec_rgb, tvec_rgb)
    err_rgb = proj_rgb - corners_rgb
    err_rgb_norm = np.linalg.norm(err_rgb, axis=1)

    overlay_path = os.path.join(folder, "rgb_pnp_overlay.png")
    stats_label = f"fx={fx_val:.1f} fy={fy_val:.1f} cx={cx_val:.1f} cy={cy_val:.1f}"
    plot_overlay_multi(
        rgb_img,
        [(corners_rgb, "lime", "detected"), (proj_rgb, "red", "projected")],
        "RGB PnP reprojection",
        overlay_path,
        stats_label,
        point_errors=err_rgb_norm,
    )
    log(f"[rgb] overlay saved -> {overlay_path}")


    """ HOW TO DO PNP METHOD OF YOU HAVE TWO CAMERAS (USAGE OF THE EXTRINSICS )"""

    right_h, right_w = right_img.shape[:2]
    K_right, d_right = get_intrinsics(calib, RIGHT_SOCKET, (right_w, right_h))
    num_right, corners_right, ids_right = detect_charuco(right_img, board)
    if num_right == 0:
        dbg = right_img.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(dbg, cv2.COLOR_BGR2GRAY), ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        show_debug(dbg, "right_markers", folder, "debug_right_markers.png")
        raise RuntimeError("No right charuco corners detected")

    matched_ids, corners_right_m, corners_rgb_m = match_charuco(ids_right, corners_right, ids_rgb, corners_rgb)
    if matched_ids is None:
        raise RuntimeError("No matching charuco IDs between right and RGB")

    R_rrgb, t_rrgb = get_extrinsics(calib, RIGHT_SOCKET, LEFT_SOCKET)
    tri_rrgb = triangulate_pair(corners_right_m, corners_rgb_m, K_right, d_right, K_rgb, d_rgb, R_rrgb, t_rrgb)
    proj_tri_rgb = project_points(tri_rrgb, K_rgb, d_rgb, cv2.Rodrigues(R_rrgb)[0], t_rrgb)
    tri_err = proj_tri_rgb - corners_rgb_m
    tri_err_norm = np.linalg.norm(tri_err, axis=1)

    tri_overlay_path = os.path.join(folder, "rgb_tri_rrgb_overlay.png")
    plot_overlay_multi(
        rgb_img,
        [(corners_rgb_m, "lime", "detected"), (proj_tri_rgb, "red", "triangulated")],
        "RGB reprojection of triangulated R-RGB points",
        tri_overlay_path,
        "",
        point_errors=tri_err_norm,
    )
    log(f"[tri] overlay saved -> {tri_overlay_path}")

    print("Saved plots:")
    print(overlay_path)
    print(tri_overlay_path)


def main():
    parser = argparse.ArgumentParser(description="Local Bosch investigation (PnP + R-RGB triangulation).")
    parser.add_argument("--folder", required=True, help="Folder containing images and calibration.json")
    parser.add_argument("--show", action="store_true", help="Show plots in a GUI window")
    parser.add_argument("--prints", action="store_true", help="Enable detailed logging")
    parser.add_argument("--debug-show", action="store_true", help="Show debug cv2 windows (or save debug images)")
    args = parser.parse_args()

    global PRINTS
    PRINTS = bool(args.prints)
    global DEBUG_SHOW
    DEBUG_SHOW = bool(args.debug_show)

    if args.show and os.environ.get("DISPLAY"):
        pass
    else:
        if args.show and not os.environ.get("DISPLAY"):
            log("[warn] DISPLAY not set; forcing non-interactive Agg backend.")
        matplotlib.use("Agg")
    global plt
    import matplotlib.pyplot as plt

    right_path, rgb_path, calib_path = resolve_inputs(args.folder)
    print(f"Using paths: {right_path}")
    print(f"Using paths: {rgb_path}")
    print(f"Using paths: {calib_path}")
    run(args.folder, right_path, rgb_path, calib_path)


if __name__ == "__main__":
    main()
