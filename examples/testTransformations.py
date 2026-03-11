import json

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
WARNED_MISSING_SET_EXTRINSICS = False


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

def _point3f_from_json(data):
    data = data or {}
    return dai.Point3f(float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("z", 0.0)))

def _socket_from_json(value) -> dai.CameraBoardSocket:
    if isinstance(value, dai.CameraBoardSocket):
        return value
    if isinstance(value, str):
        if hasattr(dai.CameraBoardSocket, value):
            return getattr(dai.CameraBoardSocket, value)
        try:
            value = int(value)
        except ValueError:
            return dai.CameraBoardSocket.AUTO
    try:
        return dai.CameraBoardSocket(int(value))
    except Exception:
        return dai.CameraBoardSocket.AUTO

def get_transformation(calib_path: str, socket: dai.CameraBoardSocket) -> dai.ImgTransformation:

    with open(calib_path, "r") as f:
        calib_data = json.load(f)

    camera_data_blob = calib_data.get("cameraData")
    if camera_data_blob is None:
        raise ValueError("Missing 'cameraData' in calibration JSON")

    socket_id = int(socket.value)
    camera_data = None

    if isinstance(camera_data_blob, dict):
        camera_data = camera_data_blob.get(str(socket_id), camera_data_blob.get(socket_id))
    elif isinstance(camera_data_blob, list):
        pair_entries = [entry for entry in camera_data_blob if isinstance(entry, (list, tuple)) and len(entry) == 2]
        if pair_entries:
            # DepthAI calibration files commonly encode cameraData as [[socket, cameraInfo], ...].
            for entry in pair_entries:
                try:
                    if int(entry[0]) == socket_id:
                        camera_data = entry[1]
                        break
                except (TypeError, ValueError):
                    continue
        # Fallback for non-standard direct list layouts indexed by socket value.
        elif 0 <= socket_id < len(camera_data_blob):
            maybe = camera_data_blob[socket_id]
            if isinstance(maybe, dict):
                camera_data = maybe

    if camera_data is None:
        raise ValueError(f"No calibration entry found for socket {socket}")

    width = int(camera_data.get("width", calib_data.get("width", 0)))
    height = int(camera_data.get("height", calib_data.get("height", 0)))
    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid frame size for socket {socket}: width={width}, height={height}")

    intrinsic_matrix = camera_data.get("intrinsicMatrix")
    if intrinsic_matrix is None:
        raise ValueError(f"Missing intrinsicMatrix for socket {socket}")

    img_transformation = dai.ImgTransformation(width, height)
    img_transformation.setIntrinsicMatrix(intrinsic_matrix)

    camera_type = camera_data.get("cameraType")
    if camera_type is not None:
        if isinstance(camera_type, dai.CameraModel):
            img_transformation.setDistortionModel(camera_type)
        else:
            img_transformation.setDistortionModel(dai.CameraModel(int(camera_type)))

    img_transformation.setDistortionCoefficients(camera_data.get("distortionCoeff", []))

    extrinsics_json = camera_data.get("extrinsics")
    if extrinsics_json:
        extrinsics = dai.Extrinsics()
        rotation_matrix = extrinsics_json.get("rotationMatrix")
        if rotation_matrix is not None:
            extrinsics.rotationMatrix = rotation_matrix
        extrinsics.translation = _point3f_from_json(extrinsics_json.get("translation", extrinsics_json.get("translationVector")))
        extrinsics.specTranslation = _point3f_from_json(extrinsics_json.get("specTranslation"))
        if hasattr(extrinsics, "toCameraSocket"):
            extrinsics.toCameraSocket = _socket_from_json(extrinsics_json.get("toCameraSocket", dai.CameraBoardSocket.AUTO.value))
        if hasattr(img_transformation, "setExtrinsics"):
            img_transformation.setExtrinsics(extrinsics)
        else:
            global WARNED_MISSING_SET_EXTRINSICS
            if not WARNED_MISSING_SET_EXTRINSICS:
                print(
                    "[debug] ImgTransformation.setExtrinsics is unavailable in this depthai build. "
                    "Inter-camera remapPointTo may be inaccurate without extrinsics."
                )
                WARNED_MISSING_SET_EXTRINSICS = True

    return img_transformation


def run(folder: str, from_path: str, to_path: str, calib_path: str, from_socket: dai.CameraBoardSocket, to_socket: dai.CameraBoardSocket) -> None:
    """Detect Charuco in from image and remap all points into to image space using ImgTransformation.remapPointTo."""
    to_img = cv2.imread(to_path, cv2.IMREAD_COLOR)
    if to_img is None:
        raise ValueError(f"Failed to read to image: {to_path}")
    from_img = cv2.imread(from_path, cv2.IMREAD_COLOR)
    if from_img is None:
        raise ValueError(f"Failed to read from image: {from_path}")

    from_transformation = get_transformation(calib_path, from_socket)
    to_transformation = get_transformation(calib_path, to_socket)

    board = cv2.aruco.CharucoBoard_create(
        NUM_SQUARES_X, NUM_SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT
    )

    num_from, corners_from, ids_from = detect_charuco(from_img, board)
    if num_from == 0:
        dbg = from_img.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(dbg, cv2.COLOR_BGR2GRAY), ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        show_debug(dbg, "from_markers", folder, "debug_from_markers.png")
        raise RuntimeError("No Charuco corners detected in from image")

    num_to, corners_to, ids_to = detect_charuco(to_img, board)
    if num_to == 0:
        dbg = to_img.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(dbg, cv2.COLOR_BGR2GRAY), ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
        show_debug(dbg, "to_markers", folder, "debug_to_markers.png")

    remapped_points = []
    for x, y in corners_from:
        src_point = dai.Point2f(float(x), float(y))
        dst_point = from_transformation.remapPointTo(to_transformation, src_point)
        remapped_points.append([dst_point.x, dst_point.y])
    remapped_points = np.asarray(remapped_points, dtype=np.float32)

    to_h, to_w = to_img.shape[:2]
    out_of_bounds_mask = (
        (remapped_points[:, 0] < 0)
        | (remapped_points[:, 0] >= to_w)
        | (remapped_points[:, 1] < 0)
        | (remapped_points[:, 1] >= to_h)
    )
    out_of_bounds_count = int(np.count_nonzero(out_of_bounds_mask))
    print(
        f"[debug] from corners={len(corners_from)} ids={len(ids_from)} | "
        f"to corners={0 if corners_to is None else len(corners_to)} ids={0 if ids_to is None else len(ids_to)}"
    )
    print(
        f"[debug] remapped x:[{float(np.min(remapped_points[:, 0])):.2f}, {float(np.max(remapped_points[:, 0])):.2f}] "
        f"y:[{float(np.min(remapped_points[:, 1])):.2f}, {float(np.max(remapped_points[:, 1])):.2f}] "
        f"out_of_bounds={out_of_bounds_count}/{len(remapped_points)} "
        f"(to frame size: {to_w}x{to_h})"
    )

    if ids_from is not None:
        print(f"[debug] from ids sample: {ids_from[:20].tolist()}")
    if ids_to is not None:
        print(f"[debug] to ids sample: {ids_to[:20].tolist()}")

    if ids_to is not None and corners_to is not None:
        matched_ids, remapped_m, corners_to_m = match_charuco(ids_from, remapped_points, ids_to, corners_to)
        if matched_ids is None:
            print("[debug] no matching ChArUco IDs between from-image remap and to-image detection")
        else:
            errors = np.linalg.norm(remapped_m - corners_to_m, axis=1)
            delta = corners_to_m - remapped_m
            print(
                f"[debug] matched={len(matched_ids)} "
                f"mean_err={float(np.mean(errors)):.3f}px "
                f"median_err={float(np.median(errors)):.3f}px "
                f"max_err={float(np.max(errors)):.3f}px"
            )
            print(
                f"[debug] delta(to-remap) mean=({float(np.mean(delta[:, 0])):.3f}, {float(np.mean(delta[:, 1])):.3f}) px "
                f"median=({float(np.median(delta[:, 0])):.3f}, {float(np.median(delta[:, 1])):.3f}) px "
                f"std=({float(np.std(delta[:, 0])):.3f}, {float(np.std(delta[:, 1])):.3f}) px"
            )
            worst_idx = np.argsort(errors)[-10:][::-1]
            for idx in worst_idx:
                rid = int(matched_ids[idx])
                src = remapped_m[idx]
                dst = corners_to_m[idx]
                err = float(errors[idx])
                print(
                    f"[debug] id={rid:4d} remap=({src[0]:8.2f},{src[1]:8.2f}) "
                    f"to_det=({dst[0]:8.2f},{dst[1]:8.2f}) err={err:7.3f}px"
                )

    overlay = to_img.copy()
    draw_points(overlay, corners_from, color=(0, 0, 255))  # red = original detections in from image
    draw_points(overlay, remapped_points, color=(0, 255, 0))  # green = remapped points in to-image space
    draw_points(overlay, corners_to, color=(255, 0, 0))  # blue = detected corners in to image for debug
    log(f"[remap] remapped {len(remapped_points)} points from from image to to-image space")

    output_path = os.path.join(folder, "charuco_remap_from_to.png")
    cv2.imwrite(output_path, overlay)
    log(f"[remap] overlay saved -> {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Remap Charuco points from one image transformation space to another.")
    parser.add_argument("--folder", required=True, help="Folder containing images and calibration.json")
    parser.add_argument("--show", action="store_true", help="Show plots in a GUI window")
    parser.add_argument("--prints", action="store_true", help="Enable detailed logging")
    parser.add_argument("--debug-show", action="store_true", help="Show debug cv2 windows (or save debug images)")
    parser.add_argument("--from-socket", default="CAM_C", help="Source camera socket (e.g. CAM_A, CAM_B, CAM_C)")
    parser.add_argument("--to-socket", default="CAM_B", help="Target camera socket (e.g. CAM_A, CAM_B, CAM_C)")
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



    from_socket = getattr(dai.CameraBoardSocket, args.from_socket, None)
    to_socket = getattr(dai.CameraBoardSocket, args.to_socket, None)
    if from_socket is None or to_socket is None:
        raise ValueError("Invalid socket name. Use values like CAM_A, CAM_B, CAM_C.")

    from_path, to_path, calib_path = resolve_inputs(args.folder)
    print(f"Using from image: {from_path}")
    print(f"Using to image: {to_path}")
    print(f"Using calibration: {calib_path}")
    print(f"Using sockets: from={from_socket}, to={to_socket}")
    run(args.folder, from_path, to_path, calib_path, from_socket, to_socket)


if __name__ == "__main__":
    main()
