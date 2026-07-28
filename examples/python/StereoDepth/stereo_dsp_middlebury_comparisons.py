import argparse
import csv
import datetime
import io
import json
import time
import zipfile
from pathlib import Path
from urllib.request import urlopen

import cv2
import depthai as dai
import numpy as np


DISPARITIES = 64
FRACTIONAL_BITS = 4
Q4_SCALE = 1 << FRACTIONAL_BITS
POSTPROCESS_NORMALIZATION = 1 << 13
POSTPROCESS_SCALE = POSTPROCESS_NORMALIZATION // ((DISPARITIES - 1) * Q4_SCALE)
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATASET = SCRIPT_DIR / "middlebury2014"
DEFAULT_OUTPUT = SCRIPT_DIR / "stereo_dsp_gpu_results"
DEFAULT_SCENES = ("Adirondack-perfect", "Motorcycle-perfect", "Pipes-perfect")
MIDDLEBURY_URL = "https://vision.middlebury.edu/stereo/data/scenes2014/zip/"
EVA_SIZE = (1280, 800)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", nargs="?", type=Path, help="Middlebury scene directory or a directory containing scenes")
    parser.add_argument("--scenes", nargs="+", help="Scene directory names to process")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=400)
    parser.add_argument("--p1", type=int, default=8)
    parser.add_argument("--p2", type=int, default=32)
    parser.add_argument("--uniqueness-ratio", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--display", action="store_true")
    return parser.parse_args()


def prepare_default_dataset(dataset):
    dataset.mkdir(parents=True, exist_ok=True)
    for scene in DEFAULT_SCENES:
        scene_path = dataset / scene
        if (scene_path / "im0.png").is_file() and (scene_path / "im1.png").is_file():
            continue
        archive_name = f"{scene}.zip"
        print(f"Downloading {archive_name}")
        with urlopen(MIDDLEBURY_URL + archive_name, timeout=120) as response:
            archive_data = response.read()
        with zipfile.ZipFile(io.BytesIO(archive_data)) as archive:
            archive.extractall(dataset)


def find_scenes(dataset, requested):
    if (dataset / "im0.png").is_file() and (dataset / "im1.png").is_file():
        scenes = [dataset]
    else:
        scenes = sorted(path for path in dataset.iterdir() if (path / "im0.png").is_file() and (path / "im1.png").is_file())
    if requested:
        by_name = {path.name: path for path in scenes}
        missing = [name for name in requested if name not in by_name]
        if missing:
            raise FileNotFoundError(f"Middlebury scenes not found: {', '.join(missing)}")
        scenes = [by_name[name] for name in requested]
    if not scenes:
        raise FileNotFoundError(f"No Middlebury scenes containing im0.png and im1.png found under {dataset}")
    return scenes


def load_pair(scene, size):
    left = cv2.imread(str(scene / "im0.png"), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(scene / "im1.png"), cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        raise RuntimeError(f"Failed to load stereo pair from {scene}")
    if left.shape != right.shape:
        raise RuntimeError(f"Input dimensions differ in {scene}: {left.shape} and {right.shape}")
    return cv2.resize(left, size, interpolation=cv2.INTER_AREA), cv2.resize(right, size, interpolation=cv2.INTER_AREA)


def confidence_threshold(uniqueness_ratio):
    threshold = (uniqueness_ratio * 255 + 99) // 100
    if threshold * 100 // 255 != uniqueness_ratio:
        raise ValueError(f"Uniqueness ratio {uniqueness_ratio} cannot be represented by the DSP_GPU confidence threshold")
    return threshold


def create_queues(stereo):
    return (
        stereo.left.createInputQueue(),
        stereo.right.createInputQueue(),
        stereo.disparity.createOutputQueue(),
        stereo.rectifiedLeft.createOutputQueue(),
        stereo.rectifiedRight.createOutputQueue(),
    )


def create_dsp_pipeline(width, height, p1, p2, uniqueness_ratio):
    pipeline = dai.Pipeline()
    stereo = pipeline.create(dai.node.StereoDepth)
    queues = create_queues(stereo)

    stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_GPU)
    stereo.setInputResolution(width, height)
    stereo.setRectification(False)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

    config = stereo.initialConfig
    config.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    config.costAggregation.p1Config.enableAdaptive = False
    config.costAggregation.p2Config.enableAdaptive = False
    config.costAggregation.p1Config.defaultValue = p1
    config.costAggregation.p2Config.defaultValue = p2
    config.setConfidenceThreshold(confidence_threshold(uniqueness_ratio))
    config.setLeftRightCheck(True)
    config.setExtendedDisparity(False)
    config.setSubpixel(True)
    config.setSubpixelFractionalBits(FRACTIONAL_BITS)
    config.setMedianFilter(dai.MedianFilter.KERNEL_3x3)
    config.postProcessing.speckleFilter.enable = False
    config.postProcessing.spatialFilter.enable = False
    config.postProcessing.temporalFilter.enable = False
    config.postProcessing.decimationFilter.decimationFactor = 1
    config.postProcessing.holeFilling.enable = False
    config.postProcessing.adaptiveMedianFilter.enable = False

    return (pipeline, *queues)


def create_eva_pipeline():
    pipeline = dai.Pipeline()
    stereo = pipeline.create(dai.node.StereoDepth)
    queues = create_queues(stereo)

    stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.EVA)
    stereo.setInputResolution(*EVA_SIZE)
    stereo.setRectification(False)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    stereo.initialConfig.setSubpixel(True)
    stereo.initialConfig.setSubpixelFractionalBits(FRACTIONAL_BITS)

    return (pipeline, *queues)


def make_frame(image, sequence_num, timestamp_ms, socket):
    frame = dai.ImgFrame()
    frame.setData(image.reshape(-1))
    frame.setTimestamp(datetime.timedelta(milliseconds=timestamp_ms))
    frame.setSequenceNum(sequence_num)
    frame.setInstanceNum(socket)
    frame.setType(dai.ImgFrame.Type.RAW8)
    frame.setWidth(image.shape[1])
    frame.setStride(image.shape[1])
    frame.setHeight(image.shape[0])
    return frame


def send_pair(left_queue, right_queue, pair, sequence_num):
    timestamp_ms = sequence_num * 50
    left_queue.send(make_frame(pair[0], sequence_num, timestamp_ms, dai.CameraBoardSocket.CAM_B))
    right_queue.send(make_frame(pair[1], sequence_num, timestamp_ms, dai.CameraBoardSocket.CAM_C))


def pad_pair_for_eva(pair):
    height, width = pair[0].shape
    if width > EVA_SIZE[0] or height > EVA_SIZE[1]:
        raise ValueError(f"EVA comparison supports inputs up to {EVA_SIZE[0]}x{EVA_SIZE[1]}")
    left = (EVA_SIZE[0] - width) // 2
    right = EVA_SIZE[0] - width - left
    top = (EVA_SIZE[1] - height) // 2
    bottom = EVA_SIZE[1] - height - top
    padded = tuple(cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_REPLICATE) for image in pair)
    return padded, (top, left, height, width)


def crop_eva(disparity, crop):
    top, left, height, width = crop
    return disparity[top : top + height, left : left + width]


def receive_frames(queue, expected_sequences, timeout):
    received = {}
    deadline = time.monotonic() + timeout
    while expected_sequences - received.keys():
        message = queue.tryGet()
        if message is None:
            if time.monotonic() >= deadline:
                missing = sorted(expected_sequences - received.keys())
                raise TimeoutError(f"Timed out waiting for sequences {missing}")
            time.sleep(0.01)
            continue
        sequence_num = message.getSequenceNum()
        if sequence_num in expected_sequences:
            received[sequence_num] = message.getFrame().copy()
    return received


def run_pipeline(bundle, pairs, timeout):
    pipeline, left_queue, right_queue, disparity_queue, rectified_left_queue, rectified_right_queue = bundle
    expected_sequences = set(range(len(pairs)))
    with pipeline:
        pipeline.start()
        for sequence_num, pair in enumerate(pairs):
            send_pair(left_queue, right_queue, pair, sequence_num)
        send_pair(left_queue, right_queue, pairs[-1], len(pairs))
        disparities = receive_frames(disparity_queue, expected_sequences, timeout)
        rectified_left = receive_frames(rectified_left_queue, expected_sequences, timeout)
        rectified_right = receive_frames(rectified_right_queue, expected_sequences, timeout)
    return disparities, rectified_left, rectified_right


def opencv_disparity(left, right, p1, p2, uniqueness_ratio):
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=DISPARITIES,
        blockSize=1,
        P1=p1,
        P2=p2,
        disp12MaxDiff=1,
        preFilterCap=31,
        uniquenessRatio=uniqueness_ratio,
        speckleWindowSize=0,
        speckleRange=0,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    return matcher.compute(left, right)


def dsp_to_q4(disparity):
    if np.any(disparity % POSTPROCESS_SCALE):
        raise RuntimeError("Post-processed DSP_GPU disparity is not aligned to the expected scale")
    return disparity // POSTPROCESS_SCALE


def compare(dsp, opencv):
    dsp = dsp.astype(np.int32)
    opencv = opencv.astype(np.int32)
    dsp = dsp[:, DISPARITIES:]
    opencv = opencv[:, DISPARITIES:]
    dsp_valid = dsp > 0
    opencv_valid = opencv > 0
    both_valid = dsp_valid & opencv_valid
    validity_disagreement = dsp_valid ^ opencv_valid
    differences = np.abs(dsp[both_valid] - opencv[both_valid])
    canonical_dsp = np.where(dsp_valid, dsp, 0)
    canonical_opencv = np.where(opencv_valid, opencv, 0)
    total = dsp.size
    compared = differences.size

    return {
        "evaluated_pixels": int(total),
        "compared": int(compared),
        "backend_valid_pct": 100.0 * np.count_nonzero(dsp_valid) / total,
        "opencv_valid_pct": 100.0 * np.count_nonzero(opencv_valid) / total,
        "validity_agreement_pct": 100.0 * (total - np.count_nonzero(validity_disagreement)) / total,
        "canonical_exact_pct": 100.0 * np.count_nonzero(canonical_dsp == canonical_opencv) / total,
        "exact_q4_pct": 100.0 * np.count_nonzero(differences == 0) / max(compared, 1),
        "within_1q4_pct": 100.0 * np.count_nonzero(differences <= 1) / max(compared, 1),
        "within_1px_pct": 100.0 * np.count_nonzero(differences <= Q4_SCALE) / max(compared, 1),
        "mae_px": float(np.mean(differences) / Q4_SCALE) if compared else 0.0,
        "rmse_px": float(np.sqrt(np.mean(differences.astype(np.float64) ** 2)) / Q4_SCALE) if compared else 0.0,
        "p95_px": float(np.percentile(differences, 95) / Q4_SCALE) if compared else 0.0,
        "p99_px": float(np.percentile(differences, 99) / Q4_SCALE) if compared else 0.0,
        "max_px": float(np.max(differences) / Q4_SCALE) if compared else 0.0,
        "differences_q4": differences,
    }


def disparity_color(disparity):
    valid = disparity > 0
    normalized = np.clip(disparity.astype(np.float32) * 255.0 / ((DISPARITIES - 1) * Q4_SCALE), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def add_label(image, label):
    labeled = image.copy()
    cv2.rectangle(labeled, (0, 0), (labeled.shape[1], 36), (0, 0, 0), -1)
    cv2.putText(labeled, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return labeled


def difference_panel(backend, opencv, label):
    both_valid = (backend > 0) & (opencv > 0)
    validity_disagreement = (backend > 0) ^ (opencv > 0)
    difference_q4 = np.abs(backend.astype(np.int32) - opencv.astype(np.int32))
    normalized = np.clip(difference_q4 * 255.0 / Q4_SCALE, 0, 255).astype(np.uint8)
    panel = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
    panel[~both_valid] = 0
    panel[validity_disagreement] = (255, 0, 255)
    return add_label(panel, label)


def make_visualization(scene_name, dsp, opencv, eva):
    dsp = dsp[:, DISPARITIES:]
    opencv = opencv[:, DISPARITIES:]
    eva = eva[:, DISPARITIES:]
    panels = (
        add_label(disparity_color(dsp), f"{scene_name}: DSP_GPU"),
        add_label(disparity_color(opencv), "OpenCV SGBM_3WAY"),
        add_label(disparity_color(eva), "EVA"),
        difference_panel(dsp, opencv, "DSP - OpenCV: 0..1 px; magenta = validity"),
        difference_panel(eva, opencv, "EVA - OpenCV: 0..1 px; magenta = validity"),
    )
    return np.hstack(panels)


def aggregate_results(results, backend):
    differences = np.concatenate([result["differences_q4"] for result in results])
    total = sum(result["evaluated_pixels"] for result in results)
    compared = differences.size
    return {
        "backend": backend,
        "scene": "ALL",
        "evaluated_pixels": total,
        "compared": int(compared),
        "backend_valid_pct": sum(result["backend_valid_pct"] * result["evaluated_pixels"] for result in results) / total,
        "opencv_valid_pct": sum(result["opencv_valid_pct"] * result["evaluated_pixels"] for result in results) / total,
        "validity_agreement_pct": sum(result["validity_agreement_pct"] * result["evaluated_pixels"] for result in results) / total,
        "canonical_exact_pct": sum(result["canonical_exact_pct"] * result["evaluated_pixels"] for result in results) / total,
        "exact_q4_pct": 100.0 * np.count_nonzero(differences == 0) / max(compared, 1),
        "within_1q4_pct": 100.0 * np.count_nonzero(differences <= 1) / max(compared, 1),
        "within_1px_pct": 100.0 * np.count_nonzero(differences <= Q4_SCALE) / max(compared, 1),
        "mae_px": float(np.mean(differences) / Q4_SCALE) if compared else 0.0,
        "rmse_px": float(np.sqrt(np.mean(differences.astype(np.float64) ** 2)) / Q4_SCALE) if compared else 0.0,
        "p95_px": float(np.percentile(differences, 95) / Q4_SCALE) if compared else 0.0,
        "p99_px": float(np.percentile(differences, 99) / Q4_SCALE) if compared else 0.0,
        "max_px": float(np.max(differences) / Q4_SCALE) if compared else 0.0,
    }


def printable_result(result):
    return {key: value for key, value in result.items() if key != "differences_q4"}


def main():
    args = parse_args()
    if args.dataset is None:
        args.dataset = DEFAULT_DATASET
        args.scenes = args.scenes or list(DEFAULT_SCENES)
        prepare_default_dataset(args.dataset)
    if args.width <= DISPARITIES or args.width % 128:
        raise ValueError("DSP_GPU input width must exceed 64 and be divisible by 128")
    if args.height <= 0:
        raise ValueError("DSP_GPU input height must be positive")
    if args.p1 < 0 or args.p2 < args.p1 or args.p2 > 255:
        raise ValueError("DSP_GPU requires 0 <= P1 <= P2 <= 255")
    if not 0 <= args.uniqueness_ratio <= 100:
        raise ValueError("Uniqueness ratio must be in [0, 100]")

    scenes = find_scenes(args.dataset, args.scenes)
    pairs = [load_pair(scene, (args.width, args.height)) for scene in scenes]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dsp_disparities, rectified_left, rectified_right = run_pipeline(
        create_dsp_pipeline(args.width, args.height, args.p1, args.p2, args.uniqueness_ratio), pairs, args.timeout
    )
    eva_padded, eva_crops = zip(*(pad_pair_for_eva(pair) for pair in pairs))
    eva_disparities, eva_rectified_left, eva_rectified_right = run_pipeline(create_eva_pipeline(), eva_padded, args.timeout)

    dsp_results = []
    eva_results = []
    overview = []
    for sequence_num, (scene, pair) in enumerate(zip(scenes, pairs)):
        dsp = dsp_to_q4(dsp_disparities[sequence_num])
        reference_pair = (rectified_left[sequence_num], rectified_right[sequence_num])
        if not np.array_equal(pair[0], reference_pair[0]) or not np.array_equal(pair[1], reference_pair[1]):
            raise RuntimeError(f"DSP_GPU changed the input pair for {scene.name} while rectification was disabled")
        padded_pair = eva_padded[sequence_num]
        if not np.array_equal(padded_pair[0], eva_rectified_left[sequence_num]) or not np.array_equal(
            padded_pair[1], eva_rectified_right[sequence_num]
        ):
            raise RuntimeError(f"EVA changed the input pair for {scene.name} while rectification was disabled")
        eva = crop_eva(eva_disparities[sequence_num], eva_crops[sequence_num])
        reference = opencv_disparity(reference_pair[0], reference_pair[1], args.p1, args.p2, args.uniqueness_ratio)

        dsp_result = compare(dsp, reference)
        dsp_result.update({"scene": scene.name, "backend": "DSP_GPU"})
        dsp_results.append(dsp_result)
        eva_result = compare(eva, reference)
        eva_result.update({"scene": scene.name, "backend": "EVA"})
        eva_results.append(eva_result)

        visualization = make_visualization(scene.name, dsp, reference, eva)
        cv2.imwrite(str(args.output_dir / f"{scene.name}_comparison.png"), visualization)
        np.save(args.output_dir / f"{scene.name}_dsp_gpu_q4.npy", dsp)
        np.save(args.output_dir / f"{scene.name}_opencv_q4.npy", reference)
        np.save(args.output_dir / f"{scene.name}_eva_q4.npy", eva)
        preview = cv2.resize(
            visualization, (visualization.shape[1] // 2, visualization.shape[0] // 2), interpolation=cv2.INTER_AREA
        )
        overview.append(preview)
        print(json.dumps(printable_result(dsp_result), sort_keys=True))
        print(json.dumps(printable_result(eva_result), sort_keys=True))
        if args.display:
            cv2.imshow("DSP_GPU vs OpenCV SGBM_3WAY vs EVA", preview)
            cv2.waitKey(0)

    dsp_aggregate = aggregate_results(dsp_results, "DSP_GPU")
    eva_aggregate = aggregate_results(eva_results, "EVA")
    rows = [printable_result(result) for result in dsp_results + eva_results] + [dsp_aggregate, eva_aggregate]
    with (args.output_dir / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    with (args.output_dir / "metrics.json").open("w") as stream:
        json.dump({"settings": vars(args) | {"dataset": str(args.dataset), "output_dir": str(args.output_dir)}, "results": rows}, stream, indent=2)
    cv2.imwrite(str(args.output_dir / "comparison_overview.png"), np.vstack(overview))
    print(json.dumps(dsp_aggregate, sort_keys=True))
    print(json.dumps(eva_aggregate, sort_keys=True))
    print(f"Results written to {args.output_dir}")


if __name__ == "__main__":
    main()
