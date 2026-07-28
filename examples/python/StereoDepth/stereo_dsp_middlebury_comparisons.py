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
GPU_SIZE = (1280, 800)
NEURAL_MODEL = dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM
NEURAL_SIZE = dai.node.NeuralDepth.getInputSize(NEURAL_MODEL)


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


def read_pfm(path):
    with path.open("rb") as stream:
        if stream.readline().rstrip() != b"Pf":
            raise RuntimeError(f"Expected a grayscale PFM file: {path}")
        width, height = map(int, stream.readline().split())
        scale = float(stream.readline())
        endian = "<" if scale < 0 else ">"
        disparity = np.fromfile(stream, endian + "f4")
    if disparity.size != width * height:
        raise RuntimeError(f"Malformed PFM payload: {path}")
    return np.flip(disparity.reshape(height, width), axis=0)


def resize_disparity_pixels(disparity, size):
    _, source_width = disparity.shape
    valid = np.isfinite(disparity) & (disparity > 0)
    values = np.where(valid, disparity, 0).astype(np.float32)
    weights = valid.astype(np.float32)
    resized_values = cv2.resize(values, size, interpolation=cv2.INTER_LINEAR)
    resized_weights = cv2.resize(weights, size, interpolation=cv2.INTER_LINEAR)
    resized = np.divide(resized_values, resized_weights, out=np.zeros_like(resized_values), where=resized_weights > 1e-6)
    resized_valid = cv2.resize(valid.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST).astype(bool)
    resized[~resized_valid] = 0
    return resized * (size[0] / source_width)


def pixels_to_q4(disparity):
    return np.rint(np.clip(disparity, 0, np.iinfo(np.uint16).max / Q4_SCALE) * Q4_SCALE).astype(np.uint16)


def resize_q4(disparity, size):
    return pixels_to_q4(resize_disparity_pixels(disparity.astype(np.float32) / Q4_SCALE, size))


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


def create_gpu_pipeline():
    pipeline = dai.Pipeline()
    calibration = pipeline.getDefaultDevice().readCalibration()
    stereo = pipeline.create(dai.node.GPUStereo)
    stereo.setRectification(False)
    stereo.left.setBlocking(True)
    stereo.right.setBlocking(True)
    stereo.left.setMaxSize(8)
    stereo.right.setMaxSize(8)
    return (
        pipeline,
        stereo.left.createInputQueue(),
        stereo.right.createInputQueue(),
        stereo.disparity.createOutputQueue(),
        calibration,
    )


def create_neural_pipeline():
    pipeline = dai.Pipeline()
    calibration = pipeline.getDefaultDevice().readCalibration()
    neural = pipeline.create(dai.node.NeuralDepth)
    neural.neuralNetwork.setModelFromDeviceZoo(NEURAL_MODEL)
    neural.rectification.setOutputSize(NEURAL_SIZE)
    neural.setRectification(False)
    neural.left.setBlocking(True)
    neural.right.setBlocking(True)
    neural.left.setMaxSize(8)
    neural.right.setMaxSize(8)
    return (
        pipeline,
        neural.left.createInputQueue(),
        neural.right.createInputQueue(),
        neural.disparity.createOutputQueue(),
        calibration,
    )


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


def calibrated_frame(image, sequence_num, timestamp_ms, socket, calibration):
    frame = make_frame(image, sequence_num, timestamp_ms, socket)
    height, width = image.shape
    transformation = dai.ImgTransformation()
    transformation.setSize(width, height)
    transformation.setSourceSize(width, height)
    transformation.setIntrinsicMatrix(calibration.getCameraIntrinsics(socket, width, height))
    extrinsics = dai.Extrinsics()
    extrinsics.toCameraSocket = dai.CameraBoardSocket.CAM_A
    extrinsics.setTransformationMatrix(calibration.getCameraExtrinsics(socket, dai.CameraBoardSocket.CAM_A))
    transformation.setExtrinsics(extrinsics)
    frame.setTransformation(transformation)
    return frame


def send_pair(left_queue, right_queue, pair, sequence_num, calibration=None):
    timestamp_ms = sequence_num * 50
    frame_factory = calibrated_frame if calibration is not None else make_frame
    extra = (calibration,) if calibration is not None else ()
    left_queue.send(frame_factory(pair[0], sequence_num, timestamp_ms, dai.CameraBoardSocket.CAM_B, *extra))
    right_queue.send(frame_factory(pair[1], sequence_num, timestamp_ms, dai.CameraBoardSocket.CAM_C, *extra))


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


def run_calibrated_pipeline(bundle, pairs, timeout):
    pipeline, left_queue, right_queue, disparity_queue, calibration = bundle
    disparities = {}
    with pipeline:
        pipeline.start()
        send_pair(left_queue, right_queue, pairs[0], 0, calibration)
        for sequence_num, pair in enumerate(pairs, 1):
            send_pair(left_queue, right_queue, pair, sequence_num, calibration)
            disparities[sequence_num - 1] = receive_frames(disparity_queue, {sequence_num}, timeout)[sequence_num]
    return disparities


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


def compare(backend, reference):
    backend = backend.astype(np.int32)
    reference = reference.astype(np.int32)
    backend = backend[:, DISPARITIES:]
    reference = reference[:, DISPARITIES:]
    backend_valid = backend > 0
    reference_valid = reference > 0
    both_valid = backend_valid & reference_valid
    validity_disagreement = backend_valid ^ reference_valid
    differences = np.abs(backend[both_valid] - reference[both_valid])
    canonical_backend = np.where(backend_valid, backend, 0)
    canonical_reference = np.where(reference_valid, reference, 0)
    total = backend.size
    compared = differences.size

    return {
        "evaluated_pixels": int(total),
        "compared": int(compared),
        "backend_valid_pct": 100.0 * np.count_nonzero(backend_valid) / total,
        "reference_valid_pct": 100.0 * np.count_nonzero(reference_valid) / total,
        "validity_agreement_pct": 100.0 * (total - np.count_nonzero(validity_disagreement)) / total,
        "canonical_exact_pct": 100.0 * np.count_nonzero(canonical_backend == canonical_reference) / total,
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


def make_visualization(scene_name, ground_truth, backends):
    ground_truth = ground_truth[:, DISPARITIES:]
    cropped = [(name, disparity[:, DISPARITIES:]) for name, disparity in backends]
    top = [add_label(disparity_color(disparity), f"{scene_name}: {name}") for name, disparity in cropped]
    top.append(add_label(disparity_color(ground_truth), f"{scene_name}: Ground truth"))
    bottom = [difference_panel(disparity, ground_truth, f"{name} - GT") for name, disparity in cropped]
    bottom.append(difference_panel(ground_truth, ground_truth, "GT - GT (0..1 px; magenta = validity)"))
    return np.vstack((np.hstack(top), np.hstack(bottom)))


def aggregate_results(results, backend, reference):
    differences = np.concatenate([result["differences_q4"] for result in results])
    total = sum(result["evaluated_pixels"] for result in results)
    compared = differences.size
    return {
        "backend": backend,
        "reference": reference,
        "scene": "ALL",
        "evaluated_pixels": total,
        "compared": int(compared),
        "backend_valid_pct": sum(result["backend_valid_pct"] * result["evaluated_pixels"] for result in results) / total,
        "reference_valid_pct": sum(result["reference_valid_pct"] * result["evaluated_pixels"] for result in results) / total,
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
    target_size = (args.width, args.height)
    pairs = [load_pair(scene, target_size) for scene in scenes]
    gpu_pairs = [load_pair(scene, GPU_SIZE) for scene in scenes]
    native_ground_truth = [read_pfm(scene / "disp0.pfm") for scene in scenes]
    ground_truth = [pixels_to_q4(resize_disparity_pixels(disparity, target_size)) for disparity in native_ground_truth]
    ground_truth_scales = {scene.name: args.width / disparity.shape[1] for scene, disparity in zip(scenes, native_ground_truth)}
    args.output_dir.mkdir(parents=True, exist_ok=True)

    dsp_disparities, rectified_left, rectified_right = run_pipeline(
        create_dsp_pipeline(args.width, args.height, args.p1, args.p2, args.uniqueness_ratio), pairs, args.timeout
    )
    eva_padded, eva_crops = zip(*(pad_pair_for_eva(pair) for pair in pairs))
    eva_disparities, eva_rectified_left, eva_rectified_right = run_pipeline(create_eva_pipeline(), eva_padded, args.timeout)
    gpu_disparities = run_calibrated_pipeline(create_gpu_pipeline(), gpu_pairs, args.timeout)
    neural_disparities = run_calibrated_pipeline(create_neural_pipeline(), pairs, args.timeout)

    backend_order = ("DSP_GPU", "OpenCV SGBM_3WAY", "EVA", "NeuralDepth MEDIUM", "GPUStereo")
    results = {backend: [] for backend in backend_order}
    dsp_opencv_results = []
    overview = []
    for sequence_num, (scene, pair, gt) in enumerate(zip(scenes, pairs, ground_truth)):
        dsp = dsp_to_q4(dsp_disparities[sequence_num])
        reference_pair = (rectified_left[sequence_num], rectified_right[sequence_num])
        if not np.array_equal(pair[0], reference_pair[0]) or not np.array_equal(pair[1], reference_pair[1]):
            raise RuntimeError(f"DSP_GPU changed the input pair for {scene.name} while rectification was disabled")
        padded_pair = eva_padded[sequence_num]
        if not np.array_equal(padded_pair[0], eva_rectified_left[sequence_num]) or not np.array_equal(
            padded_pair[1], eva_rectified_right[sequence_num]
        ):
            raise RuntimeError(f"EVA changed the input pair for {scene.name} while rectification was disabled")

        opencv = opencv_disparity(reference_pair[0], reference_pair[1], args.p1, args.p2, args.uniqueness_ratio)
        eva = crop_eva(eva_disparities[sequence_num], eva_crops[sequence_num])
        neural = resize_q4(neural_disparities[sequence_num], target_size)
        gpu = resize_q4(gpu_disparities[sequence_num], target_size)
        backends = (
            ("DSP_GPU", dsp),
            ("OpenCV SGBM_3WAY", opencv),
            ("EVA", eva),
            ("NeuralDepth MEDIUM", neural),
            ("GPUStereo", gpu),
        )

        for backend, disparity in backends:
            result = compare(disparity, gt)
            result.update({"scene": scene.name, "backend": backend, "reference": "Ground truth"})
            results[backend].append(result)
            print(json.dumps(printable_result(result), sort_keys=True))
        dsp_opencv = compare(dsp, opencv)
        dsp_opencv.update({"scene": scene.name, "backend": "DSP_GPU", "reference": "OpenCV SGBM_3WAY"})
        dsp_opencv_results.append(dsp_opencv)

        visualization = make_visualization(scene.name, gt, backends)
        cv2.imwrite(str(args.output_dir / f"{scene.name}_comparison.png"), visualization)
        for backend, disparity in backends:
            filename = backend.lower().replace(" ", "_")
            np.save(args.output_dir / f"{scene.name}_{filename}_q4.npy", disparity)
        np.save(args.output_dir / f"{scene.name}_ground_truth_q4.npy", gt)
        preview = cv2.resize(
            visualization, (visualization.shape[1] // 2, visualization.shape[0] // 2), interpolation=cv2.INTER_AREA
        )
        overview.append(preview)
        if args.display:
            cv2.imshow("Stereo backends and absolute differences to Middlebury ground truth", preview)
            cv2.waitKey(0)

    scene_rows = [printable_result(result) for backend in backend_order for result in results[backend]]
    scene_rows.extend(printable_result(result) for result in dsp_opencv_results)
    aggregates = [aggregate_results(results[backend], backend, "Ground truth") for backend in backend_order]
    aggregates.append(aggregate_results(dsp_opencv_results, "DSP_GPU", "OpenCV SGBM_3WAY"))
    rows = scene_rows + aggregates
    with (args.output_dir / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    with (args.output_dir / "metrics.json").open("w") as stream:
        settings = vars(args) | {
            "dataset": str(args.dataset),
            "output_dir": str(args.output_dir),
            "gpu_input_size": GPU_SIZE,
            "neural_model": "NEURAL_DEPTH_MEDIUM",
            "neural_input_size": NEURAL_SIZE,
            "scaling": {
                "target_size": target_size,
                "gpu_disparity_multiplier": args.width / GPU_SIZE[0],
                "neural_disparity_multiplier": args.width / NEURAL_SIZE[0],
                "ground_truth_disparity_multipliers": ground_truth_scales,
                "spatial_interpolation": "validity-weighted bilinear with nearest-neighbor validity mask",
                "output_units": "Q4 disparity at target_size",
            },
        }
        json.dump({"settings": settings, "results": rows}, stream, indent=2)
    cv2.imwrite(str(args.output_dir / "comparison_overview.png"), np.vstack(overview))
    for aggregate in aggregates:
        print(json.dumps(aggregate, sort_keys=True))
    print(f"Results written to {args.output_dir}")


if __name__ == "__main__":
    main()
