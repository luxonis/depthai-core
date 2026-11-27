"""Extended standalone OpticalFlow acceptance. Requires an idle RVC4.

Each invocation tests one geometry, so a device error cannot hide other matrix
results. End-to-end timing includes XLink transfer; it is not EVA kernel time.
No application is stopped or installed by this test.
"""
import argparse
import json
import time
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


def frame_message(image, sequence, kind):
    msg = dai.ImgFrame()
    msg.setCvFrame(image, kind)
    msg.setSequenceNum(sequence)
    stamp = timedelta(microseconds=1_000_000 + sequence * 33_333)
    msg.setTimestamp(stamp)
    msg.setTimestampDevice(stamp)
    return msg


def check_group(group, width, height, previous, current):
    assert group is not None, 'Timed out waiting for native flow'
    tensor = group['flow']
    arrays = {name: tensor.getTensor(name) for name in ('forward', 'backward', 'confidence')}
    for name in ('forward', 'backward'):
        assert arrays[name].shape == (height, width, 2)
        assert arrays[name].dtype == np.float32
        assert np.isfinite(arrays[name]).all()
    assert arrays['confidence'].shape == (height, width, 1)
    # Python getTensor() converts numeric tensor types to FP32 by default.
    assert tensor.getTensorInfo('confidence').dataType == dai.TensorInfo.DataType.U8F
    assert tensor.getTensorInfo('forward').dataType == dai.TensorInfo.DataType.FP32
    assert tensor.getTensorInfo('backward').dataType == dai.TensorInfo.DataType.FP32
    for packet in (group, tensor, group['target']):
        assert packet.getSequenceNum() == current.getSequenceNum()
        assert packet.getTimestamp() == current.getTimestamp()
        assert packet.getTimestampDevice() == current.getTimestampDevice()
    reference = group['reference']
    assert reference.getSequenceNum() == previous.getSequenceNum()
    assert reference.getTimestamp() == previous.getTimestamp()
    assert reference.getTimestampDevice() == previous.getTimestampDevice()
    assert len(reference.getData()) == len(group['target'].getData()) == 0
    return arrays


def error_stats(array, expected, roi, quality):
    vectors = array[roi]
    epe = np.linalg.norm(vectors - np.asarray(expected), axis=-1)
    valid = quality[roi] >= 100
    return {
        'expected': np.asarray(expected).tolist(), 'median_vector': np.median(vectors.reshape(-1, 2), axis=0).tolist(),
        'median_epe_px': float(np.median(epe)), 'p95_epe_px': float(np.percentile(epe, 95)),
        'fraction_within_1_5px': float(np.mean(epe < 1.5)),
        'quality_ge_100_fraction': float(valid.mean()),
        'quality_ge_100_median_epe_px': float(np.median(epe[valid])) if valid.any() else None,
    }


def synthetic(device, args):
    w, h = args.width, args.height
    base = cv2.GaussianBlur(np.random.default_rng(args.seed).integers(0, 256, (h, w), np.uint8), (3, 3), .6)
    margin = min(48, max(4, h // 8), max(4, w // 8))
    roi = (slice(margin, h-margin), slice(margin, w-margin))
    kind = getattr(dai.ImgFrame.Type, args.format)
    rows, durations, accuracy_failures = [], [], []
    with dai.Pipeline(device) as pipeline:
        node = pipeline.create(dai.node.OpticalFlow)
        incoming = node.inputImage.createInputQueue(maxSize=2, blocking=True)
        passed = node.passthroughInputImage.createOutputQueue(maxSize=2, blocking=True)
        output = node.outputFlow.createOutputQueue(maxSize=2, blocking=True)
        pipeline.start()
        previous, held = None, None
        index = 0

        def send(image, label, regions=()):
            nonlocal previous, held, index
            # Deliberate source gaps exercise actual consumed-pair metadata.
            current = frame_message(image, 10 + 3 * index, kind)
            index += 1
            started = time.perf_counter()
            incoming.send(current)
            passthrough = passed.get(timedelta(seconds=45))
            assert passthrough is not None, 'No passthrough'
            assert passthrough.getSequenceNum() == current.getSequenceNum()
            assert np.array_equal(passthrough.getCvFrame(), image), 'Input pixels changed'
            if previous is None:
                assert output.get(timedelta(milliseconds=150)) is None, 'Fake first-frame flow'
            else:
                group = output.get(timedelta(seconds=45))
                arrays = check_group(group, w, h, previous, current)
                durations.append(time.perf_counter() - started)
                if held is not None:
                    for name, saved in held[1].items():
                        assert np.array_equal(held[0].getTensor(name), saved), 'Retained buffer changed: ' + name
                if held is None:
                    held = (group['flow'], {name: value.copy() for name, value in arrays.items()})
                row = {'case': label, 'reference_sequence': previous.getSequenceNum(),
                       'target_sequence': current.getSequenceNum(), 'roundtrip_s': durations[-1], 'regions': []}
                for label_roi, region, motion in regions:
                    stats = {'region': label_roi}
                    for direction, sign in (('forward', 1), ('backward', -1)):
                        stats[direction] = error_stats(arrays[direction], np.asarray(motion)*sign,
                                                       region, arrays['confidence'][..., 0])
                    row['regions'].append(stats)
                rows.append(row)
                # Save summary before assertions so failures retain measured evidence.
                (args.output / 'pairs.json').write_text(json.dumps(rows, indent=2))
                for stats in row['regions']:
                    for direction in ('forward', 'backward'):
                        if stats[direction]['median_epe_px'] >= 1.5:
                            accuracy_failures.append(f'{label} {stats["region"]} {direction}: {stats[direction]}')
            previous = current

        send(base, 'first')
        send(base, 'identity', [('interior', roi, (0, 0))])
        for dx, dy in ((4, 0), (0, 4), (-4, 3), (8, -6), (16, 0)):
            moved = cv2.warpAffine(base, np.float32([[1, 0, dx], [0, 1, dy]]), (w, h), borderMode=cv2.BORDER_REFLECT)
            send(moved, f'translate_{dx}_{dy}', [('interior', roi, (dx, dy))])
            send(base, f'reverse_{dx}_{dy}', [('interior', roi, (-dx, -dy))])
        if w >= 640 and h >= 384:
            moved = base.copy()
            regions = []
            for x0, y0, x1, y1, dx, dy in ((w//5, h//5, 2*w//5, 4*h//5, 6, 0),
                                           (3*w//5, h//5, 4*w//5, 4*h//5, -4, 4)):
                moved[y0:y1, x0:x1] = base[y0-dy:y1-dy, x0-dx:x1-dx]
                regions.append((f'object_{dx}_{dy}', (slice(y0+24, y1-24), slice(x0+24, x1-24)), (dx, dy)))
            regions.append(('stationary_background', (slice(32, h-32), slice(24, w//5-24)), (0, 0)))
            send(moved, 'two_independent_objects', regions)
        # Textureless input is not a reliable-motion oracle, but must remain finite.
        send(np.full_like(base, 127), 'textureless_transition')
        send(np.full_like(base, 127), 'textureless_identity')
    assert not accuracy_failures, '\n'.join(accuracy_failures)
    return {'pairs': rows, 'retained_first_packet_pairs': len(rows),
            'median_roundtrip_s': float(np.median(durations)),
            'timing_scope': 'sequential host input + native compute + full tensor/passthrough transfer'}


def live(device, args):
    rows = []
    with dai.Pipeline(device) as pipeline:
        camera = pipeline.create(dai.node.Camera).build(getattr(dai.CameraBoardSocket, args.socket))
        node = pipeline.create(dai.node.OpticalFlow)
        camera.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.GRAY8, fps=30).link(node.inputImage)
        if args.mode == 'drop':
            node.inputImage.setBlocking(False)
            node.inputImage.setMaxSize(1)
        queue = node.outputFlow.createOutputQueue(maxSize=2, blocking=True)
        pipeline.start()
        held = None
        previous_target = None
        for i in range(args.frames + 5):
            if args.mode == 'drop':
                time.sleep(.12)
            group = queue.get(timedelta(seconds=45))
            assert group is not None
            arrays = check_group(group, args.width, args.height, group['reference'], group['target'])
            ref, target = group['reference'], group['target']
            assert target.getSequenceNum() > ref.getSequenceNum()
            assert target.getTimestampDevice() > ref.getTimestampDevice()
            if previous_target is not None:
                assert ref.getSequenceNum() == previous_target, 'Consumed-pair continuity broke'
            previous_target = target.getSequenceNum()
            if held is not None:
                for name, saved in held[1].items():
                    assert np.array_equal(held[0].getTensor(name), saved), 'Retained live tensor changed'
            else:
                held = (group['flow'], {name: array.copy() for name, array in arrays.items()})
            if i >= 5:
                rows.append({'received_s': time.perf_counter(), 'reference': ref.getSequenceNum(),
                             'target': target.getSequenceNum(), 'pair_interval_s':
                             (target.getTimestampDevice()-ref.getTimestampDevice()).total_seconds()})
    gaps = sum(row['target']-row['reference']-1 for row in rows)
    if args.mode == 'drop':
        assert gaps > 0, 'Drop test did not exercise a skipped input'
    elapsed = rows[-1]['received_s']-rows[0]['received_s']
    return {'pairs': rows, 'fps': (len(rows)-1)/elapsed, 'skipped_source_frames': gaps,
            'tensor_bytes_per_pair': args.width*args.height*17,
            'timing_scope': 'camera + native compute + full flow transfer to host, excluding five warmup pairs'}


def bypass(device, args):
    with dai.Pipeline(device) as pipeline:
        node = pipeline.create(dai.node.OpticalFlow)
        incoming = node.inputImage.createInputQueue()
        passed = node.passthroughInputImage.createOutputQueue()
        pipeline.start()
        for sequence in range(5):
            pixels = np.full((args.height, args.width), sequence, np.uint8)
            incoming.send(frame_message(pixels, sequence, dai.ImgFrame.Type.GRAY8))
            msg = passed.get(timedelta(seconds=10))
            assert msg is not None and msg.getSequenceNum() == sequence
            assert np.array_equal(msg.getCvFrame(), pixels)
    return {'passthrough_frames': 5, 'flow_output_connected': False}


def reject(device, args):
    messages = []
    callback = device.addLogCallback(lambda message: messages.append(message.payload))
    expected = {
        'height': 'height 128..1056', 'width': 'width 128..1920',
        'stride': 'packed input rows', 'payload': 'payload must contain',
        'type': 'requires GRAY8 or RAW8', 'message': 'requires ImgFrame messages',
        'change': 'dimensions and format must remain constant',
    }[args.invalid]
    try:
        with dai.Pipeline(device) as pipeline:
            node = pipeline.create(dai.node.OpticalFlow)
            incoming = node.inputImage.createInputQueue()
            passed = node.passthroughInputImage.createOutputQueue()
            flow = node.outputFlow.createOutputQueue()
            control = pipeline.create(dai.node.OpticalFlow)
            control_input = control.inputImage.createInputQueue()
            control_output = control.passthroughInputImage.createOutputQueue()
            pipeline.start()
            msg = frame_message(np.zeros((args.height, args.width), np.uint8), 10, dai.ImgFrame.Type.GRAY8)
            if args.invalid == 'height':
                msg.setHeight(32)
            elif args.invalid == 'width':
                msg.setWidth(639)
            elif args.invalid == 'stride':
                msg.setStride(args.width+128)
            elif args.invalid == 'payload':
                # XLink's GRAY8 alignment stage expands short input buffers;
                # RAW8 preserves the truncated payload for the node's guard.
                msg.setType(dai.ImgFrame.Type.RAW8)
                msg.setData(np.zeros(16, np.uint8))
                assert len(msg.getData()) == 16
            elif args.invalid == 'type':
                msg.setType(dai.ImgFrame.Type.NV12)
            elif args.invalid == 'message':
                msg = dai.Buffer()
            elif args.invalid == 'change':
                incoming.send(msg)
                assert passed.get(timedelta(seconds=10)) is not None
                msg = frame_message(np.zeros((args.height, args.width+128), np.uint8), 11, dai.ImgFrame.Type.GRAY8)
            incoming.send(msg)
            deadline = time.monotonic()+10
            while time.monotonic() < deadline and not any(expected in line for line in messages):
                time.sleep(.05)
            assert any(expected in line for line in messages), f'Missing expected validation error {expected!r}: {messages}'
            assert flow.tryGet() is None, 'Invalid frame produced a flow packet'
            # An independent node in this still-running pipeline must stay alive.
            pixels = np.full((args.height, args.width), 42, np.uint8)
            control_input.send(frame_message(pixels, 99, dai.ImgFrame.Type.GRAY8))
            recovered = control_output.get(timedelta(seconds=10))
            assert recovered is not None and recovered.getSequenceNum() == 99
            assert np.array_equal(recovered.getCvFrame(), pixels)
        return {'invalid_case': args.invalid, 'expected_error': expected,
                'observed_errors': [line for line in messages if expected in line],
                'independent_node_alive_after_rejection': True}
    finally:
        device.removeLogCallback(callback)


def shutdown(device, args):
    timings = []
    for index, connected_input in enumerate((False, True, True)):
        # Pipeline.stop() closes assigned devices by contract; reopen for restart.
        round_device = device if index == 0 else dai.Device(dai.DeviceInfo(args.device))
        with dai.Pipeline(round_device) as pipeline:
            node = pipeline.create(dai.node.OpticalFlow)
            queue = node.outputFlow.createOutputQueue(maxSize=1, blocking=True)
            if connected_input:
                camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                camera.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.GRAY8, fps=30).link(node.inputImage)
            else:
                incoming = node.inputImage.createInputQueue()
            pipeline.start()
            time.sleep(1)
            # Intentionally do not consume output: shutdown must unblock sends.
            started = time.perf_counter()
            pipeline.stop()
            pipeline.wait()
            elapsed = time.perf_counter()-started
            assert elapsed < 10, 'Shutdown stalled on a blocked queue'
            timings.append({'input_connected': connected_input, 'stop_s': elapsed})
    return {'shutdown_and_restart': timings}


def tracker(device, args):
    rng = np.random.default_rng(args.seed)
    base = cv2.GaussianBlur(rng.integers(0, 256, (args.height, args.width), np.uint8), (3, 3), .6)
    motions = []
    with dai.Pipeline(device) as pipeline:
        node = pipeline.create(dai.node.FeatureTracker)
        node.initialConfig.setNumTargetFeatures(256)
        estimator = dai.FeatureTrackerConfig.MotionEstimator()
        estimator.enable = True
        node.initialConfig.setMotionEstimator(estimator)
        incoming = node.inputImage.createInputQueue()
        features = node.outputFeatures.createOutputQueue()
        passed = node.passthroughInputImage.createOutputQueue()
        pipeline.start()
        previous = None
        for index, dx in enumerate((0, 0, 4, 8, 4, 0)):
            image = cv2.warpAffine(base, np.float32([[1, 0, dx], [0, 1, 0]]),
                                   (args.width, args.height), borderMode=cv2.BORDER_REFLECT)
            incoming.send(frame_message(image, index, dai.ImgFrame.Type.GRAY8))
            passthrough = passed.get(timedelta(seconds=20))
            tracked = features.get(timedelta(seconds=20))
            assert passthrough is not None and tracked is not None
            assert np.array_equal(passthrough.getCvFrame(), image)
            points = {point.id: np.array([point.position.x, point.position.y]) for point in tracked.trackedFeatures}
            if index == 0 and not points:
                continue  # Initial frame has no preceding optical-flow pair.
            assert len(points) >= 20, 'FeatureTracker no longer finds textured features'
            if previous is not None:
                shared = points.keys() & previous[0].keys()
                assert len(shared) >= 20, 'Feature IDs were not maintained'
                displacement = np.array([points[key]-previous[0][key] for key in shared])
                error = np.linalg.norm(displacement-[dx-previous[1], 0], axis=1)
                assert np.median(error) < 1.5
                motions.append({'shared_features': len(shared), 'median_epe_px': float(np.median(error))})
            previous = (points, dx)
    return {'feature_tracker_pairs': motions}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--height', type=int, required=True)
    parser.add_argument('--format', choices=('GRAY8', 'RAW8'), default='GRAY8')
    parser.add_argument('--mode', choices=('synthetic', 'live', 'drop', 'bypass', 'reject', 'shutdown', 'tracker'), default='synthetic')
    parser.add_argument('--invalid', choices=('height', 'width', 'stride', 'payload', 'type', 'message', 'change'), default='height')
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--socket', default='CAM_B', choices=('CAM_A', 'CAM_B', 'CAM_C'))
    parser.add_argument('--seed', type=int, default=27)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary = {'width': args.width, 'height': args.height, 'format': args.format,
               'mode': args.mode, 'seed': args.seed, 'passed': False}
    try:
        with dai.Device(dai.DeviceInfo(args.device)) as device:
            assert device.getPlatform() == dai.Platform.RVC4
            tests = {'synthetic': synthetic, 'live': live, 'drop': live, 'bypass': bypass,
                     'reject': reject, 'shutdown': shutdown, 'tracker': tracker}
            summary.update(tests[args.mode](device, args))
        summary['passed'] = True
    except Exception as error:
        summary['error'] = repr(error)
        raise
    finally:
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps({key: value for key, value in summary.items() if key != 'pairs'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
