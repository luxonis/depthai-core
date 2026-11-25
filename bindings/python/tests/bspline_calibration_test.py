"""Hardware-free checks of the B-spline lens model Python API."""
import unittest

import depthai as dai


def make_lens(correction=0.0):
    lens = dai.BsplineCameraCalibration()
    lens.width, lens.height = 640, 480
    lens.nx = lens.ny = 4
    lens.fovXDegrees = 80
    lens.intrinsics = [400, 410, 320, 240] + [correction, -correction] * 16
    return lens


def legacy_fields(handler):
    data = handler.eepromToJson()
    data.pop('bsplineCameraData', None)
    data['cameraData'] = sorted(data['cameraData'], key=lambda camera: camera[0])
    return data


class BsplineCalibrationTests(unittest.TestCase):
    def test_projection_and_local_correction(self):
        lens = make_lens()
        lens.validate()
        self.assertEqual(list(lens.project([0., 0., 1.])), [320., 240.])
        projected = make_lens(0.002).project([0., 0., 1.])
        self.assertAlmostEqual(projected[0], 320.8)
        self.assertAlmostEqual(projected[1], 239.18)

    def test_unproject_inverts_project(self):
        lens = make_lens(0.002)
        for pixel in ([320., 240.], [10., 20.], [630., 470.]):
            ray = lens.unproject(pixel)
            self.assertAlmostEqual(sum(v * v for v in ray), 1.0)
            back = lens.project(list(ray))
            self.assertAlmostEqual(back[0], pixel[0], places=5)
            self.assertAlmostEqual(back[1], pixel[1], places=5)

    def test_set_roundtrip_and_clear_preserve_legacy(self):
        handler = dai.CalibrationHandler()
        before = legacy_fields(handler)
        handler.setBsplineCameraCalibration(dai.CameraBoardSocket.CAM_B, make_lens(0.001))
        self.assertEqual(legacy_fields(handler), before)
        restored = dai.CalibrationHandler.fromJson(handler.eepromToJson())
        self.assertEqual(restored.getBsplineCameraCalibration(dai.CameraBoardSocket.CAM_B).width, 640)
        self.assertIsNone(restored.getBsplineCameraCalibration(dai.CameraBoardSocket.CAM_C))
        restored.clearBsplineCameraCalibration(dai.CameraBoardSocket.CAM_B)
        self.assertIsNone(restored.getBsplineCameraCalibration(dai.CameraBoardSocket.CAM_B))
        self.assertEqual(legacy_fields(restored), before)

    def test_invalid_lens_is_rejected(self):
        lens = make_lens()
        lens.intrinsics = lens.intrinsics[:-1]
        with self.assertRaises(ValueError):
            dai.CalibrationHandler().setBsplineCameraCalibration(dai.CameraBoardSocket.CAM_B, lens)

    def test_legacy_is_the_default(self):
        models = dai.node.StereoDepth.RectificationModel
        self.assertEqual(dai.StereoDepthProperties().rectificationModel, models.LEGACY)
        self.assertNotEqual(models.BSPLINE, models.LEGACY)


if __name__ == '__main__':
    unittest.main()
