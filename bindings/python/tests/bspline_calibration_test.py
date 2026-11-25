"""Hardware-free checks for the independent spline calibration Python API."""
import unittest

import depthai as dai


def make_model():
    lens = dai.BsplineCameraCalibration()
    lens.nx = lens.ny = 4
    lens.fovXDegrees = 80
    lens.intrinsics = [400, 410, 320, 240] + [0.0] * 32
    identity = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    model = dai.BsplineStereoCalibration()
    model.deviceId, model.calibrationId = 'unit-test', 'independent-model'
    model.leftSocket, model.rightSocket = dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
    model.width, model.height = 640, 480
    model.left = model.right = lens
    model.rotationLeft = model.rotationRight = model.leftToRightRotation = identity
    model.leftToRightTranslationCm = [-7.5, 0, 0]
    model.baselineCm = 7.5
    model.virtualIntrinsics = [400, 410, 320, 240]
    model.referenceSocket = dai.CameraBoardSocket.CAM_A
    model.leftToReferenceRotation = [0., -1., 0., 1., 0., 0., 0., 0., 1.]
    model.leftToReferenceTranslationCm = [1, 2, 3]
    return model


def legacy_fields(handler):
    data = handler.eepromToJson()
    data.pop('bsplineStereoCalibration', None)
    data['cameraData'] = sorted(data['cameraData'], key=lambda camera: camera[0])
    return data


class BsplineCalibrationTests(unittest.TestCase):
    def test_projection_and_local_correction(self):
        model = make_model()
        model.validate()
        self.assertEqual(list(model.left.project([0., 0., 1.])), [320., 240.])
        lens = model.left
        lens.intrinsics = lens.intrinsics[:4] + [0.002, -0.003] * 16
        projected = lens.project([0., 0., 1.])
        self.assertAlmostEqual(projected[0], 320.8)
        self.assertAlmostEqual(projected[1], 238.77)

    def test_both_camera_poses_use_the_spline_geometry(self):
        model = make_model()
        left = model.getCameraToReference(model.leftSocket)
        right = model.getCameraToReference(model.rightSocket)
        self.assertEqual([row[3] for row in left], [1, 2, 3, 1])
        self.assertEqual([row[3] for row in right], [1, 9.5, 3, 1])
        self.assertEqual([row[:3] for row in left], [row[:3] for row in right])

    def test_attach_roundtrip_and_clear_preserve_legacy(self):
        handler = dai.CalibrationHandler()
        before = legacy_fields(handler)
        handler.setBsplineStereoCalibration(make_model())
        self.assertEqual(legacy_fields(handler), before)
        restored = dai.CalibrationHandler.fromJson(handler.eepromToJson())
        self.assertEqual(restored.getBsplineStereoCalibration().baselineCm, 7.5)
        self.assertEqual(legacy_fields(restored), before)
        restored.clearBsplineStereoCalibration()
        self.assertIsNone(restored.getBsplineStereoCalibration())
        self.assertEqual(legacy_fields(restored), before)

    def test_invalid_or_old_geometry_is_rejected(self):
        model = make_model()
        model.version = 1
        with self.assertRaises(ValueError):
            model.validate()
        model = make_model()
        model.leftToReferenceTranslationCm = []
        with self.assertRaises(ValueError):
            model.getCameraToReference(model.leftSocket)
        model = make_model()
        model.baselineCm *= 2
        with self.assertRaises(ValueError):
            model.validate()

    def test_auto_retains_legacy_semantics(self):
        models = dai.node.StereoDepth.RectificationModel
        self.assertEqual(models.AUTO, models.LEGACY)
        self.assertNotEqual(models.BSPLINE, models.LEGACY)


if __name__ == '__main__':
    unittest.main()
