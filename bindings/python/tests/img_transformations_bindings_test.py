import depthai as dai


def _identity_extrinsics(from_socket=dai.CameraBoardSocket.CAM_A):
    extr = dai.Extrinsics()
    extr.rotationMatrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    extr.translation = dai.Point3f(0.0, 0.0, 0.0)
    extr.specTranslation = dai.Point3f(0.0, 0.0, 0.0)
    extr.toCameraSocket = dai.CameraBoardSocket.AUTO
    extr.lengthUnit = dai.LengthUnit.CENTIMETER
    return extr


def _build_transformation(from_socket=dai.CameraBoardSocket.CAM_A):
    tr = dai.ImgTransformation(640, 480)
    tr.setIntrinsicMatrix([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    tr.setDistortionCoefficients([0.0, 0.0, 0.0, 0.0, 0.0])
    tr.setExtrinsics(_identity_extrinsics(from_socket))
    tr.setSourceSize(640, 480)
    tr.setSize(640, 480)
    return tr


def test_extrinsics_extended_bindings_roundtrip():
    extr = _identity_extrinsics()
    assert extr.toCameraSocket == dai.CameraBoardSocket.AUTO
    assert extr.lengthUnit == dai.LengthUnit.CENTIMETER
    assert len(extr.getTransformationMatrix()) == 4
    assert len(extr.getInverseTransformationMatrix()) == 4
    assert extr.getTranslationVector() == [0.0, 0.0, 0.0]
    assert extr.isEqualExtrinsics(extr)


def test_img_transformation_extended_bindings_smoke():
    tr = _build_transformation(dai.CameraBoardSocket.CAM_A)

    tr.setCalibrationId(42)
    assert tr.getCalibrationId() == 42

    tr.addSrcCrops([])
    assert tr.isValid()
    assert tr.isEqualTransformation(tr)


def test_img_transformation_3d_methods_and_constructor_with_extrinsics():
    extr = _identity_extrinsics()
    tr1 = dai.ImgTransformation(
        640,
        480,
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]],
        dai.CameraModel.Perspective,
        [0.0, 0.0, 0.0, 0.0, 0.0],
        extr,
    )
    tr2 = _build_transformation(dai.CameraBoardSocket.CAM_B)

    p3 = dai.Point3f(0.1, 0.2, 1.0)
    p2 = tr1.project3DPoint(p3)
    assert isinstance(p2, dai.Point2f)

    p2_to = tr1.project3DPointTo(tr2, p3)
    p2_from = tr2.project3DPointFrom(tr1, p3)
    assert isinstance(p2_to, dai.Point2f)
    assert isinstance(p2_from, dai.Point2f)

    p3_to = tr1.remap3DPointTo(tr2, p3)
    p3_from = tr2.remap3DPointFrom(tr1, p3)
    assert isinstance(p3_to, dai.Point3f)
    assert isinstance(p3_from, dai.Point3f)

    matrix = tr1.getExtrinsicsTransformationMatrixTo(tr2)
    assert len(matrix) == 4
    assert len(matrix[0]) == 4
