import depthai as dai
import numpy as np

def test_pcldata_set_points():
    pointCloudData = dai.PointCloudData()

    # Points
    points = np.random.rand(1000, 3).astype(np.float32)
    pointCloudData.setPoints(points)

    assert(pointCloudData.isColor() == False)
    assert(isinstance(pointCloudData.getPoints(), np.ndarray))
    assert(pointCloudData.getPoints().dtype == points.dtype)
    assert(np.array_equal(pointCloudData.getPoints(), points))

    # RGB Points
    colors = np.random.rand(1000, 4).astype(np.uint8)
    pointCloudData.setPointsRGB(points, colors)

    assert(pointCloudData.isColor() == True)
    assert(isinstance(pointCloudData.getPoints(), np.ndarray))
    assert(isinstance(pointCloudData.getPointsRGB()[0], np.ndarray))
    assert(isinstance(pointCloudData.getPointsRGB()[1], np.ndarray))
    assert(pointCloudData.getPoints().dtype == points.dtype)
    assert(pointCloudData.getPointsRGB()[0].dtype == points.dtype)
    assert(pointCloudData.getPointsRGB()[1].dtype == colors.dtype)
    assert(np.array_equal(pointCloudData.getPoints(), points))
    assert(np.array_equal(pointCloudData.getPointsRGB()[0], points))
    assert(np.array_equal(pointCloudData.getPointsRGB()[1], colors))


if __name__ == '__main__':
  test_pcldata_set_points()