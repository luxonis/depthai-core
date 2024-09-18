import cv2
import depthai as dai
import numpy as np
import pytest

DEBUG = False
def generate_test_image():
    # Generate a 100x100 image with a gradient from 0 to 255
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    for i in range(100):
        for j in range(100):
            image[i, j, 0] = i
            image[i, j, 1] = i + j
            image[i, j, 2] = j
    return image

@pytest.mark.parametrize("size", [(1000, 1000), (2000, 2000), (500, 500)])
@pytest.mark.parametrize("type", [dai.ImgFrame.Type.BGR888p, dai.ImgFrame.Type.BGR888i, dai.ImgFrame.Type.RGB888i, dai.ImgFrame.Type.RGB888i,
                                  dai.ImgFrame.Type.YUV420p, dai.ImgFrame.Type.NV12, dai.ImgFrame.Type.NV21])
                                  # The other types are not supported by getCvFrame() yet
                                  # dai.ImgFrame.Type.YUV400p, dai.ImgFrame.Type.YUV422i, dai.ImgFrame.Type.YUV422p, dai.ImgFrame.Type.YUV444p, dai.ImgFrame.Type.YUV444i])
def test_dai_image_conversion(size, type):
    image = generate_test_image()

    largeImage = cv2.resize(image, size)
    if DEBUG:
        cv2.imshow("largeImage", largeImage)

    # Create a DAI ImgFrame and set the frame
    imgDai = dai.ImgFrame()
    imgDai.setCvFrame(largeImage, type)

    # Get the frame back from ImgFrame
    recoveredImage = imgDai.getCvFrame()

    if DEBUG:
        cv2.imshow("recoveredImage", recoveredImage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    # Compare the original large_image and the recovered_image_bgr
    # The tolerance level for comparison
    tolerance = 3

    # Calculate the mean squared error between the images
    mse = np.mean((largeImage.astype("float") - recoveredImage.astype("float")) ** 2)
    assert mse <= tolerance, f"Images differ significantly with MSE: {mse}"

if __name__ == "__main__":
    test_dai_image_conversion((1000, 1000), dai.ImgFrame.Type.BGR888p)