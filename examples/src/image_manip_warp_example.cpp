#include <iostream>
#include <cstdio>

#include "utility.hpp"

#include "depthai/depthai.hpp"

struct warpFourPointTest {
    std::vector<dai::Point2f> points;
    bool normalizedCoords;
    const char *description;
};

#define ROTATE_RATE_MAX 5.0
#define ROTATE_RATE_INC 0.1
#define KEY_ROTATE_DECR 'z'
#define KEY_ROTATE_INCR 'x'

#define RESIZE_MAX_W 800
#define RESIZE_MAX_H 600
#define RESIZE_FACTOR_MAX 5
#define KEY_RESIZE_INC   'v'

/* The crop points are specified in clockwise order,
 * with first point mapped to output top-left, as:
 *   P0  ->  P1
 *    ^       v
 *   P3  <-  P2
 */
#define P0 {0, 0}  // top-left
#define P1 {1, 0}  // top-right
#define P2 {1, 1}  // bottom-right
#define P3 {0, 1}  // bottom-left
std::vector<warpFourPointTest> warpList = {
  //{{{  0,  0},{  1,  0},{  1,  1},{  0,  1}}, true, "passthrough"},
  //{{{  0,  0},{639,  0},{639,479},{  0,479}}, false,"passthrough (pixels)"},
    {{P0, P1, P2, P3}, true, "1.passthrough"},
    {{P3, P0, P1, P2}, true, "2.rotate 90"},
    {{P2, P3, P0, P1}, true, "3.rotate 180"},
    {{P1, P2, P3, P0}, true, "4.rotate 270"},
    {{P1, P0, P3, P2}, true, "5.horizontal mirror"},
    {{P3, P2, P1, P0}, true, "6.vertical flip"},
    {{{-0.1,-0.1},{1.1,-0.1},{1.1,1.1},{-0.1,1.1}}, true, "7.add black borders"},
    {{{-0.3, 0},{1, 0},{1.3, 1},{0, 1}}, true, "8.parallelogram transform"},
    {{{-0.2, 0},{1.8, 0},{1, 1},{0, 1}}, true, "9.trapezoid transform"},
};
#define KEY_WARP_TEST_CYCLE 'c'

void printControls() {
    printf("\n=== Controls:\n");
    printf(" %c -rotated rectangle crop, decrease rate\n", KEY_ROTATE_DECR);
    printf(" %c -rotated rectangle crop, increase rate\n", KEY_ROTATE_INCR);
    printf(" %c -warp 4-point transform, cycle through modes\n", KEY_WARP_TEST_CYCLE);
    printf(" %c -resize cropped region, or disable resize\n", KEY_RESIZE_INC);
    printf(" h -print controls (help)\n");
}

int main(){
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto camOut = pipeline.create<dai::node::XLinkOut>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto manipCfg = pipeline.create<dai::node::XLinkIn>();
    auto manipOut = pipeline.create<dai::node::XLinkOut>();

    camOut->setStreamName("preview");
    manipOut->setStreamName("manip");
    manipCfg->setStreamName("manipCfg");

    colorCam->setPreviewSize(640, 480);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    manip->setMaxOutputFrameSize(2000*1500*3);

    /* Link nodes: CAM --> XLINK(preview)
     *                \--> manip -> XLINK(manipOut)
     *        manipCfg ---/
     */
    colorCam->preview.link(camOut->input);
    colorCam->preview.link(manip->inputImage);
    manip->out.link(manipOut->input);
    manipCfg->out.link(manip->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);
    device.startPipeline();

    // Create input & output queues
    auto previewQueue = device.getOutputQueue("preview", 8, false);
    auto manipQueue = device.getOutputQueue("manip", 8, false);
    auto manipInQueue = device.getInputQueue("manipCfg");

    std::vector<decltype(previewQueue)> frameQueues {previewQueue, manipQueue};

    // keep processing data
    int key = -1;
    float angleDeg = 0;
    float rotateRate = 1.0;
    int resizeFactor = 0, resizeX, resizeY;
    bool testFourPt = false;
    int warpIdx = -1;

    printControls();

    while (key != 'q') {
        if (key >= 0) {
            printf("Pressed: %c | ", key);
            if (key == KEY_ROTATE_DECR || key == KEY_ROTATE_INCR) {
                if (key == KEY_ROTATE_DECR) {
                    if (rotateRate > -ROTATE_RATE_MAX)
                        rotateRate -= ROTATE_RATE_INC;
                } else if (key == KEY_ROTATE_INCR) {
                    if (rotateRate <  ROTATE_RATE_MAX)
                        rotateRate += ROTATE_RATE_INC;
                }
                testFourPt = false;
                printf("Crop rotated rectangle, rate: %g degrees", rotateRate);
            } else if (key == KEY_RESIZE_INC) {
                resizeFactor++;
                if (resizeFactor > RESIZE_FACTOR_MAX) {
                    resizeFactor = 0;
                    printf("Crop region not resized");
                } else {
                    resizeX = RESIZE_MAX_W / resizeFactor;
                    resizeY = RESIZE_MAX_H / resizeFactor;
                    printf("Crop region resized to: %d x %d", resizeX, resizeY);
                }
            } else if (key == KEY_WARP_TEST_CYCLE) {
                resizeFactor = 0; // Disable resizing initially
                warpIdx = (warpIdx + 1) % warpList.size();
                printf("Warp 4-point transform, %s", warpList[warpIdx].description);
                testFourPt = true;
            } else if (key == 'h') {
                printControls();
            }
            printf("\n");
        }

        // Send an updated config with continuous rotate, or after a key press
        if (key >= 0 || (!testFourPt && std::abs(rotateRate) > 0.0001)) {
            dai::ImageManipConfig cfg;
            if (testFourPt) {
                cfg.setWarpTransformFourPoints(warpList[warpIdx].points,
                                               warpList[warpIdx].normalizedCoords);
            } else {
                angleDeg += rotateRate;
                dai::RotatedRect rr = {
                        {320, 240}, // center
                        {640, 480}, //{400, 400}, // size
                        angleDeg
                };
                bool normalized = false;
                cfg.setCropRotatedRect(rr, normalized);
            }
            if (resizeFactor > 0) {
                cfg.setResize(resizeX, resizeY);
            }
            //cfg.setWarpBorderFillColor(255, 0, 0);
            //cfg.setWarpBorderReplicatePixels();
            manipInQueue->send(cfg);
        }

        for (const auto& q : frameQueues) {
            auto img = q->get<dai::ImgFrame>();
            auto mat = toMat(img->getData(), img->getWidth(), img->getHeight(), 3, 1);
            cv::imshow(q->getName(), mat);
        }
        key = cv::waitKey(1);
    }
}
