#include <opencv2/imgcodecs.hpp>
#include <utility>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/node/host/XLinkOutHost.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "opencv2/opencv.hpp"

class ImgManipTestIn : public dai::NodeCRTP<dai::node::ThreadedHostNode, ImgManipTestIn> {
    using Type = dai::ImgFrame::Type;
    std::string fpath;

   public:
    explicit ImgManipTestIn(std::string fpath) : fpath(std::move(fpath)) {}
    Output outImg{*this, {.name = "outImg", .types = {{dai::DatatypeEnum::ImgFrame, false}}}};
    Output outCfg{*this, {.name = "outCfg", .types = {{dai::DatatypeEnum::ImageManipConfig, false}}}};
    void run() override {
        cv::Mat img = cv::imread(fpath);
        if(img.empty()) {
            throw std::runtime_error("File not found: " + fpath);
        }
        std::vector<Type> types = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::YUV420p, Type::NV12, Type::GRAY8};
        for(int i = 0; i < types.size(); ++i) {
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setCvFrame(img, types[i]);
            for(int j = 0; j < types.size(); ++j) {
                if (i == j) continue;
                auto cfg = std::make_shared<dai::ImageManipConfig>();
                cfg->setFrameType(types[j]);
                outCfg.send(cfg);
                outImg.send(frame);
            }
        }
    }
};

class ImgManipTestOut : public dai::NodeCRTP<dai::node::ThreadedHostNode, ImgManipTestOut> {
    using Type = dai::ImgFrame::Type;
    std::string fpath;

   public:
    explicit ImgManipTestOut(std::string fpath) : fpath(std::move(fpath)) {}
    Input inImg{*this, {.name = "inImg", .types = {{dai::DatatypeEnum::ImgFrame, false}}, .waitForMessage = true}};
    void run() override {
        cv::Mat img = cv::imread(fpath);
        if(img.empty()) {
            throw std::runtime_error("File not found: " + fpath);
        }
        std::vector<Type> types = {Type::RGB888i, Type::RGB888p, Type::BGR888i, Type::BGR888p, Type::YUV420p, Type::NV12, Type::GRAY8};
        for(int i = 0; i < types.size(); ++i) {
            for(int j = 0; j < types.size(); ++j) {
                if (i == j) continue;
                auto image = inImg.get<dai::ImgFrame>();
                if(image->getType() != types[j]) {
                    image = inImg.get<dai::ImgFrame>();
                }
                assert(image->getType() == types[j] || types[j] == Type::GRAY8);
                cv::Mat imgCv = image->getCvFrame();
                if(imgCv.empty()) {
                    throw std::runtime_error("Empty frame");
                } else if(imgCv.channels() == 1) {
                    cv::cvtColor(imgCv, imgCv, cv::COLOR_GRAY2BGR);
                }
                assert(img.cols == imgCv.cols);
                assert(img.rows == imgCv.rows);
                cv::Mat mergedImage(img.rows, img.cols + imgCv.cols, img.type());

                // Copy the first image to the left half of the merged image
                cv::Mat left(mergedImage, cv::Rect(0, 0, img.cols, img.rows));
                img.copyTo(left);

                // Copy the second image to the right half of the merged image
                cv::Mat right(mergedImage, cv::Rect(img.cols, 0, imgCv.cols, imgCv.rows));
                imgCv.copyTo(right);

                // if ((int)types[j] == 2 || (int)types[j] == 22)
                cv::imshow("Merged (" + std::to_string((int)types[i]) + " to " + std::to_string((int)types[j]) + ")", mergedImage);
                cv::waitKey(1000);
            }
        }
    }
};

int main(int argc, char** argv) {
    auto device = std::make_shared<dai::Device>("10.12.103.126");
    dai::Pipeline pipeline(device);

    std::string fpath = argc > 1 ? argv[1] : "image.jpg";

    auto imgManip = pipeline.create<dai::node::ImageManip>()->build();
    auto testerIn = pipeline.create<ImgManipTestIn>(fpath);
    auto testerOut = pipeline.create<ImgManipTestOut>(fpath);
    testerIn->outImg.link(imgManip->inputImage);
    testerIn->outCfg.link(imgManip->inputConfig);
    imgManip->out.link(testerOut->inImg);

    pipeline.start();
    pipeline.wait();
}
