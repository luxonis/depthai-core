#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include "depthai-shared/pb/properties/StereoDepthProperties.hpp"

namespace dai {
namespace node {

    class StereoDepth : public Node {
        dai::StereoDepthProperties properties;

        std::string getName() override;
        std::vector<Output> getOutputs() override;
        std::vector<Input> getInputs() override;
        nlohmann::json getProperties() override;
        std::shared_ptr<Node> clone() override;

       public:
        StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

        Input left{*this, "left", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
        Input right{*this, "right", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}}};
        Output depth{*this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output disparity{*this, "disparity", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output syncedLeft{*this, "syncedLeft", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output syncedRight{*this, "syncedRight", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output rectifiedLeft{*this, "rectifiedLeft", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
        Output rectifiedRight{*this, "rectifiedRight", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

        // Specify local filesystem path to the calibration file. Empty string to use EEPROM
        void loadCalibrationFile(const std::string& path);
        // Specify calibration data as a vector of bytes. Empty vector to use EEPROM
        void loadCalibrationData(const std::vector<std::uint8_t>& data);
        // Specify that a passthrough/dummy calibration should be used, when input frames
        // are already rectified (e.g. sourced from recordings on the host)
        void setEmptyCalibration(void);
        // Optional (taken from MonoCamera nodes if they exist)
        void setInputResolution(int width, int height);

        void setMedianFilter(StereoDepthProperties::MedianFilter median);
        void setConfidenceThreshold(int confThr);
        void setLeftRightCheck(bool enable);
        void setSubpixel(bool enable);
        void setExtendedDisparity(bool enable);
        void setRectifyEdgeFillColor(int color);
        void setRectifyMirrorFrame(bool enable);
        void setOutputRectified(bool enable);
        void setOutputDepth(bool enable);
    };

}  // namespace node
}  // namespace dai
