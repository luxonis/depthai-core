#pragma once

#include <opencv2/imgcodecs.hpp>

#include "ReplayArchiveFixture.hpp"

namespace replaytest {

class ImageArchive : public Directory {
   public:
    ImageArchive(const std::string& imageExtension, const std::string& archiveExtension, bool wrapped, int width = 18, int height = 10) {
        std::vector<Member> members{{"./", {}, true}};
        if(wrapped) members.push_back({"./frames/", {}, true});
        const std::string prefix = wrapped ? "./frames/" : "./";
        // Neither member order nor lexicographic order matches numeric frame order.
        for(const auto number : {10, 30, 2}) {
            const auto value = number == 2 ? 40 : number == 10 ? 100 : 160;
            const cv::Mat image(height, width, CV_8UC3, cv::Scalar(value, value, value));
            std::vector<std::uint8_t> bytes;
            if(!cv::imencode(imageExtension, image, bytes)) throw std::runtime_error("Could not encode test image");
            members.push_back({prefix + "cam_" + std::to_string(number) + imageExtension, std::move(bytes)});
        }
        archive = path / ("frames" + archiveExtension);
        writeArchive(archive, members);
    }

    std::filesystem::path archive;
};

}  // namespace replaytest
