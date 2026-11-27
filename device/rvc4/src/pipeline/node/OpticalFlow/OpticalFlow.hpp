#pragma once
#include "depthai/pipeline/node/OpticalFlow.hpp"
namespace dai {
namespace gate {
class OpticalFlow : public dai::node::OpticalFlow {
   public:
    constexpr static const char* NAME = dai::node::OpticalFlow::NAME;
    OpticalFlow() : OpticalFlow(std::make_unique<Properties>()) {}
    explicit OpticalFlow(std::unique_ptr<Properties> props) : dai::node::OpticalFlow(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;
};
}  // namespace gate
}  // namespace dai
