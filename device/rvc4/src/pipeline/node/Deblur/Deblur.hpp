#pragma once
#include "depthai/pipeline/node/Deblur.hpp"
namespace dai {
namespace gate {
class Deblur : public dai::node::Deblur {
   public:
    constexpr static const char* NAME = dai::node::Deblur::NAME;
    Deblur() : Deblur(std::make_unique<Properties>()) {}
    explicit Deblur(std::unique_ptr<Properties> props) : dai::node::Deblur(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;
};
}  // namespace gate
}  // namespace dai
