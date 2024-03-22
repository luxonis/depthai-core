#pragma once

#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {

class DeviceNode : public ThreadedNode {
   private:
    std::shared_ptr<Device> device;

   public:
    DeviceNode() = delete;
    virtual ~DeviceNode() = default;
    // virtual 'run' method
    virtual void run() override;

    copyable_unique_ptr<Properties> propertiesHolder;

    // Get properties
    virtual Properties& getProperties();

   protected:
    DeviceNode(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool conf);
    DeviceNode(std::unique_ptr<Properties> props, bool conf);
};

// Node CRTP class
template <typename Base, typename Derived, typename Props>
class DeviceNodeCRTP : public Base {
   public:
    using Properties = Props;
    virtual ~DeviceNodeCRTP() = default;
    /// Underlying properties
    Properties& properties;
    const char* getName() const override {
        return Derived::NAME;
    };
    // std::unique_ptr<Node> clone() const override {
    //     return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    // };
    void build() {}

    // No public constructor, only a factory function.
    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<Derived> create(Args&&... args) {
        auto n = std::make_shared<Derived>(std::forward(args)...);
        n->build();
        return n;
    }

    // No public constructor, only a factory function.
    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<Derived> create(std::shared_ptr<Device> device, Args&&... args) {
        // auto n = std::make_shared<Derived>(device, std::forward<Args>(args)...);
        auto n = std::shared_ptr<Derived>(new Derived(device, std::forward<Args>(args)...));
        n->build();
        return n;
    }
    [[nodiscard]] static std::shared_ptr<Derived> create(std::unique_ptr<Properties> props) {
        // auto n = std::shared_ptr<Derived>(new Derived(props));
        auto n = std::shared_ptr<Derived>(new Derived(props));
        // Configure mode, don't build
        // n->build();
        return n;
    }

   protected:
    DeviceNodeCRTP() : Base(std::make_unique<Props>(), false), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(const std::shared_ptr<Device>& device)
        : Base(device, std::make_unique<Props>(), false), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(std::unique_ptr<Properties> props) : Base(std::move(props), true), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(std::unique_ptr<Properties> props, bool confMode)
        : Base(std::move(props), confMode), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}

    friend Derived;
    friend Base;
    friend PipelineImpl;
};

}  // namespace dai
