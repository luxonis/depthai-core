#pragma once

#include <chrono>
#include <memory>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

namespace spdlog {
class async_logger;
}  // namespace spdlog

namespace dai {

class DeviceNode : public ThreadedNode {
   protected:
    std::shared_ptr<Device> device;

   public:
    DeviceNode() = delete;
    virtual ~DeviceNode() = default;
    // virtual 'run' method
    virtual void run() override;

    bool runOnHost() const override {
        // By default, don't allow running on host, but can be overridden
        return false;
    }

    /**
     * @brief Get device for this node
     *
     * @return shared pointer to device
     */
    const std::shared_ptr<Device> getDevice() const;

    copyable_unique_ptr<Properties> propertiesHolder;

    // Get properties
    virtual Properties& getProperties();

    void setLogLevel(dai::LogLevel level) override;
    virtual dai::LogLevel getLogLevel() const override;

   protected:
    void logTiming(const std::shared_ptr<spdlog::async_logger>& logger,
                   std::chrono::steady_clock::time_point tAbsoluteBeginning,
                   std::chrono::steady_clock::time_point tGotInput,
                   std::chrono::steady_clock::time_point tProcessed,
                   std::chrono::steady_clock::time_point tAbsoluteEnd);
    DeviceNode(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool conf);
    DeviceNode(std::unique_ptr<Properties> props, bool conf);

    template <typename T>
    friend class Subnode;
    friend class Pipeline;
    friend class PipelineImpl;

    /**
     * @brief Set device for this node
     *
     * @param device: shared pointer to device
     */
    void setDevice(const std::shared_ptr<Device>& device);
};

// Node CRTP class
template <typename Base, typename Derived, typename Props, bool BuiltInNode = true>
class DeviceNodeCRTP : public Base {
   public:
    using Properties = Props;
    virtual ~DeviceNodeCRTP() = default;
    /// Underlying properties
    Properties& properties;
    DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
    const char* getName() const override {
        return Derived::NAME;
    };
    DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING

    bool isBuiltInNode() const override {
        return BuiltInNode;
    }

    // std::unique_ptr<Node> clone() const override {
    //     return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    // };

    // No public constructor, only a factory function.
    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<Derived> create(Args&&... args) {
        auto nodePtr = std::shared_ptr<Derived>(new Derived(std::forward<Args>(args)...));
        nodePtr->buildInternal();
        return nodePtr;
    }

    // No public constructor, only a factory function.
    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<Derived> create(std::shared_ptr<Device> device, Args&&... args) {
        auto nodePtr = std::shared_ptr<Derived>(new Derived(device, std::forward<Args>(args)...));
        nodePtr->buildInternal();
        return nodePtr;
    }
    [[nodiscard]] static std::shared_ptr<Derived> create(std::unique_ptr<Properties> props) {
        return std::shared_ptr<Derived>(new Derived(props));
    }

   protected:
    DeviceNodeCRTP() : Base(std::make_unique<Props>(), false), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(const std::shared_ptr<Device>& device)
        : Base(device, std::make_unique<Props>(), false), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(std::unique_ptr<Properties> props) : Base(std::move(props), true), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(std::unique_ptr<Properties> props, bool confMode)
        : Base(std::move(props), confMode), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    DeviceNodeCRTP(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool confMode)
        : Base(device, std::move(props), confMode), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    friend Derived;
    friend Base;
    friend PipelineImpl;
};

class HostRunnable {
   public:
    HostRunnable() = default;
    virtual ~HostRunnable();  // Has to be virtual for std::dynamic_cast to be used reliably
};
}  // namespace dai
