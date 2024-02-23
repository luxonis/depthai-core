#pragma once

#include "depthai/pipeline/ThreadedNode.hpp"

namespace dai {

class DeviceNode : public ThreadedNode {
   public:
    using ThreadedNode::ThreadedNode;
    virtual ~DeviceNode() = default;

    // virtual 'run' method
    virtual void run() override;

    copyable_unique_ptr<Properties> propertiesHolder;

    // Get properties
    virtual Properties& getProperties();

   protected:
    DeviceNode(std::unique_ptr<Properties> props, bool conf);
};

// Node CRTP class
template <typename Base, typename Derived, typename Props>
class NodeCRTP : public Base {
   public:
    using Properties = Props;
    virtual ~NodeCRTP() = default;
    /// Underlying properties
    Properties& properties;
    const char* getName() const override {
        return Derived::NAME;
    };
    std::unique_ptr<Node> clone() const override {
        return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    };
    void build() {}

    // No public constructor, only a factory function.
    [[nodiscard]] static std::shared_ptr<Derived> create() {
        auto n = std::make_shared<Derived>();
        n->build();
        return n;
    }
    [[nodiscard]] static std::shared_ptr<Derived> create(std::unique_ptr<Properties> props) {
        auto n = std::shared_ptr<Derived>(new Derived(props));
        // Configure mode, don't build
        // n->build();
        return n;
    }

   protected:
    NodeCRTP() : Base(std::make_unique<Props>(), false), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    NodeCRTP(std::unique_ptr<Properties> props) : Base(std::move(props), true), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}
    NodeCRTP(std::unique_ptr<Properties> props, bool confMode)
        : Base(std::move(props), confMode), properties(static_cast<Properties&>(*DeviceNode::propertiesHolder)) {}

    friend Derived;
    friend Base;
};

}  // namespace dai
