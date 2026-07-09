#include "depthai/pipeline/Subnode.hpp"

#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/properties/DeviceNodeGroupProperties.hpp"

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/

class CustomDeviceSubnode : public dai::DeviceNode {
   public:
    CustomDeviceSubnode() : DeviceNode(std::make_unique<dai::DeviceNodeGroupProperties>(), false) {}
    CustomDeviceSubnode(std::shared_ptr<dai::Device> device) : DeviceNode(device, std::make_unique<dai::DeviceNodeGroupProperties>(), false) {}

    const char* getName() const override {
        return "CustomDeviceSubnode";
    };

    void buildInternal() override {
        buildInternalCalled = true;
    }

    bool buildInternalCalled = false;
};

class CustomHostSubnode : public dai::Node {
   public:
    CustomHostSubnode() : Node() {}

    Output out{*this, {"out", DEFAULT_GROUP, {{{dai::DatatypeEnum::Buffer, true}}}}};

    bool runOnHost() const override {
        return true;
    }

    const char* getName() const override {
        return "CustomHostSubnode";
    };

    void buildInternal() override {
        buildInternalCalled = true;
    }

    bool buildInternalCalled = false;
};

class CustomDeviceNode : public dai::DeviceNode {
   public:
    CustomDeviceNode() : DeviceNode(std::make_unique<dai::DeviceNodeGroupProperties>(), false) {}
    CustomDeviceNode(std::shared_ptr<dai::Device> device) : DeviceNode(device, std::make_unique<dai::DeviceNodeGroupProperties>(), false) {}

    const char* getName() const override {
        return "CustomDeviceNode";
    };

    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<CustomDeviceNode> create(std::shared_ptr<dai::Device> device, Args&&... args) {
        auto nodePtr = std::make_shared<CustomDeviceNode>(device, std::forward<Args>(args)...);
        nodePtr->buildInternal();
        return nodePtr;
    }

    dai::Subnode<CustomDeviceSubnode> deviceSubnode{*this, "deviceSubnode"};
    dai::Subnode<CustomHostSubnode> hostSubnode{*this, "hostSubnode"};

    void buildInternal() override {
        buildInternalCalled = true;
    }

    bool buildInternalCalled = false;
};

class CustomThreadedHostNode : public dai::node::CustomThreadedNode<CustomThreadedHostNode> {
   public:
    constexpr static const char* NAME = "CustomThreadedHostNode";

    CustomThreadedHostNode() {}

    void run() override {}

    dai::Subnode<dai::node::Sync> syncSubnode{*this, "syncSubnode"};  // HostRunnable
};

class LazyThreadedHostNode : public dai::node::CustomThreadedNode<LazyThreadedHostNode> {
   public:
    constexpr static const char* NAME = "LazyThreadedHostNode";

    void run() override {}

    dai::Subnode<CustomHostSubnode>& ensureHostSubnode() {
        if(!hostSubnode) {
            hostSubnode = std::make_unique<dai::Subnode<CustomHostSubnode>>(*this, "hostSubnode");
        }
        return *hostSubnode;
    }

   private:
    std::unique_ptr<dai::Subnode<CustomHostSubnode>> hostSubnode;
};

class RejectingThreadedHostNode : public dai::node::CustomThreadedNode<RejectingThreadedHostNode> {
   public:
    constexpr static const char* NAME = "RejectingThreadedHostNode";

    void run() override {}

    void createInvalidDeviceSubnode() {
        invalidSubnode = std::make_unique<dai::Subnode<CustomDeviceSubnode>>(*this, "invalidSubnode");
    }

    bool hasInvalidSubnode() const {
        return invalidSubnode != nullptr;
    }

   private:
    std::unique_ptr<dai::Subnode<CustomDeviceSubnode>> invalidSubnode;
};
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/

TEST_CASE("Subnode inherits device from parent and buildInternal is called") {
    // Create pipeline
    dai::Pipeline p;
    auto customNode = p.create<CustomDeviceNode>();
    auto device = p.getDefaultDevice();

    // Device is properly set
    REQUIRE(customNode->getDevice() == device);
    REQUIRE(customNode->deviceSubnode->getDevice() == device);

    // Build internal is called
    REQUIRE(customNode->buildInternalCalled);
    REQUIRE(customNode->deviceSubnode->buildInternalCalled);
    REQUIRE(customNode->hostSubnode->buildInternalCalled);
}

TEST_CASE("Host-runnable subnode has its device set if parent is not a device node") {
    // Create pipeline
    dai::Pipeline p;
    auto customNode = p.create<CustomThreadedHostNode>();
    auto device = p.getDefaultDevice();

    // Device is properly set
    REQUIRE(customNode->syncSubnode->getDevice() == device);
}

TEST_CASE("Lazy subnode added after parent creation is adopted into the pipeline") {
    dai::Pipeline p(false);
    auto customNode = p.create<LazyThreadedHostNode>();

    REQUIRE(p.getAllNodes().size() == 1);
    REQUIRE(customNode->getAllNodes().empty());

    auto& hostSubnode = customNode->ensureHostSubnode();

    REQUIRE(p.getAllNodes().size() == 2);
    REQUIRE(customNode->getAllNodes().size() == 1);
    REQUIRE_NOTHROW(hostSubnode->out.createOutputQueue());
}

TEST_CASE("Invalid non-host-runnable device subnode does not mutate parent graph") {
    dai::Pipeline p(false);
    auto customNode = p.create<RejectingThreadedHostNode>();

    REQUIRE(p.getAllNodes().size() == 1);
    REQUIRE(customNode->getAllNodes().empty());

    REQUIRE_THROWS(customNode->createInvalidDeviceSubnode());
    REQUIRE_FALSE(customNode->hasInvalidSubnode());
    REQUIRE(p.getAllNodes().size() == 1);
    REQUIRE(customNode->getAllNodes().empty());
}
