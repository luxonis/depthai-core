#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

#include "common/ModelType.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/properties/SCRFDParserProperties.hpp"
#include "nn_archive/NNArchive.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "spdlog/fmt/fmt.h"
// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {

namespace node {

class SCRFDParser : public DeviceNodeCRTP<DeviceNode, SCRFDParser, SCRFDParserProperties>, public HostRunnable {
   private:
   public:
    constexpr static const char* NAME = "SCRFDParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    void run() override;

    //

    void buildInternal() override;
};

}  // namespace node

}  // namespace dai