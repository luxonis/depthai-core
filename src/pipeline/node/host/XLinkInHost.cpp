#include "depthai/pipeline/node/host/XLinkInHost.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

namespace dai {
XLinkInHost::XLinkInHost(std::shared_ptr<XLinkConnection> conn, std::string streamName) : conn(std::move(conn)), streamName(std::move(streamName)){};

void XLinkInHost::run() {
    // Create a stream for the connection
    XLinkStream stream(std::move(conn), streamName, 1);
    while(isRunning()) {}
    }
}
}  // namespace dai