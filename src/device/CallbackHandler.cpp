#include "depthai/device/CallbackHandler.hpp"

#include <cassert>

#include "depthai/xlink/XLinkConstants.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"
namespace dai {

void CallbackHandler::setCallback(std::function<std::shared_ptr<ADatatype>(std::shared_ptr<ADatatype>)> cb) {
    callback = std::move(cb);
}

CallbackHandler::CallbackHandler(std::shared_ptr<XLinkConnection> conn,
                                 const std::string& streamName,
                                 std::function<std::shared_ptr<ADatatype>(std::shared_ptr<ADatatype>)> cb)
    : connection(std::move(conn)), callback(std::move(cb)) {
    t = std::thread([this, streamName]() {
        try {
            XLinkStream stream(connection, streamName, device::XLINK_USB_BUFFER_MAX_SIZE);

            while(running) {
                auto packet = stream.readMove();
                const auto data = StreamMessageParser::parseMessage(&packet);

                auto toSend = callback(std::move(data));

                auto serialized = StreamMessageParser::serializeMessage(toSend);

                stream.write(serialized);
            }

        } catch(const std::exception&) {
            // TODO(themarpe) - throw an exception
            assert(0 && "TODO");
        }
    });
}

CallbackHandler::~CallbackHandler() {
    running = false;  // Detach: no way to unblock the underlying XLinkReadData
    t.detach();
}

}  // namespace dai
