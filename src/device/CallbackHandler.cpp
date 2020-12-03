#include "depthai/device/CallbackHandler.hpp"

#include "pipeline/datatype/StreamPacketParser.hpp"

namespace dai {

void CallbackHandler::setCallback(std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb) {
    callback = std::move(cb);
}

CallbackHandler::CallbackHandler(std::shared_ptr<XLinkConnection> conn,
                                 const std::string& streamName,
                                 std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb)
    : connection(std::move(conn)), callback(std::move(cb)) {
    // creates a thread which reads from queue and writes to xlink
    t = std::thread([this, streamName]() {
        try {
            // open stream with 1B write size (no writing will happen here)
            connection->openStream(streamName, XLINK_USB_BUFFER_MAX_SIZE);

            while(running) {
                // read packet
                auto* packet = connection->readFromStreamRaw(streamName);
                // parse packet
                auto data = parsePacket(packet);
                // release packet
                connection->readFromStreamRawRelease(streamName);

                // CALLBACK
                auto toSend = callback(data);

                auto serialized = serializeData(toSend);

                // Write packet back
                connection->writeToStream(streamName, serialized);
            }

            connection->closeStream(streamName);

        } catch(const std::exception& ex) {
            // TODO(themarpe) - throw an exception
            assert(0 && "TODO");
        }
    });
}

CallbackHandler::~CallbackHandler() {
    // detach from thread, because currently no way to unblock underlying XLinkReadData
    running = false;
    t.detach();
}

}  // namespace dai
