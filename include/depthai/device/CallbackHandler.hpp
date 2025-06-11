#pragma once

// std
#include <functional>
#include <memory>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

class CallbackHandler {
    std::thread t;
    std::atomic<bool> running{true};
    std::shared_ptr<XLinkConnection> connection;
    std::function<std::shared_ptr<ADatatype>(std::shared_ptr<ADatatype>)> callback;

   public:
    void setCallback(std::function<std::shared_ptr<ADatatype>(std::shared_ptr<ADatatype>)> cb);
    CallbackHandler(std::shared_ptr<XLinkConnection> conn,
                    const std::string& streamName,
                    std::function<std::shared_ptr<ADatatype>(std::shared_ptr<ADatatype>)> cb);
    ~CallbackHandler();
};

}  // namespace dai
