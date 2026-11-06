#pragma once

#include <string>
#include <utility>

#include "nanorpc/core/client.h"

namespace dai {
namespace detail {

template <typename Client, typename... Args>
auto callDeviceRpc(Client& client, const std::string& name, Args&&... args) {
    // Otherwise a leading uint64_t payload selects nanorpc's explicit-ID
    // overload, sending the payload as the method ID instead of an argument.
    return client.call(name, nanorpc::core::hash_id(name), std::forward<Args>(args)...);
}

}  // namespace detail
}  // namespace dai
