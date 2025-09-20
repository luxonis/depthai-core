// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

#include <string>

#include "depthai/utility/export.hpp"
namespace dai {

DEPTHAI_API bool initialize();
DEPTHAI_API bool initialize(std::string additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
DEPTHAI_API bool initialize(const char* additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
DEPTHAI_API bool initialize(void* javavm);

}  // namespace dai
