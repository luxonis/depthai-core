// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

#include <string>

namespace dai {

bool initialize();
bool initialize(const std::string& additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
bool initialize(const char* additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
bool initialize(void* javavm);

}  // namespace dai
