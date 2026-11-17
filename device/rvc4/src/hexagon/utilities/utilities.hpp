#pragma once

#include <string>

namespace hexagon {

// Name of a FastRPC error code (the AEE_* codes of the Hexagon SDK), for log messages
std::string errorToString(int errorCode);

}  // namespace hexagon
