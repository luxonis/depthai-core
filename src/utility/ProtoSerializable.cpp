#include "depthai/utility/ProtoSerializable.hpp"

namespace dai {

#if defined(__clang__)
ProtoSerializable::~ProtoSerializable() = default;
#endif

}  // namespace dai