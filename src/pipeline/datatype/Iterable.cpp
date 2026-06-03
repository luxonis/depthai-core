#include "depthai/pipeline/datatype/Iterable.hpp"

#include <stdexcept>

namespace dai {

std::size_t Iterable::getSize() const {
    return 1;
}

}  // namespace dai