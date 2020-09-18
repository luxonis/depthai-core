#include "types.hpp"


unsigned size_of_type(const TensorDataType& type)
{
    auto it = c_type_size.find(type);
    assert(it != c_type_size.end());
    return it->second;
};

