#include <assert.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "nnet/tensor_info.hpp"

std::ostream &operator<<(std::ostream &os, TensorInfo const &t_info)
{
    os << "Name: " << t_info.tensor_name << std::endl;
    os << "Index: " << t_info.tensor_idx << std::endl;
    os << "Element type: " << type_to_string.at(t_info.tensor_data_type) << std::endl;
    os << "Element size: " << " " << t_info.tensor_element_size << ((t_info.tensor_element_size == 1) ? "byte" : " bytes") << std::endl;
    os << "Offset: " << t_info.tensor_offset << " " << ((t_info.tensor_offset <= 1) ? "byte" : " bytes") << std::endl;
    os << "Dimensions: ";
    os << "[";
    for (int idx = 0; idx < t_info.tensor_dimensions.size(); idx++)
    {
        auto dimension = t_info.tensor_dimensions[idx];
        if (idx == t_info.tensor_dimensions.size() - 1)
        {
            os << dimension << "]" << std::endl;
        }
        else
        {
            os << dimension << ", ";
        }
    }
    return os;
}