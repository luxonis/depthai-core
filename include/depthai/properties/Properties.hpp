#pragma once

#include "depthai/utility/Serialization.hpp"

namespace dai {

/// Base Properties structure
struct Properties {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type) const = 0;
    virtual ~Properties();
};

/// Serializable properties
template <typename Base, typename Derived>
struct PropertiesSerializable : Base {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type = SerializationType::LIBNOP) const override;
};

template <typename Base, typename Derived>  // Not inline: extern templates stop per-unit instantiation; see src/utility/serialization/
void PropertiesSerializable<Base, Derived>::serialize(std::vector<std::uint8_t>& data, SerializationType type) const {
    utility::serialize(static_cast<const Derived&>(*this), data, type);
}

}  // namespace dai