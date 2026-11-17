#pragma once

#include "depthai/utility/Serialization.hpp"

namespace dai {

/// Base Properties structure
struct Properties {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type) const = 0;
    virtual std::unique_ptr<Properties> clone() const = 0;
    virtual ~Properties();
};

/// Serializable properties
template <typename Base, typename Derived>
struct PropertiesSerializable : Base {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type = SerializationType::LIBNOP) const override;

    virtual std::unique_ptr<Properties> clone() const override;
};

// Not inline, so that the extern template declarations in the properties headers stop every
// unit from instantiating the serializers. Explicit instantiations: src/utility/serialization/.
template <typename Base, typename Derived>
void PropertiesSerializable<Base, Derived>::serialize(std::vector<std::uint8_t>& data, SerializationType type) const {
    utility::serialize(static_cast<const Derived&>(*this), data, type);
}

template <typename Base, typename Derived>
std::unique_ptr<Properties> PropertiesSerializable<Base, Derived>::clone() const {
    return std::make_unique<Derived>(static_cast<const Derived&>(*this));
}

}  // namespace dai