#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace utility {
namespace serialization {

// Member I of the libnop member list has the name I of DEPTHAI_SERIALIZE_FIELD_NAMES, because both
// come from the field list of the same *_DECL macro. The expressions match the inline macros.
template <typename Members, typename T, std::size_t... I>
void toJson(nlohmann::json& j, const T& t, std::index_sequence<I...>) {
    const char* const* names = depthaiSerializeFieldNames(&t);
    ((void)(j[names[I]] = Members::template At<I>::Resolve(t)), ...);
}

template <typename Members, typename T, std::size_t... I>
void fromJson(const nlohmann::json& j, T& t, std::index_sequence<I...>) {
    const char* const* names = depthaiSerializeFieldNames(&t);
    ((void)j.at(names[I]).get_to(*Members::template At<I>::Resolve(&t)), ...);
}

template <typename Members, typename T, std::size_t... I>
void toJsonOptional(nlohmann::json& j, const T& t, std::index_sequence<I...>) {
    const char* const* names = depthaiSerializeFieldNames(&t);
    ((void)nlohmann::to_json(j[names[I]], Members::template At<I>::Resolve(t)), ...);
}

template <typename Members, typename T, std::size_t... I>
void fromJsonOptional(const nlohmann::json& j, T& t, std::index_sequence<I...>) {
    const char* const* names = depthaiSerializeFieldNames(&t);
    ((j.contains(names[I]) ? (void)j[names[I]].get_to(*Members::template At<I>::Resolve(&t)) : (void)0), ...);
}

}  // namespace serialization
}  // namespace utility
}  // namespace dai

// Definitions for the *_DECL macros of depthai/utility/Serialization.hpp.
// Use them in the namespace of the type, with the type name as seen from that namespace.
// The member list is named directly: nop::MemberListTraits also requires a default constructor.
// Scope is empty in the namespace of the type, or that namespace followed by :: outside of it.
#define DEPTHAI_SERIALIZE_JSON_DEFINE(Scope, Type, Members, toJsonFn, fromJsonFn)                             \
    void Scope to_json(nlohmann::json& j, const Type& t) {                                                    \
        ::dai::utility::serialization::toJsonFn<Members>(j, t, std::make_index_sequence<Members::Count>{});   \
    }                                                                                                         \
    void Scope from_json(const nlohmann::json& j, Type& t) {                                                  \
        ::dai::utility::serialization::fromJsonFn<Members>(j, t, std::make_index_sequence<Members::Count>{}); \
    }

#define DEPTHAI_SERIALIZE_EXT_DEFINE(Type) DEPTHAI_SERIALIZE_JSON_DEFINE(, Type, ::nop::ExternalMemberTraits<Type>::MemberList, toJson, fromJson)

#define DEPTHAI_SERIALIZE_OPTIONAL_EXT_DEFINE(Type) \
    DEPTHAI_SERIALIZE_JSON_DEFINE(, Type, ::nop::ExternalMemberTraits<Type>::MemberList, toJsonOptional, fromJsonOptional)

#define DEPTHAI_SERIALIZE_OPTIONAL_DEFINE(Type) DEPTHAI_SERIALIZE_JSON_DEFINE(, Type, Type::NOP__MEMBERS, toJsonOptional, fromJsonOptional)

#define DEPTHAI_SERIALIZE_DEFINE(Type)                                          \
    DEPTHAI_SERIALIZE_JSON_DEFINE(, Type, Type::NOP__MEMBERS, toJson, fromJson) \
    std::string Type::str() const {                                             \
        nlohmann::json j = *this;                                               \
        return ::dai::utility::jsonDisplay(j);                                  \
    }

// One line for each PropertiesSerializable type, at global scope: an explicit instantiation cannot be in a nested
// namespace such as dai::beta. It defines the destructor (the key function, so the vtable is in this unit), the json
// conversions, and the instantiation that the extern template of the properties header refers to.
// Ns is the namespace of the type, for example dai or dai::beta.
#define DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(Ns, Name)                                                                  \
    Ns::Name::~Name() = default;                                                                                       \
    DEPTHAI_SERIALIZE_JSON_DEFINE(Ns::, Ns::Name, ::nop::ExternalMemberTraits<Ns::Name>::MemberList, toJson, fromJson) \
    template struct ::dai::PropertiesSerializable<::dai::Properties, Ns::Name>;
