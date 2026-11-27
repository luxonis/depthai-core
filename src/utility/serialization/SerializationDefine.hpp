#pragma once

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "depthai/utility/Serialization.hpp"

namespace dai {
namespace utility {
namespace serialization {

template <typename Members, typename T, std::size_t... I>  // Member I has name I: both come from the field list of the same *_DECL macro
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

#define DEPTHAI_SERIALIZE_PROPERTIES_DEFINE(Ns, Name)                                                                  \
    Ns::Name::~Name() = default;                                                                                       \
    DEPTHAI_SERIALIZE_JSON_DEFINE(Ns::, Ns::Name, ::nop::ExternalMemberTraits<Ns::Name>::MemberList, toJson, fromJson) \
    template struct ::dai::PropertiesSerializable<::dai::Properties, Ns::Name>;  // Use at global scope: this instantiation cannot be in dai::beta
