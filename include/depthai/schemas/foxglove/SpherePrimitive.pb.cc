// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: foxglove/SpherePrimitive.proto

#include "foxglove/SpherePrimitive.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
#include "google/protobuf/generated_message_tctable_impl.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::google::protobuf;
namespace _pbi = ::google::protobuf::internal;
namespace _fl = ::google::protobuf::internal::field_layout;
namespace foxglove {

inline constexpr SpherePrimitive::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : _cached_size_{0},
        pose_{nullptr},
        size_{nullptr},
        color_{nullptr} {}

template <typename>
PROTOBUF_CONSTEXPR SpherePrimitive::SpherePrimitive(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct SpherePrimitiveDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SpherePrimitiveDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~SpherePrimitiveDefaultTypeInternal() {}
  union {
    SpherePrimitive _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SpherePrimitiveDefaultTypeInternal _SpherePrimitive_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_foxglove_2fSpherePrimitive_2eproto[1];
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_foxglove_2fSpherePrimitive_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_foxglove_2fSpherePrimitive_2eproto = nullptr;
const ::uint32_t TableStruct_foxglove_2fSpherePrimitive_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    PROTOBUF_FIELD_OFFSET(::foxglove::SpherePrimitive, _impl_._has_bits_),
    PROTOBUF_FIELD_OFFSET(::foxglove::SpherePrimitive, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::foxglove::SpherePrimitive, _impl_.pose_),
    PROTOBUF_FIELD_OFFSET(::foxglove::SpherePrimitive, _impl_.size_),
    PROTOBUF_FIELD_OFFSET(::foxglove::SpherePrimitive, _impl_.color_),
    0,
    1,
    2,
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        {0, 11, -1, sizeof(::foxglove::SpherePrimitive)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::foxglove::_SpherePrimitive_default_instance_._instance,
};
const char descriptor_table_protodef_foxglove_2fSpherePrimitive_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\036foxglove/SpherePrimitive.proto\022\010foxglo"
    "ve\032\024foxglove/Color.proto\032\023foxglove/Pose."
    "proto\032\026foxglove/Vector3.proto\"p\n\017SphereP"
    "rimitive\022\034\n\004pose\030\001 \001(\0132\016.foxglove.Pose\022\037"
    "\n\004size\030\002 \001(\0132\021.foxglove.Vector3\022\036\n\005color"
    "\030\003 \001(\0132\017.foxglove.Colorb\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_foxglove_2fSpherePrimitive_2eproto_deps[3] =
    {
        &::descriptor_table_foxglove_2fColor_2eproto,
        &::descriptor_table_foxglove_2fPose_2eproto,
        &::descriptor_table_foxglove_2fVector3_2eproto,
};
static ::absl::once_flag descriptor_table_foxglove_2fSpherePrimitive_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_foxglove_2fSpherePrimitive_2eproto = {
    false,
    false,
    231,
    descriptor_table_protodef_foxglove_2fSpherePrimitive_2eproto,
    "foxglove/SpherePrimitive.proto",
    &descriptor_table_foxglove_2fSpherePrimitive_2eproto_once,
    descriptor_table_foxglove_2fSpherePrimitive_2eproto_deps,
    3,
    1,
    schemas,
    file_default_instances,
    TableStruct_foxglove_2fSpherePrimitive_2eproto::offsets,
    file_level_metadata_foxglove_2fSpherePrimitive_2eproto,
    file_level_enum_descriptors_foxglove_2fSpherePrimitive_2eproto,
    file_level_service_descriptors_foxglove_2fSpherePrimitive_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_foxglove_2fSpherePrimitive_2eproto_getter() {
  return &descriptor_table_foxglove_2fSpherePrimitive_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_foxglove_2fSpherePrimitive_2eproto(&descriptor_table_foxglove_2fSpherePrimitive_2eproto);
namespace foxglove {
// ===================================================================

class SpherePrimitive::_Internal {
 public:
  using HasBits = decltype(std::declval<SpherePrimitive>()._impl_._has_bits_);
  static constexpr ::int32_t kHasBitsOffset =
    8 * PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_._has_bits_);
  static const ::foxglove::Pose& pose(const SpherePrimitive* msg);
  static void set_has_pose(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::foxglove::Vector3& size(const SpherePrimitive* msg);
  static void set_has_size(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::foxglove::Color& color(const SpherePrimitive* msg);
  static void set_has_color(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::foxglove::Pose& SpherePrimitive::_Internal::pose(const SpherePrimitive* msg) {
  return *msg->_impl_.pose_;
}
const ::foxglove::Vector3& SpherePrimitive::_Internal::size(const SpherePrimitive* msg) {
  return *msg->_impl_.size_;
}
const ::foxglove::Color& SpherePrimitive::_Internal::color(const SpherePrimitive* msg) {
  return *msg->_impl_.color_;
}
void SpherePrimitive::clear_pose() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.pose_ != nullptr) _impl_.pose_->Clear();
  _impl_._has_bits_[0] &= ~0x00000001u;
}
void SpherePrimitive::clear_size() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.size_ != nullptr) _impl_.size_->Clear();
  _impl_._has_bits_[0] &= ~0x00000002u;
}
void SpherePrimitive::clear_color() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.color_ != nullptr) _impl_.color_->Clear();
  _impl_._has_bits_[0] &= ~0x00000004u;
}
SpherePrimitive::SpherePrimitive(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:foxglove.SpherePrimitive)
}
inline PROTOBUF_NDEBUG_INLINE SpherePrimitive::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : _has_bits_{from._has_bits_},
        _cached_size_{0} {}

SpherePrimitive::SpherePrimitive(
    ::google::protobuf::Arena* arena,
    const SpherePrimitive& from)
    : ::google::protobuf::Message(arena) {
  SpherePrimitive* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);
  ::uint32_t cached_has_bits = _impl_._has_bits_[0];
  _impl_.pose_ = (cached_has_bits & 0x00000001u)
                ? CreateMaybeMessage<::foxglove::Pose>(arena, *from._impl_.pose_)
                : nullptr;
  _impl_.size_ = (cached_has_bits & 0x00000002u)
                ? CreateMaybeMessage<::foxglove::Vector3>(arena, *from._impl_.size_)
                : nullptr;
  _impl_.color_ = (cached_has_bits & 0x00000004u)
                ? CreateMaybeMessage<::foxglove::Color>(arena, *from._impl_.color_)
                : nullptr;

  // @@protoc_insertion_point(copy_constructor:foxglove.SpherePrimitive)
}
inline PROTOBUF_NDEBUG_INLINE SpherePrimitive::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : _cached_size_{0} {}

inline void SpherePrimitive::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  ::memset(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, pose_),
           0,
           offsetof(Impl_, color_) -
               offsetof(Impl_, pose_) +
               sizeof(Impl_::color_));
}
SpherePrimitive::~SpherePrimitive() {
  // @@protoc_insertion_point(destructor:foxglove.SpherePrimitive)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void SpherePrimitive::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  delete _impl_.pose_;
  delete _impl_.size_;
  delete _impl_.color_;
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void SpherePrimitive::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.SpherePrimitive)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      ABSL_DCHECK(_impl_.pose_ != nullptr);
      _impl_.pose_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      ABSL_DCHECK(_impl_.size_ != nullptr);
      _impl_.size_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      ABSL_DCHECK(_impl_.color_ != nullptr);
      _impl_.color_->Clear();
    }
  }
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* SpherePrimitive::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<2, 3, 3, 0, 2> SpherePrimitive::_table_ = {
  {
    PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_._has_bits_),
    0, // no _extensions_
    3, 24,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967288,  // skipmap
    offsetof(decltype(_table_), field_entries),
    3,  // num_field_entries
    3,  // num_aux_entries
    offsetof(decltype(_table_), aux_entries),
    &_SpherePrimitive_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    {::_pbi::TcParser::MiniParse, {}},
    // .foxglove.Pose pose = 1;
    {::_pbi::TcParser::FastMtS1,
     {10, 0, 0, PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.pose_)}},
    // .foxglove.Vector3 size = 2;
    {::_pbi::TcParser::FastMtS1,
     {18, 1, 1, PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.size_)}},
    // .foxglove.Color color = 3;
    {::_pbi::TcParser::FastMtS1,
     {26, 2, 2, PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.color_)}},
  }}, {{
    65535, 65535
  }}, {{
    // .foxglove.Pose pose = 1;
    {PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.pose_), _Internal::kHasBitsOffset + 0, 0,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // .foxglove.Vector3 size = 2;
    {PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.size_), _Internal::kHasBitsOffset + 1, 1,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // .foxglove.Color color = 3;
    {PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.color_), _Internal::kHasBitsOffset + 2, 2,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
  }}, {{
    {::_pbi::TcParser::GetTable<::foxglove::Pose>()},
    {::_pbi::TcParser::GetTable<::foxglove::Vector3>()},
    {::_pbi::TcParser::GetTable<::foxglove::Color>()},
  }}, {{
  }},
};

::uint8_t* SpherePrimitive::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.SpherePrimitive)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // .foxglove.Pose pose = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        1, _Internal::pose(this),
        _Internal::pose(this).GetCachedSize(), target, stream);
  }

  // .foxglove.Vector3 size = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        2, _Internal::size(this),
        _Internal::size(this).GetCachedSize(), target, stream);
  }

  // .foxglove.Color color = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        3, _Internal::color(this),
        _Internal::color(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.SpherePrimitive)
  return target;
}

::size_t SpherePrimitive::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.SpherePrimitive)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // .foxglove.Pose pose = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.pose_);
    }

    // .foxglove.Vector3 size = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.size_);
    }

    // .foxglove.Color color = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.color_);
    }

  }
  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData SpherePrimitive::_class_data_ = {
    SpherePrimitive::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* SpherePrimitive::GetClassData() const {
  return &_class_data_;
}

void SpherePrimitive::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<SpherePrimitive*>(&to_msg);
  auto& from = static_cast<const SpherePrimitive&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.SpherePrimitive)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _this->_internal_mutable_pose()->::foxglove::Pose::MergeFrom(
          from._internal_pose());
    }
    if (cached_has_bits & 0x00000002u) {
      _this->_internal_mutable_size()->::foxglove::Vector3::MergeFrom(
          from._internal_size());
    }
    if (cached_has_bits & 0x00000004u) {
      _this->_internal_mutable_color()->::foxglove::Color::MergeFrom(
          from._internal_color());
    }
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void SpherePrimitive::CopyFrom(const SpherePrimitive& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.SpherePrimitive)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool SpherePrimitive::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* SpherePrimitive::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void SpherePrimitive::InternalSwap(SpherePrimitive* PROTOBUF_RESTRICT other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  ::google::protobuf::internal::memswap<
      PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.color_)
      + sizeof(SpherePrimitive::_impl_.color_)
      - PROTOBUF_FIELD_OFFSET(SpherePrimitive, _impl_.pose_)>(
          reinterpret_cast<char*>(&_impl_.pose_),
          reinterpret_cast<char*>(&other->_impl_.pose_));
}

::google::protobuf::Metadata SpherePrimitive::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_foxglove_2fSpherePrimitive_2eproto_getter, &descriptor_table_foxglove_2fSpherePrimitive_2eproto_once,
      file_level_metadata_foxglove_2fSpherePrimitive_2eproto[0]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"