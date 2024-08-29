// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: foxglove/Grid.proto

#include "foxglove/Grid.pb.h"

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

inline constexpr Grid::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : _cached_size_{0},
        fields_{},
        frame_id_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        data_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        timestamp_{nullptr},
        pose_{nullptr},
        cell_size_{nullptr},
        column_count_{0u},
        row_stride_{0u},
        cell_stride_{0u} {}

template <typename>
PROTOBUF_CONSTEXPR Grid::Grid(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct GridDefaultTypeInternal {
  PROTOBUF_CONSTEXPR GridDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~GridDefaultTypeInternal() {}
  union {
    Grid _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 GridDefaultTypeInternal _Grid_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_foxglove_2fGrid_2eproto[1];
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_foxglove_2fGrid_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_foxglove_2fGrid_2eproto = nullptr;
const ::uint32_t TableStruct_foxglove_2fGrid_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_._has_bits_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.timestamp_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.frame_id_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.pose_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.column_count_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.cell_size_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.row_stride_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.cell_stride_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.fields_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Grid, _impl_.data_),
    0,
    ~0u,
    1,
    ~0u,
    2,
    ~0u,
    ~0u,
    ~0u,
    ~0u,
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        {0, 17, -1, sizeof(::foxglove::Grid)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::foxglove::_Grid_default_instance_._instance,
};
const char descriptor_table_protodef_foxglove_2fGrid_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\023foxglove/Grid.proto\022\010foxglove\032!foxglov"
    "e/PackedElementField.proto\032\023foxglove/Pos"
    "e.proto\032\026foxglove/Vector2.proto\032\037google/"
    "protobuf/timestamp.proto\"\206\002\n\004Grid\022-\n\ttim"
    "estamp\030\001 \001(\0132\032.google.protobuf.Timestamp"
    "\022\020\n\010frame_id\030\002 \001(\t\022\034\n\004pose\030\003 \001(\0132\016.foxgl"
    "ove.Pose\022\024\n\014column_count\030\004 \001(\007\022$\n\tcell_s"
    "ize\030\005 \001(\0132\021.foxglove.Vector2\022\022\n\nrow_stri"
    "de\030\006 \001(\007\022\023\n\013cell_stride\030\007 \001(\007\022,\n\006fields\030"
    "\010 \003(\0132\034.foxglove.PackedElementField\022\014\n\004d"
    "ata\030\t \001(\014b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_foxglove_2fGrid_2eproto_deps[4] =
    {
        &::descriptor_table_foxglove_2fPackedElementField_2eproto,
        &::descriptor_table_foxglove_2fPose_2eproto,
        &::descriptor_table_foxglove_2fVector2_2eproto,
        &::descriptor_table_google_2fprotobuf_2ftimestamp_2eproto,
};
static ::absl::once_flag descriptor_table_foxglove_2fGrid_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_foxglove_2fGrid_2eproto = {
    false,
    false,
    417,
    descriptor_table_protodef_foxglove_2fGrid_2eproto,
    "foxglove/Grid.proto",
    &descriptor_table_foxglove_2fGrid_2eproto_once,
    descriptor_table_foxglove_2fGrid_2eproto_deps,
    4,
    1,
    schemas,
    file_default_instances,
    TableStruct_foxglove_2fGrid_2eproto::offsets,
    file_level_metadata_foxglove_2fGrid_2eproto,
    file_level_enum_descriptors_foxglove_2fGrid_2eproto,
    file_level_service_descriptors_foxglove_2fGrid_2eproto,
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
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_foxglove_2fGrid_2eproto_getter() {
  return &descriptor_table_foxglove_2fGrid_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_foxglove_2fGrid_2eproto(&descriptor_table_foxglove_2fGrid_2eproto);
namespace foxglove {
// ===================================================================

class Grid::_Internal {
 public:
  using HasBits = decltype(std::declval<Grid>()._impl_._has_bits_);
  static constexpr ::int32_t kHasBitsOffset =
    8 * PROTOBUF_FIELD_OFFSET(Grid, _impl_._has_bits_);
  static const ::google::protobuf::Timestamp& timestamp(const Grid* msg);
  static void set_has_timestamp(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::foxglove::Pose& pose(const Grid* msg);
  static void set_has_pose(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::foxglove::Vector2& cell_size(const Grid* msg);
  static void set_has_cell_size(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::google::protobuf::Timestamp& Grid::_Internal::timestamp(const Grid* msg) {
  return *msg->_impl_.timestamp_;
}
const ::foxglove::Pose& Grid::_Internal::pose(const Grid* msg) {
  return *msg->_impl_.pose_;
}
const ::foxglove::Vector2& Grid::_Internal::cell_size(const Grid* msg) {
  return *msg->_impl_.cell_size_;
}
void Grid::clear_timestamp() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.timestamp_ != nullptr) _impl_.timestamp_->Clear();
  _impl_._has_bits_[0] &= ~0x00000001u;
}
void Grid::clear_pose() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.pose_ != nullptr) _impl_.pose_->Clear();
  _impl_._has_bits_[0] &= ~0x00000002u;
}
void Grid::clear_cell_size() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.cell_size_ != nullptr) _impl_.cell_size_->Clear();
  _impl_._has_bits_[0] &= ~0x00000004u;
}
void Grid::clear_fields() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  _impl_.fields_.Clear();
}
Grid::Grid(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:foxglove.Grid)
}
inline PROTOBUF_NDEBUG_INLINE Grid::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : _has_bits_{from._has_bits_},
        _cached_size_{0},
        fields_{visibility, arena, from.fields_},
        frame_id_(arena, from.frame_id_),
        data_(arena, from.data_) {}

Grid::Grid(
    ::google::protobuf::Arena* arena,
    const Grid& from)
    : ::google::protobuf::Message(arena) {
  Grid* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);
  ::uint32_t cached_has_bits = _impl_._has_bits_[0];
  _impl_.timestamp_ = (cached_has_bits & 0x00000001u)
                ? CreateMaybeMessage<::google::protobuf::Timestamp>(arena, *from._impl_.timestamp_)
                : nullptr;
  _impl_.pose_ = (cached_has_bits & 0x00000002u)
                ? CreateMaybeMessage<::foxglove::Pose>(arena, *from._impl_.pose_)
                : nullptr;
  _impl_.cell_size_ = (cached_has_bits & 0x00000004u)
                ? CreateMaybeMessage<::foxglove::Vector2>(arena, *from._impl_.cell_size_)
                : nullptr;
  ::memcpy(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, column_count_),
           reinterpret_cast<const char *>(&from._impl_) +
               offsetof(Impl_, column_count_),
           offsetof(Impl_, cell_stride_) -
               offsetof(Impl_, column_count_) +
               sizeof(Impl_::cell_stride_));

  // @@protoc_insertion_point(copy_constructor:foxglove.Grid)
}
inline PROTOBUF_NDEBUG_INLINE Grid::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : _cached_size_{0},
        fields_{visibility, arena},
        frame_id_(arena),
        data_(arena) {}

inline void Grid::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  ::memset(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, timestamp_),
           0,
           offsetof(Impl_, cell_stride_) -
               offsetof(Impl_, timestamp_) +
               sizeof(Impl_::cell_stride_));
}
Grid::~Grid() {
  // @@protoc_insertion_point(destructor:foxglove.Grid)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void Grid::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  _impl_.frame_id_.Destroy();
  _impl_.data_.Destroy();
  delete _impl_.timestamp_;
  delete _impl_.pose_;
  delete _impl_.cell_size_;
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void Grid::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.Grid)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.fields_.Clear();
  _impl_.frame_id_.ClearToEmpty();
  _impl_.data_.ClearToEmpty();
  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      ABSL_DCHECK(_impl_.timestamp_ != nullptr);
      _impl_.timestamp_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      ABSL_DCHECK(_impl_.pose_ != nullptr);
      _impl_.pose_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      ABSL_DCHECK(_impl_.cell_size_ != nullptr);
      _impl_.cell_size_->Clear();
    }
  }
  ::memset(&_impl_.column_count_, 0, static_cast<::size_t>(
      reinterpret_cast<char*>(&_impl_.cell_stride_) -
      reinterpret_cast<char*>(&_impl_.column_count_)) + sizeof(_impl_.cell_stride_));
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* Grid::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<4, 9, 4, 38, 2> Grid::_table_ = {
  {
    PROTOBUF_FIELD_OFFSET(Grid, _impl_._has_bits_),
    0, // no _extensions_
    9, 120,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294966784,  // skipmap
    offsetof(decltype(_table_), field_entries),
    9,  // num_field_entries
    4,  // num_aux_entries
    offsetof(decltype(_table_), aux_entries),
    &_Grid_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    {::_pbi::TcParser::MiniParse, {}},
    // .google.protobuf.Timestamp timestamp = 1;
    {::_pbi::TcParser::FastMtS1,
     {10, 0, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.timestamp_)}},
    // string frame_id = 2;
    {::_pbi::TcParser::FastUS1,
     {18, 63, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.frame_id_)}},
    // .foxglove.Pose pose = 3;
    {::_pbi::TcParser::FastMtS1,
     {26, 1, 1, PROTOBUF_FIELD_OFFSET(Grid, _impl_.pose_)}},
    // fixed32 column_count = 4;
    {::_pbi::TcParser::FastF32S1,
     {37, 63, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.column_count_)}},
    // .foxglove.Vector2 cell_size = 5;
    {::_pbi::TcParser::FastMtS1,
     {42, 2, 2, PROTOBUF_FIELD_OFFSET(Grid, _impl_.cell_size_)}},
    // fixed32 row_stride = 6;
    {::_pbi::TcParser::FastF32S1,
     {53, 63, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.row_stride_)}},
    // fixed32 cell_stride = 7;
    {::_pbi::TcParser::FastF32S1,
     {61, 63, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.cell_stride_)}},
    // repeated .foxglove.PackedElementField fields = 8;
    {::_pbi::TcParser::FastMtR1,
     {66, 63, 3, PROTOBUF_FIELD_OFFSET(Grid, _impl_.fields_)}},
    // bytes data = 9;
    {::_pbi::TcParser::FastBS1,
     {74, 63, 0, PROTOBUF_FIELD_OFFSET(Grid, _impl_.data_)}},
    {::_pbi::TcParser::MiniParse, {}},
    {::_pbi::TcParser::MiniParse, {}},
    {::_pbi::TcParser::MiniParse, {}},
    {::_pbi::TcParser::MiniParse, {}},
    {::_pbi::TcParser::MiniParse, {}},
    {::_pbi::TcParser::MiniParse, {}},
  }}, {{
    65535, 65535
  }}, {{
    // .google.protobuf.Timestamp timestamp = 1;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.timestamp_), _Internal::kHasBitsOffset + 0, 0,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // string frame_id = 2;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.frame_id_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
    // .foxglove.Pose pose = 3;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.pose_), _Internal::kHasBitsOffset + 1, 1,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // fixed32 column_count = 4;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.column_count_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kFixed32)},
    // .foxglove.Vector2 cell_size = 5;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.cell_size_), _Internal::kHasBitsOffset + 2, 2,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // fixed32 row_stride = 6;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.row_stride_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kFixed32)},
    // fixed32 cell_stride = 7;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.cell_stride_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kFixed32)},
    // repeated .foxglove.PackedElementField fields = 8;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.fields_), -1, 3,
    (0 | ::_fl::kFcRepeated | ::_fl::kMessage | ::_fl::kTvTable)},
    // bytes data = 9;
    {PROTOBUF_FIELD_OFFSET(Grid, _impl_.data_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kBytes | ::_fl::kRepAString)},
  }}, {{
    {::_pbi::TcParser::GetTable<::google::protobuf::Timestamp>()},
    {::_pbi::TcParser::GetTable<::foxglove::Pose>()},
    {::_pbi::TcParser::GetTable<::foxglove::Vector2>()},
    {::_pbi::TcParser::GetTable<::foxglove::PackedElementField>()},
  }}, {{
    "\15\0\10\0\0\0\0\0\0\0\0\0\0\0\0\0"
    "foxglove.Grid"
    "frame_id"
  }},
};

::uint8_t* Grid::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.Grid)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // .google.protobuf.Timestamp timestamp = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        1, _Internal::timestamp(this),
        _Internal::timestamp(this).GetCachedSize(), target, stream);
  }

  // string frame_id = 2;
  if (!this->_internal_frame_id().empty()) {
    const std::string& _s = this->_internal_frame_id();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "foxglove.Grid.frame_id");
    target = stream->WriteStringMaybeAliased(2, _s, target);
  }

  // .foxglove.Pose pose = 3;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        3, _Internal::pose(this),
        _Internal::pose(this).GetCachedSize(), target, stream);
  }

  // fixed32 column_count = 4;
  if (this->_internal_column_count() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFixed32ToArray(
        4, this->_internal_column_count(), target);
  }

  // .foxglove.Vector2 cell_size = 5;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        5, _Internal::cell_size(this),
        _Internal::cell_size(this).GetCachedSize(), target, stream);
  }

  // fixed32 row_stride = 6;
  if (this->_internal_row_stride() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFixed32ToArray(
        6, this->_internal_row_stride(), target);
  }

  // fixed32 cell_stride = 7;
  if (this->_internal_cell_stride() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFixed32ToArray(
        7, this->_internal_cell_stride(), target);
  }

  // repeated .foxglove.PackedElementField fields = 8;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_fields_size()); i < n; i++) {
    const auto& repfield = this->_internal_fields().Get(i);
    target = ::google::protobuf::internal::WireFormatLite::
        InternalWriteMessage(8, repfield, repfield.GetCachedSize(), target, stream);
  }

  // bytes data = 9;
  if (!this->_internal_data().empty()) {
    const std::string& _s = this->_internal_data();
    target = stream->WriteBytesMaybeAliased(9, _s, target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.Grid)
  return target;
}

::size_t Grid::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.Grid)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .foxglove.PackedElementField fields = 8;
  total_size += 1UL * this->_internal_fields_size();
  for (const auto& msg : this->_internal_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSize(msg);
  }
  // string frame_id = 2;
  if (!this->_internal_frame_id().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_frame_id());
  }

  // bytes data = 9;
  if (!this->_internal_data().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::BytesSize(
                                    this->_internal_data());
  }

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // .google.protobuf.Timestamp timestamp = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.timestamp_);
    }

    // .foxglove.Pose pose = 3;
    if (cached_has_bits & 0x00000002u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.pose_);
    }

    // .foxglove.Vector2 cell_size = 5;
    if (cached_has_bits & 0x00000004u) {
      total_size +=
          1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.cell_size_);
    }

  }
  // fixed32 column_count = 4;
  if (this->_internal_column_count() != 0) {
    total_size += 5;
  }

  // fixed32 row_stride = 6;
  if (this->_internal_row_stride() != 0) {
    total_size += 5;
  }

  // fixed32 cell_stride = 7;
  if (this->_internal_cell_stride() != 0) {
    total_size += 5;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData Grid::_class_data_ = {
    Grid::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* Grid::GetClassData() const {
  return &_class_data_;
}

void Grid::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<Grid*>(&to_msg);
  auto& from = static_cast<const Grid&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.Grid)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_internal_mutable_fields()->MergeFrom(
      from._internal_fields());
  if (!from._internal_frame_id().empty()) {
    _this->_internal_set_frame_id(from._internal_frame_id());
  }
  if (!from._internal_data().empty()) {
    _this->_internal_set_data(from._internal_data());
  }
  cached_has_bits = from._impl_._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _this->_internal_mutable_timestamp()->::google::protobuf::Timestamp::MergeFrom(
          from._internal_timestamp());
    }
    if (cached_has_bits & 0x00000002u) {
      _this->_internal_mutable_pose()->::foxglove::Pose::MergeFrom(
          from._internal_pose());
    }
    if (cached_has_bits & 0x00000004u) {
      _this->_internal_mutable_cell_size()->::foxglove::Vector2::MergeFrom(
          from._internal_cell_size());
    }
  }
  if (from._internal_column_count() != 0) {
    _this->_internal_set_column_count(from._internal_column_count());
  }
  if (from._internal_row_stride() != 0) {
    _this->_internal_set_row_stride(from._internal_row_stride());
  }
  if (from._internal_cell_stride() != 0) {
    _this->_internal_set_cell_stride(from._internal_cell_stride());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void Grid::CopyFrom(const Grid& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.Grid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool Grid::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* Grid::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void Grid::InternalSwap(Grid* PROTOBUF_RESTRICT other) {
  using std::swap;
  auto* arena = GetArena();
  ABSL_DCHECK_EQ(arena, other->GetArena());
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  _impl_.fields_.InternalSwap(&other->_impl_.fields_);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.frame_id_, &other->_impl_.frame_id_, arena);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.data_, &other->_impl_.data_, arena);
  ::google::protobuf::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Grid, _impl_.cell_stride_)
      + sizeof(Grid::_impl_.cell_stride_)
      - PROTOBUF_FIELD_OFFSET(Grid, _impl_.timestamp_)>(
          reinterpret_cast<char*>(&_impl_.timestamp_),
          reinterpret_cast<char*>(&other->_impl_.timestamp_));
}

::google::protobuf::Metadata Grid::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_foxglove_2fGrid_2eproto_getter, &descriptor_table_foxglove_2fGrid_2eproto_once,
      file_level_metadata_foxglove_2fGrid_2eproto[0]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"