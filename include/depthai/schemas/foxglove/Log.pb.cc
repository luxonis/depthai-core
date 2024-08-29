// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: foxglove/Log.proto

#include "foxglove/Log.pb.h"

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

inline constexpr Log::Impl_::Impl_(
    ::_pbi::ConstantInitialized) noexcept
      : _cached_size_{0},
        message_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        name_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        file_(
            &::google::protobuf::internal::fixed_address_empty_string,
            ::_pbi::ConstantInitialized()),
        timestamp_{nullptr},
        level_{static_cast< ::foxglove::Log_Level >(0)},
        line_{0u} {}

template <typename>
PROTOBUF_CONSTEXPR Log::Log(::_pbi::ConstantInitialized)
    : _impl_(::_pbi::ConstantInitialized()) {}
struct LogDefaultTypeInternal {
  PROTOBUF_CONSTEXPR LogDefaultTypeInternal() : _instance(::_pbi::ConstantInitialized{}) {}
  ~LogDefaultTypeInternal() {}
  union {
    Log _instance;
  };
};

PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT
    PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 LogDefaultTypeInternal _Log_default_instance_;
}  // namespace foxglove
static ::_pb::Metadata file_level_metadata_foxglove_2fLog_2eproto[1];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_foxglove_2fLog_2eproto[1];
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_foxglove_2fLog_2eproto = nullptr;
const ::uint32_t TableStruct_foxglove_2fLog_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(
    protodesc_cold) = {
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_._has_bits_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _internal_metadata_),
    ~0u,  // no _extensions_
    ~0u,  // no _oneof_case_
    ~0u,  // no _weak_field_map_
    ~0u,  // no _inlined_string_donated_
    ~0u,  // no _split_
    ~0u,  // no sizeof(Split)
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.timestamp_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.level_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.message_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.name_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.file_),
    PROTOBUF_FIELD_OFFSET(::foxglove::Log, _impl_.line_),
    0,
    ~0u,
    ~0u,
    ~0u,
    ~0u,
    ~0u,
};

static const ::_pbi::MigrationSchema
    schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
        {0, 14, -1, sizeof(::foxglove::Log)},
};

static const ::_pb::Message* const file_default_instances[] = {
    &::foxglove::_Log_default_instance_._instance,
};
const char descriptor_table_protodef_foxglove_2fLog_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\022foxglove/Log.proto\022\010foxglove\032\037google/p"
    "rotobuf/timestamp.proto\"\341\001\n\003Log\022-\n\ttimes"
    "tamp\030\001 \001(\0132\032.google.protobuf.Timestamp\022\""
    "\n\005level\030\002 \001(\0162\023.foxglove.Log.Level\022\017\n\007me"
    "ssage\030\003 \001(\t\022\014\n\004name\030\004 \001(\t\022\014\n\004file\030\005 \001(\t\022"
    "\014\n\004line\030\006 \001(\007\"L\n\005Level\022\013\n\007UNKNOWN\020\000\022\t\n\005D"
    "EBUG\020\001\022\010\n\004INFO\020\002\022\013\n\007WARNING\020\003\022\t\n\005ERROR\020\004"
    "\022\t\n\005FATAL\020\005b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_foxglove_2fLog_2eproto_deps[1] =
    {
        &::descriptor_table_google_2fprotobuf_2ftimestamp_2eproto,
};
static ::absl::once_flag descriptor_table_foxglove_2fLog_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_foxglove_2fLog_2eproto = {
    false,
    false,
    299,
    descriptor_table_protodef_foxglove_2fLog_2eproto,
    "foxglove/Log.proto",
    &descriptor_table_foxglove_2fLog_2eproto_once,
    descriptor_table_foxglove_2fLog_2eproto_deps,
    1,
    1,
    schemas,
    file_default_instances,
    TableStruct_foxglove_2fLog_2eproto::offsets,
    file_level_metadata_foxglove_2fLog_2eproto,
    file_level_enum_descriptors_foxglove_2fLog_2eproto,
    file_level_service_descriptors_foxglove_2fLog_2eproto,
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
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_foxglove_2fLog_2eproto_getter() {
  return &descriptor_table_foxglove_2fLog_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_foxglove_2fLog_2eproto(&descriptor_table_foxglove_2fLog_2eproto);
namespace foxglove {
const ::google::protobuf::EnumDescriptor* Log_Level_descriptor() {
  ::google::protobuf::internal::AssignDescriptors(&descriptor_table_foxglove_2fLog_2eproto);
  return file_level_enum_descriptors_foxglove_2fLog_2eproto[0];
}
PROTOBUF_CONSTINIT const uint32_t Log_Level_internal_data_[] = {
    393216u, 0u, };
bool Log_Level_IsValid(int value) {
  return 0 <= value && value <= 5;
}
#if (__cplusplus < 201703) && \
  (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

constexpr Log_Level Log::UNKNOWN;
constexpr Log_Level Log::DEBUG;
constexpr Log_Level Log::INFO;
constexpr Log_Level Log::WARNING;
constexpr Log_Level Log::ERROR;
constexpr Log_Level Log::FATAL;
constexpr Log_Level Log::Level_MIN;
constexpr Log_Level Log::Level_MAX;
constexpr int Log::Level_ARRAYSIZE;

#endif  // (__cplusplus < 201703) &&
        // (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
// ===================================================================

class Log::_Internal {
 public:
  using HasBits = decltype(std::declval<Log>()._impl_._has_bits_);
  static constexpr ::int32_t kHasBitsOffset =
    8 * PROTOBUF_FIELD_OFFSET(Log, _impl_._has_bits_);
  static const ::google::protobuf::Timestamp& timestamp(const Log* msg);
  static void set_has_timestamp(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::google::protobuf::Timestamp& Log::_Internal::timestamp(const Log* msg) {
  return *msg->_impl_.timestamp_;
}
void Log::clear_timestamp() {
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  if (_impl_.timestamp_ != nullptr) _impl_.timestamp_->Clear();
  _impl_._has_bits_[0] &= ~0x00000001u;
}
Log::Log(::google::protobuf::Arena* arena)
    : ::google::protobuf::Message(arena) {
  SharedCtor(arena);
  // @@protoc_insertion_point(arena_constructor:foxglove.Log)
}
inline PROTOBUF_NDEBUG_INLINE Log::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility, ::google::protobuf::Arena* arena,
    const Impl_& from)
      : _has_bits_{from._has_bits_},
        _cached_size_{0},
        message_(arena, from.message_),
        name_(arena, from.name_),
        file_(arena, from.file_) {}

Log::Log(
    ::google::protobuf::Arena* arena,
    const Log& from)
    : ::google::protobuf::Message(arena) {
  Log* const _this = this;
  (void)_this;
  _internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(
      from._internal_metadata_);
  new (&_impl_) Impl_(internal_visibility(), arena, from._impl_);
  ::uint32_t cached_has_bits = _impl_._has_bits_[0];
  _impl_.timestamp_ = (cached_has_bits & 0x00000001u)
                ? CreateMaybeMessage<::google::protobuf::Timestamp>(arena, *from._impl_.timestamp_)
                : nullptr;
  ::memcpy(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, level_),
           reinterpret_cast<const char *>(&from._impl_) +
               offsetof(Impl_, level_),
           offsetof(Impl_, line_) -
               offsetof(Impl_, level_) +
               sizeof(Impl_::line_));

  // @@protoc_insertion_point(copy_constructor:foxglove.Log)
}
inline PROTOBUF_NDEBUG_INLINE Log::Impl_::Impl_(
    ::google::protobuf::internal::InternalVisibility visibility,
    ::google::protobuf::Arena* arena)
      : _cached_size_{0},
        message_(arena),
        name_(arena),
        file_(arena) {}

inline void Log::SharedCtor(::_pb::Arena* arena) {
  new (&_impl_) Impl_(internal_visibility(), arena);
  ::memset(reinterpret_cast<char *>(&_impl_) +
               offsetof(Impl_, timestamp_),
           0,
           offsetof(Impl_, line_) -
               offsetof(Impl_, timestamp_) +
               sizeof(Impl_::line_));
}
Log::~Log() {
  // @@protoc_insertion_point(destructor:foxglove.Log)
  _internal_metadata_.Delete<::google::protobuf::UnknownFieldSet>();
  SharedDtor();
}
inline void Log::SharedDtor() {
  ABSL_DCHECK(GetArena() == nullptr);
  _impl_.message_.Destroy();
  _impl_.name_.Destroy();
  _impl_.file_.Destroy();
  delete _impl_.timestamp_;
  _impl_.~Impl_();
}

PROTOBUF_NOINLINE void Log::Clear() {
// @@protoc_insertion_point(message_clear_start:foxglove.Log)
  PROTOBUF_TSAN_WRITE(&_impl_._tsan_detect_race);
  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.message_.ClearToEmpty();
  _impl_.name_.ClearToEmpty();
  _impl_.file_.ClearToEmpty();
  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    ABSL_DCHECK(_impl_.timestamp_ != nullptr);
    _impl_.timestamp_->Clear();
  }
  ::memset(&_impl_.level_, 0, static_cast<::size_t>(
      reinterpret_cast<char*>(&_impl_.line_) -
      reinterpret_cast<char*>(&_impl_.level_)) + sizeof(_impl_.line_));
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::google::protobuf::UnknownFieldSet>();
}

const char* Log::_InternalParse(
    const char* ptr, ::_pbi::ParseContext* ctx) {
  ptr = ::_pbi::TcParser::ParseLoop(this, ptr, ctx, &_table_.header);
  return ptr;
}


PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1
const ::_pbi::TcParseTable<3, 6, 1, 36, 2> Log::_table_ = {
  {
    PROTOBUF_FIELD_OFFSET(Log, _impl_._has_bits_),
    0, // no _extensions_
    6, 56,  // max_field_number, fast_idx_mask
    offsetof(decltype(_table_), field_lookup_table),
    4294967232,  // skipmap
    offsetof(decltype(_table_), field_entries),
    6,  // num_field_entries
    1,  // num_aux_entries
    offsetof(decltype(_table_), aux_entries),
    &_Log_default_instance_._instance,
    ::_pbi::TcParser::GenericFallback,  // fallback
  }, {{
    {::_pbi::TcParser::MiniParse, {}},
    // .google.protobuf.Timestamp timestamp = 1;
    {::_pbi::TcParser::FastMtS1,
     {10, 0, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.timestamp_)}},
    // .foxglove.Log.Level level = 2;
    {::_pbi::TcParser::SingularVarintNoZag1<::uint32_t, offsetof(Log, _impl_.level_), 63>(),
     {16, 63, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.level_)}},
    // string message = 3;
    {::_pbi::TcParser::FastUS1,
     {26, 63, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.message_)}},
    // string name = 4;
    {::_pbi::TcParser::FastUS1,
     {34, 63, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.name_)}},
    // string file = 5;
    {::_pbi::TcParser::FastUS1,
     {42, 63, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.file_)}},
    // fixed32 line = 6;
    {::_pbi::TcParser::FastF32S1,
     {53, 63, 0, PROTOBUF_FIELD_OFFSET(Log, _impl_.line_)}},
    {::_pbi::TcParser::MiniParse, {}},
  }}, {{
    65535, 65535
  }}, {{
    // .google.protobuf.Timestamp timestamp = 1;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.timestamp_), _Internal::kHasBitsOffset + 0, 0,
    (0 | ::_fl::kFcOptional | ::_fl::kMessage | ::_fl::kTvTable)},
    // .foxglove.Log.Level level = 2;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.level_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kOpenEnum)},
    // string message = 3;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.message_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
    // string name = 4;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.name_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
    // string file = 5;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.file_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kUtf8String | ::_fl::kRepAString)},
    // fixed32 line = 6;
    {PROTOBUF_FIELD_OFFSET(Log, _impl_.line_), -1, 0,
    (0 | ::_fl::kFcSingular | ::_fl::kFixed32)},
  }}, {{
    {::_pbi::TcParser::GetTable<::google::protobuf::Timestamp>()},
  }}, {{
    "\14\0\0\7\4\4\0\0"
    "foxglove.Log"
    "message"
    "name"
    "file"
  }},
};

::uint8_t* Log::_InternalSerialize(
    ::uint8_t* target,
    ::google::protobuf::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:foxglove.Log)
  ::uint32_t cached_has_bits = 0;
  (void)cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  // .google.protobuf.Timestamp timestamp = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::InternalWriteMessage(
        1, _Internal::timestamp(this),
        _Internal::timestamp(this).GetCachedSize(), target, stream);
  }

  // .foxglove.Log.Level level = 2;
  if (this->_internal_level() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
        2, this->_internal_level(), target);
  }

  // string message = 3;
  if (!this->_internal_message().empty()) {
    const std::string& _s = this->_internal_message();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "foxglove.Log.message");
    target = stream->WriteStringMaybeAliased(3, _s, target);
  }

  // string name = 4;
  if (!this->_internal_name().empty()) {
    const std::string& _s = this->_internal_name();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "foxglove.Log.name");
    target = stream->WriteStringMaybeAliased(4, _s, target);
  }

  // string file = 5;
  if (!this->_internal_file().empty()) {
    const std::string& _s = this->_internal_file();
    ::google::protobuf::internal::WireFormatLite::VerifyUtf8String(
        _s.data(), static_cast<int>(_s.length()), ::google::protobuf::internal::WireFormatLite::SERIALIZE, "foxglove.Log.file");
    target = stream->WriteStringMaybeAliased(5, _s, target);
  }

  // fixed32 line = 6;
  if (this->_internal_line() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteFixed32ToArray(
        6, this->_internal_line(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target =
        ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
            _internal_metadata_.unknown_fields<::google::protobuf::UnknownFieldSet>(::google::protobuf::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:foxglove.Log)
  return target;
}

::size_t Log::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:foxglove.Log)
  ::size_t total_size = 0;

  ::uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string message = 3;
  if (!this->_internal_message().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_message());
  }

  // string name = 4;
  if (!this->_internal_name().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_name());
  }

  // string file = 5;
  if (!this->_internal_file().empty()) {
    total_size += 1 + ::google::protobuf::internal::WireFormatLite::StringSize(
                                    this->_internal_file());
  }

  // .google.protobuf.Timestamp timestamp = 1;
  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size +=
        1 + ::google::protobuf::internal::WireFormatLite::MessageSize(*_impl_.timestamp_);
  }

  // .foxglove.Log.Level level = 2;
  if (this->_internal_level() != 0) {
    total_size += 1 +
                  ::_pbi::WireFormatLite::EnumSize(this->_internal_level());
  }

  // fixed32 line = 6;
  if (this->_internal_line() != 0) {
    total_size += 5;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::google::protobuf::Message::ClassData Log::_class_data_ = {
    Log::MergeImpl,
    nullptr,  // OnDemandRegisterArenaDtor
};
const ::google::protobuf::Message::ClassData* Log::GetClassData() const {
  return &_class_data_;
}

void Log::MergeImpl(::google::protobuf::Message& to_msg, const ::google::protobuf::Message& from_msg) {
  auto* const _this = static_cast<Log*>(&to_msg);
  auto& from = static_cast<const Log&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:foxglove.Log)
  ABSL_DCHECK_NE(&from, _this);
  ::uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_message().empty()) {
    _this->_internal_set_message(from._internal_message());
  }
  if (!from._internal_name().empty()) {
    _this->_internal_set_name(from._internal_name());
  }
  if (!from._internal_file().empty()) {
    _this->_internal_set_file(from._internal_file());
  }
  if ((from._impl_._has_bits_[0] & 0x00000001u) != 0) {
    _this->_internal_mutable_timestamp()->::google::protobuf::Timestamp::MergeFrom(
        from._internal_timestamp());
  }
  if (from._internal_level() != 0) {
    _this->_internal_set_level(from._internal_level());
  }
  if (from._internal_line() != 0) {
    _this->_internal_set_line(from._internal_line());
  }
  _this->_internal_metadata_.MergeFrom<::google::protobuf::UnknownFieldSet>(from._internal_metadata_);
}

void Log::CopyFrom(const Log& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:foxglove.Log)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

PROTOBUF_NOINLINE bool Log::IsInitialized() const {
  return true;
}

::_pbi::CachedSize* Log::AccessCachedSize() const {
  return &_impl_._cached_size_;
}
void Log::InternalSwap(Log* PROTOBUF_RESTRICT other) {
  using std::swap;
  auto* arena = GetArena();
  ABSL_DCHECK_EQ(arena, other->GetArena());
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.message_, &other->_impl_.message_, arena);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.name_, &other->_impl_.name_, arena);
  ::_pbi::ArenaStringPtr::InternalSwap(&_impl_.file_, &other->_impl_.file_, arena);
  ::google::protobuf::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Log, _impl_.line_)
      + sizeof(Log::_impl_.line_)
      - PROTOBUF_FIELD_OFFSET(Log, _impl_.timestamp_)>(
          reinterpret_cast<char*>(&_impl_.timestamp_),
          reinterpret_cast<char*>(&other->_impl_.timestamp_));
}

::google::protobuf::Metadata Log::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_foxglove_2fLog_2eproto_getter, &descriptor_table_foxglove_2fLog_2eproto_once,
      file_level_metadata_foxglove_2fLog_2eproto[0]);
}
// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"