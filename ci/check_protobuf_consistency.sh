#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PROTO_HPP="${REPO_ROOT}/src/utility/ProtoSerialize.hpp"
PROTO_CPP="${REPO_ROOT}/src/utility/ProtoSerialize.cpp"
DATATYPE_SRC_DIR="${REPO_ROOT}/src/pipeline/datatype"

TMP_DIR="$(mktemp -d)"
cleanup() {
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

GET_DECLS="${TMP_DIR}/get_decls.txt"
GET_DEFS="${TMP_DIR}/get_defs.txt"
SET_DECLS="${TMP_DIR}/set_decls.txt"
SET_DEFS="${TMP_DIR}/set_defs.txt"
PROTO_DATATYPES="${TMP_DIR}/proto_datatypes.txt"
SCHEMA_ENUMS="${TMP_DIR}/schema_enums.txt"
DESER_TRUE_ENUMS="${TMP_DIR}/deser_true_enums.txt"

extract_get_specializations_declared() {
    rg -NoI --replace '$1' '^[[:space:]]*std::unique_ptr<google::protobuf::Message>[[:space:]]+getProtoMessage\(const ([A-Za-z0-9_]+)\*' "${PROTO_HPP}" \
        | grep -vx 'T' | sort -u > "${GET_DECLS}"
}

extract_get_specializations_defined() {
    rg -NoI --replace '$1' '^[[:space:]]*std::unique_ptr<google::protobuf::Message>[[:space:]]+getProtoMessage\(const ([A-Za-z0-9_]+)\*' "${PROTO_CPP}" \
        | grep -vx 'T' | sort -u > "${GET_DEFS}"
}

extract_set_specializations_declared() {
    rg -NoI --replace '$1' '^[[:space:]]*void[[:space:]]+setProtoMessage\(([A-Za-z0-9_]+)&' "${PROTO_HPP}" \
        | grep -vx 'T' | sort -u > "${SET_DECLS}"
}

extract_set_specializations_defined() {
    rg -NoI --replace '$1' '^[[:space:]]*void[[:space:]]+setProtoMessage\(([A-Za-z0-9_]+)&' "${PROTO_CPP}" \
        | grep -vx 'T' | sort -u > "${SET_DEFS}"
}

extract_proto_datatype_classes() {
    rg -NoI --replace '$1' '^[[:space:]]*std::vector<std::uint8_t>[[:space:]]+([A-Za-z0-9_]+)::serializeProto\([^)]*\)[[:space:]]+const[[:space:]]+\{' "${DATATYPE_SRC_DIR}" -g '*.cpp' \
        | sort -u > "${PROTO_DATATYPES}"
}

extract_schema_name_to_datatype_enums() {
    awk '
        /DatatypeEnum schemaNameToDatatype\(const std::string& schemaName\)/ { in_func = 1; next }
        in_func && /^\}/ { in_func = 0 }
        in_func {
            while(match($0, /DatatypeEnum::[A-Za-z0-9_]+/)) {
                print substr($0, RSTART + 14, RLENGTH - 14)
                $0 = substr($0, RSTART + RLENGTH)
            }
        }
    ' "${PROTO_CPP}" | sort -u > "${SCHEMA_ENUMS}"
}

extract_deserialization_supported_true_enums() {
    awk '
        /bool deserializationSupported\(DatatypeEnum datatype\)/ { in_func = 1; before_true = 1; next }
        in_func && /^\}/ { in_func = 0 }
        in_func {
            if($0 ~ /return true;/) before_true = 0
            if(before_true) {
                while(match($0, /case DatatypeEnum::[A-Za-z0-9_]+:/)) {
                    print substr($0, RSTART + 19, RLENGTH - 20)
                    $0 = substr($0, RSTART + RLENGTH)
                }
            }
        }
    ' "${PROTO_CPP}" | sort -u > "${DESER_TRUE_ENUMS}"
}

check_subset() {
    local subset="$1"
    local superset="$2"
    local description="$3"
    local missing
    missing="$(comm -23 "${subset}" "${superset}")"
    if [[ -n "${missing}" ]]; then
        echo "ERROR: ${description}"
        echo "${missing}" | sed 's/^/  - /'
        return 1
    fi
    return 0
}

check_equal_sets() {
    local left="$1"
    local right="$2"
    local description="$3"

    local only_left only_right
    only_left="$(comm -23 "${left}" "${right}")"
    only_right="$(comm -13 "${left}" "${right}")"

    if [[ -n "${only_left}" || -n "${only_right}" ]]; then
        echo "ERROR: ${description}"
        if [[ -n "${only_left}" ]]; then
            echo "  Present only in first set:"
            echo "${only_left}" | sed 's/^/    - /'
        fi
        if [[ -n "${only_right}" ]]; then
            echo "  Present only in second set:"
            echo "${only_right}" | sed 's/^/    - /'
        fi
        return 1
    fi
    return 0
}

main() {
    extract_get_specializations_declared
    extract_get_specializations_defined
    extract_set_specializations_declared
    extract_set_specializations_defined
    extract_proto_datatype_classes
    extract_schema_name_to_datatype_enums
    extract_deserialization_supported_true_enums

    local failed=0

    check_equal_sets "${GET_DECLS}" "${GET_DEFS}" \
        "Mismatch between declared and defined getProtoMessage specializations." || failed=1

    check_equal_sets "${SET_DECLS}" "${SET_DEFS}" \
        "Mismatch between declared and defined setProtoMessage specializations." || failed=1

    check_subset "${PROTO_DATATYPES}" "${GET_DECLS}" \
        "Datatypes implementing serializeProto are missing getProtoMessage specializations." || failed=1

    check_subset "${SCHEMA_ENUMS}" "${GET_DECLS}" \
        "schemaNameToDatatype maps to enums without matching getProtoMessage specializations." || failed=1

    check_subset "${DESER_TRUE_ENUMS}" "${SET_DECLS}" \
        "deserializationSupported(true) enums are missing setProtoMessage specializations." || failed=1

    check_subset "${DESER_TRUE_ENUMS}" "${SCHEMA_ENUMS}" \
        "deserializationSupported(true) enums are missing schemaNameToDatatype mappings." || failed=1

    if [[ "${failed}" -ne 0 ]]; then
        echo
        echo "Protobuf consistency check FAILED."
        return 1
    fi

    echo "Protobuf consistency check PASSED."
}

main "$@"
