#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ENUM_HPP="${REPO_ROOT}/include/depthai/pipeline/datatype/DatatypeEnum.hpp"
ENUM_CPP="${REPO_ROOT}/src/pipeline/datatype/DatatypeEnum.cpp"
PARSER_CPP="${REPO_ROOT}/src/pipeline/datatype/StreamMessageParser.cpp"
DATATYPE_INCLUDE_DIR="${REPO_ROOT}/include/depthai/pipeline/datatype"
DATATYPE_SRC_DIR="${REPO_ROOT}/src/pipeline/datatype"

TMP_DIR="$(mktemp -d)"
cleanup() {
    rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

ENUM_VALUES="${TMP_DIR}/enum_values.txt"
HIERARCHY_KEYS="${TMP_DIR}/hierarchy_keys.txt"
HIERARCHY_CHILDREN="${TMP_DIR}/hierarchy_children.txt"
PARSER_CASES="${TMP_DIR}/parser_cases.txt"
RETURNED_ENUMS="${TMP_DIR}/returned_enums.txt"
ENUM_VALUES_NO_ADATATYPE="${TMP_DIR}/enum_values_no_adatype.txt"

extract_enum_values() {
    awk '
        /enum class DatatypeEnum/ { in_enum = 1; next }
        in_enum && /^[[:space:]]*};/ { in_enum = 0 }
        in_enum {
            sub(/\/\/.*/, "", $0)
            gsub(/[[:space:]]/, "", $0)
            if($0 == "") next
            sub(/,.*/, "", $0)
            if($0 != "") print $0
        }
    ' "${ENUM_HPP}" | sort -u > "${ENUM_VALUES}"
}

extract_hierarchy_keys() {
    sed -n 's/^[[:space:]]*{DatatypeEnum::\([A-Za-z0-9_]*\),.*/\1/p' "${ENUM_CPP}" | sort -u > "${HIERARCHY_KEYS}"
}

extract_hierarchy_children() {
    sed -n 's/^[[:space:]]*DatatypeEnum::\([A-Za-z0-9_]*\),.*/\1/p' "${ENUM_CPP}" | sort -u > "${HIERARCHY_CHILDREN}"
}

extract_parser_cases() {
    sed -n 's/^[[:space:]]*case DatatypeEnum::\([A-Za-z0-9_]*\):.*/\1/p' "${PARSER_CPP}" | sort -u > "${PARSER_CASES}"
}

extract_returned_datatype_enums() {
    rg -NoI --replace '$1' 'return DatatypeEnum::([A-Za-z0-9_]+);' \
        "${DATATYPE_INCLUDE_DIR}" "${DATATYPE_SRC_DIR}" -g '*.hpp' -g '*.cpp' \
        | sort -u > "${RETURNED_ENUMS}"
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

main() {
    extract_enum_values
    extract_hierarchy_keys
    extract_hierarchy_children
    extract_parser_cases
    extract_returned_datatype_enums

    grep -vx 'ADatatype' "${ENUM_VALUES}" > "${ENUM_VALUES_NO_ADATATYPE}"

    local failed=0

    check_subset "${ENUM_VALUES}" "${HIERARCHY_KEYS}" \
        "DatatypeEnum values missing as keys in DatatypeEnum.cpp hierarchy map." || failed=1

    check_subset "${ENUM_VALUES_NO_ADATATYPE}" "${HIERARCHY_CHILDREN}" \
        "DatatypeEnum values missing from DatatypeEnum.cpp hierarchy child entries." || failed=1

    check_subset "${ENUM_VALUES}" "${PARSER_CASES}" \
        "DatatypeEnum values missing from StreamMessageParser switch cases." || failed=1

    check_subset "${RETURNED_ENUMS}" "${ENUM_VALUES}" \
        "DatatypeEnum values returned by datatype classes are missing from DatatypeEnum.hpp." || failed=1

    if [[ "${failed}" -ne 0 ]]; then
        echo
        echo "Datatype enum consistency check FAILED."
        return 1
    fi

    echo "Datatype enum consistency check PASSED."
}

main "$@"
