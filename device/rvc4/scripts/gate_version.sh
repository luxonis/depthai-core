#!/bin/sh

gate_version_get() {
  json=$(wget -qO- http://localhost:11492/api/v1/version)
  version=$(echo "$json" | sed -n 's/.*"version_gate"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')
  if [ -z "$version" ]; then
    echo "ERROR: Could not parse version_gate from JSON: $json" >&2
    echo "0.0.0"
  else
    echo $version
  fi
}

gate_version_to_num() {
  # Convert version string to a zero-padded number for numeric comparison
  # For example: “0.0.23” → 000000023 by using three digits per field.
  # Beware max version number per field is 999 !!!
  IFS=.
  set -- $1
  printf "%03d%03d%03d\n" "$1" "$2" "$3"
}
