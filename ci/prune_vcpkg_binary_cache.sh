#!/bin/bash
# Prune a vcpkg 'files' binary cache after a successful vcpkg install.
#
# Removes archives that the install did not use, so the saved CI cache holds only the
# current package ABIs and does not grow on each dependency, compiler or tool change.
# Writes added=<n>, removed=<n> and changed=true|false to $GITHUB_OUTPUT, so CI saves a
# new cache only when the content changed.
#
# The archive name is the package ABI hash: <cache>/<abi[0:2]>/<abi>.zip, and the
# installed packages list their ABI in <vcpkg-installed-dir>/vcpkg/status ("Abi: " lines).
#
# Usage: prune_vcpkg_binary_cache.sh <cache-dir> <vcpkg-installed-dir> <marker-file>
#   <marker-file> must be touched before the install; newer archives count as added.
set -euo pipefail

cache_dir="$1"
status_file="$2/vcpkg/status"
marker="$3"

# Git Bash on Windows: convert the Windows paths so find/test work on them.
if command -v cygpath >/dev/null 2>&1; then
    cache_dir=$(cygpath -u "$cache_dir")
    status_file=$(cygpath -u "$status_file")
    marker=$(cygpath -u "$marker")
fi

used_abis=$(grep '^Abi: ' "$status_file" | cut -d' ' -f2 | sort -u) || true
if [[ -z "$used_abis" ]]; then
    echo "error: no package ABIs found in $status_file" >&2
    exit 1
fi

added=0
removed=0
while IFS= read -r -d '' archive; do
    if ! grep -qxF "$(basename "$archive" .zip)" <<< "$used_abis"; then
        rm -f "$archive"
        removed=$((removed + 1))
    elif [[ "$archive" -nt "$marker" ]]; then
        added=$((added + 1))
    fi
done < <(find "$cache_dir" -name '*.zip' -print0 2>/dev/null)

kept=$(find "$cache_dir" -name '*.zip' 2>/dev/null | wc -l | tr -d ' ')
echo "vcpkg binary cache: $added archive(s) added, $removed removed, $kept kept ($(grep -c . <<< "$used_abis") packages installed)"
changed=false
if (( added > 0 || removed > 0 )); then
    changed=true
fi
{
    echo "added=$added"
    echo "removed=$removed"
    echo "changed=$changed"
} >> "${GITHUB_OUTPUT:-/dev/null}"
