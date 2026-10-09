#!/bin/bash
# Prune a vcpkg 'files' binary cache after a successful vcpkg install.
#
# Removes archives that the install did not use, so the saved CI cache holds only the
# current package ABIs and does not grow on each dependency or compiler change.
# Writes changed=true to $GITHUB_OUTPUT if archives were added or removed, so CI saves
# a new cache only when the content changed.
#
# Usage: prune_vcpkg_binary_cache.sh <cache-dir> <vcpkg-installed-dir> <marker-file>
#   <marker-file> must be touched before the install; newer archives count as added.
set -euo pipefail

cache_dir="$1"
status_file="$2/vcpkg/status"
marker="$3"

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

echo "vcpkg binary cache: $added archive(s) added, $removed removed"
changed=false
if (( added > 0 || removed > 0 )); then
    changed=true
fi
echo "changed=$changed" >> "${GITHUB_OUTPUT:-/dev/null}"
