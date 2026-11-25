#!/bin/bash
# Writes the notices of the public build image (Dockerfile.sdk) into <out dir>: where the license texts and the
# source code of each part of the image are. The Dockerfile runs it as the last step of the release image, on the
# software that is installed in the image at that time, so the lists are always those of the image.
#
# Usage: write-notices.sh <out dir> <LICENSE file of depthai-core>
#
# Output:
#   NOTICE.md             The parts of the image, their license texts and their source code
#   ubuntu-packages.tsv   Each installed Ubuntu package: version, source package, source version, license file,
#                         link to the source on Launchpad
#   LICENSE-depthai-core  The license of the depthai-core files in the image
#
# Environment (set by Dockerfile.sdk): FASTCV_URL, FASTCV_SHA256, RVC4_OS_VERSION, RVC4_CORE_REVISION.
#
# The script stops with an error when a part has no license text: an installed Ubuntu package without
# /usr/share/doc/<package>/copyright that still has files in the image, numpy without its license file, or the
# sysroot without NOTICE.md.
set -euo pipefail

if [ $# -ne 2 ]; then
    echo "usage: $0 <out dir> <LICENSE file of depthai-core>" >&2
    exit 2
fi
OUT=$1
CORE_LICENSE=$2
: "${FASTCV_URL:?}" "${FASTCV_SHA256:?}" "${RVC4_OS_VERSION:?}"
CORE_REVISION=${RVC4_CORE_REVISION:-unknown}
SYSROOT=${DEPTHAI_DEVICE_RVC4_SYSROOT:-/opt/rvc4-sysroot}

mkdir -p "$OUT"
cp "$CORE_LICENSE" "$OUT/LICENSE-depthai-core"

# Ubuntu packages. A package without a copyright file is accepted only when the image has none of its files:
# Dockerfile.sdk removes the files of the C library packages of the cross compiler, so the image does not contain them.
packages_tsv="$OUT/ubuntu-packages.tsv"
removed=()
printf 'package\tversion\tsource\tsource-version\tlicense\tsource-code\n' > "$packages_tsv"
while IFS=$'\t' read -r package version source source_version; do
    copyright="/usr/share/doc/$package/copyright"
    if [ -f "$copyright" ]; then
        license=$copyright
    else
        left=$(dpkg -L "$package" | while read -r f; do if [ -f "$f" ] || [ -L "$f" ]; then echo "$f"; fi; done)
        if [ -n "$left" ]; then
            echo "error: package $package has files in the image but no $copyright:" >&2
            echo "$left" | head -5 >&2
            exit 1
        fi
        license="(not in the image: Dockerfile.sdk removes its files)"
        removed+=("$package")
    fi
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$package" "$version" "$source" "$source_version" "$license" \
        "https://launchpad.net/ubuntu/+source/$source/$source_version" >> "$packages_tsv"
done < <(dpkg-query -W -f='${Package}\t${Version}\t${source:Package}\t${source:Version}\n' | sort)
package_count=$(($(wc -l < "$packages_tsv") - 1))

# numpy, installed with pip from PyPI. Its wheel bundles OpenBLAS and the GCC runtime libraries libgfortran and
# libquadmath (numpy.libs); the license file of the wheel lists them with their licenses and sources.
numpy_info=$(python3 -I -c '
import importlib.metadata as m
d = m.distribution("numpy")
files = [f for f in (d.files or []) if f.name.startswith(("LICENSE", "COPYING"))]
print(d.version, d.locate_file(files[0]) if files else "")') || { echo "error: numpy is not installed" >&2; exit 1; }
read -r numpy_version numpy_license <<< "$numpy_info"
if [ -z "${numpy_license:-}" ] || [ ! -f "$numpy_license" ]; then
    echo "error: numpy $numpy_version has no license file" >&2
    exit 1
fi
numpy_dir=$(python3 -I -c 'import numpy, os; print(os.path.dirname(os.path.dirname(numpy.__file__)))')
openblas=$(ls "$numpy_dir"/numpy.libs/libscipy_openblas*.so 2>/dev/null | head -1 || true)
openblas_version=""
if [ -n "$openblas" ]; then
    openblas_version=$(grep -a -o -m1 'OpenBLAS [0-9][0-9.]*[0-9]' "$openblas" | head -1 | cut -d' ' -f2 || true)
    test -n "$openblas_version" || { echo "error: no OpenBLAS version in $openblas" >&2; exit 1; }
fi
bundled=$(cd "$numpy_dir/numpy.libs" 2>/dev/null && ls) || bundled=""

for f in "$SYSROOT/NOTICE.md" "$SYSROOT/MANIFEST.json" "$SYSROOT/usr/share/licenses/fastcv/LICENSE" \
    "$SYSROOT/usr/share/licenses/fastcv/NOTICE"; do
    test -f "$f" || { echo "error: no $f" >&2; exit 1; }
done

if [ "$CORE_REVISION" = unknown ]; then
    core_source="https://github.com/luxonis/depthai-core (the revision of this image is not known)"
else
    core_source="https://github.com/luxonis/depthai-core/tree/$CORE_REVISION/device/rvc4/docker"
fi

{
    cat <<EOF
# Notices of the DepthAI RVC4 firmware build image

This image builds the DepthAI RVC4 firmware: Ubuntu 22.04 with the cross compiler of Ubuntu, and the public sysroot
of Luxonis OS $RVC4_OS_VERSION. It is made from \`device/rvc4/docker/Dockerfile.sdk\` of depthai-core
(revision $CORE_REVISION). Each part of the image is under its own license. This file tells where the license texts
and the source code of each part are. The \`-dev\` image is the same image plus the sanitizer libraries of the cross
compiler (files of the Ubuntu packages in the list below).

| Part | Location in the image | License texts | Source code |
|---|---|---|---|
| $package_count Ubuntu 22.04 packages: the base system, the build tools, Python 3, the cross compiler \`g++-11-aarch64-linux-gnu\` and binutils | \`/usr\`, \`/etc\`, \`/var\` | \`/usr/share/doc/<package>/copyright\` | The source package and the version of each package, with a link to its source on Launchpad: \`ubuntu-packages.tsv\` in this folder |
| numpy $numpy_version (wheel from PyPI): headers and module for the Python bindings of the firmware | \`$numpy_dir/numpy\`, \`numpy.libs\` | \`$numpy_license\` | See "numpy" below |
| FastCV headers (Qualcomm Innovation Center) | \`$SYSROOT/usr/include/fastcv\` | \`$SYSROOT/usr/share/licenses/fastcv/LICENSE\` and \`NOTICE\` (BSD-3-Clause-Clear) | $FASTCV_URL (sha256 $FASTCV_SHA256) |
| depthai-core files: \`/usr/local/bin/install-hexagon-sdk.sh\` and this notice | \`/usr/local/bin\`, this folder | \`LICENSE-depthai-core\` in this folder (MIT) | $core_source |
| Public sysroot of Luxonis OS $RVC4_OS_VERSION | \`$SYSROOT\` | \`$SYSROOT/NOTICE.md\` lists each project and its license; \`$SYSROOT/MANIFEST.json\` lists each file | \`$SYSROOT/NOTICE.md\` |

## Ubuntu packages

Ubuntu keeps the source code of each package version in its archive. \`ubuntu-packages.tsv\` gives, for each
installed package, the source package and its version, and the Launchpad page of that source version, where the
source files can be downloaded. With \`deb-src\` entries in the APT sources, \`apt-get source <source>=<version>\` gets
the same source.
EOF
    if [ ${#removed[@]} -gt 0 ]; then
        cat <<EOF

These packages are installed, but the image does not contain their files: Dockerfile.sdk removes them in the same step
that installs them, because the firmware build uses the C library, libstdc++ and libgcc_s of the sysroot.
$(printf -- '- `%s`\n' "${removed[@]}")
EOF
    fi
    cat <<EOF

## numpy

numpy $numpy_version is the binary wheel of PyPI, without changes. Its license file (\`$numpy_license\`) has the
license of numpy and of each library that the wheel bundles, with the location of its source.

- numpy source: https://pypi.org/project/numpy/$numpy_version/#files (source archive) and
  https://github.com/numpy/numpy/tree/v$numpy_version
EOF
    if [ -n "$openblas_version" ]; then
        echo "- OpenBLAS $openblas_version (BSD-3-Clause), \`numpy.libs/$(basename "$openblas")\`: https://github.com/OpenMathLib/OpenBLAS/tree/v$openblas_version"
    fi
    for lib in $bundled; do
        case "$lib" in
            libgfortran*) echo "- GCC runtime library libgfortran (GPL-3.0-or-later WITH GCC-exception-3.1), \`numpy.libs/$lib\`: https://gcc.gnu.org/git/?p=gcc.git;a=tree;f=libgfortran" ;;
            libquadmath*) echo "- GCC quad-precision math library libquadmath (LGPL-2.1-or-later), \`numpy.libs/$lib\`: https://gcc.gnu.org/git/?p=gcc.git;a=tree;f=libquadmath" ;;
            libscipy_openblas*) ;;
            *) echo "- \`numpy.libs/$lib\`: see the license file of numpy" ;;
        esac
    done
    cat <<EOF

## Not in the image

- Qualcomm AI Runtime SDK (QAIRT) and Hexagon SDK: Luxonis may not redistribute them. The firmware build downloads the
  parts of QAIRT that it uses to a cache on the host after the user accepts the license terms of Qualcomm, and
  \`install-hexagon-sdk.sh\` installs a Hexagon SDK that the user got from Qualcomm.
- The prebuilt vendor module of the firmware: the firmware build downloads it after the user accepts its license
  agreement (see \`device/rvc4/README.md\` of depthai-core).
EOF
} > "$OUT/NOTICE.md"

echo "Notices written to $OUT: $package_count Ubuntu packages (${#removed[@]} without files in the image), numpy $numpy_version"
