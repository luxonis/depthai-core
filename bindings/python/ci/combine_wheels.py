#!/usr/bin/env python3

import logging
import zipfile
import argparse
import glob
import os
import time
import tempfile
import subprocess
import sys
import re

from collections import defaultdict
from dataclasses import dataclass

_logger = logging.getLogger("merger")


def _extract_versions(tag: str) -> tuple[int, ...]:
    """Extract versions from a tag.

    >>> _extract_versions("cp39")
    (39,)
    >>> _extract_versions("cp36.cp37t")
    (36, 37)
    >>> _extract_versions("")
    ()
    """
    versions = list()
    REGEX = r"^[a-zA-Z]+(\d+)"
    for part in tag.split("."):
        match = re.match(REGEX, part)
        assert match is not None, (
            f"Expected {REGEX} match for part: {part} (tag: {tag})"
        )
        versions.append(int(match.group(1)))
    return tuple(versions)


@dataclass
class WheelInfo:
    """
    Wheel information, extracted from the wheel name.
    For example, for the wheel name: depthai-2.30.0.0-cp313-cp313t-manylinux_2_28_x86_64.whl
    corresponds to the following information:
        wheel_dvb: "depthai-2.30.0.0"
        python_tag: "cp313"
        abi_tag: "cp313t"
        platform_tag: "manylinux_2_28_x86_64"
    """

    wheel_dvb: str
    python_tag: str
    abi_tag: str
    platform_tag: str

    @property
    def wheel_name(self) -> str:
        return (
            f"{self.wheel_dvb}-{self.python_tag}-{self.abi_tag}-{self.platform_tag}.whl"
        )

    @property
    def python_versions(self) -> tuple[int, ...]:
        return _extract_versions(self.python_tag)

    @property
    def abi_versions(self) -> tuple[int, ...]:
        return _extract_versions(self.abi_tag)

    @staticmethod
    def from_wheel_name(wheel_name: str) -> "WheelInfo":
        # All wheels end with .whl
        if not wheel_name.endswith(".whl"):
            raise ValueError(f"Wheel name must end with .whl: {wheel_name}")

        # Remove the .whl extension
        wheel_name = wheel_name[: -len(".whl")]

        # Split wheel into parts: <wheel_dvb>-<python_tag>-<abi_tag>-<platform_tag>
        parts = wheel_name.split("-")
        if not len(parts) >= 4:
            raise ValueError(f"Wheel name must have at least 4 parts: {wheel_name}")

        return WheelInfo(
            wheel_dvb="-".join(parts[:-3]),
            python_tag=parts[-3],
            abi_tag=parts[-2],
            platform_tag=parts[-1],
        )

    @staticmethod
    def combine(infos: list["WheelInfo"], sort: bool = True) -> "WheelInfo":
        assert len(infos) > 0, "Expected at least one wheel info"
        infos = sorted(infos) if sort else infos

        # Basic sanity checks
        assert all(info.wheel_dvb == infos[0].wheel_dvb for info in infos)
        assert all(info.platform_tag == infos[0].platform_tag for info in infos)

        # Wheel dvb and platform tags remain the same
        combined_wheel_dvb = infos[0].wheel_dvb
        combined_platform_tag = infos[0].platform_tag

        # Combine python and abi tags
        combined_python_tag = ".".join([info.python_tag for info in infos])
        combined_abi_tag = ".".join([info.abi_tag for info in infos])

        return WheelInfo(
            wheel_dvb=combined_wheel_dvb,
            python_tag=combined_python_tag,
            abi_tag=combined_abi_tag,
            platform_tag=combined_platform_tag,
        )

    def __eq__(self, other):
        assert isinstance(other, WheelInfo), f"Expected WheelInfo, got {type(other)}"
        return self.wheel_name == other.wheel_name

    def __lt__(self, other):
        assert isinstance(other, WheelInfo), f"Expected WheelInfo, got {type(other)}"
        my_versions = [self.python_versions, self.abi_versions]
        other_versions = [other.python_versions, other.abi_versions]
        return my_versions < other_versions


def find_all_wheels(input_folder: str, sort: bool = True) -> list[WheelInfo]:
    # Find all wheels: just the filenames, not the entire paths
    wheel_files = glob.glob(os.path.join(input_folder, "*.whl"))
    wheel_files = [os.path.basename(file) for file in wheel_files]

    # Extract wheel information and sort them
    infos = [WheelInfo.from_wheel_name(file) for file in wheel_files]
    infos = sorted(infos) if sort else infos

    return infos


def _combine_wheels_linux(input_folder, output_folder, strip):
    _logger.info("Combining wheels for Linux!")
    _logger.info(f"Stripping unneeded debug symbols: {strip}")

    # Find all wheel infos in the input folder
    infos = find_all_wheels(input_folder)
    assert len(infos) > 0, "Expected at least one wheel info"
    _logger.debug(
        f"Found infos in {input_folder}:\n{'\n'.join([str(info) for info in infos])}"
    )

    # Combined output wheel info
    combined_info = WheelInfo.combine(infos)
    _logger.info(f"Combined wheel name: {combined_info.wheel_name}")
    _logger.debug(f"Combined info: {combined_info}")

    def _patch_and_strip(file_path, dynlib_map, strip_debug):
        """
        Patch binary file to replace dynamic library dependencies and optionally strip debug symbols.
        """
        for old_lib, new_lib in dynlib_map.items():
            _logger.debug(f"Patching {file_path} to replace {old_lib} with {new_lib}")
            subprocess.run(
                ["patchelf", "--replace-needed", old_lib, new_lib, file_path],
                check=True,
            )
        if strip_debug:
            _logger.info(f"Stripping debug symbols from {os.path.basename(file_path)}")
            for strip_cmd in [["strip", "--strip-unneeded"], ["llvm-strip", "--strip-unneeded"]]:
                try:
                    subprocess.run(strip_cmd + [file_path], check=True)
                except Exception as e:
                    _logger.debug(f"Stripping failed for {file_path}: {e}")

    def _unmangle_libs(lib_dir, rename_map, magic_hash):
        """
        libusb-1.0.3323.so -> libusb-1.0.3323-<magic_hash>.so
        Unmangle library names using the given magic hash and fill the rename map.
        Returns a list of new unmangled library names.
        """
        new_libs = []
        for lib in os.listdir(lib_dir):
            base, ext = lib.split(".", 1)
            base = "".join(base.split("-")[:-1])
            new_name = f"{base}-{magic_hash}.{ext}"
            rename_map[lib] = new_name
            new_libs.append((lib, new_name))
        return new_libs

    common_magic_hash = str(time.perf_counter())[-5:]

    with tempfile.TemporaryDirectory() as temp_dir:
        dynlib_renames = defaultdict(dict)

        output_zip_path = os.path.join(
            output_folder,
            f"{combined_info.wheel_dvb}-{combined_info.python_tag}-{combined_info.abi_tag}-{combined_info.platform_tag}.whl"
        )
        _logger.info(f"Combined wheel name: {os.path.basename(output_zip_path)}")
        output_zip = zipfile.ZipFile(output_zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9)

        for wheel_info in infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            # Extract wheel contents
            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            files = os.listdir(wheel_extract_dir)
            so_files = [f for f in files if f.endswith(".so") and "cpython" in f]
            libs_dirs = [f for f in files if f.endswith(".libs")]

            assert len(so_files) == 1, f"Expected 1 .so file, got {len(so_files)}"
            assert len(libs_dirs) == 1, f"Expected 1 .libs folder, got {len(libs_dirs)}"
            so_file = so_files[0]
            libs_dir = libs_dirs[0]

            # Gather files for the output wheel (Python files, metadata, etc)
            wheel_files_to_copy = [so_file] + [
                f for f in files if not f.endswith(".so") and not f.endswith(".libs")
            ]

            # Unmangle wheel's bundled libraries
            lib_dir_path = os.path.join(wheel_extract_dir, libs_dir)
            dynlib_renames[wheel_info.wheel_name] = {}
            unmangled = _unmangle_libs(lib_dir_path, dynlib_renames[wheel_info.wheel_name], common_magic_hash)

            # Actually rename the files and patch the binaries
            # First, patch the .so and the .so inside .libs
            libs_full_paths = [
                os.path.join(lib_dir_path, old_lib)
                for old_lib, _ in unmangled
            ]
            files_to_patch = [os.path.join(wheel_extract_dir, so_file)] + libs_full_paths
            for bin_file in files_to_patch:
                _patch_and_strip(
                    bin_file,
                    dynlib_renames[wheel_info.wheel_name],
                    strip
                )

            # Now rename the libs
            for old_lib, new_lib in unmangled:
                src = os.path.join(lib_dir_path, old_lib)
                dst = os.path.join(lib_dir_path, new_lib)
                os.rename(src, dst)
                if strip:
                    _patch_and_strip(dst, {}, strip)

            # Write main files to combined wheel
            for filename in wheel_files_to_copy:
                write_to_zip(output_zip, wheel_extract_dir, filename)
            # Write the unmangled libs folder
            write_to_zip(output_zip, wheel_extract_dir, libs_dir)

        output_zip.close()
        _logger.info("Output zip closed")
        _logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        _logger.info(f"Combined wheel saved to {output_zip_path}")


def _combine_wheels_windows(args, wheel_infos):
    _logger.info("Combining wheels for Windows!")
    from delvewheel import _dll_utils as dll_utils

    if args.strip_unneeded:
        _logger.warning(
            "Stripping unneeded debug symbols is not supported on Windows. Ignoring --strip-unneeded."
        )
        args.strip_unneeded = False

    # Make sure that on linux, all the wheels have the same platform tag
    platform_tags = set(wheel_info.platform_tag for wheel_info in wheel_infos)
    if len(platform_tags) > 1:
        raise ValueError(
            f"All wheels must have the same platform tag. Found: {platform_tags}"
        )

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:
        dynlib_renames = defaultdict(lambda: defaultdict(dict))

        common_magic_hash = str(time.perf_counter())[-5:]

        combined_python_tag = ".".join(
            [wheel_info.python_tag for wheel_info in wheel_infos]
        )
        combined_abi_tag = ".".join([wheel_info.abi_tag for wheel_info in wheel_infos])
        combined_platform_tag = wheel_infos[0].platform_tag

        _logger.info(f"Combined python tag: {combined_python_tag}")
        _logger.info(f"Combined abi tag: {combined_abi_tag}")
        _logger.info(f"Combined platform tag: {combined_platform_tag}")
        _logger.info(f"Wheel DVB: {wheel_infos[0].wheel_dvb}")

        # Create a zip file for the combined wheel
        combined_wheel_name = f"{wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        _logger.info(f"Combined wheel name: {combined_wheel_name}")

        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(
            output_zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9
        )

        # Extract each wheel into a subdirectory named after the wheel
        for wheel_info in wheel_infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            ## Get the path to the wheel's dynamic library
            extracted_files = os.listdir(wheel_extract_dir)
            wheel_dylib_path = [f for f in extracted_files if f.endswith(".pyd")]
            assert len(wheel_dylib_path) == 1, (
                f"Expected 1 .pyd file, got {len(wheel_dylib_path)}"
            )
            wheel_dylib_path = wheel_dylib_path[0]

            ## get .libs folder
            wheel_libs_path = [f for f in extracted_files if f.endswith(".data")]
            assert len(wheel_libs_path) == 1, (
                f"Expected 1 .data folder, got {len(wheel_libs_path)}"
            )
            wheel_libs_path = wheel_libs_path[0]

            ## get python dll
            python_dll_path = [
                f
                for f in extracted_files
                if f.endswith(".dll") and f.startswith("python3")
            ]
            assert len(python_dll_path) == 1, (
                f"Expected 1 python3.dll file, got {len(python_dll_path)}"
            )
            python_dll_path = python_dll_path[0]

            ## All folders and files that will be copied to the output folder
            wheel_files_to_copy = [wheel_dylib_path, python_dll_path]
            wheel_files_to_copy.extend(
                [
                    f
                    for f in extracted_files
                    if not f.endswith(".pyd")
                    and not f.endswith(".dll")
                    and not f.endswith(".data")
                ]
            )

            ## Unmangle the wheel's dynamic library names
            bundled_libs = os.listdir(
                os.path.join(wheel_extract_dir, wheel_libs_path, "platlib")
            )
            unmangled_libs = list()
            for lib in bundled_libs:
                base, ext = ".".join(lib.split(".")[:-1]), "." + lib.split(".")[-1]
                base = (
                    "".join(base.split("-")[:-1]) if len(base.split("-")) > 1 else base
                )  # remove hash, or reuse base name if there is no hash
                unmangled_libs.append(base + "-" + common_magic_hash + ext)
                dynlib_renames[wheel_info.wheel_name][lib] = unmangled_libs[-1]

            # Update the dynamic library to link against the unmangled libraries
            wheel_dylib_full_path = os.path.join(wheel_extract_dir, wheel_dylib_path)
            for file in [wheel_dylib_full_path] + [
                os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", f)
                for f in os.listdir(
                    os.path.join(wheel_extract_dir, wheel_libs_path, "platlib")
                )
            ]:
                try:
                    _logger.info(f"Patching {file}")
                    needed = dll_utils.get_direct_mangleable_needed(file, {}, {})
                    name_map = {
                        old_lib: dynlib_renames[wheel_info.wheel_name][old_lib]
                        for old_lib in needed
                    }
                    dll_utils.replace_needed(file, needed, name_map, strip=False)
                except Exception as e:
                    _logger.error(f"Error patching {file}: {e}")
                    continue

            for file in wheel_files_to_copy:
                write_to_zip(output_zip, wheel_extract_dir, file)

            for lib in bundled_libs:
                lib_path = os.path.join(
                    wheel_extract_dir, wheel_libs_path, "platlib", lib
                )
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(
                    wheel_extract_dir, wheel_libs_path, "platlib", new_lib_name
                )
                _logger.info(f"Renaming {lib_path} to {new_lib_path}")
                os.rename(lib_path, new_lib_path)

            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path)

        output_zip.close()
        _logger.info("Output zip closed")
        _logger.info(
            f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB"
        )
        _logger.info(f"Combined wheel saved to {output_zip_path}")


def _combine_wheels_macos(input_folder, output_folder, strip):
    _logger.info("Combining wheels for macOS!")

    ## Filter wheel infos based on their platform tag.
    unique_tags = list(set(wheel_info.platform_tag for wheel_info in all_wheel_infos))

    _logger.info(f"Found {len(unique_tags)} unique platform tags: {unique_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:
        combined_python_tag = ".".join(
            [wheel_info.python_tag for wheel_info in all_wheel_infos]
        )
        combined_abi_tag = ".".join(
            [wheel_info.abi_tag for wheel_info in all_wheel_infos]
        )
        combined_platform_tag = ".".join(unique_tags)

        _logger.info(f"Combined python tag: {combined_python_tag}")
        _logger.info(f"Combined abi tag: {combined_abi_tag}")
        _logger.info(f"Combined platform tag: {combined_platform_tag}")
        _logger.info(f"Wheel DVB: {all_wheel_infos[0].wheel_dvb}")

        # Create a zip file for the combined wheel
        combined_wheel_name = f"{all_wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        _logger.info(f"Combined wheel name: {combined_wheel_name}")

        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(
            output_zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9
        )

        # Extract each wheel into a subdirectory named after the wheel
        for wheel_info in all_wheel_infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            ## Just copy everything over to the output zip
            for file in os.listdir(wheel_extract_dir):
                if file.endswith(".dylib"):
                    continue  # .dylib files are already contained within the "platform.dylibs" folder (put there by delocate)
                # Strip debug symbols from .so files
                file_path = os.path.join(wheel_extract_dir, file)
                if args.strip_debug and file.endswith(".so"):
                    _logger.info(f"Stripping debug symbols from {file}")
                    subprocess.run(["strip", "-S", file_path], check=True)
                write_to_zip(output_zip, wheel_extract_dir, file)

            # Strip debug symbols from bundled dylibs
            if args.strip_debug:
                dylibs_folder = os.path.join(wheel_extract_dir, "depthai.libs")
                if os.path.exists(dylibs_folder):
                    for dylib in os.listdir(dylibs_folder):
                        if dylib.endswith(".dylib"):
                            dylib_path = os.path.join(dylibs_folder, dylib)
                            _logger.info(f"Stripping debug symbols from {dylib}")
                            subprocess.run(["strip", "-S", dylib_path], check=True)

        output_zip.close()
        _logger.info("Output zip closed")
        _logger.info(
            f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB"
        )
        _logger.info(f"Combined wheel saved to {output_zip_path}")


def write_to_zip(zip_file: zipfile.ZipFile, path: str, file: str):
    file_path = os.path.join(path, file)
    if os.path.isdir(file_path):
        for root, _, files in os.walk(file_path):
            for f in files:
                arcname = file + "/" + os.path.relpath(os.path.join(root, f), file_path)
                arcname = (
                    arcname.replace("\\", "/") if sys.platform == "win32" else arcname
                )
                try:
                    # This will throw a KeyError if the file is not in the zip, in that case we write the file to the zip
                    zip_file.getinfo(arcname)
                except KeyError:
                    root = root.replace("\\", "/") if sys.platform == "win32" else root
                    zip_file.write(os.path.join(root, f), arcname)
    else:
        try:
            # This will throw a KeyError if the file is not in the zip, in that case we write the file to the zip
            zip_file.getinfo(file)
        except KeyError:
            zip_file.write(file_path, file)


def combine_wheels(
    input_folder: str,
    output_folder: str = ".",
    strip: bool = False,
    log_level: str = "INFO",
):
    ## Set up logging
    FORMAT = "[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(name)s] %(message)s"
    DATEFMT = "%Y-%m-%d %H:%M:%S"
    logging.basicConfig(level=log_level.upper(), format=FORMAT, datefmt=DATEFMT)

    if sys.platform == "linux":
        _combine_wheels_linux(input_folder, output_folder, strip)
    elif sys.platform == "win32":
        _combine_wheels_windows(input_folder, output_folder, strip)
    elif sys.platform == "darwin":
        _combine_wheels_macos(input_folder, output_folder, strip)
    else:
        raise ValueError(f"Unsupported platform: {sys.platform}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input_folder",
        type=str,
        help="Path with wheels to combine.",
        required=True,
    )
    parser.add_argument(
        "--output_folder",
        type=str,
        default=".",
        help="Path to the folder where the combined wheel will be saved",
    )
    parser.add_argument(
        "--strip",
        action="store_true",
        help="Strip unnecessary symbols from libraries to reduce size. Supported on Linux and MacOS. This option is ignored on Windows.",
    )
    parser.add_argument("--log_level", type=str, default="INFO", help="Log level")
    parser.add_argument(
        "--self-test", action="store_true", help="Run internal tests and exit"
    )
    args = parser.parse_args()

    # Perform self test
    if args.self_test:
        _run_self_test()
        exit(0)

    # Combine wheels
    combine_wheels(
        input_folder=args.input_folder,
        output_folder=args.output_folder,
        strip=args.strip,
        log_level=args.log_level,
    )
