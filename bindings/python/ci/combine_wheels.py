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
import random

from collections import defaultdict

from dataclasses import dataclass

logger = logging.getLogger("merger")

@dataclass
class WheelInfo:
    wheel_name: str
    wheel_path: str
    wheel_dvb: str
    python_tag: str
    abi_tag: str
    platform_tag: str

def _python_tag_sort_key(tag: str):
    key_parts = []
    for component in tag.split("."):
        match = re.match(r"([a-zA-Z]+)(\d+)$", component)
        if match:
            prefix, numeric = match.groups()
            key_parts.append((prefix, int(numeric)))
        else:
            key_parts.append((component, 0))
    return tuple(key_parts)

def _run_self_test():
    logger.info("Running combine_wheels self-test")
    simple_tags = ["cp39", "cp310", "cp311", "cp38"]
    expected_simple = ["cp38", "cp39", "cp310", "cp311"]
    actual_simple = sorted(simple_tags, key=_python_tag_sort_key)
    assert actual_simple == expected_simple, f"Simple ordering failed: {actual_simple}"

    composite_tags = ["cp39.cp310", "cp39.cp312", "cp38.cp310", "cp310.cp311", "cp39.cp311"]
    expected_composite = ["cp38.cp310", "cp39.cp310", "cp39.cp311", "cp39.cp312", "cp310.cp311"]
    actual_composite = sorted(composite_tags, key=_python_tag_sort_key)
    assert actual_composite == expected_composite, f"Composite ordering failed: {actual_composite}"

    wheels = [
        WheelInfo(
            wheel_name=f"depthai-foo-{tag}-abi-{idx}",
            wheel_path=f"/tmp/{tag}-{idx}.whl",
            wheel_dvb="depthai-foo",
            python_tag=tag,
            abi_tag=f"abi{idx}",
            platform_tag="plat",
        )
        for idx, tag in enumerate(composite_tags)
    ]
    random.shuffle(wheels)
    wheels.sort(key=lambda info: _python_tag_sort_key(info.python_tag))
    assert [w.python_tag for w in wheels] == expected_composite, "WheelInfo sorting failed"
    logger.info("Self-test passed")

def combine_wheels_linux(args, wheel_infos):

    logger.info("Combining wheels for Linux!")

    # Make sure that on linux, all the wheels have the same platform tag
    platform_tags = set(wheel_info.platform_tag for wheel_info in wheel_infos)
    if len(platform_tags) > 1:
        raise ValueError(f"All wheels must have the same platform tag. Found: {platform_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        dynlib_renames = defaultdict(lambda: defaultdict(dict))

        common_magic_hash = str(time.perf_counter())[-5:]

        combined_python_tag = ".".join([wheel_info.python_tag for wheel_info in wheel_infos])
        combined_abi_tag = ".".join([wheel_info.abi_tag for wheel_info in wheel_infos])
        combined_platform_tag = wheel_infos[0].platform_tag

        logger.info(f"Combined python tag: {combined_python_tag}")
        logger.info(f"Combined abi tag: {combined_abi_tag}")
        logger.info(f"Combined platform tag: {combined_platform_tag}")
        logger.info(f"Wheel DVB: {wheel_infos[0].wheel_dvb}")

        # Create a zip file for the combined wheel
        combined_wheel_name = f"{wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        logger.info(f"Combined wheel name: {combined_wheel_name}")

        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(output_zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9)

        # Extract each wheel into a subdirectory named after the wheel
        for wheel_info in wheel_infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, 'r') as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            ## Get the path to the wheel's dynamic library
            extracted_files = os.listdir(wheel_extract_dir)
            wheel_dylib_path = [f for f in extracted_files if f.endswith(".so") and "cpython" in f]
            assert len(wheel_dylib_path) == 1, f"Expected 1 .so file, got {len(wheel_dylib_path)}"
            wheel_dylib_path = wheel_dylib_path[0]

            ## get .libs folder
            wheel_libs_path = [f for f in extracted_files if f.endswith(".libs")]
            assert len(wheel_libs_path) == 1, f"Expected 1 .libs folder, got {len(wheel_libs_path)}"
            wheel_libs_path = wheel_libs_path[0]

            ## All folders and files that will be copied to the output folder
            wheel_files_to_copy = [wheel_dylib_path]
            wheel_files_to_copy.extend([f for f in extracted_files if not f.endswith(".so") and not f.endswith(".libs")])

            ## Unmangle the wheel's dynamic library names
            bundled_libs = os.listdir(os.path.join(wheel_extract_dir, wheel_libs_path))
            unmangled_libs = list()
            for lib in bundled_libs:
                base, ext = lib.split(".", 1)
                base = "".join(base.split("-")[:-1])
                unmangled_libs.append(base + "-" + common_magic_hash + "." + ext)
                dynlib_renames[wheel_info.wheel_name][lib] = unmangled_libs[-1]
            # Update the dynamic library to link against the unmangled libraries
            wheel_dylib_full_path = os.path.join(wheel_extract_dir, wheel_dylib_path)
            for file in [wheel_dylib_full_path] + [os.path.join(wheel_extract_dir, wheel_libs_path, f) for f in os.listdir(os.path.join(wheel_extract_dir, wheel_libs_path))]:
                for old_lib, new_lib in dynlib_renames[wheel_info.wheel_name].items():
                    logger.debug(f"Patching {file} to replace {old_lib} with {new_lib}")
                    subprocess.run(['patchelf', '--replace-needed', old_lib, new_lib, file], check=True)

            # Strip debug symbols from the main dynamic library
            if args.strip_debug:
                logger.info(f"Stripping debug symbols from {wheel_dylib_path}")
                subprocess.run(['strip', '--strip-unneeded', wheel_dylib_full_path], check=True)
                subprocess.run(['llvm-strip', '--strip-unneeded', wheel_dylib_full_path], check=True) # strip leads to "ELF load command address/offset not page-aligned" errors

            for file in wheel_files_to_copy:
                write_to_zip(output_zip, wheel_extract_dir, file)

            # Write the .libs folder contents to the zip
            for lib in bundled_libs:
                lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, lib)
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, new_lib_name)
                os.rename(lib_path, new_lib_path)
                if args.strip_debug:
                    logger.info(f"Stripping debug symbols from {new_lib_name}")
                    subprocess.run(['strip', '--strip-unneeded', new_lib_path], check=True)
                    subprocess.run(['llvm-strip', '--strip-unneeded', new_lib_path], check=True) # strip leads to "ELF load command address/offset not page-aligned" errors
                
            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")

def combine_wheels_windows(args, wheel_infos):

    logger.info("Combining wheels for Windows!")
    from delvewheel import _dll_utils as dll_utils

    # Make sure that on linux, all the wheels have the same platform tag
    platform_tags = set(wheel_info.platform_tag for wheel_info in wheel_infos)
    if len(platform_tags) > 1:
        raise ValueError(f"All wheels must have the same platform tag. Found: {platform_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        dynlib_renames = defaultdict(lambda: defaultdict(dict))

        common_magic_hash = str(time.perf_counter())[-5:]

        combined_python_tag = ".".join([wheel_info.python_tag for wheel_info in wheel_infos])
        combined_abi_tag = ".".join([wheel_info.abi_tag for wheel_info in wheel_infos])
        combined_platform_tag = wheel_infos[0].platform_tag

        logger.info(f"Combined python tag: {combined_python_tag}")
        logger.info(f"Combined abi tag: {combined_abi_tag}")
        logger.info(f"Combined platform tag: {combined_platform_tag}")
        logger.info(f"Wheel DVB: {wheel_infos[0].wheel_dvb}")

        # Create a zip file for the combined wheel
        combined_wheel_name = f"{wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        logger.info(f"Combined wheel name: {combined_wheel_name}")

        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(output_zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9)

        # Extract each wheel into a subdirectory named after the wheel
        for wheel_info in wheel_infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, 'r') as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            ## Get the path to the wheel's dynamic library
            extracted_files = os.listdir(wheel_extract_dir)
            wheel_dylib_path = [f for f in extracted_files if f.endswith(".pyd")]
            assert len(wheel_dylib_path) == 1, f"Expected 1 .pyd file, got {len(wheel_dylib_path)}"
            wheel_dylib_path = wheel_dylib_path[0]

            ## get .libs folder
            wheel_libs_path = [f for f in extracted_files if f.endswith(".data")]
            assert len(wheel_libs_path) == 1, f"Expected 1 .data folder, got {len(wheel_libs_path)}"
            wheel_libs_path = wheel_libs_path[0]

            ## get python dll
            python_dll_path = [f for f in extracted_files if f.endswith(".dll") and f.startswith("python3")]
            assert len(python_dll_path) == 1, f"Expected 1 python3.dll file, got {len(python_dll_path)}"
            python_dll_path = python_dll_path[0]

            ## All folders and files that will be copied to the output folder
            wheel_files_to_copy = [wheel_dylib_path, python_dll_path]
            wheel_files_to_copy.extend([f for f in extracted_files if not f.endswith(".pyd") and not f.endswith(".dll") and not f.endswith(".data")])

             ## Unmangle the wheel's dynamic library names
            bundled_libs = os.listdir(os.path.join(wheel_extract_dir, wheel_libs_path, "platlib"))
            unmangled_libs = list()
            for lib in bundled_libs:
                base, ext = ".".join(lib.split(".")[:-1]), "." + lib.split(".")[-1]
                base = "".join(base.split("-")[:-1]) if len(base.split("-")) > 1 else base # remove hash, or reuse base name if there is no hash
                unmangled_libs.append(base + "-" + common_magic_hash + ext)
                dynlib_renames[wheel_info.wheel_name][lib] = unmangled_libs[-1]

            # Update the dynamic library to link against the unmangled libraries
            wheel_dylib_full_path = os.path.join(wheel_extract_dir, wheel_dylib_path)
            for file in [wheel_dylib_full_path] + [os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", f) for f in os.listdir(os.path.join(wheel_extract_dir, wheel_libs_path, "platlib"))]:
                try:
                    logger.info(f"Patching {file}")
                    needed = dll_utils.get_direct_mangleable_needed(file, {}, {})
                    name_map = {old_lib: dynlib_renames[wheel_info.wheel_name][old_lib] for old_lib in needed}
                    dll_utils.replace_needed(file, needed, name_map, strip=False)
                except Exception as e:
                    logger.error(f"Error patching {file}: {e}")
                    continue

            for file in wheel_files_to_copy:
                write_to_zip(output_zip, wheel_extract_dir, file)

            for lib in bundled_libs:
                lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", lib)
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", new_lib_name)
                logger.info(f"Renaming {lib_path} to {new_lib_path}")
                os.rename(lib_path, new_lib_path)

            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")

def combine_wheels_macos(args, all_wheel_infos):

    logger.info("Combining wheels for macOS!")

    ## Filter wheel infos based on their platform tag.
    unique_tags = list(set(wheel_info.platform_tag for wheel_info in all_wheel_infos))

    logger.info(f"Found {len(unique_tags)} unique platform tags: {unique_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        combined_python_tag = ".".join([wheel_info.python_tag for wheel_info in all_wheel_infos])
        combined_abi_tag = ".".join([wheel_info.abi_tag for wheel_info in all_wheel_infos])
        combined_platform_tag = ".".join(unique_tags)

        logger.info(f"Combined python tag: {combined_python_tag}")
        logger.info(f"Combined abi tag: {combined_abi_tag}")
        logger.info(f"Combined platform tag: {combined_platform_tag}")
        logger.info(f"Wheel DVB: {all_wheel_infos[0].wheel_dvb}")

        # Create a zip file for the combined wheel
        combined_wheel_name = f"{all_wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        logger.info(f"Combined wheel name: {combined_wheel_name}")

        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(output_zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9)

        # Extract each wheel into a subdirectory named after the wheel
        for wheel_info in all_wheel_infos:
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(wheel_info.wheel_path, 'r') as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            ## Just copy everything over to the output zip
            for file in os.listdir(wheel_extract_dir):
                if file.endswith(".dylib"):
                    continue # .dylib files are already contained within the "platform.dylibs" folder (put there by delocate)
                # Strip debug symbols from .so files
                file_path = os.path.join(wheel_extract_dir, file)
                if args.strip_debug and file.endswith(".so"):
                    logger.info(f"Stripping debug symbols from {file}")
                    subprocess.run(['strip', '-S', file_path], check=True)
                write_to_zip(output_zip, wheel_extract_dir, file)

            # Strip debug symbols from bundled dylibs
            if args.strip_debug:
                dylibs_folder = os.path.join(wheel_extract_dir, "depthai.libs")
                if os.path.exists(dylibs_folder):
                    for dylib in os.listdir(dylibs_folder):
                        if dylib.endswith(".dylib"):
                            dylib_path = os.path.join(dylibs_folder, dylib)
                            logger.info(f"Stripping debug symbols from {dylib}")
                            subprocess.run(['strip', '-S', dylib_path], check=True)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")


def write_to_zip(zip_file: zipfile.ZipFile, path: str, file: str):
    file_path = os.path.join(path, file)
    if os.path.isdir(file_path):
        for root, _, files in os.walk(file_path):
            for f in files:
                arcname = file + "/" + os.path.relpath(os.path.join(root, f), file_path)
                arcname = arcname.replace("\\", "/") if sys.platform == "win32" else arcname
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


def main(args: argparse.Namespace):

    FORMAT = "[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(name)s] %(message)s"
    DATEFMT = "%Y-%m-%d %H:%M:%S"
    logging.basicConfig(
        level=args.log_level.upper(),
        format=FORMAT,
        datefmt=DATEFMT
    )
    if args.self_test:
        _run_self_test()
        return
    if not args.input_folder:
        raise ValueError("--input_folder is required unless --self-test is specified")
    ## Get a list of all wheels in the input folder
    wheels = glob.glob(os.path.join(args.input_folder, "*.whl"))

    ## Extract the wheel info from the wheels
    wheel_infos = []
    for wheel_path in wheels:
        wheel_name = os.path.basename(wheel_path)[:-4] # remove the .whl extension
        parts = wheel_name.split('-')
        wheel_dvb = "-".join(parts[:2])
        python_tag = parts[-3]
        abi_tag = parts[-2]
        platform_tag = parts[-1]
        wheel_infos.append(WheelInfo(
            wheel_name=wheel_name,
            wheel_path=wheel_path,
            wheel_dvb=wheel_dvb,
            python_tag=python_tag,
            abi_tag=abi_tag, 
            platform_tag=platform_tag
        ))
    wheel_infos.sort(key=lambda info: _python_tag_sort_key(info.python_tag))

    if sys.platform == "linux":
        combine_wheels_linux(args, wheel_infos)
    elif sys.platform == "win32":
        combine_wheels_windows(args, wheel_infos)
    elif sys.platform == "darwin":
        combine_wheels_macos(args, wheel_infos)
    else:
        raise ValueError(f"Unsupported platform: {sys.platform}")



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_folder", type=str, help="Path to the folder containing already repaired wheels for individual python versions that are to be combined into a single wheel")
    parser.add_argument("--output_folder", type=str, default=".", help="Path to the folder where the combined wheel will be saved")
    parser.add_argument("--strip-unneeded", action="store_true", dest="strip_debug", help="Strip unneeded debug symbols from libraries to reduce size")
    parser.add_argument("--log_level", type=str, default="INFO", help="Log level")
    parser.add_argument("--self-test", action="store_true", help="Run internal tests and exit")
    args = parser.parse_args()
    main(args)
