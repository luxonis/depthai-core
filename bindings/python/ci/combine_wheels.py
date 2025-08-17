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

from collections import defaultdict

from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class WheelInfo:
    wheel_name: str
    wheel_path: str
    wheel_dvb: str
    python_tag: str
    abi_tag: str
    platform_tag: str

def write_to_zip(zip_file: zipfile.ZipFile, root_path: str, file_or_folder: str, exclude_fn = lambda path: False):
    """
    Writes a file or directory to a zip archive, avoiding duplicate entries.

    If the specified file is a directory, recursively adds all files within the directory
    to the zip archive, preserving the directory structure. If the file is a regular file,
    adds it directly to the zip archive.

    Note: root_path corresponds to "." in the zip file.

    exclude_fn is a function that determines whether a file should be written to the zip file.
    """
    host_path = os.path.join(root_path, file_or_folder)
    if os.path.isdir(host_path):
        for f in glob.glob(os.path.join(host_path, '**'), recursive=True):
            if os.path.isfile(f) and not exclude_fn(f):
                zip_path = file_or_folder + "/" + os.path.relpath(f, host_path)
                zip_path = zip_path.replace("\\", "/") if sys.platform == "win32" else zip_path
                try:
                    zip_file.getinfo(zip_path)
                except KeyError:
                    logger.debug(f"Writing {f} to {zip_path}")
                    zip_file.write(f, zip_path)
    elif os.path.isfile(host_path) and not exclude_fn(host_path):
        zip_path = file_or_folder
        zip_path = zip_path.replace("\\", "/") if sys.platform == "win32" else zip_path
        try:
            zip_file.getinfo(zip_path)
        except KeyError:
            logger.debug(f"Writing {host_path} to {zip_path}")
            zip_file.write(host_path, zip_path)
    else:
        raise ValueError(f"File or folder {host_path} does not exist")

def combine_wheels_linux(args, wheel_infos):
    """
        The following wheel structure is expected (wheels are expected to be pre-repaired):
            - depthai/ 
            - depthai-*.dist.info/
            - depthai_cli/
            - depthai.libs/

        Strategy:
            - We pick everything from the depthai/ folder except any .so files that are not the cpython wheel
                - these .so files namely are already included in the depthai.libs folder
            - We copy the entire depthai-*.dist.info/ folder
            - We copy the entire depthai_cli/ folder
            - depthai.libs/ is copied in such a way that removes library duplicates (we introduce a custom magic hash)
    """

    logger.info("Combining wheels for Linux!")

    # Make sure that on linux, all the wheels have the same platform tag
    platform_tags = set(wheel_info.platform_tag for wheel_info in wheel_infos)
    if len(platform_tags) > 1:
        raise ValueError(f"All wheels must have the same platform tag. Found: {platform_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        # Book-keeping
        dynlib_renames = defaultdict(lambda: defaultdict(dict))
        common_magic_hash = str(time.perf_counter())[-5:]

        # Get combined wheel info
        combined_python_tag = ".".join([wheel_info.python_tag for wheel_info in wheel_infos])
        combined_abi_tag = ".".join([wheel_info.abi_tag for wheel_info in wheel_infos])
        combined_platform_tag = wheel_infos[0].platform_tag

        logger.info(f"Combined python tag: {combined_python_tag}")
        logger.info(f"Combined abi tag: {combined_abi_tag}")
        logger.info(f"Combined platform tag: {combined_platform_tag}")
        logger.info(f"Wheel DVB: {wheel_infos[0].wheel_dvb}")

        # Generate final wheel name
        combined_wheel_name = f"{wheel_infos[0].wheel_dvb}-{combined_python_tag}-{combined_abi_tag}-{combined_platform_tag}.whl"
        logger.info(f"Combined wheel name: {combined_wheel_name}")

        # Create final wheel zip file
        output_zip_path = os.path.join(args.output_folder, combined_wheel_name)
        output_zip = zipfile.ZipFile(output_zip_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9)

        for wheel_info in wheel_infos:

            # Extract wheel
            wheel_extract_dir = os.path.join(temp_dir, wheel_info.wheel_name)
            logger.info(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            os.makedirs(wheel_extract_dir, exist_ok=True)
            with zipfile.ZipFile(wheel_info.wheel_path, 'r') as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            # Copy over the `depthai-*.dist.info`
            dist_info_path = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".dist-info")]
            assert len(dist_info_path) == 1, f"Expected 1 .dist-info folder, got {len(dist_info_path)}"
            logger.info(f"Copying `{dist_info_path[0]}` folder, wheel: {wheel_info.wheel_name}")
            write_to_zip(output_zip, wheel_extract_dir, dist_info_path[0])

            # Copy over the `depthai_cli`
            logger.info(f"Copying `depthai_cli` folder, wheel: {wheel_info.wheel_name}")
            write_to_zip(output_zip, wheel_extract_dir, "depthai_cli")

            # Find full path to the cpython wheel .so file
            depthai_files = os.listdir(os.path.join(wheel_extract_dir, "depthai"))
            cpython_so_files = [f for f in depthai_files if f.endswith(".so") and "cpython" in f]
            assert len(cpython_so_files) == 1, f"Expected 1 .so file, got {len(cpython_so_files)}"
            cpython_so_path = os.path.join(wheel_extract_dir, "depthai", cpython_so_files[0])
            logger.info(f"Found cpython wheel .so file: {cpython_so_path}")

            # Rename all the .so files from the `depthai` folder and relink them. Moreover, relink the cpython wheel .so file
            depthai_libs_path = os.path.join(wheel_extract_dir, "depthai.libs")
            depthai_libs = os.listdir(depthai_libs_path)
            for lib in depthai_libs:
                base, ext = lib.split(".", 1)
                new_lib = "-".join(base.split("-")[:-1]) + "-" + common_magic_hash + "." + ext # remove old hash and replace with new one

                # Rename the library
                new_lib_path = os.path.join(depthai_libs_path, new_lib)
                logger.info(f"Renaming {lib} to {new_lib}")
                os.rename(os.path.join(depthai_libs_path, lib), new_lib_path)

                # Relink the cpython wheel .so file
                command = f"patchelf --replace-needed {lib} {new_lib} {cpython_so_path}"
                logger.debug(f"Running command: {command}")
                subprocess.run(command, shell=True, check=True)

                # Relink the rest of the libraries
                for other_lib in os.listdir(depthai_libs_path):
                    if other_lib == lib:
                        continue
                    command = f"patchelf --replace-needed {lib} {new_lib} {os.path.join(depthai_libs_path, other_lib)}"
                    logger.debug(f"Running command: {command}")
                    subprocess.run(command, shell=True, check=True)

            # Copy over the `depthai.libs` folder
            logger.info(f"Copying `depthai.libs` folder, wheel: {wheel_info.wheel_name}")
            write_to_zip(output_zip, wheel_extract_dir, "depthai.libs")

            # Copy over the `depthai` without the .so files (except the cpython .so file)
            logger.info(f"Copying `depthai` folder, wheel: {wheel_info.wheel_name}")
            def exclude_fn(path):
                base = os.path.basename(path)
                if ".so" in base and "cpython" not in base:
                    return True
                return False
            write_to_zip(output_zip, wheel_extract_dir, "depthai", exclude_fn)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")

def combine_wheels_windows(args, wheel_infos):
    """
    The following wheel structure is expected:
        - depthai 
        - depthai-*.dist.info
        - depthai_cli
    """

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
                base = "".join(base.split("-")[:-1]) # remove hash
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
    """
    The following wheel structure is expected:
        - depthai 
        - depthai-*.dist.info
        - depthai_cli
    """

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
                write_to_zip(output_zip, wheel_extract_dir, file)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")



def main(args: argparse.Namespace):

    FORMAT = "[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(name)s] %(message)s"
    DATEFMT = "%Y-%m-%d %H:%M:%S"
    logging.basicConfig(
        level=args.log_level.upper(),
        format=FORMAT,
        datefmt=DATEFMT
    )
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
    parser.add_argument("--input_folder", type=str, required=True, help="Path to the folder containing already repaired wheels for individual python versions that are to be combined into a single wheel")
    parser.add_argument("--output_folder", type=str, default=".", help="Path to the folder where the combined wheel will be saved")
    parser.add_argument("--strip_unneeded", action="store_true", help="Strip the libraries to reduce size")
    parser.add_argument("--log_level", type=str, default="INFO", help="Log level")
    args = parser.parse_args()
    main(args)