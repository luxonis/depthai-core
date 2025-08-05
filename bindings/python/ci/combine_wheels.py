#!/usr/bin/env python3

import logging
import zipfile
import argparse
import glob
import os
import time
import tempfile
import subprocess

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

def write_to_zip(zip_file: zipfile.ZipFile, path: str, file: str):
    file_path = os.path.join(path, file)
    if os.path.isdir(file_path):
        for root, _, files in os.walk(file_path):
            for f in files:
                arcname = os.path.join(file, f)
                try:
                    # This will throw a KeyError if the file is not in the zip, in that case we write the file to the zip
                    zip_file.getinfo(arcname)
                except KeyError:
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
    
    for wheel_info in wheel_infos:
        logger.info(f"Found wheel: {wheel_info}")


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

            for file in wheel_files_to_copy:
                write_to_zip(output_zip, wheel_extract_dir, file)

            # Write the .libs folder contents to the zip
            for lib in bundled_libs:
                lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, lib)
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, new_lib_name)
                os.rename(lib_path, new_lib_path)
                if args.strip_unneeded:
                    logger.info(f"Stripping {new_lib_path}")
                    subprocess.run(['strip', '--strip-unneeded', new_lib_path], check=True)
                
            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path)

        output_zip.close()
        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_folder", type=str, required=True, help="Path to the folder containing already repaired wheels for individual python versions that are to be combined into a single wheel")
    parser.add_argument("--output_folder", type=str, default=".", help="Path to the folder where the combined wheel will be saved")
    parser.add_argument("--strip_unneeded", action="store_true", help="Strip the libraries to reduce size")
    parser.add_argument("--log_level", type=str, default="INFO", help="Log level")
    args = parser.parse_args()
    main(args)