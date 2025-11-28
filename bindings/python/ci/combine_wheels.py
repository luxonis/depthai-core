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
import hashlib
import base64

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

def _sha256_hash_b64url(data: bytes) -> str:
    """Return sha256 hash in base64url encoding (no padding) as used in RECORD."""
    digest = hashlib.sha256(data).digest()
    b64 = base64.urlsafe_b64encode(digest).decode('ascii')
    return b64.rstrip('=')

def _generate_combined_wheel_content(wheel_infos, first_wheel_extract_dir: str):
    """Generate combined WHEEL file content (multi-Tag)."""
    # Find .dist-info in first wheel
    dist_info_dirs = [d for d in os.listdir(first_wheel_extract_dir) if d.endswith(".dist-info")]
    if not dist_info_dirs:
        raise FileNotFoundError("No .dist-info in first wheel")
    dist_info_name = dist_info_dirs[0]
    wheel_file_path = os.path.join(first_wheel_extract_dir, dist_info_name, "WHEEL")
    with open(wheel_file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    non_tag_lines = [line for line in lines if not line.startswith("Tag:")]
    tag_lines = set()
    for info in wheel_infos:
        py_tags = info.python_tag.split(".")
        abi_tags = info.abi_tag.split(".")
        platform_tags = info.platform_tag.split(".")
        n = max(len(py_tags), len(abi_tags), len(platform_tags))
        py_tags = py_tags * n if len(py_tags) == 1 else py_tags[:n]
        abi_tags = abi_tags * n if len(abi_tags) == 1 else abi_tags[:n]
        platform_tags = platform_tags * n if len(platform_tags) == 1 else platform_tags[:n]
        for i in range(n):
            tag = f"{py_tags[i]}-{abi_tags[i]}-{platform_tags[i]}"
            tag_lines.add(f"Tag: {tag}\n")
    return "".join(non_tag_lines + sorted(tag_lines)), dist_info_name

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

        # Extract first wheel to get .dist-info metadata
        first_wheel_info = wheel_infos[0]
        first_wheel_extract_dir = os.path.join(temp_dir, "first_wheel")
        os.makedirs(first_wheel_extract_dir, exist_ok=True)
        with zipfile.ZipFile(first_wheel_info.wheel_path, 'r') as z:
            z.extractall(first_wheel_extract_dir)

        combined_wheel_content, dist_info_name = _generate_combined_wheel_content(wheel_infos, first_wheel_extract_dir)

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
        file_records = {}

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
                write_to_zip(output_zip, wheel_extract_dir, file, file_records, dist_info_name)

            # Write the .libs folder contents to the zip
            for lib in bundled_libs:
                lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, lib)
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, new_lib_name)
                os.rename(lib_path, new_lib_path)
                if args.strip_unneeded:
                    logger.info(f"Stripping {new_lib_path}")
                    subprocess.run(['strip', '--strip-unneeded', new_lib_path], check=True)
                
            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path, file_records, dist_info_name)

        # Write .dist-info (only from first wheel)
        write_to_zip(output_zip, first_wheel_extract_dir, dist_info_name, file_records, dist_info_name)

        # Manually write combined WHEEL
        wheel_arcname = f"{dist_info_name}/WHEEL"
        output_zip.writestr(wheel_arcname, combined_wheel_content.encode("utf-8"))
        file_records[wheel_arcname] = (combined_wheel_content.encode("utf-8"), len(combined_wheel_content))

        output_zip.close()

        # Generate and write RECORD
        record_lines = []
        record_arcname = f"{dist_info_name}/RECORD"
        for arcname in sorted(file_records):
            if arcname == record_arcname:
                record_lines.append(f"{arcname},,")
            else:
                data, size = file_records[arcname]
                hash_b64 = _sha256_hash_b64url(data)
                record_lines.append(f"{arcname},sha256={hash_b64},{size}")
        record_content = "\n".join(record_lines) + "\n"
        with zipfile.ZipFile(output_zip_path, 'a', zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
            zf.writestr(record_arcname, record_content.encode("utf-8"))

        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"✅ Combined wheel saved to {output_zip_path}")

def combine_wheels_windows(args, wheel_infos):

    logger.info("Combining wheels for Windows!")
    from delvewheel import _dll_utils as dll_utils

    # Make sure that on windows, all the wheels have the same platform tag
    platform_tags = set(wheel_info.platform_tag for wheel_info in wheel_infos)
    if len(platform_tags) > 1:
        raise ValueError(f"All wheels must have the same platform tag. Found: {platform_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        dynlib_renames = defaultdict(lambda: defaultdict(dict))

        # Extract first wheel to get .dist-info metadata
        first_wheel_info = wheel_infos[0]
        first_wheel_extract_dir = os.path.join(temp_dir, "first_wheel")
        os.makedirs(first_wheel_extract_dir, exist_ok=True)
        with zipfile.ZipFile(first_wheel_info.wheel_path, 'r') as z:
            z.extractall(first_wheel_extract_dir)

        combined_wheel_content, dist_info_name = _generate_combined_wheel_content(wheel_infos, first_wheel_extract_dir)

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
        file_records = {}

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
                base = "".join(base.split("-")[:-1]) if len(base.split("-")) > 1 else base
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
                write_to_zip(output_zip, wheel_extract_dir, file, file_records, dist_info_name)

            for lib in bundled_libs:
                lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", lib)
                new_lib_name = dynlib_renames[wheel_info.wheel_name][lib]
                new_lib_path = os.path.join(wheel_extract_dir, wheel_libs_path, "platlib", new_lib_name)
                logger.info(f"Renaming {lib_path} to {new_lib_path}")
                os.rename(lib_path, new_lib_path)

            write_to_zip(output_zip, wheel_extract_dir, wheel_libs_path, file_records, dist_info_name)

        # Write .dist-info (only from first wheel)
        write_to_zip(output_zip, first_wheel_extract_dir, dist_info_name, file_records, dist_info_name)

        # Manually write combined WHEEL
        wheel_arcname = f"{dist_info_name}/WHEEL"
        output_zip.writestr(wheel_arcname, combined_wheel_content.encode("utf-8"))
        file_records[wheel_arcname] = (combined_wheel_content.encode("utf-8"), len(combined_wheel_content))

        output_zip.close()

        # Generate and write RECORD
        record_lines = []
        record_arcname = f"{dist_info_name}/RECORD"
        for arcname in sorted(file_records):
            if arcname == record_arcname:
                record_lines.append(f"{arcname},,")
            else:
                data, size = file_records[arcname]
                hash_b64 = _sha256_hash_b64url(data)
                record_lines.append(f"{arcname},sha256={hash_b64},{size}")
        record_content = "\n".join(record_lines) + "\n"
        with zipfile.ZipFile(output_zip_path, 'a', zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
            zf.writestr(record_arcname, record_content.encode("utf-8"))

        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"✅ Combined wheel saved to {output_zip_path}")

def combine_wheels_macos(args, all_wheel_infos):

    logger.info("Combining wheels for macOS!")

    ## Filter wheel infos based on their platform tag.
    unique_tags = list(set(wheel_info.platform_tag for wheel_info in all_wheel_infos))

    logger.info(f"Found {len(unique_tags)} unique platform tags: {unique_tags}")

    ## Create a temporary directory for extracting wheels
    with tempfile.TemporaryDirectory() as temp_dir:

        # Extract first wheel to get .dist-info metadata
        first_wheel_info = all_wheel_infos[0]
        first_wheel_extract_dir = os.path.join(temp_dir, "first_wheel")
        os.makedirs(first_wheel_extract_dir, exist_ok=True)
        with zipfile.ZipFile(first_wheel_info.wheel_path, 'r') as z:
            z.extractall(first_wheel_extract_dir)

        combined_wheel_content, dist_info_name = _generate_combined_wheel_content(all_wheel_infos, first_wheel_extract_dir)

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
        file_records = {}

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
                write_to_zip(output_zip, wheel_extract_dir, file, file_records, dist_info_name)

        # Ensure .dist-info from first wheel is used (in case later wheels overwrote)
        write_to_zip(output_zip, first_wheel_extract_dir, dist_info_name, file_records, dist_info_name)

        # Manually write combined WHEEL
        wheel_arcname = f"{dist_info_name}/WHEEL"
        output_zip.writestr(wheel_arcname, combined_wheel_content.encode("utf-8"))
        file_records[wheel_arcname] = (combined_wheel_content.encode("utf-8"), len(combined_wheel_content))

        output_zip.close()

        # Generate and write RECORD
        record_lines = []
        record_arcname = f"{dist_info_name}/RECORD"
        for arcname in sorted(file_records):
            if arcname == record_arcname:
                record_lines.append(f"{arcname},,")
            else:
                data, size = file_records[arcname]
                hash_b64 = _sha256_hash_b64url(data)
                record_lines.append(f"{arcname},sha256={hash_b64},{size}")
        record_content = "\n".join(record_lines) + "\n"
        with zipfile.ZipFile(output_zip_path, 'a', zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
            zf.writestr(record_arcname, record_content.encode("utf-8"))

        logger.info("Output zip closed")
        logger.info(f"Output zip size: {os.path.getsize(output_zip_path) / (1024 * 1024):.2f} MB")
        logger.info(f"Combined wheel saved to {output_zip_path}")


def write_to_zip(zip_file: zipfile.ZipFile, path: str, file: str, file_records: dict, dist_info_name: str = None):
    file_path = os.path.join(path, file)
    if os.path.isdir(file_path):
        for root, _, files in os.walk(file_path):
            for f in files:
                arcname = file + "/" + os.path.relpath(os.path.join(root, f), file_path)
                arcname = arcname.replace("\\", "/") if sys.platform == "win32" else arcname
                
                if arcname.endswith(("/WHEEL", "/RECORD")):
                    continue
                    
                try:
                    zip_file.getinfo(arcname)
                except KeyError:
                    full_path = os.path.join(root, f)
                    with open(full_path, "rb") as infile:
                        data = infile.read()
                    zip_file.writestr(arcname, data)
                    if not arcname.endswith("/RECORD"):
                        file_records[arcname] = (data, len(data))
    else:
        # Single file
        arcname = file.replace("\\", "/")
        if arcname.endswith("/WHEEL") or arcname.endswith("/RECORD"):
            return
            
        try:
            zip_file.getinfo(file)
        except KeyError:
            with open(file_path, "rb") as infile:
                data = infile.read()
            zip_file.writestr(file, data)
            file_records[file] = (data, len(data))


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
        if len(parts) < 3:
            raise ValueError(f"Invalid wheel filename: {wheel_path}")
        platform_tag = parts[-1]
        abi_tag = parts[-2]
        python_tag = parts[-3]
        wheel_dvb = "-".join(parts[:-3])
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
    parser.add_argument("--strip_unneeded", action="store_true", help="Strip the libraries to reduce size")
    parser.add_argument("--log_level", type=str, default="INFO", help="Log level")
    parser.add_argument("--self-test", action="store_true", help="Run internal tests and exit")
    args = parser.parse_args()
    main(args)
