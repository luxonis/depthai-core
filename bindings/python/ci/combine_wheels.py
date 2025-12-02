#!/usr/bin/env python3

import logging
import zipfile
import glob
import os
import time
import tempfile
import subprocess
import sys
import re
import shutil
import hashlib
import base64
from collections import namedtuple
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
        assert match is not None, f"Expected {REGEX} match for part: {part} (tag: {tag})"
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
        return f"{self.wheel_dvb}-{self.python_tag}-{self.abi_tag}-{self.platform_tag}.whl"

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
    def combine(
        infos: list["WheelInfo"],
        sort: bool = True,
        require_dvb_uniqueness: bool = True,
        require_platform_uniqueness: bool = True,
    ) -> "WheelInfo":
        assert len(infos) > 0, "Expected at least one wheel info"
        infos = sorted(infos) if sort else infos

        # Basic sanity checks
        if require_dvb_uniqueness:
            assert all(info.wheel_dvb == infos[0].wheel_dvb for info in infos)
        if require_platform_uniqueness:
            assert all(info.platform_tag == infos[0].platform_tag for info in infos)

        # Combine wheel dvb and platform tags
        combined_wheel_dvb = ".".join(set([info.wheel_dvb for info in infos]))
        combined_platform_tag = ".".join(set([info.platform_tag for info in infos]))

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


def _calculate_file_hash(file_path):
    """Calculate the SHA256 hash of a file."""
    hash_sha256 = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_sha256.update(chunk)
    digest = hash_sha256.digest()
    b64 = base64.urlsafe_b64encode(digest).decode("ascii")
    return b64.rstrip("=")


def _generate_record_file(staging_dir: str, dist_info_dir: str):
    """
    Generate a RECORD file for the wheel as per PEP 427.
    The RECORD file lists every file in the wheel except itself.
    """
    record_path = os.path.join(dist_info_dir, "RECORD")
    with open(record_path, "w", encoding="utf-8") as record_file:
        # Add entries for all files in the wheel
        for root, _, files in os.walk(staging_dir):
            for file in files:
                # Skip the RECORD file itself
                if file == "RECORD":
                    continue
                
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, staging_dir).replace("\\", "/")
                
                # Calculate hash and size
                file_hash = _calculate_file_hash(file_path)
                file_size = os.path.getsize(file_path)
                
                # Write to RECORD file
                record_file.write(f"{rel_path},sha256={file_hash},{file_size}\n")
        
        # Add entry for the RECORD file itself (hash and size are empty as per PEP 427)
        record_rel_path = os.path.relpath(record_path, staging_dir)
        record_file.write(f"{record_rel_path},,\n")
    
    return record_path


def _update_wheel_file(dist_info_dir: str, combined_infos: list[WheelInfo]):
    """
    Update the WHEEL file in the .dist-info directory with new, combined tags.
    This preserves other metadata in the file.
    """
    wheel_path = os.path.join(dist_info_dir, "WHEEL")
    if not os.path.exists(wheel_path):
        _logger.warning("WHEEL file not found in dist-info directory. Skipping update.")
        return

    _logger.info(f"Updating tags in {wheel_path}")
    
    # Read existing content
    with open(wheel_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # Filter out old Tag lines
    new_lines = [line for line in lines if not line.startswith("Tag:")]
    
    # Generate new Tag lines
    for wheel_info in combined_infos:
        new_lines.append(f"Tag: {wheel_info.python_tag}-{wheel_info.abi_tag}-{wheel_info.platform_tag}\n")

    # Write the updated content back
    with open(wheel_path, "w") as f:
        f.writelines(new_lines)


def _combine_wheels_linux(input_folder, output_folder, strip):
    """
    Each wheel is expected to have the following structure:
    <wheel_name>.whl
    ├── <project_name>.libs
    │   ├── <lib1>.so
    │   ├── <lib2>.so
    │   ├── ...
    ├── <package_name>-<version>.dist-info
    │   ├── WHEEL
    │   ├── METADATA
    │   ├── RECORD
    │   └── ...
    ├── <cpython_lib>.so
    ├── <extra_lib_1>.so # this will get removed (it should be part of the libs folder)
    ├── <extra_lib_2>.so # this will get removed (it should be part of the libs folder)
    ├── <extra_lib_n>.so # this will get removed (it should be part of the libs folder)
    ├── <other_files>
    ├── ...

    The combination process works as follows:
    1. Each wheel is extracted to a separate directory (in a temporary directory)
    2. In each directory, the libraries in the .libs folder are renamed to the new names (name remangling)
    3. In each directory, the cpython library is relinked to the new names
    4. In each directory, the other libraries are relinked to the new names
    5. In each directory, the debug symbols are stripped for all libraries (if strip is True)
    6. A staging directory is created. A new .dist-info directory is created within it.
    7. Metadata (excluding WHEEL/RECORD) is copied from the first wheel's .dist-info to the new one.
    8. The WHEEL file in the new .dist-info is updated with combined tags.
    9. All other files (libs, .so files) are copied to the staging directory.
    10. The RECORD file is generated for PEP 427 compliance.
    11. The staging directory is zipped into a single wheel.
    """

    _logger.info("Combining wheels for Linux!")
    _logger.info(f"Stripping unneeded debug symbols: {strip}")

    # Find all wheel infos in the input folder
    infos = find_all_wheels(input_folder)
    assert len(infos) > 0, "Expected at least one wheel info"
    _logger.debug(f"Found infos in {input_folder}: {infos}")

    # Combined output wheel info
    combined_info = WheelInfo.combine(infos)
    _logger.info(f"Combined wheel name: {combined_info.wheel_name}")
    _logger.debug(f"Combined info: {combined_info}")

    def _remangle_name(name, magic_hash):
        """Remangle a library name to a new name.
        For example, if the library name is "libfoo-<original_hash>.so.1.2.0", the new name will be "libfoo-<new_hash>.so.1.2.0".
        """
        base, ext = name.split(".", 1)
        base = "".join(base.split("-")[:-1])
        new_name = f"{base}-{magic_hash}.{ext}"
        return new_name

    with tempfile.TemporaryDirectory() as temp_dir:
        # Create extraction directory for individual wheels
        extract_dir = os.path.join(temp_dir, "extracted")
        os.makedirs(extract_dir, exist_ok=True)

        # Create staging directory for the combined wheel
        staging_dir = os.path.join(temp_dir, "staging")
        os.makedirs(staging_dir, exist_ok=True)

        ExtractedWheel = namedtuple("ExtractedWheel", ["wheel_info", "wheel_extract_dir", "cpython_lib", "libs_folder", "dist_info_dir"])

        # Step: Each wheel is extracted to a separate directory
        extracted_wheels = []
        _logger.info("Step: Extracting all wheels")
        for wheel_info in infos:
            wheel_extract_dir = os.path.join(extract_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            # Extract wheel contents
            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(os.path.join(input_folder, wheel_info.wheel_name), "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            # Find cpython library
            cpython_libs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".so") and "cpython" in f]
            assert len(cpython_libs) == 1, f"Expected 1 .so file with 'cpython' in the name, got {len(cpython_libs)}"
            cpython_lib = cpython_libs[0]
            _logger.debug(f"Found cpython lib: {cpython_lib}")

            # Find libs folder
            libs_folders = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".libs")]
            assert len(libs_folders) == 1, f"Expected 1 .libs folder, got {len(libs_folders)}"
            libs_folder = libs_folders[0]
            _logger.debug(f"Found libs folder: {libs_folder}")

            # Find .dist-info directory
            dist_info_dirs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".dist-info")]
            assert len(dist_info_dirs) == 1, f"Expected 1 .dist-info directory, got {len(dist_info_dirs)}"
            dist_info_dir = dist_info_dirs[0]
            _logger.debug(f"Found dist-info directory: {dist_info_dir}")

            # Remove extra .so files (any .so file already is part of the .libs folder)
            extra_libs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".so") and "cpython" not in f]
            _logger.debug(f"Found extra libs: {extra_libs}. Removing them.")
            for lib in extra_libs:
                os.remove(os.path.join(wheel_extract_dir, lib))

            extracted_wheels.append(ExtractedWheel(wheel_info, wheel_extract_dir, cpython_lib, libs_folder, dist_info_dir))

        # Step: In each folder, rename libraries in the .libs folder to the new names
        magic_hash = str(time.perf_counter())[-5:] + "abc"
        _logger.info("Step: Renaming libraries in the .libs folder | Magic hash: %s", magic_hash)
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, libs_folder, dist_info_dir = extracted_wheel

            # Generate rename mapping for libraries in the .libs folder
            name_map = dict()
            for lib in os.listdir(os.path.join(wheel_extract_dir, libs_folder)):
                original_name = lib
                new_name = _remangle_name(original_name, magic_hash)
                name_map[original_name] = new_name

            # Rename libraries in the .libs folder to the new names
            for original_name, new_name in name_map.items():
                rename_from = os.path.join(wheel_extract_dir, libs_folder, original_name)
                rename_to = os.path.join(wheel_extract_dir, libs_folder, new_name)
                os.rename(rename_from, rename_to)

            # Relink all libraries
            libs = os.listdir(os.path.join(wheel_extract_dir, libs_folder))
            for lib in libs:  # for each library, relink it to the new name
                lib_path = os.path.join(wheel_extract_dir, libs_folder, lib)
                for old_name, new_name in name_map.items():
                    subprocess.run(["patchelf", "--replace-needed", old_name, new_name, lib_path], check=True)

            # Relink the cpython library
            for old_name, new_name in name_map.items():
                cpython_lib_path = os.path.join(wheel_extract_dir, cpython_lib)
                subprocess.run(["patchelf", "--replace-needed", old_name, new_name, cpython_lib_path], check=True)

            # Strip debug symbols for all libraries
            if strip:
                for lib in libs:
                    _logger.info(f"Stripping debug symbols from {lib}")
                    lib_path = os.path.join(wheel_extract_dir, libs_folder, lib)
                    subprocess.run(["strip", "-S", lib_path], check=True)
                    subprocess.run(["llvm-strip", "-S", lib_path], check=True)

            # Strip debug symbols for the cpython library
            if strip:
                _logger.info(f"Stripping debug symbols from {cpython_lib}")
                cpython_lib_path = os.path.join(wheel_extract_dir, cpython_lib)
                subprocess.run(["strip", "-S", cpython_lib_path], check=True)
                subprocess.run(["llvm-strip", "-S", cpython_lib_path], check=True)

        # Step: Prepare staging directory's .dist-info
        _logger.info("Step: Preparing .dist-info directory in staging area")
        # The name of the .dist-info directory is based on the combined package name and version
        new_dist_info_path = os.path.join(staging_dir, extracted_wheels[0].dist_info_dir)
        os.makedirs(new_dist_info_path, exist_ok=True)

        # Copy metadata files from the first wheel, excluding RECORD
        first_wheel_dist_info = os.path.join(extracted_wheels[0].wheel_extract_dir, extracted_wheels[0].dist_info_dir)
        for item in os.listdir(first_wheel_dist_info):
            if item.lower() not in ["record"]:
                shutil.copy2(os.path.join(first_wheel_dist_info, item), new_dist_info_path)

        # Step: Copy all other content to the staging directory
        _logger.info("Step: Copying all other content to the staging directory")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, libs_folder, dist_info_dir = extracted_wheel
            for item in os.listdir(wheel_extract_dir):
                # Skip the old .dist-info directory
                if item.endswith(".dist-info"):
                    continue
                src_path = os.path.join(wheel_extract_dir, item)
                dest_path = os.path.join(staging_dir, item)
                if os.path.isdir(src_path):
                    shutil.copytree(src_path, dest_path, dirs_exist_ok=True)
                else:
                    shutil.copy2(src_path, dest_path)

        # Step: Update WHEEL and generate RECORD files for PEP 427 compliance
        _logger.info("Step: Updating WHEEL and generating RECORD file for PEP 427 compliance")
        _update_wheel_file(new_dist_info_path, infos)
        _generate_record_file(staging_dir, new_dist_info_path)

        # Step: zip the staging directory
        _logger.info("Step: Zipping the staging directory")
        zip_path = os.path.join(output_folder, combined_info.wheel_name)
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as output_zip:
            for root, dirs, files in os.walk(staging_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, staging_dir)
                    output_zip.write(file_path, arcname)
        _logger.info("Output zip created")
        _logger.info(f"Output zip size: {os.path.getsize(zip_path) / (1024 * 1024):.2f} MB")
        _logger.info(f"Combined wheel saved to {zip_path}")


def _combine_wheels_macos(input_folder, output_folder, strip):
    """
    Each wheel is expected to have the following structure:
    <wheel_name>.whl
    ├── palace.dylibs
    │   ├── <lib1>.dylib
    │   ├── <lib2>.dylib
    │   ├── ...
    ├── <package_name>-<version>.dist-info
    │   ├── WHEEL
    │   ├── METADATA
    │   ├── RECORD
    │   └── ...
    ├── <cpython_lib>.so
    ├── <extra_lib_1>.dylib # this will get removed (it should be part of the libs folder)
    ├── <extra_lib_2>.dylib # this will get removed (it should be part of the libs folder)
    ├── <extra_lib_n>.dylib # this will get removed (it should be part of the libs folder)
    ├── <other_files>
    ├── ...

    The combination process works as follows:
    1. Each wheel is extracted to a separate directory (in a temporary directory)
    2. Extra .dylib files are removed (any .dylib file already is part of the .libs folder)
    3. A staging directory is created. A new .dist-info directory is created within it.
    4. Metadata (excluding WHEEL/RECORD) is copied from the first wheel's .dist-info to the new one.
    5. The WHEEL file in the new .dist-info is updated with combined tags.
    6. All other files (libs, .dylib files) are copied to the staging directory.
    7. The RECORD file is generated for PEP 427 compliance.
    8. The staging directory is zipped into a single wheel.
    """
    _logger.info("Combining wheels for MacOS!")
    _logger.info(f"Stripping unneeded debug symbols: {strip}")

    # Find all wheel infos in the input folder
    infos = find_all_wheels(input_folder)
    assert len(infos) > 0, "Expected at least one wheel info"
    _logger.debug(f"Found infos in {input_folder}: {infos}")

    # Combined output wheel info - platforms on MacOS are not unique
    combined_info = WheelInfo.combine(infos, require_platform_uniqueness=False)
    _logger.info(f"Combined wheel name: {combined_info.wheel_name}")
    _logger.debug(f"Combined info: {combined_info}")

    with tempfile.TemporaryDirectory() as temp_dir:
        # Create extraction directory for individual wheels
        extract_dir = os.path.join(temp_dir, "extracted")
        os.makedirs(extract_dir, exist_ok=True)

        # Create staging directory for the combined wheel
        staging_dir = os.path.join(temp_dir, "staging")
        os.makedirs(staging_dir, exist_ok=True)

        ExtractedWheel = namedtuple(
            "ExtractedWheel", ["wheel_info", "wheel_extract_dir", "cpython_lib", "palace_dylibs", "dist_info_dir"]
        )

        # Step: Each wheel is extracted to a separate directory
        extracted_wheels = []
        _logger.info("Step: Extracting all wheels")
        for wheel_info in infos:
            wheel_extract_dir = os.path.join(extract_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            # Extract wheel contents
            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(os.path.join(input_folder, wheel_info.wheel_name), "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            # Find cpython library
            cpython_libs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".so") and "cpython" in f]
            assert len(cpython_libs) == 1, f"Expected 1 .so file with 'cpython' in the name, got {len(cpython_libs)}"
            cpython_lib = cpython_libs[0]
            _logger.debug(f"Found cpython lib: {cpython_lib}")

            # Find palace.dylibs
            palace_dylibs = "palace.dylibs"
            _logger.debug(f"Found palace.dylibs: {palace_dylibs}")

            # Find .dist-info directory
            dist_info_dirs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".dist-info")]
            assert len(dist_info_dirs) == 1, f"Expected 1 .dist-info directory, got {len(dist_info_dirs)}"
            dist_info_dir = dist_info_dirs[0]
            _logger.debug(f"Found dist-info directory: {dist_info_dir}")

            # Remove extra .dylib files (any .dylib file already is part of the palace.dylibs folder)
            extra_libs = [f for f in os.listdir(wheel_extract_dir) if f.endswith(".dylib")]
            _logger.debug(f"Found extra libs: {extra_libs}. Removing them.")
            for lib in extra_libs:
                os.remove(os.path.join(wheel_extract_dir, lib))

            extracted_wheels.append(ExtractedWheel(wheel_info, wheel_extract_dir, cpython_lib, palace_dylibs, dist_info_dir))

        # Step: In each folder, strip debug symbols for all libraries
        _logger.info("Step: Stripping debug symbols for all libraries")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, palace_dylibs, dist_info_dir = extracted_wheel

            # Strip debug symbols for all libraries
            if strip:
                for lib in os.listdir(os.path.join(wheel_extract_dir, palace_dylibs)):
                    _logger.info(f"Stripping debug symbols from {lib}")
                    lib_path = os.path.join(wheel_extract_dir, palace_dylibs, lib)
                    subprocess.run(["strip", "-S", lib_path], check=True)

            # Strip debug symbols for the cpython library
            # TODO: Invalid signature error, fix this
            _logger.debug(f"Found cpython lib: {cpython_lib}, but not stripping debug symbols for it")

        # Step: Prepare staging directory's .dist-info
        _logger.info("Step: Preparing .dist-info directory in staging area")
        new_dist_info_path = os.path.join(staging_dir, extracted_wheels[0].dist_info_dir)
        os.makedirs(new_dist_info_path, exist_ok=True)

        # Copy metadata files from the first wheel, excluding RECORD
        first_wheel_dist_info = os.path.join(extracted_wheels[0].wheel_extract_dir, extracted_wheels[0].dist_info_dir)
        for item in os.listdir(first_wheel_dist_info):
            if item.lower() not in ["record"]:
                shutil.copy2(os.path.join(first_wheel_dist_info, item), new_dist_info_path)

        # Step: Copy all other content to the staging directory
        _logger.info("Step: Copying all other content to the staging directory")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, palace_dylibs, dist_info_dir = extracted_wheel
            for item in os.listdir(wheel_extract_dir):
                if item.endswith(".dist-info"):
                    continue
                src_path = os.path.join(wheel_extract_dir, item)
                dest_path = os.path.join(staging_dir, item)
                if os.path.isdir(src_path):
                    shutil.copytree(src_path, dest_path, dirs_exist_ok=True)
                else:
                    shutil.copy2(src_path, dest_path)

        # Step: Update WHEEL and generate RECORD files for PEP 427 compliance
        _logger.info("Step: Updating WHEEL and generating RECORD file for PEP 427 compliance")
        _update_wheel_file(new_dist_info_path, infos)
        _generate_record_file(staging_dir, new_dist_info_path)

        # Step: zip the staging directory
        _logger.info("Step: Zipping the staging directory")
        zip_path = os.path.join(output_folder, combined_info.wheel_name)
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as output_zip:
            for root, dirs, files in os.walk(staging_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, staging_dir)
                    output_zip.write(file_path, arcname)
        _logger.info("Output zip created")
        _logger.info(f"Output zip size: {os.path.getsize(zip_path) / (1024 * 1024):.2f} MB")
        _logger.info(f"Combined wheel saved to {zip_path}")


def _combine_wheels_windows(input_folder, output_folder, strip):
    """
    Each wheel is expected to have the following structure:
    <wheel_name>.whl
    ├── <wheel_name>.data
    │   ├── platlib
    │   │   ├── <lib1>.dll
    │   │   ├── <lib2>.dll
    │   │   ├── ...
    ├── <package_name>-<version>.dist-info
    │   ├── WHEEL
    │   ├── METADATA
    │   ├── RECORD
    │   └── ...
    ├── <cpython_lib>.pyd
    ├── python3X.dll
    ├── <other_files>
    ├── <extra_lib_1>.dll # this will get removed (it should be part of the platlib folder)
    ├── <extra_lib_2>.dll # this will get removed (it should be part of the platlib folder)
    ├── <extra_lib_n>.dll # this will get removed (it should be part of the platlib folder)
    ├── ...

    The combination process works as follows:
    1. Each wheel is extracted to a separate directory (in a temporary directory)
    2. In each directory, the libraries in the .data/platlib folder are renamed (remangled with common hash)
    3. In each directory, the .pyd file and bundled DLLs are patched to link to the new names
    4. A staging directory is created. A new .dist-info directory is created within it.
    5. Metadata (excluding WHEEL/RECORD) is copied from the first wheel's .dist-info to the new one.
    6. The WHEEL file in the new .dist-info is updated with combined tags.
    7. All other files (libs, .dll files) are copied to the staging directory.
    8. The RECORD file is generated for PEP 427 compliance.
    9. The staging directory is zipped into a single wheel.
    """

    _logger.info("Combining wheels for Windows!")
    _logger.info(f"Stripping unneeded debug symbols: {strip}")
    if strip:
        _logger.warning("Stripping debug symbols is not supported on Windows")

    try:
        from delvewheel import _dll_utils as dll_utils
    except ImportError:
        _logger.error("delvewheel is required for Windows wheel combining. Install it with: pip install delvewheel")
        raise

    # Find all wheel infos in the input folder
    infos = find_all_wheels(input_folder)
    assert len(infos) > 0, "Expected at least one wheel info"
    _logger.debug(f"Found infos in {input_folder}: {infos}")

    # Combined output wheel info
    combined_info = WheelInfo.combine(infos)
    _logger.info(f"Combined wheel name: {combined_info.wheel_name}")
    _logger.debug(f"Combined info: {combined_info}")

    def _remangle_name(lib, new_hash):
        """Remangle a DLL name to a new name.
        Example: "libfoo-<original_hash>.dll" becomes "libfoo-<new_hash>.dll"
        """
        base = ".".join(lib.split(".")[:-1])
        ext = "." + lib.split(".")[-1]
        base = (
            "".join(base.split("-")[:-1]) if len(base.split("-")) > 1 else base
        )  # remove existing hash, or reuse base name if there is no hash
        return f"{base}-{new_hash}{ext}"

    with tempfile.TemporaryDirectory() as temp_dir:
        # Create extraction directory for individual wheels
        extract_dir = os.path.join(temp_dir, "extracted")
        os.makedirs(extract_dir, exist_ok=True)

        # Create staging directory for the combined wheel
        staging_dir = os.path.join(temp_dir, "staging")
        os.makedirs(staging_dir, exist_ok=True)

        ExtractedWheel = namedtuple(
            "ExtractedWheel", ["wheel_info", "wheel_extract_dir", "cpython_lib", "python_dll", "data_folder", "dist_info_dir"]
        )

        # Step: Each wheel is extracted to a separate directory
        extracted_wheels = []
        _logger.info("Step: Extracting all wheels")
        for wheel_info in infos:
            wheel_extract_dir = os.path.join(extract_dir, wheel_info.wheel_name)
            os.makedirs(wheel_extract_dir, exist_ok=True)

            # Extract wheel contents
            _logger.debug(f"Extracting {wheel_info.wheel_name} to {wheel_extract_dir}")
            with zipfile.ZipFile(os.path.join(input_folder, wheel_info.wheel_name), "r") as wheel_zip:
                wheel_zip.extractall(wheel_extract_dir)

            extracted_files = os.listdir(wheel_extract_dir)

            # Find .pyd file (Windows Python extension)
            pyd_files = [f for f in extracted_files if f.endswith(".pyd")]
            assert len(pyd_files) == 1, f"Expected 1 .pyd file, got {len(pyd_files)}"
            cpython_lib = pyd_files[0]
            _logger.debug(f"Found .pyd file: {cpython_lib}")

            # Find python3X.dll
            python_dlls = [f for f in extracted_files if f.endswith(".dll") and f.startswith("python3")]
            assert len(python_dlls) == 1, f"Expected 1 python3X.dll file, got {len(python_dlls)}"
            python_dll = python_dlls[0]
            _logger.debug(f"Found python dll: {python_dll}")

            # Find .data folder
            data_folders = [f for f in extracted_files if f.endswith(".data")]
            assert len(data_folders) == 1, f"Expected 1 .data folder, got {len(data_folders)}"
            data_folder = data_folders[0]
            _logger.debug(f"Found data folder: {data_folder}")

            # Find .dist-info directory
            dist_info_dirs = [f for f in extracted_files if f.endswith(".dist-info")]
            assert len(dist_info_dirs) == 1, f"Expected 1 .dist-info directory, got {len(dist_info_dirs)}"
            dist_info_dir = dist_info_dirs[0]
            _logger.debug(f"Found dist-info directory: {dist_info_dir}")

            # Delete extra DLLs (already in the .data/platlib folder)
            extra_dlls = [f for f in extracted_files if f.endswith(".dll") and (f != python_dll)]
            _logger.debug(f"Found extra DLLs: {extra_dlls}. Removing them.")
            for dll in extra_dlls:
                os.remove(os.path.join(wheel_extract_dir, dll))

            extracted_wheels.append(ExtractedWheel(wheel_info, wheel_extract_dir, cpython_lib, python_dll, data_folder, dist_info_dir))

        # Step: Rename libraries in the .data/platlib folder to new names (remangling)
        magic_hash = str(time.perf_counter())[-5:] + "win"
        _logger.info(f"Step: Renaming libraries in .data/platlib folders | Magic hash: {magic_hash}")

        # Store rename mappings for each wheel
        all_name_maps = {}
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, python_dll, data_folder, dist_info_dir = extracted_wheel

            platlib_dir = os.path.join(wheel_extract_dir, data_folder, "platlib")
            if not os.path.exists(platlib_dir):
                _logger.warning(f"No platlib directory found in {data_folder}")
                all_name_maps[wheel_info.wheel_name] = {}
                continue

            # Generate rename mapping for DLLs in platlib
            name_map = {}
            bundled_dlls = os.listdir(platlib_dir)
            for dll in bundled_dlls:
                if not dll.endswith(".dll"):
                    continue
                original_name = dll
                new_name = _remangle_name(original_name, magic_hash)
                name_map[original_name] = new_name
                _logger.debug(f"Will rename: {original_name} -> {new_name}")

            all_name_maps[wheel_info.wheel_name] = name_map

        # Step: Patch all binaries to use the new DLL names
        _logger.info("Step: Patching binaries with new DLL names")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, python_dll, data_folder, dist_info_dir = extracted_wheel
            name_map = all_name_maps[wheel_info.wheel_name]

            if not name_map:
                continue

            platlib_dir = os.path.join(wheel_extract_dir, data_folder, "platlib")

            # Patch the .pyd file
            pyd_path = os.path.join(wheel_extract_dir, cpython_lib)
            try:
                _logger.info(f"Patching {cpython_lib}")
                needed = dll_utils.get_direct_mangleable_needed(pyd_path, {}, {})
                pyd_name_map = {old: name_map[old] for old in needed if old in name_map}
                if pyd_name_map:
                    dll_utils.replace_needed(pyd_path, list(pyd_name_map.keys()), pyd_name_map, strip=False)
            except Exception as e:
                _logger.error(f"Error patching {cpython_lib}: {e}")

            # Patch all DLLs in platlib
            for dll in os.listdir(platlib_dir):
                if not dll.endswith(".dll"):
                    continue

                dll_path = os.path.join(platlib_dir, dll)
                try:
                    _logger.info(f"Patching {dll}")
                    needed = dll_utils.get_direct_mangleable_needed(dll_path, {}, {})
                    dll_name_map = {old: name_map[old] for old in needed if old in name_map}
                    if dll_name_map:
                        dll_utils.replace_needed(dll_path, list(dll_name_map.keys()), dll_name_map, strip=False)
                except Exception as e:
                    _logger.error(f"Error patching {dll}: {e}")

        # Step: Actually rename the DLL files
        _logger.info("Step: Renaming DLL files")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, python_dll, data_folder, dist_info_dir = extracted_wheel
            name_map = all_name_maps[wheel_info.wheel_name]

            if not name_map:
                continue

            platlib_dir = os.path.join(wheel_extract_dir, data_folder, "platlib")

            for original_name, new_name in name_map.items():
                old_path = os.path.join(platlib_dir, original_name)
                new_path = os.path.join(platlib_dir, new_name)
                if os.path.exists(old_path):
                    _logger.debug(f"Renaming {original_name} -> {new_name}")
                    os.rename(old_path, new_path)

        # Step: Prepare staging directory's .dist-info
        _logger.info("Step: Preparing .dist-info directory in staging area")
        new_dist_info_path = os.path.join(staging_dir, extracted_wheels[0].dist_info_dir)
        os.makedirs(new_dist_info_path, exist_ok=True)

        # Copy metadata files from the first wheel, excluding RECORD
        first_wheel_dist_info = os.path.join(extracted_wheels[0].wheel_extract_dir, extracted_wheels[0].dist_info_dir)
        for item in os.listdir(first_wheel_dist_info):
            if item.lower() not in ["record"]:
                shutil.copy2(os.path.join(first_wheel_dist_info, item), new_dist_info_path)

        # Step: Copy all other content to the staging directory
        _logger.info("Step: Copying all other content to the staging directory")
        for extracted_wheel in extracted_wheels:
            wheel_info, wheel_extract_dir, cpython_lib, python_dll, data_folder, dist_info_dir = extracted_wheel
            for item in os.listdir(wheel_extract_dir):
                if item.endswith(".dist-info"):
                    continue
                src_path = os.path.join(wheel_extract_dir, item)
                dest_path = os.path.join(staging_dir, item)
                if os.path.isdir(src_path):
                    shutil.copytree(src_path, dest_path, dirs_exist_ok=True)
                else:
                    shutil.copy2(src_path, dest_path)

        # Step: Update WHEEL and generate RECORD files for PEP 427 compliance
        _logger.info("Step: Updating WHEEL and generating RECORD file for PEP 427 compliance")
        _update_wheel_file(new_dist_info_path, infos)
        _generate_record_file(staging_dir, new_dist_info_path)

        # Step: zip the staging directory
        _logger.info("Step: Zipping the staging directory")
        zip_path = os.path.join(output_folder, combined_info.wheel_name)
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as output_zip:
            for root, dirs, files in os.walk(staging_dir):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, staging_dir)
                    output_zip.write(file_path, arcname)
        _logger.info("Output zip created")
        _logger.info(f"Output zip size: {os.path.getsize(zip_path) / (1024 * 1024):.2f} MB")
        _logger.info(f"Combined wheel saved to {zip_path}")


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

    # Combine wheels for the current platform
    if sys.platform == "linux":
        _combine_wheels_linux(input_folder, output_folder, strip)
    elif sys.platform == "darwin":
        _combine_wheels_macos(input_folder, output_folder, strip)
    elif sys.platform == "win32":
        _combine_wheels_windows(input_folder, output_folder, strip)
    else:
        raise ValueError(f"Unsupported platform: {sys.platform}")


if __name__ == "__main__":
    import argparse

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
    args = parser.parse_args()

    # Combine wheels
    combine_wheels(
        input_folder=args.input_folder,
        output_folder=args.output_folder,
        strip=args.strip,
        log_level=args.log_level,
    )