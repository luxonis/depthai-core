import sys
import os
import re

# Use absolute path for project_root
project_root = os.path.abspath(os.path.dirname(__file__))
version_depthai_core_path = os.path.join(project_root, "..", "..", "CMakeLists.txt")

# Regex pattern to parse the version and pre-release details
cmake_lists_txt_version_pattern = r'project\s*\(\s*[^)]*VERSION\s+"?([\d\.]+)"?\s*[^)]*\)'
cmake_lists_txt_pre_release_pattern = r'set\s*\(\s*DEPTHAI_PRE_RELEASE_TYPE\s+"([^"]*)"\s*\)'
cmake_lists_txt_pre_release_version_pattern = r'set\s*\(\s*DEPTHAI_PRE_RELEASE_VERSION\s+"([^"]*)"\s*\)'

def get_version_from_cmake_lists(path, pattern):
    try:
        with open(path, 'r') as file:
            content = file.read()
        match = re.search(pattern, content, flags=re.IGNORECASE)
        if match:
            return match.group(1)
        else:
            raise ValueError(f"No match found for pattern in {path}")
    except FileNotFoundError:
        sys.exit(f"Error: File not found {path}")
    except Exception as e:
        sys.exit(f"Error while reading from {path}: {str(e)}")

def get_package_version():
    version_core = get_version_from_cmake_lists(version_depthai_core_path, cmake_lists_txt_version_pattern)
    pre_release_type = get_version_from_cmake_lists(version_depthai_core_path, cmake_lists_txt_pre_release_pattern)
    pre_release_version = get_version_from_cmake_lists(version_depthai_core_path, cmake_lists_txt_pre_release_version_pattern)

    if version_core:
        if pre_release_type and pre_release_version:
            version_core += f"-{pre_release_type}.{pre_release_version}"
        return version_core
    else:
        return None

def get_package_dev_version(commit_hash):
    package_version = get_package_version()
    if package_version:
        return f"{package_version}.dev0+{commit_hash}"
    else:
        return None

if __name__ == '__main__':
    commit_hash = "123abc"  # Placeholder for actual commit hash
    print("Package Version:", get_package_version())
    print("Development Package Version:", get_package_dev_version(commit_hash))
