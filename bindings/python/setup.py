import os
import re
import sys
import platform
import shutil
import subprocess
import find_version

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
try:
    from setuptools.command.bdist_wheel import bdist_wheel
except ImportError:  # setuptools < 70.1
    from wheel.bdist_wheel import bdist_wheel
from packaging.version import Version
from pathlib import Path

MODULE_NAME = 'depthai'
DEPTHAI_CLI_MODULE_NAME = 'depthai_cli'

here = os.path.abspath(os.path.dirname(__file__))
version_file = os.path.join(here, "generated", "version.py")
os.makedirs(os.path.join(here, "generated"), exist_ok=True)
if os.environ.get('CI') != None :
    final_version = find_version.get_package_version()
    if os.environ.get('BUILD_COMMIT_HASH') != None:
        final_version = find_version.get_package_dev_version(os.environ['BUILD_COMMIT_HASH'])
    with open(version_file, 'w') as vf :
        vf.write("__version__ = '" + final_version + "'")
elif os.path.exists("../../.git"):
    commit_hash = 'dev'
    try:
        commit_hash = (
            subprocess.check_output(
                ["git", "rev-parse", "HEAD"], stderr=subprocess.STDOUT
            )
            .splitlines()[0]
            .decode()
        )
    except subprocess.CalledProcessError as e:
        commit_hash = 'dev'
    final_version = find_version.get_package_dev_version(commit_hash)

    with open(version_file, 'w') as vf :
        vf.write("__version__ = '" + final_version + "'")


if os.path.isfile(version_file) == False :
    final_version = find_version.get_package_dev_version('dev')
    with open(version_file, 'w') as vf :
        vf.write("__version__ = '" + final_version + "'")

exec(open(version_file).read())
buildCommitHash = None
if len(__version__.split("+")) > 1 :
    buildCommitHash = __version__.split("+")[1]

repo_root = os.path.abspath(os.path.join(here, os.pardir, os.pardir))
source_license = os.path.join(repo_root, "LICENSE")
source_notices_dir = os.path.join(repo_root, "notices")
wheel_license_output_dir = os.path.join(here, "generated", "wheel_licenses")
wheel_sbom_output_dir = os.path.join(here, "generated", "wheel_sbom")
packaged_license = os.path.join(here, "LICENSE")
packaged_notices_dir = os.path.join(here, "notices")

def _stage_license_files():
    if not os.path.isfile(source_license):
        raise RuntimeError(f"Root license file does not exist: {source_license}")
    shutil.copy2(source_license, packaged_license)

    if os.path.isdir(source_notices_dir):
        if os.path.isdir(packaged_notices_dir):
            shutil.rmtree(packaged_notices_dir)
        shutil.copytree(source_notices_dir, packaged_notices_dir)

def _license_files():
    if not os.path.isdir(packaged_notices_dir):
        return ["LICENSE"]
    notice_files = sorted(f"notices/{path.name}" for path in Path(packaged_notices_dir).iterdir() if path.is_file())
    if not notice_files:
        return ["LICENSE"]
    return ["LICENSE", *notice_files]

_stage_license_files()


long_description = open("README.md", "r", encoding="utf-8").read()

MACOSX_DEPLOYMENT_TARGETS = {
    "arm64": "11.0",
    "x86_64": "11.0",
}
MACOS_SETTINGS = {}

def _configure_macos_build_settings():
    arch = platform.machine().lower()
    arch = "arm64" if arch == "arm64" else "x86_64"
    default_target = MACOSX_DEPLOYMENT_TARGETS[arch]
    deployment_target = os.environ.get("MACOSX_DEPLOYMENT_TARGET", default_target)
    os.environ["MACOSX_DEPLOYMENT_TARGET"] = deployment_target

    wheel_tag = f"macosx-{deployment_target}-{arch}"
    os.environ["_PYTHON_HOST_PLATFORM"] = wheel_tag
    os.environ.setdefault("ARCHFLAGS", f"-arch {arch}")
    os.environ.setdefault("CMAKE_OSX_ARCHITECTURES", arch)
    return {
        "arch": arch,
        "deployment_target": deployment_target,
        "wheel_tag": wheel_tag,
    }

if sys.platform == "darwin":
    MACOS_SETTINGS = _configure_macos_build_settings()

def _read_cmake_cache(build_dir):
    cache_path = os.path.join(build_dir, "CMakeCache.txt")
    if not os.path.isfile(cache_path):
        return {}

    values = {}
    with open(cache_path, "r", encoding="utf-8", errors="replace") as cache_file:
        for line in cache_file:
            line = line.strip()
            if not line or line.startswith(("#", "//")) or "=" not in line:
                continue
            key_type, value = line.split("=", 1)
            key = key_type.split(":", 1)[0]
            values[key] = value
    return values

def _generate_wheel_license_notices(build_dir):
    cache = _read_cmake_cache(build_dir)
    install_root = cache.get("VCPKG_INSTALLED_DIR") or os.path.join(build_dir, "vcpkg_installed")
    triplet = cache.get("VCPKG_TARGET_TRIPLET") or os.environ.get("VCPKG_DEFAULT_TRIPLET")

    if not triplet:
        candidates = sorted(Path(install_root).glob("*/share"))
        if candidates:
            triplet = candidates[0].parent.name

    if not triplet:
        raise RuntimeError("Cannot determine VCPKG_TARGET_TRIPLET for third-party notice generation.")

    script_path = os.path.join(repo_root, "scripts", "generate_third_party_notices.py")
    cmd = [
        sys.executable,
        script_path,
        "--repo-root", repo_root,
        "--build-dir", build_dir,
        "--install-root", install_root,
        "--triplet", triplet,
        "--output-dir", wheel_license_output_dir,
    ]
    subprocess.check_call(cmd)
    _stage_license_files()

def _generate_wheel_sbom(build_dir):
    if os.path.isdir(wheel_sbom_output_dir):
        shutil.rmtree(wheel_sbom_output_dir)
    script_path = os.path.join(repo_root, "scripts", "generate_sbom.py")
    try:  # Best effort: a wheel without an SBOM works; CI checks it (scripts/verify_license_artifact.py)
        subprocess.check_call([
            sys.executable,
            script_path,
            "build",
            "--wheel",
            "--repo-root", repo_root,
            "--build-dir", build_dir,
            "--version", __version__,
            "--output-dir", wheel_sbom_output_dir,
        ])
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"warning: no SBOM for this wheel: {error}")
        if os.path.isdir(wheel_sbom_output_dir):
            shutil.rmtree(wheel_sbom_output_dir)

class BdistWheelWithSbom(bdist_wheel):
    def write_wheelfile(self, wheelfile_base, *args, **kwargs):  # PEP 770; runs for both egg-info and pip dist-info
        super().write_wheelfile(wheelfile_base, *args, **kwargs)
        if not os.path.isdir(wheel_sbom_output_dir):
            return
        sbom_dir = os.path.join(wheelfile_base, "sboms")
        os.makedirs(sbom_dir, exist_ok=True)
        for name in sorted(os.listdir(wheel_sbom_output_dir)):
            if name.endswith((".spdx.json", ".cdx.json")):
                shutil.copy2(os.path.join(wheel_sbom_output_dir, name), sbom_dir)

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):

    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = Version(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < Version('3.2.0'):
                raise RuntimeError("CMake >= 3.2.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        if ext.name == DEPTHAI_CLI_MODULE_NAME:
            cam_test_path      = os.path.join(str(repo_root), "utilities", "cam_test.py")
            cam_test_dest = os.path.join(self.build_lib, DEPTHAI_CLI_MODULE_NAME, "cam_test.py")
            cam_test_gui_path  = os.path.join(str(repo_root), "utilities", "cam_test_gui.py")
            cam_test_gui_dest = os.path.join(self.build_lib, DEPTHAI_CLI_MODULE_NAME, "cam_test_gui.py")
            stress_test_path   = os.path.join(str(repo_root), "utilities", "stress_test.py")
            stress_test_dest = os.path.join(self.build_lib, DEPTHAI_CLI_MODULE_NAME, "stress_test.py")
            flash_network_bootloader_path = os.path.join(str(repo_root), "utilities", "flash_network_bootloader.py")
            flash_network_bootloader_dest = os.path.join(self.build_lib, DEPTHAI_CLI_MODULE_NAME, "flash_network_bootloader.py")
            files_to_copy = [
                (cam_test_path, cam_test_dest),
                (cam_test_gui_path, cam_test_gui_dest),
                (stress_test_path, stress_test_dest),
                (flash_network_bootloader_path, flash_network_bootloader_dest),
            ]
            for src, dst in files_to_copy:
                with open(src, "r") as f:
                    with open(dst, "w") as f2:
                        f2.write(f.read())
            return

        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        if not extdir.endswith(os.path.sep):  # Trailing separator: needed to auto-detect auxiliary native libs
            extdir += os.path.sep

        cmake_args = []
        build_args = []
        env = os.environ.copy()

        cmake_args += ['-DDEPTHAI_BUILD_PYTHON=ON']
        cmake_args += ['-DDEPTHAI_ENABLE_EVENTS_MANAGER=ON']
        if env.get('DEPTHAI_BUILD_BETA') == 'ON':
            cmake_args += ['-DDEPTHAI_BUILD_BETA=ON']

        if env.get("CI") is not None:  # Shared libs only in CI, for downstream wheel bundling
            cmake_args += ['-DBUILD_SHARED_LIBS=ON']

        cmake_args += ['-DDEPTHAI_MERGED_TARGET=ON']
        if env.get('DEPTHAI_BUILD_BASALT') == 'ON':
            cmake_args += ['-DDEPTHAI_BASALT_SUPPORT=ON']
        if env.get('DEPTHAI_BUILD_PCL') == 'ON':
            cmake_args += ['-DDEPTHAI_PCL_SUPPORT=ON']
        if env.get('DEPTHAI_BUILD_RTABMAP') == 'ON':
            cmake_args += ['-DDEPTHAI_RTABMAP_SUPPORT=ON']
        if env.get('DEPTHAI_BUILD_KOMPUTE') == 'ON':
            cmake_args += ['-DDEPTHAI_KOMPUTE_SUPPORT=ON']
        build_args += ['--target=depthai']


        cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir, '-DPYTHON_EXECUTABLE=' + sys.executable]
        if platform.system() == "Windows":
            env = {key:env[key] for key in env if key.upper() != 'pythonLocation'.upper()}
        env['pythonLocation'] = str(Path(sys.executable).parent.absolute())


        if buildCommitHash != None :
            cmake_args += ['-DDEPTHAI_PYTHON_COMMIT_HASH=' + buildCommitHash]

        if 'DEPTHAI_PYTHON_DOCSTRINGS_INPUT' in os.environ:
            cmake_args += ['-DDEPTHAI_PYTHON_DOCSTRINGS_INPUT='+os.environ['DEPTHAI_PYTHON_DOCSTRINGS_INPUT']]

        if 'DEPTHAI_INSTALLATION_DIR' in os.environ:
            cmake_args += ['-DDEPTHAI_PYTHON_USE_FIND_PACKAGE=ON']
            cmake_args += ['-DCMAKE_PREFIX_PATH='+os.environ['DEPTHAI_INSTALLATION_DIR']]

        cfg = 'Debug' if self.debug else 'Release'
        cmake_args += ['-DPIP_TEMP_LIB_FOLDER=' + os.path.abspath(os.path.join(self.build_temp, cfg))]
        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        cmake_args += ['-DDEPTHAI_VCPKG_INTERNAL_ONLY=OFF']
        cmake_args += ['-DDEPTHAI_EXCLUDE_ARCHIVE_LIBS_SYMBOLS=ON']
        cmake_args += ['-DDEPTHAI_LINK_GC_SECTIONS=ON']  # Linux; removes much only with EXCLUDE_ARCHIVE_LIBS_SYMBOLS
        build_args += ['--config', cfg]
        if not self.debug:
            rootDirectory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            cmake_args += [
                '-DVCPKG_OVERLAY_TRIPLETS={}'.format(
                    os.path.join(rootDirectory, 'cmake', 'triplets', 'release')
                )
            ]
        freeMemory = 4000
        if platform.system() == "Linux":
            try:
                freeMemory = int(os.popen("free -m").readlines()[1].split()[6])
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                freeMemory = 4000

        if 'CMAKE_BUILD_ARGS' in os.environ:
            build_args += [os.environ['CMAKE_BUILD_ARGS']]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]


            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            else:
                cmake_args += ['-A', 'Win32']

            build_args += ['--', '/m']
        else:
            if sys.platform == 'darwin':
                mac_arch = MACOS_SETTINGS.get('arch', platform.machine())
                deployment_target = MACOS_SETTINGS.get('deployment_target')
                wheel_tag = MACOS_SETTINGS.get('wheel_tag')
                if wheel_tag:
                    env['_PYTHON_HOST_PLATFORM'] = wheel_tag
                if deployment_target:
                    env['MACOSX_DEPLOYMENT_TARGET'] = deployment_target
                env.setdefault('ARCHFLAGS', os.environ.get('ARCHFLAGS', f'-arch {mac_arch}'))
                cmake_arch = os.environ.get('CMAKE_OSX_ARCHITECTURES', mac_arch)
                if not any(arg.startswith('-DCMAKE_OSX_ARCHITECTURES=') for arg in cmake_args):
                    cmake_args += [f'-DCMAKE_OSX_ARCHITECTURES={cmake_arch}']
                if deployment_target and not any(arg.startswith('-DCMAKE_OSX_DEPLOYMENT_TARGET=') for arg in cmake_args):
                    cmake_args += [f'-DCMAKE_OSX_DEPLOYMENT_TARGET={deployment_target}']

            max_threads = os.cpu_count()
            num_threads = (freeMemory // 2000)
            num_threads = min(num_threads, max_threads)
            if num_threads <= 0:
                num_threads = 1
            build_args += ['--', '-j' + str(num_threads)]
            cmake_args += ['-DHUNTER_JOBS_NUMBER=' + str(num_threads)]

        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''), self.distribution.get_version())

        if 'CMAKE_ARGS' in os.environ:
            cmake_args += [os.environ['CMAKE_ARGS']]
        if ("free-threading build" in sys.version):
            cmake_args += ["-DDEPTHAI_PYTHON_GIL_FREE=ON"]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        if os.environ.get('DEPTHAI_CONFIGURE_ONLY') == '1':  # Configure installs the deps; for vcpkg_cache_warm.yml
            print('DEPTHAI_CONFIGURE_ONLY=1: dependencies installed and project configured, not building')
            return
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp, env=env)
        _generate_wheel_license_notices(self.build_temp)
        _generate_wheel_sbom(self.build_temp)

cmdclass = {
    'build_ext': CMakeBuild,
    'bdist_wheel': BdistWheelWithSbom,
}

setup(
    name=MODULE_NAME,
    version=__version__,
    author='Luxonis',
    author_email='support@luxonis.com',
    description='DepthAI Python Library',
    license="MIT",
    license_files=_license_files(),
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/luxonis/depthai-core/tree/main/bindings/python",
    ext_modules=[
        CMakeExtension(MODULE_NAME, str(Path(__file__).absolute().parent.parent.parent.absolute())),
        Extension(DEPTHAI_CLI_MODULE_NAME, sources=[])
    ],
    cmdclass=cmdclass,
    packages=[DEPTHAI_CLI_MODULE_NAME],
    zip_safe=False,
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Information Technology",
        "Intended Audience :: Science/Research",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: Unix",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "Programming Language :: Python :: 3.14",
        "Programming Language :: Python :: 3.15",
        "Programming Language :: C++",
        "Programming Language :: Python :: Implementation :: CPython",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development",
    ],
    python_requires='>=3.9',
    install_requires=[
        "numpy<3.0.0",
    ],
    entry_points={
        "console_scripts": [
            f'depthai={DEPTHAI_CLI_MODULE_NAME}.depthai_cli:cli'
        ]
    }
)
