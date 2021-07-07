# DepthAI C++ Library

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

Core C++ library

## Documentation

Documentation is available over at [Luxonis DepthAI API](https://docs.luxonis.com/projects/api/en/latest/)

## Disclaimer
DepthAI library doesn't yet provide API stability guarantees. While we take care to properly deprecate old functions, some changes might still be breaking. We expect to provide API stability from version 3.0.0 onwards.

## Dependencies
- cmake >= 3.4
- libusb1 development package (MacOS & Linux only)
- C/C++14 compiler
- [optional] OpenCV 4

MacOS: `brew install libusb`

Linux: `sudo apt install libusb-1.0-0-dev`

## Integration

### CMake

Targets available to link to are:
 - depthai::core - Core library, without using opencv internally
 - depthai::opencv - Core + support for opencv related helper functions (requires OpenCV4)

#### Using find_package

Build static or dynamic version of library and install (See: [Building](##building) and [Installing](##installing))

Add `find_package` and `target_link_libraries` to your project
```
find_package(depthai CONFIG REQUIRED)
...
target_link_libraries([my-app] PRIVATE depthai::opencv)
```

And point CMake to either build directory or install directory:
```
-D depthai_DIR=depthai-core/build
```
or
```
-D depthai_DIR=depthai-core/build/install/lib/cmake/depthai
```

If library was installed to default search path like `/usr/local` on Linux, specifying `depthai_DIR` isn't necessary as CMake will find it automatically.

#### Using add_subdirectory

This method is more intrusive but simpler as it doesn't require building the library separately.

Add `add_subdirectory` which points to `depthai-core` folder **before** project command. Then link to any required targets.
```
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)
...
project(my-app)
...
target_link_libraries([my-app] PRIVATE depthai::opencv)
```

### Non-CMake integration (Visual Studio, Xcode, CodeBlocks, ...)

To integrate into a different build system than CMake, prefered way is compiling as dynamic library and setting correct build options.
1. First build as dynamic library: [Building Dynamic library](###dynamic-library)
2. Then install: [Installing](##installing)
3. Set needed library directories:
    - `build/install/lib` (for linking to either depthai-core or depthai-opencv)
    - `build/install/bin` (for .dll's)
4. And include directories
    - `build/install/include` (library headers)
    - `build/install/include/depthai-shared/3rdparty` (shared 3rdparty headers)
    - `build/install/lib/cmake/depthai/dependencies/include` (dependency headers)
5. Add the following defines
    - `XLINK_USE_MX_ID_NAME=ON`
    - `__PC__=ON`

> ℹ️ Threading library might need to be linked to explicitly.

> ℹ️ Check `build/depthai-core-integration.txt` or `build/depthai-opencv-integration.txt` for up to date define options.
The generated integration file also specifies include paths without requiring installation.

## Building

Make sure submodules are updated
```
git submodule update --init --recursive
```

> ℹ️ To speed up build times, use `cmake --build build --parallel [num CPU cores]` (CMake >= 3.12).
For older versions use: Linux/macOS: `cmake --build build -- -j[num CPU cores]`, MSVC: `cmake --build build -- /MP[num CPU cores]`

### Static library
```
cmake -H. -Bbuild
cmake --build build
```

### Dynamic library
```
cmake -H. -Bbuild -D BUILD_SHARED_LIBS=ON
cmake --build build
```
## Installing

To install specify optional prefix and build target install
```
cmake -H. -Bbuild -D CMAKE_INSTALL_PREFIX=[path/to/install/dir]
cmake --build build
cmake --build build --target install
```

If `CMAKE_INSTALL_PREFIX` isn't specified, the library is installed under build folder `install`.

## Running tests

To run the tests build the library with the following options
```
cmake -H. -Bbuild -D DEPTHAI_TEST_EXAMPLES=ON -D DEPTHAI_BUILD_TESTS=ON -D DEPTHAI_BUILD_EXAMPLES=ON
cmake --build build
```

Then navigate to `build` folder and run `ctest`
```
cd build
ctest
```

## Style check

The library uses clang format to enforce a certain coding style.
If a style check is failing, run the `clangformat` target, check the output and push changes.

To use this target clang format must be installed, preferably clang-format-10
```
sudo apt install clang-format-10
```

And to apply formatting
```
cmake --build build --target clangformat
```

## Documentation generation

Doxygen is used to generate documentation. Follow [doxygen download](https://www.doxygen.nl/download.html#srcbin) and install the required binaries for your platform.

After that specify CMake define `-D DEPTHAI_BUILD_DOCS=ON` and build the target `doxygen`

## Debugging tips

Debugging can be done using **Visual Studio Code** and either **GDB** or **LLDB** (extension 'CodeLLDB').
LLDB in some cases was much faster to step with and resolved more `incomplete_type` variables than GDB. Your mileage may vary though.


If there is a need to step into **Hunter** libraries, that can be achieved by removing previous built artifacts
```
rm -r ~/.hunter
```

And configuring the project with the following CMake option set to `ON`
```
cmake . -D HUNTER_KEEP_PACKAGE_SOURCES=ON
```

This retains the libraries source code, so that debugger can step through it (the paths are already set up correctly)


## Troubleshooting

### Hunter
Hunter is a CMake-only dependency manager for C/C++ projects.

If you are stuck with error message which mentions external libraries (subdirectory of `.hunter`) like the following:
```
/usr/bin/ld: /home/[user]/.hunter/_Base/062a19a/ccfed35/a84a713/Install/lib/liblzma.a(stream_flags_decoder.c.o): warning: relocation against `lzma_footer_magic' in read-only section `.text'
```

Try erasing the **Hunter** cache folder.

Linux/MacOS:
```
rm -r ~/.hunter
```
Windows:
```
del C:/.hunter
```
or
```
del C:/[user]/.hunter
```
