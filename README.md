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
- C/C++11 compiler
- [optional] OpenCV 4

MacOS: `brew install libusb`

Linux: `sudo apt install libusb-1.0-0-dev`

## Using as library

To use this library in your own project you can use CMake `add_subdirectory` pointing to the root of this repository
Optionally append `EXCLUDE_FROM_ALL` to hide depthai-core related targets, etc...
```
add_subdirectory(depthai-core)
```
or
```
add_subdirectory(depthai-core EXCLUDE_FROM_ALL)
```
And at the end link to your target (PUBLIC or PRIVATE depending on your needs)
```
target_link_libraries(my-app PUBLIC depthai-core)
```

## Building

Make sure submodules are updated 
```
git submodule update --init --recursive
```

**Static library** 
```
mkdir build && cd build
cmake ..
cmake --build . --parallel 8
```

**Dynamic library**
```
mkdir build && cd build
cmake .. -D BUILD_SHARED_LIBS=ON
cmake --build . --parallel 8
```
## Installing

To install specify optional prefix and build target install
```
cmake .. -D CMAKE_INSTALL_PREFIX=[path/to/install/dir]
cmake --build . --parallel 8
cmake --build . --target install --parallel 8
```

## Running tests

To run the tests build the library with the following options
```
mkdir build_tests && cd build_tests
cmake .. -D DEPTHAI_TEST_EXAMPLES=ON -D DEPTHAI_BUILD_TESTS=ON -D DEPTHAI_BUILD_EXAMPLES=ON
cmake --build . --parallel 8
ctest
```

## Style check

The library uses clang format to enforce a certain style. 
If a style check is failing, run the `clangformat` target, check the output and push changes

To use this target clang format must be installed, preferably clang-format-10
```
sudo apt install clang-format-10
```

And to apply formatting
```
cmake --build [build/dir] --target clangformat
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
