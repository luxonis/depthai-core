# DepthAI C++ Library

Core C++ library

## Dependencies
- cmake >= 3.2
- libusb1 development package
- C/C++11 compiler
 
MacOS: `brew install libusb`

Linux: `sudo apt install libusb-1.0-0-dev`

## Using as library

To use this library in your own project you can use cmake add_subdirectory pointing to the root of this repository
 - CMake: add_subdirectory(depthai-core)

See [example/CMakeLists.txt](example/CMakeLists.txt)

## Building

To build the library from source perform the following:
```
mkdir build && cd build
cmake ..
cmake --build . --parallel
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
