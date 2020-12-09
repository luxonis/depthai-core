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

To build the static version of the library from source perform the following:
```
mkdir build && cd build
cmake ..
cmake --build . --parallel
```
And for the dynamic version of the library:
```
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
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

## Debugging tips

Debugging can be done using **Visual Studio Code** and either **GDB** or **LLDB** (extension 'CodeLLDB').
LLDB in some cases was much faster to step with and resolved more `incomplete_type` variables than GDB. Your mileage may vary though.


If there is a need to step into **Hunter** libraries, that can be achieved by removing previous built artifacts
```
rm -r ~/.hunter
```

And configuring the project with the following CMake option set to `ON`
```
cmake . -DHUNTER_KEEP_PACKAGE_SOURCES=ON
```

This retains the libraries source code, so that debugger can step through it (the paths are already set up correctly)
