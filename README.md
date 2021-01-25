# DepthAI C++ Library

Core C++ library

## Alpha Disclaimer
DepthAI library is currently in alpha for version 0.x.y. We are still making breaking API changes and expect to get to stable API by version 1.0.0.

## Dependencies
- cmake >= 3.2
- libusb1 development package
- C/C++11 compiler
 
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

To build the static version of the library from source perform the following:

Make sure submodules are updated 
```
git submodule update --init --recursive
```

Configure and build
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
