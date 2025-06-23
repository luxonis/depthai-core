# uvcgadget - UVC gadget C library

uvcgadget is a pure C library that implements handling of UVC gadget functions.

## Utilities

- uvc-gadget - Sample test application

## Build instructions:

To compile:

```
$ meson build
$ ninja -C build
```

## Cross compiling instructions:

Cross compilation can be managed by meson. Please read the directions at
https://mesonbuild.com/Cross-compilation.html for detailed guidance on using
meson.

In brief summary:
```
$ meson build --cross <meson cross file>
$ ninja -C build
```
