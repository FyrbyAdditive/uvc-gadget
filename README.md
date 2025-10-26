# uvcgadget - UVC gadget C library

uvcgadget is a pure C library that implements handling of UVC gadget functions.
This is a fork of https://gitlab.freedesktop.org/camera/uvc-gadget to add just pan and tilt (so far).
The servos currently function via a separate http controller application that is not yet published.

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
