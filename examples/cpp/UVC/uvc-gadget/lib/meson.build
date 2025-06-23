# SPDX-License-Identifier: CC0-1.0

libuvcgadget_sources = files([
  'configfs.c',
  'events.c',
  'jpg-source.c',
  'slideshow-source.c',
  'stream.c',
  'test-source.c',
  'timer.c',
  'uvc.c',
  'v4l2.c',
  'v4l2-source.c',
  'video-buffers.c',
  'video-source.c',
])

if libcamera.found()
  libuvcgadget_sources += files(['libcamera-source.cpp'])
endif

if libjpeg.found() and threads.found()
  libuvcgadget_sources += files(['mjpeg_encoder.cpp'])
endif

libuvcgadget = shared_library('uvcgadget',
                              libuvcgadget_sources,
                              dependencies : [libcamera, libjpeg, threads],
                              version : uvc_gadget_version,
                              install : true,
                              include_directories : [includes, config_includes])

libuvcgadget_dep = declare_dependency(sources : [
                                          uvcgadget_public_headers,
                                     ],
                                     include_directories : includes,
                                     link_with : libuvcgadget)

pkg_mod = import('pkgconfig')
pkg_mod.generate(libuvcgadget,
                 description : 'UVC Gadget support library',
                 subdirs : 'uvcgadget')

meson.override_dependency('uvcgadget', libuvcgadget_dep)
