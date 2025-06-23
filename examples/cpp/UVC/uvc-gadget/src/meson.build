# SPDX-License-Identifier: CC0-1.0

libuvcgadget = dependency('uvcgadget')

gadget = executable('uvc-gadget', 'main.c',
                    dependencies : [
                        libuvcgadget,
                    ],
                    include_directories : [includes, config_includes],
                    install : true)
