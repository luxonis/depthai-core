# SPDX-License-Identifier: CC0-1.0

project('uvc-gadget', 'c', 'cpp',
    meson_version : '>= 0.56',
    version : '0.4.0',
    default_options : [
        'werror=true',
        'warning_level=2',
        'cpp_std=c++17',
    ],
    license : 'LGPL 2.1+')

# Generate version information. The uvc_gadget_git_version variable contains the
# full version with git patch count and SHA1 (e.g. 1.2.3+211-c94a24f4), while
# the uvc_gadget_version variable contains the major.minor.patch (e.g. 1.2.3)
# only. If the source tree isn't under git control, or if it matches the last
# git version tag, the build metadata (e.g. +211-c94a24f4) is omitted from
# uvc_gadget_git_version.
uvc_gadget_git_version = run_command('scripts/gen-version.sh',
                                    meson.project_build_root(),
                                    meson.project_source_root(),
                                    check: false).stdout().strip()
if uvc_gadget_git_version == ''
    uvc_gadget_git_version = meson.project_version()
endif

uvc_gadget_version = uvc_gadget_git_version.split('+')[0]

# A shallow clone, or a clone without a reachable tag equivalent to the
# meson.project_version() could leave the project in a mis-described state.
# Produce a warning in this event, and fix to a best effort.
if uvc_gadget_version != meson.project_version()
    warning('The sources disagree about the version: '
            + uvc_gadget_version + ' != ' + meson.project_version())

    summary({'uvc-gadget git version' : uvc_gadget_git_version,
             'Source version match' : false,
            },
            bool_yn : true, section : 'Versions')

    # Replace the version components reported by git with the release version,
    # but keep all trailing information supplied by git.
    uvc_gadget_git_version = (meson.project_version() +
                              uvc_gadget_git_version.strip(uvc_gadget_version))
    uvc_gadget_version = meson.project_version()

    # Append a marker to show we have modified this version string
    uvc_gadget_git_version += '-nvm'
endif

summary({ 'Sources': uvc_gadget_git_version, }, section : 'Versions')

# Configure the build environment.
cc = meson.get_compiler('c')

libcamera = dependency('libcamera', required : false)
libjpeg = dependency('libjpeg', required : false)
threads = dependency('threads', required : false)

conf = configuration_data()

if libcamera.found()
  conf.set('HAVE_LIBCAMERA', true)
endif

if libjpeg.found() and threads.found()
    conf.set('CONFIG_CAN_ENCODE', true)
endif

configure_file(output : 'config.h', configuration : conf)
config_includes = include_directories('.')

subdir('include')

subdir('lib')
subdir('src')
