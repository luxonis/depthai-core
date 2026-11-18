# Maturity level "snapshot" / "release"
# set(DEPTHAI_DEVICE_KB_MATURITY "release")
set(DEPTHAI_DEVICE_RVC4_MATURITY "snapshot")

# Firmware package that the host library embeds. Empty (the default): the build builds the firmware from device/rvc4
# of this checkout (cmake/DepthaiDeviceRvc4FirmwareBuild.cmake, needs bash and Docker). A version, for example
# -D DEPTHAI_DEVICE_RVC4_VERSION=0.0.1+<commit>, downloads that package from the artifact server.
set(DEPTHAI_DEVICE_RVC4_VERSION "" CACHE STRING "RVC4 firmware package to download; empty: build it from device/rvc4")

# Prebuilt vendor module (licensee-only code of the RVC4 firmware) that a firmware build of device/rvc4 links:
# depthai-device-rvc4-vendor-<version>[-asan-ubsan|-tsan].tar.xz (cmake/DepthaiDeviceRvc4VendorDownloader.cmake)
set(DEPTHAI_DEVICE_RVC4_VENDOR_VERSION "0.0.1+440991f38212ae738b08a930feff9b66061dee0c")
