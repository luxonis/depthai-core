# Maturity level "snapshot" / "release"
# set(DEPTHAI_DEVICE_KB_MATURITY "release")
set(DEPTHAI_DEVICE_RVC4_MATURITY "snapshot")

# Firmware package that the host library embeds. Empty (the default): the build builds the firmware from device/rvc4
# of this checkout (cmake/DepthaiDeviceRvc4FirmwareBuild.cmake, needs bash and Docker). A version, for example
# -D DEPTHAI_DEVICE_RVC4_VERSION=0.0.1+<commit>, downloads that package from the artifact server.
set(DEPTHAI_DEVICE_RVC4_VERSION "" CACHE STRING "RVC4 firmware package to download; empty: build it from device/rvc4")

# Prebuilt vendor module (licensee-only code of the RVC4 firmware) that a firmware build of device/rvc4 links:
# depthai-device-rvc4-vendor-<version>[-asan-ubsan|-tsan].tar.xz (cmake/DepthaiDeviceRvc4VendorDownloader.cmake)
set(DEPTHAI_DEVICE_RVC4_VENDOR_VERSION "0.0.1+0a6d9452cc8af555f16d33d1e7c2bc551cc7110d")

# Public build image of the RVC4 firmware (device/rvc4/docker/Dockerfile.sdk) that the build scripts of device/rvc4 use
# when DEPTHAI_DEVICE_SDK_IMAGE is not set: ghcr.io/luxonis/depthai-device-sdk:os-<Luxonis OS version>-<first 12
# characters of the commit of the vendor repository that made the public sysroot>. The sanitizer targets use the same
# tag with -dev.
# device/rvc4/docker/sdk-images.sh builds and pushes it; a fixed tag is never moved. The OS release of its sysroot must
# be the one of the vendor module above: the firmware build compares them.
set(DEPTHAI_DEVICE_RVC4_SDK_IMAGE "ghcr.io/luxonis/depthai-device-sdk:os-1.37.0-f610793ec1db")
