# Backport https://github.com/luxonis/libusb/commit/745c272a2f0305c4ab162b7fb1b2edc7578c6b77
# to b7e4548958325b18feb73977163ad44398099534. Failed claims must preserve
# the endpoint table shared with an already open device handle.
file(SHA256 "${CMAKE_CURRENT_LIST_FILE}" patch_hash)
if(NOT patch_hash STREQUAL DEPTHAI_LIBUSB_PATCH_HASH)
    message(FATAL_ERROR "libusb patch changed; rerun the DepthAI CMake configuration")
endif()

set(source_file "${CMAKE_CURRENT_SOURCE_DIR}/libusb/os/windows_winusb.c")
set(patched_hash "ad6b2f73e2aac5bef3f877267096a9ed5c2afb7cb1e80b7c4c59cca37ee92d0b")
file(SHA256 "${source_file}" source_hash)
if(source_hash STREQUAL patched_hash)
    return()
endif()

file(READ "${source_file}" source)
# Move endpoint cleanup from the claim/altsetting wrappers into the function
# called only after the backend operation succeeds.
string(REPLACE
    "\tsafe_free(priv->usb_interface[iface].endpoint);\n\tpriv->usb_interface[iface].nb_endpoints = 0;\n\n"
    "" source "${source}"
)
string(REPLACE
    "\tif_desc = &conf_desc->interface[iface].altsetting[altsetting];\n\tsafe_free(priv->usb_interface[iface].endpoint);"
    "\tif_desc = &conf_desc->interface[iface].altsetting[altsetting];"
    source "${source}"
)
string(REPLACE
    "\tr = libusb_get_active_config_descriptor(dev_handle->dev, &conf_desc);"
    "\tsafe_free(priv->usb_interface[iface].endpoint);\n\tpriv->usb_interface[iface].nb_endpoints = 0;\n\n\tr = libusb_get_active_config_descriptor(dev_handle->dev, &conf_desc);"
    source "${source}"
)
string(SHA256 source_hash "${source}")
if(NOT source_hash STREQUAL patched_hash)
    message(FATAL_ERROR "Unexpected libusb source; cannot apply the Windows interface claim fix")
endif()
file(WRITE "${source_file}" "${source}")
