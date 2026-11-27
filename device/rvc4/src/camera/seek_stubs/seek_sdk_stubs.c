/* Loader stubs for the Seek SDK (libseekcamera.so.5.2): built three times with the SONAMEs of its DT_NEEDED entries
 * libv4l2.so.0, libv4l2subdev.so.0 and libmediactl.so.0. The firmware replaces every function libseekcamera imports
 * from these libraries right after dlopen() (src/camera/SeekEagle.cpp), so none of these is ever called. */
#include <errno.h>
#include <stddef.h>

int v4l2_open(const char* file, int flags, ...) {
    (void)file;
    (void)flags;
    errno = ENODEV;
    return -1;
}
int v4l2_close(int fd) {
    (void)fd;
    errno = EBADF;
    return -1;
}
int v4l2_ioctl(int fd, unsigned long request, ...) {
    (void)fd;
    (void)request;
    errno = ENOTTY;
    return -1;
}
int v4l2_munmap(void* start, size_t length) {
    (void)start;
    (void)length;
    errno = EINVAL;
    return -1;
}
void* media_device_new(const char* devnode) {
    (void)devnode;
    return NULL;
}
int media_device_enumerate(void* media) {
    (void)media;
    return -ENODEV;
}
unsigned int media_get_entities_count(void* media) {
    (void)media;
    return 0;
}
void* media_get_entity(void* media, unsigned int index) {
    (void)media;
    (void)index;
    return NULL;
}
void* media_get_entity_by_name(void* media, const char* name) {
    (void)media;
    (void)name;
    return NULL;
}
const void* media_entity_get_info(void* entity) {
    (void)entity;
    return NULL;
}
int media_parse_setup_link(void* media, const char* p, char** endp) {
    (void)media;
    if(endp != NULL) *endp = (char*)p;
    return -ENODEV;
}
void media_device_unref(void* media) {
    (void)media;
}
void media_debug_set_handler(void* media, void* handler, void* priv) {
    (void)media;
    (void)handler;
    (void)priv;
}
int v4l2_subdev_parse_setup_formats(void* media, int which, const char* p) {
    (void)media;
    (void)which;
    (void)p;
    return -ENODEV;
}
