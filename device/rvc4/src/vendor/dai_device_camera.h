/*
 * C interface of the camera HAL names of the vendor module. See dai_device_common.h for the rules.
 *
 * The camera HAL of the device OS has vendor tags: metadata entries that the firmware reads and writes (for example
 * the manual white balance). The public code uses the Luxonis names below; the vendor module gives the name of each
 * tag in the HAL ("<section>.<name>"), and the public code looks up the tag id with that name at run time.
 * New tags get new numbers at the end; a number never changes its meaning.
 */
#ifndef DAI_DEVICE_CAMERA_H
#define DAI_DEVICE_CAMERA_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Static characteristics */
#define DAI_CAMERA_TAG_SENSOR_NAME 0 /* name of the sensor of a camera */

/* Session parameters */
#define DAI_CAMERA_TAG_OFFLINE_FRONT_END 1         /* process the frames offline in the image front end */
#define DAI_CAMERA_TAG_REALTIME_BAYER_PROCESSING 2 /* run the camera on the Bayer processing stage */

/* Capture settings */
#define DAI_CAMERA_TAG_OVERRIDE_SENSOR_FRAME_DURATION 3
#define DAI_CAMERA_TAG_DISABLE_FPS_LIMITS 4
#define DAI_CAMERA_TAG_SHARPNESS 5
#define DAI_CAMERA_TAG_ISO_EXPOSURE_PRIORITY_SELECT 6 /* the priority of the ISO / exposure time control */
#define DAI_CAMERA_TAG_ISO_VALUE 7
#define DAI_CAMERA_TAG_ISO_EXPOSURE_PRIORITY 8
#define DAI_CAMERA_TAG_EXPOSURE_METERING_MODE 9
#define DAI_CAMERA_TAG_MANUAL_WB_MODE 10
#define DAI_CAMERA_TAG_MANUAL_WB_COLOR_TEMPERATURE 11
#define DAI_CAMERA_TAG_CONTRAST 12
#define DAI_CAMERA_TAG_SATURATION 13
#define DAI_CAMERA_TAG_EXPOSURE_TIME_LIMIT 14 /* the limit of the exposure time of the auto exposure */
#define DAI_CAMERA_TAG_ISO_LIMIT 15           /* the ISO range of the auto exposure */

/* Capture results */
#define DAI_CAMERA_TAG_AWB_COLOR_TEMPERATURE 16 /* the color temperature that the auto white balance selected */
#define DAI_CAMERA_TAG_SENSOR_READOUT 17        /* the sensor readout data of a frame */

/* Number of tags of this version of the interface */
#define DAI_CAMERA_TAG_COUNT 18

/* The name of a tag in the camera HAL, "<section>.<name>"; NULL when the vendor module does not know the tag */
const char* dai_device_camera_tag_name(uint32_t tag);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_CAMERA_H */
