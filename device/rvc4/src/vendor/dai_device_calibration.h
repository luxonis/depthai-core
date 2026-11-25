/*
 * C interface of the calibration overrides of the vendor module. See dai_device_common.h for the rules.
 *
 * For some devices, the vendor module has a calibration that replaces the calibration in the EEPROM of the device.
 * At start, the firmware uses it in place of the EEPROM calibration.
 */
#ifndef DAI_DEVICE_CALIBRATION_H
#define DAI_DEVICE_CALIBRATION_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The calibration of the device with this serial number, as the JSON text of dai::CalibrationHandler
 * (CalibrationHandler::fromJson). NULL when the device has no override, or when serial_number is NULL.
 * The text is static: do not free it. */
const char* dai_device_calibration_override(const char* serial_number);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_CALIBRATION_H */
