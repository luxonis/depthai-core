# Projectors

Allow to observe the intensity of the DOT and FLOOD projectors on OAK cameras:

Supported devices:
- OAK 4 D Pro
- OAK-D Pro W PoE
- OAK-D Pro W
- OAK-D Pro PoE
- OAK-D Pro

Certain RVC2 EL2086 board variants, including OAK-ToF models using the Samsung S5K63D sensor, limit combined dot/flood power to prevent power crashes.
To bypass that budget, set `dangerousIrPowerOverride` before constructing the device, then use the normal intensity setters:

```cpp
dai::Device::Config config;
config.dangerousIrPowerOverride = true;  // DANGEROUS: full power can crash the device
auto device = std::make_shared<dai::Device>(config);
device->setIrLaserDotProjectorIntensity(1.0f);
device->setIrFloodLightIntensity(1.0f);
```

The flag defaults to `false` and requires updated RVC2 firmware; opting in with unsupported firmware fails device initialization.
Individual emitter current and duty-cycle limits still apply. The override lasts until firmware reboot, including across calibration reads.
It applies to the auxiliary dot/flood emitters; it does not change the ToF sensor's illumination power.

For hardware validation, run `logging_test '[.dangerous]'` on a 63D with updated firmware.
This explicit test checks cap warnings and calculated emitter currents in both modes using 30% dot and flood requests, exceeding the normal 45% shared budget.
