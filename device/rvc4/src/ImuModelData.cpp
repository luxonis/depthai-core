#include "ImuModelData.hpp"

namespace dai {

const ImuNoiseParameters kLsm6ModelParams =  // LSM6DSV noise parameters: not stored in EEPROM, but part of the IMU calibration data
    {.name = "LSM6",
     .accelerometer = {.x = {.noiseDensity = 0.0265581027090947f, .randomWalk = 0.000010093945255527f, .biasStability = 0.00908348979687702f},
                       .y = {.noiseDensity = 0.0248946412822367f, .randomWalk = 0.0000102126378842322f, .biasStability = 0.0115752711877888f},
                       .z = {.noiseDensity = 0.0325070710518501f, .randomWalk = 0.000020894079359336f, .biasStability = 0.0154330840172653f}},
     .gyroscope = {.x = {.noiseDensity = 0.16962473704429f, .randomWalk = 0.0415246279899921f, .biasStability = 1.9938014488291f},
                   .y = {.noiseDensity = 0.162065795356986f, .randomWalk = 0.0553796640289008f, .biasStability = 1.94429340491937f},
                   .z = {.noiseDensity = 0.141726916752152f, .randomWalk = 0.0522673667125027f, .biasStability = 2.20512465921074f}}};

const ImuNoiseParameters& getDefaultImuNoiseParams() {  // RVC4 supports LSM6 only
    return kLsm6ModelParams;
}
}
