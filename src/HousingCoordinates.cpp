// depthai
#include "depthai/common/HousingCoordinateSystem.hpp"

namespace dai {

static const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>> HOUSING_COORDINATES = {
    {
        "OAK-4-D-AF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 64.030000000000001f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-D-FF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 64.030000000000001f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-D-W",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 62.039999999999999f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.170000000000002f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.170000000000002f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-PRO-AF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 64.030000000000001f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-PRO-FF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 64.030000000000001f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.829999999999998f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-PRO-W",
        {
            {dai::HousingCoordinateSystem::CAM_A, {37.5f, 11.23f, 62.039999999999999f}},
            {dai::HousingCoordinateSystem::CAM_B, {0.0f, 11.23f, 60.170000000000002f}},
            {dai::HousingCoordinateSystem::CAM_C, {75.0f, 11.23f, 60.170000000000002f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {37.5f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_B, {0.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_C, {75.0f, 11.23f, 66.5f}},
            {dai::HousingCoordinateSystem::IMU, {85.239999999999995f, 9.2599999999999998f, 55.140000000000001f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {75.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {0.0f, 32.740000000000002f, 19.390000000000001f}},
            {dai::HousingCoordinateSystem::VESA_D, {64.340000000000003f, 32.740000000000002f, 25.52f}},
            {dai::HousingCoordinateSystem::VESA_E, {75.0f, 32.740000000000002f, 19.390000000000001f}},
        }
    },
    {
        "OAK-4-S-AF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {25.0f, 10.199999999999999f, 67.620000000000005f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {25.0f, 10.199999999999999f, 69.5f}},
            {dai::HousingCoordinateSystem::IMU, {12.699999999999999f, 30.550000000000001f, 59.93f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {50.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {5.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_D, {25.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_E, {45.0f, 39.299999999999997f, 32.590000000000003f}},
        }
    },
    {
        "OAK-4-S-FF",
        {
            {dai::HousingCoordinateSystem::CAM_A, {25.0f, 10.199999999999999f, 67.620000000000005f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {25.0f, 10.199999999999999f, 69.5f}},
            {dai::HousingCoordinateSystem::IMU, {12.699999999999999f, 30.550000000000001f, 59.93f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {50.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {5.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_D, {25.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_E, {45.0f, 39.299999999999997f, 32.590000000000003f}},
        }
    },
    {
        "OAK-4-S-W",
        {
            {dai::HousingCoordinateSystem::CAM_A, {25.0f, 10.199999999999999f, 65.629999999999995f}},
            {dai::HousingCoordinateSystem::FRONT_CAM_A, {25.0f, 10.199999999999999f, 69.5f}},
            {dai::HousingCoordinateSystem::IMU, {12.699999999999999f, 30.550000000000001f, 59.93f}},
            {dai::HousingCoordinateSystem::VESA_A, {0.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_B, {50.0f, 0.0f, 0.0f}},
            {dai::HousingCoordinateSystem::VESA_C, {5.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_D, {25.0f, 39.299999999999997f, 32.590000000000003f}},
            {dai::HousingCoordinateSystem::VESA_E, {45.0f, 39.299999999999997f, 32.590000000000003f}},
        }
    },
};

const std::unordered_map<std::string, std::unordered_map<dai::HousingCoordinateSystem, std::array<float, 3>>>& getHousingCoordinates() {
    return HOUSING_COORDINATES;
}

} // namespace dai
