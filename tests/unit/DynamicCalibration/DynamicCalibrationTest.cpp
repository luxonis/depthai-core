#include <gtest/gtest.h>

#include "depthai/pipeline/node/DynamicCalibration.hpp"

class MockDynamicCalibration : public dai::node::DynamicCalibration {
   public:
    Properties& getPropertiesPublic() {
        return getProperties();
    }
};

// Test case for the add function
TEST(DynamicCalibration, SetAndGetRunOnHost) {
    dai::node::DynamicCalibration dynCalib;
    dynCalib.setRunOnHost(true);
    EXPECT_TRUE(dynCalib.runOnHost());

    dynCalib.setRunOnHost(false);
    EXPECT_FALSE(dynCalib.runOnHost());
}

TEST(DynamicCalibration, SetPerformanceMode) {
    MockDynamicCalibration dynCalib;
    dynCalib.setPerformanceMode(dai::DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS);
    EXPECT_EQ(dynCalib.getPropertiesPublic().performanceMode, dai::DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS);
}

TEST(DynamicCalibration, SetContinousMode) {
    MockDynamicCalibration dynCalib;
    dynCalib.setContinousMode();
    EXPECT_EQ(dynCalib.getPropertiesPublic().recalibrationMode, dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS);
}

TEST(DynamicCalibration, SetTimeFrequency) {
    MockDynamicCalibration dynCalib;
    dynCalib.setTimeFrequency(42);
    EXPECT_EQ(dynCalib.getPropertiesPublic().timeFrequency, 42);
}
