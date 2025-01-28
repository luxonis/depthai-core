#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;



    // Pipeline is defined, now we can connect to the device
    dai::Device d;

    auto imuExtr = d.readCalibration().getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_B, true);

    for(auto& row : imuExtr) {
        for(auto& val : row) {
            cout << val << " ";
        }
        cout << endl;
    }
    return 0;
}
