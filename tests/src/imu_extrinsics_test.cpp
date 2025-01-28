#include <cstdio>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

struct ImuEntry {
    std::string boardName;
    std::string revision;
    std::string imuName;
    // 3 rows × 4 columns: each row is {r0, r1, r2, t}
    std::array<std::array<float, 4>, 3> extrinsics3x4;
};

// A helper function to make a 3×4 from a 3×3 rotation + translation.
constexpr std::array<std::array<float, 4>, 3> make3x4(const std::array<std::array<float, 3>, 3>& rot, const std::array<float, 3>& trans) {
    return {{{{rot[0][0], rot[0][1], rot[0][2], trans[0]}}, {{rot[1][0], rot[1][1], rot[1][2], trans[1]}}, {{rot[2][0], rot[2][1], rot[2][2], trans[2]}}}};
}

int main() {
    std::vector<ImuEntry> imuEntries;
    //-----------------------------------------------------------------------------
    // OAK-D S2/Pro (R6 - generic)
    //   boardName="DM9098", revision="R6M2E6"
    //   rotation=[ [0,1,0], [1,0,0], [0,0,-1] ]
    //   BNO086 => trans={7.5448, -0.2, -0.0448}
    //   BMI270 => trans={6.935,  -0.2, -0.565 }

    {
        std::string board = "DM9098";
        std::string rev = "R6M2E6";

        // Common rotation
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D S2/Pro (R3 - generic)
    //   boardName="DM9098", revision="R3M2E3"
    //   same rotation, same translations
    {
        std::string board = "DM9098";
        std::string rev = "R3M2E3";

        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D S2 (EEPROMv6)
    //   boardName="OAK-D-S2", revision="R2M0E2"
    {
        std::string board = "OAK-D-S2";
        std::string rev = "R2M0E2";

        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086 => trans={7.5448, -0.2, -0.0448}
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270 => trans={6.935, -0.2, -0.565}
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D W (EEPROMv6)
    //   boardName="OAK-D-W", revision="R3M2E2"
    {
        std::string board = "OAK-D-W";
        std::string rev = "R3M2E2";

        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086 => trans={7.5448, -0.2, -0.0448}
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270 => trans={6.935, -0.2, -0.565}
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D Pro (EEPROMv6)  <-- This is the one you said was missing
    //   boardName="OAK-D-PRO", revision="R3M1E3"
    //   rotation = [ [0,1,0], [1,0,0], [0,0,-1] ]
    //   BNO086 => trans={7.5448, -0.2, -0.0448}
    //   BMI270 => trans={6.935, -0.2, -0.565}

    {
        std::string board = "OAK-D-PRO";
        std::string rev = "R3M1E3";

        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D Pro W (EEPROMv6)
    //   boardName="OAK-D Pro-W", revision="R3M2E2"
    //   same rotation, same pattern
    {
        std::string board = "OAK-D Pro-W";
        std::string rev = "R3M2E2";

        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};

        // BNO086 => trans={7.5448, -0.2, -0.0448}
        {
            std::array<float, 3> trans = {7.5448f, -0.2f, -0.0448f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270 => trans={6.935, -0.2, -0.565}
        {
            std::array<float, 3> trans = {6.935f, -0.2f, -0.565f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D-LITE (DeviceEntry)
    //   board="OAK-D-LITE", revision=""
    //   BNO086 => rot=[[0,1,0],[1,0,0],[0,0,-1]], trans={0.0541, 0.2, -0.368}
    //   BMI270 => rot=[[1,0,0],[0,0,-1],[0,1,0]], trans={0.6, -0.2, -0.6439}

    {
        std::string board = "OAK-D-LITE";
        std::string rev = "";  // Not specified

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rotBno = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> transBno = {0.0541f, 0.2f, -0.368f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rotBno, transBno)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rotBmi = {{{{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}, {{0.f, 1.f, 0.f}}}};
            std::array<float, 3> transBmi = {0.6f, -0.2f, -0.6439f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rotBmi, transBmi)});
        }
    }

    //-----------------------------------------------------------------------------
    // OAK-D Pro PoE boards (R3, R4, R5, R7...) etc.
    //   boardName="NG9097", revision= ...
    //   Each time, if it has BNO086, we add an entry; if it has BMI270, we add another entry.
    //   Already in the snippet "oak_d_pro_poe_r3", "oak_d_pro_poe_r4", "oak_d_pro_poe_r5", "oak_d_pro_poe_r7", etc.
    //   We'll do them quickly here.

    {
        // R3M2E2 => BNO086 => rot={{0,-1,0},{-1,0,0},{0,0,-1}}, trans={7.75f,-0.2f,2.0264f}
        //          BMI270 => rot={{0,1,0}, {1,0,0}, {0,0,-1}},   trans={8.5475f,-0.2f,1.6464f}
        std::string board = "NG9097";
        std::string rev = "R3M2E2";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    {
        // R4M2E4 => same pattern
        std::string board = "NG9097";
        std::string rev = "R4M2E4";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    {
        // R5M3E5 => same pattern
        std::string board = "NG9097";
        std::string rev = "R5M3E5";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    {
        // R7M4E6 => same pattern
        std::string board = "NG9097";
        std::string rev = "R7M4E6";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "OAK-D-LITE";
        std::string rev = "";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rotBno = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> transBno = {0.0541f, 0.2f, -0.368f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rotBno, transBno)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rotBmi = {{{{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}, {{0.f, 1.f, 0.f}}}};
            std::array<float, 3> transBmi = {0.6f, -0.2f, -0.6439f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rotBmi, transBmi)});
        }
    }

    {
        std::string board = "NG9097";
        std::string rev = "R3M2E2";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "NG9097";
        std::string rev = "R4M2E4";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "NG9097";
        std::string rev = "R5M3E5";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "NG9097";
        std::string rev = "R7M4E6";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "BC2087";
        std::string rev = "R0M0E0";

        std::array<std::array<float, 3>, 3> rot = {{{{1.f, 0.f, 0.f}}, {{0.f, -1.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        // BNO086
        {
            std::array<float, 3> trans = {12.1465f, 0.f, -1.2616f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<float, 3> trans = {9.2731f, 0.2f, -0.7456f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "BC2087";
        std::string rev = "R1M1E1";

        std::array<std::array<float, 3>, 3> rot = {{{{1.f, 0.f, 0.f}}, {{0.f, -1.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        // BNO086
        {
            std::array<float, 3> trans = {12.1465f, 0.f, -1.2616f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<float, 3> trans = {9.2731f, 0.2f, -0.7456f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }

    {
        // Common rotation
        std::array<std::array<float, 3>, 3> rot = {{{{1.f, 0.f, 0.f}}, {{0.f, -1.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        // BNO086 => trans={12.1465, 0.0, -1.2616}
        // BMI270 => trans={9.2731, 0.2, -0.7456}

        // R2M1E2
        {
            std::string board = "BC2087";
            std::string rev = "R2M1E2";

            // BNO086
            {
                std::array<float, 3> trans = {12.1465f, 0.f, -1.2616f};
                imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
            }
            // BMI270
            {
                std::array<float, 3> trans = {9.2731f, 0.2f, -0.7456f};
                imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
            }
        }

        // R3M1E3
        {
            std::string board = "BC2087";
            std::string rev = "R3M1E3";

            // BNO086
            {
                std::array<float, 3> trans = {12.1465f, 0.f, -1.2616f};
                imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
            }
            // BMI270
            {
                std::array<float, 3> trans = {9.2731f, 0.2f, -0.7456f};
                imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
            }
        }
    }

    {
        std::string board = "NG9097";
        std::string rev = "R3M2E2";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        std::string board = "NG9097";
        std::string rev = "R4M2E3";

        // BNO086
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, -1.f, 0.f}}, {{-1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {7.75f, -0.2f, 2.0264f};
            imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
        }
        // BMI270
        {
            std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
            std::array<float, 3> trans = {8.5475f, -0.2f, 1.6464f};
            imuEntries.push_back({board, rev, "BMI270", make3x4(rot, trans)});
        }
    }
    {
        auto addSr = [&](std::string rev) {
            // BNO086
            {
                std::array<std::array<float, 3>, 3> rot = {{{{1.f, 0.f, 0.f}}, {{0.f, -1.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
                std::array<float, 3> trans = {-0.1731f, 0.f, 1.0175f};
                imuEntries.push_back({"DM2080", rev, "BNO086", make3x4(rot, trans)});
            }
            // BMI270
            {
                std::array<std::array<float, 3>, 3> rot = {{{{1.f, 0.f, 0.f}}, {{0.f, -1.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
                std::array<float, 3> trans = {-0.24f, 0.f, 1.0f};
                imuEntries.push_back({"DM2080", rev, "BMI270", make3x4(rot, trans)});
            }
        };

        addSr("R0M0E0");
        addSr("R1M1E1");
        addSr("R3M2E3");
        addSr("R4M2E4");
    }
    {
        std::string board = "EL2086";
        std::string rev = "R0M0E0";
        // rotation={{0,1,0},{1,0,0},{0,0,-1}}, trans={0.2915, -0.069, 0.2832}
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }
    {
        // We'll do: board="EL2086", rev="R0M0E0-nIR-C81M00-00", BNO086
        std::string board = "EL2086";
        std::string rev = "R0M0E0-nIR-C81M00-00";
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }

    {
        std::string board = "EL2086";
        std::string rev = "R1M1E1";
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }

    {
        std::string board = "EL2086";
        std::string rev = "R1M1E1-nIR-C81M00-00";
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }

    {
        std::string board = "EL2086";
        std::string rev = "R3M2E3";
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }

    {
        std::string board = "EL2086";
        std::string rev = "R3M2E3-mono";
        std::array<std::array<float, 3>, 3> rot = {{{{0.f, 1.f, 0.f}}, {{1.f, 0.f, 0.f}}, {{0.f, 0.f, -1.f}}}};
        std::array<float, 3> trans = {0.2915f, -0.069f, 0.2832f};
        imuEntries.push_back({board, rev, "BNO086", make3x4(rot, trans)});
    }
    // Pipeline is defined, now we can connect to the device
    dai::Device d;

    auto imuExtr = d.readCalibration().getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_B, true);
    auto imuType = d.getConnectedIMU();

    auto boardName = d.readCalibration().getEepromData().boardName;
    auto revision = d.readCalibration().getEepromData().boardRev;
    std::cout << "IMU type: " << imuType << " board name: " << boardName << " revision: " << revision << std::endl;
    std::cout << "Calib extr" << std::endl;
    for(auto& e : imuEntries) {
        if(e.imuName == imuType && e.boardName == boardName && e.revision == revision) {
            std::cout << "Found calibration for " << imuType << " on " << boardName << " rev " << revision << std::endl;
            for(auto& row : e.extrinsics3x4) {
                for(auto& val : row) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
            }
        }
    }
    for(auto& row : imuExtr) {
        for(auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
