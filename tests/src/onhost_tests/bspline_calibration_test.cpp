// dai::BsplineCameraCalibration against mrcal 2.5.2: pixels from mrcal.project for the same lens. The control points
// come from a formula so the test can rebuild them; rays include points far outside the grid (extrapolation).
#include <catch2/catch_all.hpp>
#include <cmath>

#include "depthai/common/BsplineCalibration.hpp"

TEST_CASE("B-spline lens model matches mrcal projection and inverts it") {
    dai::BsplineCameraCalibration lens;
    lens.width = 1280;
    lens.height = 800;
    lens.nx = 20;
    lens.ny = 12;
    lens.fovXDegrees = 106;
    lens.intrinsics = {800., 805., 640., 400.};
    for(int i = 0; i < 2 * lens.nx * lens.ny; ++i) lens.intrinsics.push_back(0.02 * std::sin(1.7 * i + 0.3));
    lens.validate();

    // mrcal.project(rays, "LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=20_Ny=12_fov_x_deg=106", intrinsics)
    const std::vector<std::pair<std::array<double, 3>, std::array<double, 2>>> golden = {
        {{0.0, 0.0, 1.0}, {638.6443487017414, 399.8500920736482}},
        {{0.3, -0.2, 1.0}, {869.2423530397881, 247.2188528465151}},
        {{-0.7, 0.45, 1.0}, {151.89535377328053, 712.7533453503304}},
        {{1.4, 0.9, 1.0}, {1401.1615253960204, 894.4574787020676}},
        {{-2.5, -1.6, 1.0}, {24.097546795522817, -737.5988433571092}},
        {{0.05, 1.2, 1.0}, {713.9428224385545, 1179.687254396894}},
        {{3.0, 2.0, 1.0}, {1368.2098517067698, 3263.33458940758}},
        {{-0.9, -0.6, 0.4}, {-110.41869264592958, -462.711155560644}},
    };
    for(const auto& [ray, pixel] : golden) {
        CAPTURE(ray[0], ray[1], ray[2]);
        const auto projected = lens.project(ray);
        CHECK(projected[0] == Catch::Approx(pixel[0]).margin(1e-8));
        CHECK(projected[1] == Catch::Approx(pixel[1]).margin(1e-8));
    }
    // Inside the image, unproject returns the ray that projects back to the pixel
    for(const auto& [ray, pixel] : golden) {
        if(pixel[0] < 0 || pixel[0] >= lens.width || pixel[1] < 0 || pixel[1] >= lens.height) continue;
        const auto back = lens.project(lens.unproject(pixel));
        CHECK(back[0] == Catch::Approx(pixel[0]).margin(1e-5));
        CHECK(back[1] == Catch::Approx(pixel[1]).margin(1e-5));
    }
}
