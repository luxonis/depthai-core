#include "depthai/common/BsplineCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dai {
namespace {

void require(bool condition, const char* message) {
    if(!condition) throw std::invalid_argument(message);
}

std::array<double, 4> cubicBasis(double t) {
    const double t2 = t * t, t3 = t2 * t;
    return {{(1 - 3 * t + 3 * t2 - t3) / 6, (4 - 6 * t2 + 3 * t3) / 6, (1 + 3 * t + 3 * t2 - 3 * t3) / 6, t3 / 6}};
}

std::array<double, 4> cubicBasisDerivative(double t) {
    const double t2 = t * t;
    return {{(-3 + 6 * t - 3 * t2) / 6, (-12 * t + 9 * t2) / 6, (3 + 6 * t - 9 * t2) / 6, 3 * t2 / 6}};
}

// Pixel of the stereographic point u, and its 2x2 Jacobian d(pixel)/d(u) if jacobian is set
std::array<double, 2> pixelOf(const BsplineCameraCalibration& c, double ux, double uy, std::array<double, 4>* jacobian) {
    constexpr double pi = 3.14159265358979323846;  // M_PI is not standard (MSVC)
    const double density = (c.nx - 3) / (4 * std::tan(c.fovXDegrees * pi / 720));
    const double ix = ux * density + (c.nx - 1) / 2.0, iy = uy * density + (c.ny - 1) / 2.0;
    require(std::isfinite(ix) && std::isfinite(iy), "Nonfinite spline grid coordinate");
    // Clamp as double first, so that very large rays cannot overflow int
    const int x0 = static_cast<int>(std::clamp(ix, 1.0, double(c.nx - 3)));
    const int y0 = static_cast<int>(std::clamp(iy, 1.0, double(c.ny - 3)));
    const auto bx = cubicBasis(ix - x0), by = cubicBasis(iy - y0);
    const auto dbx = cubicBasisDerivative(ix - x0), dby = cubicBasisDerivative(iy - y0);
    std::array<double, 2> delta{{0, 0}};
    std::array<double, 4> dDelta{{0, 0, 0, 0}};  // d(delta_axis)/d(ux), d(delta_axis)/d(uy) for axis x, y
    for(int y = 0; y < 4; ++y) {
        for(int x = 0; x < 4; ++x) {
            for(int axis = 0; axis < 2; ++axis) {
                const double control = c.intrinsics[4 + 2 * ((y0 - 1 + y) * c.nx + x0 - 1 + x) + axis];
                delta[axis] += by[y] * bx[x] * control;
                dDelta[2 * axis] += by[y] * dbx[x] * control * density;
                dDelta[2 * axis + 1] += dby[y] * bx[x] * control * density;
            }
        }
    }
    const double fx = c.intrinsics[0], fy = c.intrinsics[1];
    if(jacobian) *jacobian = {{fx * (1 + dDelta[0]), fx * dDelta[1], fy * dDelta[2], fy * (1 + dDelta[3])}};
    return {{fx * (ux + delta[0]) + c.intrinsics[2], fy * (uy + delta[1]) + c.intrinsics[3]}};
}

}  // namespace

void BsplineCameraCalibration::validate() const {
    require(width > 0 && height > 0 && nx >= 4 && ny >= 4 && nx <= 128 && ny <= 128 && std::isfinite(fovXDegrees) && fovXDegrees > 0 && fovXDegrees < 180
                && intrinsics.size() == 4 + 2 * static_cast<size_t>(nx) * ny,
            "Invalid cubic splined-stereographic resolution/dimensions/FOV/parameter count");
    require(std::all_of(intrinsics.begin(), intrinsics.end(), [](double v) { return std::isfinite(v); }), "Nonfinite spline intrinsics");
    require(intrinsics[0] > 0 && intrinsics[1] > 0, "Nonpositive spline focal length");
}

std::array<double, 2> BsplineCameraCalibration::project(const std::array<double, 3>& ray) const {
    const double denominator = std::hypot(std::hypot(ray[0], ray[1]), ray[2]) + ray[2];
    require(std::isfinite(denominator) && denominator > 0, "Invalid spline projection ray");
    return pixelOf(*this, 2 * ray[0] / denominator, 2 * ray[1] / denominator, nullptr);
}

std::array<double, 3> BsplineCameraCalibration::unproject(const std::array<double, 2>& pixel) const {
    // Newton on pixelOf(u) = pixel from the uncorrected stereographic point. The step is halved while it does not
    // reduce the error, which keeps it converging where the spline extrapolates (image corners of wide lenses).
    double ux = (pixel[0] - intrinsics[2]) / intrinsics[0], uy = (pixel[1] - intrinsics[3]) / intrinsics[1];
    std::array<double, 4> j;
    auto p = pixelOf(*this, ux, uy, &j);
    double ex = p[0] - pixel[0], ey = p[1] - pixel[1], error = ex * ex + ey * ey;
    for(int iteration = 0; iteration < 50 && error >= 1e-12; ++iteration) {
        const double det = j[0] * j[3] - j[1] * j[2];
        if(!(std::abs(det) > 0)) break;
        const double dx = (j[3] * ex - j[1] * ey) / det, dy = (-j[2] * ex + j[0] * ey) / det;
        double step = 1;
        for(int halving = 0; halving < 30; ++halving, step /= 2) {
            std::array<double, 4> jNext;
            const auto next = pixelOf(*this, ux - step * dx, uy - step * dy, &jNext);
            const double rx = next[0] - pixel[0], ry = next[1] - pixel[1];
            if(rx * rx + ry * ry < error) {
                ux -= step * dx;
                uy -= step * dy;
                ex = rx;
                ey = ry;
                error = rx * rx + ry * ry;
                j = jNext;
                break;
            }
        }
    }
    if(error >= 1e-12) throw std::runtime_error("B-spline unprojection did not converge");
    // Inverse stereographic projection to a unit ray
    const double s = (ux * ux + uy * uy) / 4;
    return {{ux / (1 + s), uy / (1 + s), (1 - s) / (1 + s)}};
}

}  // namespace dai
