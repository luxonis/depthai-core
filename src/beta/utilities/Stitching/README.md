# Camera models in the Stitching node

The Stitching node describes every frame it emits with an `ImgTransformation`, so that the panorama or the bird's-eye
view can be placed in space like any other camera image. The confusing part of that metadata is the
`dai::CameraModel` it carries, because the field is called `distortionModel` while a panorama has no distortion. This
note explains what the value means for the inputs and the output of the node.

## One enum, two meanings

A `dai::CameraModel` names a **projection surface** and a **distortion model** at the same time
(see [`CameraModel.hpp`](../../../../include/depthai/common/CameraModel.hpp)):

| CameraModel       | Projection surface | Distortion model (what the coefficients parametrize)                       |
|-------------------|--------------------|-----------------------------------------------------------------------------|
| `Perspective`     | pinhole plane      | Brown-Conrady, OpenCV layout (k1, k2, p1, p2, k3, k4, k5, k6, s1-s4, taux, tauy) |
| `Fisheye`         | pinhole plane      | Kannala-Brandt, OpenCV fisheye layout (k1, k2, k3, k4)                       |
| `RadialDivision`  | pinhole plane      | radial division (not evaluated by depthai-core yet)                          |
| `Equirectangular` | sphere             | none                                                                        |
| `Cylindrical`     | cylinder           | none                                                                        |

- The projection surface defines how a direction `(x, y, z)` in the camera frame maps to normalized coordinates and
  therefore what `fx, fy, cx, cy` mean: focal length and principal point on a pinhole plane, radius of the sphere or
  cylinder in pixels and the pixel the Z axis projects to on a panorama surface.
- The distortion model defines the formula the distortion coefficients feed. Only the pinhole plane carries distortion
  in depthai. Empty or all-zero coefficients make any pinhole model an undistorted pinhole camera.

There is intentionally no separate projection enum and distortion enum. Every supported combination has one name, and
the one place where the combination is visible is `ImgTransformation::getDistortionModel()`, whose name predates the
panorama surfaces. Read it as "camera model", not as "lens distortion".

## What the node accepts

`setCameraModel()` (only in `Mode::PANORAMA`) chooses the surface the inputs are warped onto, which is also the camera
model of the output:

| `setCameraModel()`  | OpenCV warper          | Output                                     |
|---------------------|------------------------|--------------------------------------------|
| `Equirectangular`   | `cv::SphericalWarper`  | sphere, the default, same as OpenCV         |
| `Cylindrical`       | `cv::CylindricalWarper`| cylinder                                   |
| `Perspective`       | `cv::PlaneWarper`      | pinhole plane, no distortion coefficients  |
| `Fisheye`, `RadialDivision` | rejected       | they are distortions of the pinhole plane, there is no surface to render onto |

The mapping lives in `createWarper()` in [`StitchingCompositing.hpp`](StitchingCompositing.hpp).

The **inputs** are ordinary camera frames and may use any pinhole model. Visual registration (`cv::Stitcher`) ignores
their intrinsics and distortion altogether. Calibrated composition (`setUseInputCalibration(true)`) reads the input
intrinsics and rotations and requires the inputs to be undistorted, since the warper only models the surface; request
undistorted camera outputs for that.

## What the node emits

| Mode                      | Output `CameraModel`                          | Coefficients | Intrinsics                                              |
|---------------------------|-----------------------------------------------|--------------|---------------------------------------------------------|
| `PANORAMA`                | the value of `setCameraModel()`               | none         | `fx = fy =` warper scale (radius in pixels), `(cx, cy)` where the panorama Z axis lands |
| `PLANAR_PROJECTION`       | `Perspective`                                 | none         | the virtual pinhole camera of `setView()`               |

`panoramaTransformation()` in [`Stitching.cpp`](Stitching.cpp) builds the panorama metadata from the OpenCV warper
canvas; the only subtlety is the spherical warper measuring its polar angle from the -Y axis while the equirectangular
latitude is measured from the XZ plane, which shifts `cy` by `scale * pi / 2`.

So for a panorama, `getDistortionModel()` returns `Equirectangular` or `Cylindrical` and `getDistortionCoefficients()`
is empty. That is correct and intended: the value tells the consumer which surface to wrap the image onto, and the
empty coefficients tell it that no lens model is involved.
