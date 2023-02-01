^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthai
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.20.2 (2023-01-31)
-----------
* Fix for ColorCamera at high resolution while using isp scaling
* Fix for OV9282 SW sync on devices with OV9782 RGB camera
* Fix for IMX378/477/577 on sockets other than CAM_A (RGB)
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin

2.20.1 (2023-01-29)
-----------
* Modified OpenVINO::VERSION_UNIVERSAL API improvements / backward compatibility
* Bootloader version 0.0.24 (fixes for standalone / flashed usecases)
* [FW] Status LEDs on some additional devices
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin

2.19.1 (2022-11-28)
-----------
* Added Device getDeviceName API
* OAK-FFC 4P (R5M1E5) IR/Dot support
* Additional Stability bugfixes to go along with 2.19.0 for PoE devices
* Protected productName field in EEPROM
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin


2.19.0 (2022-09-20)
-----------
* Stability improvements #616
* isUserBootloaderSupported API
* Device.setTimesync(true/false) convenience function to enable or disable subsequent timesyncing
* Windows improvements with listing BOOTED devices ("udev permissions" issue)
* Fix OV9282 as MonoCamera on RGB socket (issue was black image)
* Fix crash under high load (regression with camera events streaming)
* Fix YOLOv5/7 decoding in case of a single class
* Fix image size when decimation filter is enabled
* Fix for certain OV9782 and OV9282 permutations/configs
* Reset Device timestamp on boot to zero
* Reworded "No available devices" error message when there are other connected devices connected.
* Update CI to Node16 compatible actions


2.17.4 (2022-09-20)
-----------
* DEPTHAI_WATCHDOG=0 bugfix (245fb57)
* V5 Calibration flashing fix
* FW log level bugfix (#587)
* Updated DeviceBootloader::Config to retain existing values
* PoE watchdog issues addressed (74b699c)
* XLink - kernel driver detach (fixes some USB connectivity issues) (ba9bd8b)
* Added EEPROM clear capability
* Added missing installation of DLL files (#550)
* Asset RPC refactor
* Exposed Device::getAllConnectedDevices()
* Exposed FW & BL versions
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin


2.17.3 (2022-08-05)
-----------
* Updated FW - BMI270 IMU improvements
* Added seq & timestamps for more messages
* New boards support
* Windows DLL improvements (install libusb dll alongside libdepthai-core.dll)
* XLink - improved connecting directly to given IP
* StereoDepth ImgFrame metadata w/h when decimation filter is enabled
* Intrinsic read fix #379
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin, Sachin Guruswamy


2.17.0 (2022-07-11)
-----------
* Support for new S2/Pro devices
* FW: support for OAK-D-S2 / OAK-D-Pro using the latest board DM9098 R6M2E6
* Handle new resolutions THE_720_P and THE_800_P for ColorCamera, applicable to OV9782 on RGB/center socket
* StereoDepth: Add option for disparity shift to reduce minimum depth
* StereoDepth: extended and subpixel mode can be enabled simultaneously
* YoloV6 support
* Refactor ImageManip node
* macOS / Linux shared library and CI improvements
* Bootloader improvements
* Flash boot improvements
* Bootloader improvements (capability to flash and boot from eMMC)
* Flashed application information
* Memory querying
* XLink device search race fix
* Capability to flash BoardConfig along with the Pipeline
* Added host monitor thread to disconnect offline PoE devices
* Contributors: Alex Bougdan, Szabolcs Gergely, Martin Peterlin, Sachin Guruswamy


2.16.0 (2022-06-15)
-----------
* OpenVINO 2022.1 support
* XLink device search improvements
* cross subnets for PoE devices
* drastically reduced discovery time
* Separated name / IP and MXID, to be able to query both USB path / IP and MXID
* Android support
* libusb dependency is now managed by Hunter
* IMU FW update for BNO sensor
* Added DetectionParser node as a standalone component of DetectionNetwork
* StereoDepth - subpixel fractional bits API
* VideoEncoder - lifted 16 frame pool limitation
* Contributors: Sachin, Sachin Guruswamy

2.15.5 (2022-06-02)
-----------
* EEPROM FIX
* Json fix (`#478 <https://github.com/luxonis/depthai-core/issues/478>`_)
  * Fixed nlohmann json < v3.9.0 compat and toolchain generation
  * turn off clang format
  Co-authored-by: Martin Peterlin <martin.peterlin7@gmail.com>
  Co-authored-by: TheMarpe <martin@luxonis.com>
* Empty-Commit
* Update package.xml
* Contributors: Sachin, Sachin Guruswamy

2.15.4 (2022-05-09)
-------------------
* Release 2.15.4
* Update docs; removed unsupported AprilTag families
* FW: VideoEncoder: fix keyframe rate config, fix resource computations for JPEG
  (e.g: MJPEG for 4K video 30fps + MJPEG for still 12MP ~1fps)
  properly set resources used to allow
* Update FW
* Update FW; change behavior of stereo rectification based on stereo camera FOV
* Merge 'origin/poe_mtu_sysctl' into develop - `#428 <https://github.com/luxonis/depthai-core/issues/428>`_
  Improve PoE throughput and latency for some usecases
* Update XLink to set TCP_NODELAY, reducing latency
* Merge 'origin/develop' into poe_mtu_sysctl
* Merge branch 'eeprom_version_v7' into develop
* Merge branch 'develop' into eeprom_version_v7
* Merge branch 'json_compat' into develop
* Lowered minimum required nlohmann json version to 3.6.0
* Set RGB aligned depth output to match mono camera
* Merge 'ov7251_configurable_fps' into develop - `#455 <https://github.com/luxonis/depthai-core/issues/455>`_
* Update FW: fix overriding useHomographyRectification behaviour specified in docs when custom mesh is provided
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#459 <https://github.com/luxonis/depthai-core/issues/459>`_ from diablodale/fix458-cmaketest-flags
  reduce num conforming tests; add missing _conforming test suffix
* reduce tests for MSVC conforming preprocessor
  - drastically reduce number of tests run for
  MSVC conforming preprocessor
  https://github.com/luxonis/depthai-core/pull/459#issuecomment-1108649206
  - add option to test harness that indicates
  when a test is run with the MSVC conforming preprocessor
* Updated flashing permissions
* Fix RGB alignment remapping when configured color camera resolution is different from calibration one
* Updated Bootloader to v0.0.18
* Updated FW with device EEPROM handling fixes
* strengthen test for device construct+q+frame
* Updated bootloader with PCIe internal clock fixes
* Added capability to create CalibrationHandler from json
* Fixed factory reset functionality and exposed more functions
* Updated BL with more build information and new EEPROM data support
* Updated EEPROM and added another level of permissions
* add missing _conforming suffix to tests cmake
  - fixes `luxonis/depthai-core#458 <https://github.com/luxonis/depthai-core/issues/458>`_
* Merge pull request `#457 <https://github.com/luxonis/depthai-core/issues/457>`_ from luxonis/rgb_alignment
  Enable RGB alignment for spatial detection examples
* Enable RGB alignment for spatial detection examples
* Merge pull request `#454 <https://github.com/luxonis/depthai-core/issues/454>`_ from diablodale/test-device-queues1
  test case for Device constructor not calling tryStartPipeline()
* test case for Device constructor not tryStartPipeline()
  - catch bug and prevent regression as discussed
  https://github.com/luxonis/depthai-core/commit/7257b95ecfb8dcb77c075e196ac774cc05cb8bc6#commitcomment-71730879
* Merge remote-tracking branch 'origin/main' into HEAD
* Update FW: configurable FPS for OV7251: max 99 for 480p, 117 for 400p
* Added bindings and support for new EEPROM version
* WIP - modify behavior to be backwards compatible and add checks if calibration is available
* Added additional EEPROM functionality
* Applied formatting
* Merge branch 'main' into develop
* Update FW: improve PoE throughput and latency (set net.inet.tcp.delayed_ack=0),
  add config for MTU (not advised to change for now) and other sysctl params
* Contributors: Dale Phurrough, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.15.3 (2022-04-22)
-------------------
* Add explicit documentation about loadMesh behavior; specify that only the first 8 distortion coefficients are used
* Merge pull request `#456 <https://github.com/luxonis/depthai-core/issues/456>`_ from luxonis/macos_ci_test
  Fix failing CI for MacOS
* Extend useHomographyRectification documentation with more details
* Remove brew update
* Bump version to 2.15.3
* Merge branch 'release_2.15.3' into main
* Clarify docs for homography rectification default behavior
* Merge pull request `#437 <https://github.com/luxonis/depthai-core/issues/437>`_ from luxonis/warp_mesh_on_device
  Add on-device mesh generator for Stereo
* Disable mesh rectification by default; fix error reporting when RGB alignment is enabled and left-right check disabled
* Fix styling
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'serialization_type' into develop
* Fixed incorrect Device constructors not starting the pipeline and creating queues
* Fixed device Clock.now in Script node to match messages timestamps
* Modifed serializeToJson to create a json object instead
* Added Clock.now bindings on device
* Added capability to serialize pipeline to json
* Merge pull request `#424 <https://github.com/luxonis/depthai-core/issues/424>`_ from luxonis/bmi270_support
  IMU: Bmi270 support
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#449 <https://github.com/luxonis/depthai-core/issues/449>`_ from luxonis/openvino_no_blob
  Openvino: Fix error reporting when blob is not set
* Removed DEPTHAI_NODISCARD for docs generation
* Updated libnop  (`#448 <https://github.com/luxonis/depthai-core/issues/448>`_)
  * Updated libnop with C++20 fixes and added fs test targeting C++20
  * Added a guard for non-existent tests
  * Modified tests to not require higher CMake version
* Fix openvino get version
* Openvino: Fix error reporting when blob is not set
* Removed deprecated StereoDepth API
* new class `dai::Path` for APIs that accept path/filenames (`#384 <https://github.com/luxonis/depthai-core/issues/384>`_)
  * initial dai::Path and test cases
  - fixes `luxonis/depthai-core#352 <https://github.com/luxonis/depthai-core/issues/352>`_
  * move codecvt from header -> cpp
  * add Path::string() and u8string()
  - to enable display/log of Path
  * fmt for dai::Path; NN::setBlobPath(dai::Path)
  * dia::path throws like std::fs::path
  * c++17, pub/pvt header, test cmake c++ std level
  - enable c++17 std::filesystem support and test cases
  - split header into public/private parts
  - cmake for test cases now supports optional
  c++ standard level param
  * verify c++ std compiler support for tests
  - add COMPILER_SUPPORTS_CXX{14,17,20,23} vars
  to Flags.cmake and can be used everywhere
  * add dai::Path::empty()
  * add dai::Path to Device, DeviceBase, Resources
  - simplify Device, DeviceBase constructors by delegating
  - add is_same<> template on constructors with bool param to
  prevent implicit convert of almost everything to bool
  - make two DeviceInfo constructors explicit to prevent their use in
  implicit conversion
  - relevant test cases
  - fix minor throw text bugs
  * fix Device usb2Mode sigs, add test case
  * add dai::Path to CalibrationHandler
  * minor refactor dai::Path
  * enable 2 Calibration+1 Bootloader example
  * add dai::Path to DeviceBootloader, XLinkConnection
  - plus test cases
  * add dai::Path to Pipeline, StereoDepth, AssetManager
  - plus test cases
  * add dai::Path to dai::Script + test cases
  * linux fixes for test cases, and c++14 type_traits
  * add doxygen to dai::Path
  * detect compiler c++ std level and update cmake
  * fix preprocessor flag for tests on MSVC
  - fixes luxonis/`depthai-core/issues#408 <https://github.com/depthai-core/issues/issues/408>`_
  * partial dai::Path support for c++20 utf-8
  - unable to fully test due to bug `#407 <https://github.com/luxonis/depthai-core/issues/407>`_
  * add windows header define WIN32_LEAN_AND_MEAN
  * rename macro to DEPTHAI_NODISCARD
  - review feedback
* Apply style
* Add on-device mesh generator
* Initial BMI270 support
* Contributors: Dale Phurrough, Martin Peterlin, SzabolcsGergely, TheMarpe, szabi-luxonis

2.15.2 (2022-03-30)
-------------------
* Release v2.15.2
* Merge pull request `#439 <https://github.com/luxonis/depthai-core/issues/439>`_ from 0xMihir/main
  Bump Hunter version
* chore: bump Hunter version
  Adds support for MSVC 1931
  Using 0.24.0 doesn't work because there's a duplication error in the nlohmann/json library hunter config file
* std::exchange needs <utility> to be included (`#435 <https://github.com/luxonis/depthai-core/issues/435>`_)
  * std::exchange needs <utility> to be included
  Without <utility> it is gives "error: ‘exchange’ is not a member of ‘std’" errors.
  Ref : https://en.cppreference.com/w/cpp/utility/exchange
  * clang format fix
* Merge branch 'main' into develop
* Fixes `#436 <https://github.com/luxonis/depthai-core/issues/436>`_ - removes temporary warning log in StereoDepth
* Updated XLink - removed dependency on pthread_getname_np
* Merge branch 'device_is_closed_fix' into develop
* Fixed XLink issue with not erroring on write failures
* Openvino: improve error logging for out of memory cases
* Modified to store fisheye Camera model
  * Add getter for distortion model in CalibrationHandler
  * Pad distortion coefficients with 0's if there's less than 14
  * Only return first four distortion coefficients for Fisheye distortion
* Merge pull request `#430 <https://github.com/luxonis/depthai-core/issues/430>`_ from luxonis/custom_depth_unit
  Customizable depth unit
* Change metre to meter
* Change millimetre to depth unit where it's applicable in docs
* Add setter/getter utility function for depth unit
* Add customizable depth unit
* Merge pull request `#427 <https://github.com/luxonis/depthai-core/issues/427>`_ from luxonis/warp_improvements
  Warp engine improvements for RGB alignment/stereo rectification
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Warp engine improvements for RGB alignment/stereo rectification
* Contributors: Mihir Patil, Onuralp Sezer, SzabolcsGergely, TheMarpe, slitcch, szabi-luxonis

2.15.1 (2022-03-16)
-------------------
* Merge branch 'release_2.15.1' into main
* Merge pull request `#426 <https://github.com/luxonis/depthai-core/issues/426>`_ from luxonis/focal_length_from_calib
  Use focal length from calibration by default for Stereo node
* Merge pull request `#422 <https://github.com/luxonis/depthai-core/issues/422>`_ from luxonis/fix_calib_rgb_translation
  Calib data RGB spec translation fix for some CM3/CM4 boards
* Set focal length from calibration as default for stereo node
* Update FW: fix StereoDepth crash with missing EEPROM, report error if missing,
  do not rotate RGB (AUTO orientation) on OAK-D(-Lite) if EEPROM is missing
* Merge 'origin/develop' into fix_calib_rgb_translation
* Bump version to 2.15.1
* Merge pull request `#421 <https://github.com/luxonis/depthai-core/issues/421>`_ from luxonis/confidence_map_alignment_opt
  Optimizing the confidence map alignment
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk: Merge confidence_map_alignment_opt into develop
* Merge branch 'xlink_race_improvements' into develop
* Update FW: patch EEPROM data for incorrectly programmed RGB spec translation,
  for manually ran calibration on CM3/CM4 boards.
  Was leading to wrong RGB-depth alignment
* Update FW/mdk - Optimizing the confidence map alignment(CleanUp - remove global variables)
* Adding a new StereoDepth example rgb_depth_confidence_aligned.cpp for aligning the rgb, depth and confidence frames
* Updated XLink with only required changes
* Update FW/mdk - Optimizing the confidence map alignment(CleanUp logs)
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk - Optimizing the confidence map alignment
* Updated XLink with some race fixes and other improvements
* Updated FW with thermal protection mechanism
* Merge pull request `#398 <https://github.com/luxonis/depthai-core/issues/398>`_ from diablodale/fix390-callback-moves
  move semantics with DataOutputQueue::addCallback()
* Merge pull request `#417 <https://github.com/luxonis/depthai-core/issues/417>`_ from ibaiGorordo/patch-1
  Fix Readme links
* Fix Readme links
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk - Optimizing the confidence map alignment
* Fix default temporal/spatial filter values when subpixel is enabled
* Merge pull request `#403 <https://github.com/luxonis/depthai-core/issues/403>`_ from diablodale/fix314-cmpjson
  verify device json in example script_json_comm
* Merge pull request `#409 <https://github.com/luxonis/depthai-core/issues/409>`_ from diablodale/fix408-conformflag
  correct test cmake to add preproc conform flag
* correct test cmake to add preproc conform flag
  - fixes `luxonis/depthai-core#408 <https://github.com/luxonis/depthai-core/issues/408>`_
* Updated release template
* Merge branch 'main' into develop
* verify device json in example script_json_comm
  - related to https://github.com/luxonis/depthai-core/issues/314#issuecomment-1007463313
* move semantics in DataOutputQueue::addCallback()
  - fixes `luxonis/depthai-core#390 <https://github.com/luxonis/depthai-core/issues/390>`_
  - minor move tweaks using callbacks
* Contributors: Dale Phurrough, Ibai Gorordo, OanaMariaVatavu, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.15.0 (2022-02-23)
-------------------
* Merge branch 'release_2.15.0' into main
* Bump version to 2.15.0
* Merge pull request `#287 <https://github.com/luxonis/depthai-core/issues/287>`_ from luxonis/oak-d-pro_develop
  Support for OAK-D Pro
* Merge 'origin/develop' into oak-d-pro_develop
* `getIrDrivers` -> vector of tuples (driverName, i2cBus, i2cAddr). Return if setIrDot/Flood succeeded
* Merge pull request `#401 <https://github.com/luxonis/depthai-core/issues/401>`_ from luxonis/openvino_version_deprecation
  Deprecate OpenVINO 2020.4, 2021.1, 2021.2, 2021.3
* Merge 'origin/develop' into oak-d-pro_develop
* Merge pull request `#405 <https://github.com/luxonis/depthai-core/issues/405>`_ from luxonis/cfg_fps_lite
  Configurable RGB FPS on Lite devices, fix RGB orientation
* Updated test suite for new env var naming
* Renamed env variable and updated README regarding testing
* Update FW: RGB orientation fix for OAK-1 (new versions) and OAK-1-Lite
* Merge 'origin/develop' into cfg_fps_lite
* Merge 'origin/develop' into oak-d-pro_develop
* IR driver: remove raw register access API
* IR driver RPC: add `getIrDrivers()`, update description with limits
* Deprecate OpenVINO 2020.4, 2021.1, 2021.2, 2022.3
* Merge pull request `#389 <https://github.com/luxonis/depthai-core/issues/389>`_ from luxonis/imu_device_ts
  Add device monotonic timestamp to IMU reports
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add dot-projector/flood-illuminator brightness control
* Merge 'origin/develop' into oak-d-pro_develop
* Revert "USB bootloader with support for missing 24MHz USB osc (OAK-D Pro-PoE initial flashing)"
  This reverts commit 96691b9a8295c54bea1c04c20bc4ad60091ca536.
* Update FW: ColorCamera memory optimization when `still` is not connected,
  properly handle width-multiple-of-32 (only needed for VideoEncoder), don't enforce it on `video` by default,
  allow `still` size smaller than `video`
* Modify IMU example: GYRO at 400 hz to avoid spikes
* Update shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#387 <https://github.com/luxonis/depthai-core/issues/387>`_ from luxonis/subpixel_after_lr_check
  Do subpixel interpolation once when LR-check is enabled
* Fix formatting
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated shared
* Fixed crash when device watchdog is disabled
* Merge pull request `#394 <https://github.com/luxonis/depthai-core/issues/394>`_ from luxonis/skip_invalid_devices
  Invalid device warnings suppression
* Suppressed redundant warnings for invalid devices
* Merge pull request `#382 <https://github.com/luxonis/depthai-core/issues/382>`_ from diablodale/fix300-errorname
  skip devices named "<error>" in device search
* Merge branch 'testing_ci' into develop
* Merge branch 'fw_stability_fix' into develop
* Fixes a stability issue bug that affected PoE devices mostly
* Added testing CI
* Add on-device python bindings
* Add device monotonic timestamp to IMU reports
* Update FW, fix for custom alinment subpixel interpolation
* Do subpixel interpolation after LR-check; improves performance to 30fps@800p
* Merge pull request `#378 <https://github.com/luxonis/depthai-core/issues/378>`_ from diablodale/fix366-qsize0
  enable queues of size=0 which only do callbacks
* Merge branch 'fix334-interopt-dll' into develop
* Merge pull request `#361 <https://github.com/luxonis/depthai-core/issues/361>`_ from luxonis/rgbd_depth_align_optimize
  Rgbd-depth alignment optimization
* Update mdk - RGB-depth alignment
* Merge remote-tracking branch 'origin/develop' into rgbd_depth_align_optimize
* enable queues of size=0 which only do callbacks
  - minor optimize LockingQueue
  - fixes `luxonis/depthai-core#366 <https://github.com/luxonis/depthai-core/issues/366>`_
* Modified env variable logging verbosity
* log envvar values at TRACE level only (`#381 <https://github.com/luxonis/depthai-core/issues/381>`_)
  - fixes `luxonis/depthai-core#380 <https://github.com/luxonis/depthai-core/issues/380>`_
* Update FW - adding the RGB scaling factor for the RGB-depth center alignment
* skip devices named "<error>" in device search
  - partial fix `luxonis/depthai-core#300 <https://github.com/luxonis/depthai-core/issues/300>`_
* minor cleanup examples and tests
  - most fixes are signed/unsigned comparison corrections
* fix Win MSVC cmake INTERPROCEDURAL_OPTIMIZATION
  - workaround MSVC incompat BUILD_SHARED_LIBS +
  WINDOWS_EXPORT_ALL_SYMBOLS +
  INTERPROCEDURAL_OPTIMIZATION
  - fixes `luxonis/depthai-core#334 <https://github.com/luxonis/depthai-core/issues/334>`_
  - includes pr feedback
* Merge remote-tracking branch 'origin/develop' into rgbd_depth_align_optimize
* Merge pull request `#375 <https://github.com/luxonis/depthai-core/issues/375>`_ from luxonis/swap_imu_raw_accelerometer_axis
  Swap ACCELEROMETER_RAW x and y axis to match ACCELEROMETER
* Swap ACCELEROMETER_RAW x and y axis to match ACCELEROMETER
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#374 <https://github.com/luxonis/depthai-core/issues/374>`_ from luxonis/reenable_feature_tracker_metadata
  Reenable feature tracker metadata; change default CPU to CSS for Script node
* Update shared to match FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed non-cv support ImgFrame header
* Added initial setter chaining for messages
* Merge branch 'nndata_sequence_num'
* Updated style
* Merge branch 'openvino_blob' into develop
* Change bootloader shared submodule to match develop
* Merge remote-tracking branch 'origin/develop' into HEAD
* Modified docs and type of exception being thrown
* Documented Blob fields and added blob file size check
* Merge remote-tracking branch 'jdavidberger/develop'
* Updated libnop with renamed Nil enum
* Merge branch 'msvc_traditional' into develop
* Removed the need for conforming MSVC preprocessor
* Added capability to read blob information
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed env var usage
* Fixed typo
* Added search time env variable and moved querying of env variables to happen once
* Reverted back search timings in USB protocol case
* Added means of downselecting protocol used by XLink
* Merge branch 'watchdog_protection' into develop
* Merge remote-tracking branch 'origin/poe_improvements' into develop
* Update FW with clock related tweaks for feature tracker
* Updated flash_bootloader example
* Update shared/FW w/ build fix
* Improved PoE interaction
* Merge pull request `#359 <https://github.com/luxonis/depthai-core/issues/359>`_ from luxonis/subpixel_docs_fix
  Fix subpixel fractional bits documentation
* Update FW with stereo fixes for instance number; RGB depth alignment
* Change the resolution to 720p for the RGB-depth alignment example
* Update FW: optimized RGB-depth alignment
* Fix subpixel fractional bits documentation
* Rename AprilTagData to AprilTags
* Merge pull request `#166 <https://github.com/luxonis/depthai-core/issues/166>`_ from luxonis/gen2_apriltag
  Add apriltag support
* Update FW to latest develop
* Add example for advanced settings
* Update FW/shared/examples
* Update docs/FW
* Expose all config options for april tag detection algorithm
* Update FW/shared with fixes for TAG_CIR49H12 TAG_CUST48H12 TAG_STAND41H12 TAG_STAND52H13
* Update FW/shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Apply formatting
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#353 <https://github.com/luxonis/depthai-core/issues/353>`_ from luxonis/depth_docs
  Added some clarifications to depth docs
* Added some clarifications to depth docs
* Update FW with object tracker KCF fixes
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Specify minimum nlohmann version
* Merge pull request `#350 <https://github.com/luxonis/depthai-core/issues/350>`_ from luxonis/focal_from_intrinsics
  Use focal length from calibration intrinsics for fisheye cameras
* Update FW with fisheye lens detection and override option: setFocalLengthFromCalibration
* Get focal length from calibration intrinsics
* workaround bootloader-shared var init bug (`#347 <https://github.com/luxonis/depthai-core/issues/347>`_)
  - manually init class vars as workaround for
  https://github.com/luxonis/depthai-bootloader-shared/issues/4
* Update XLink
* Updated .gitmodules
* Update FW/XLink to latest
* Add openvino 2021.4.2 support
* Revert XLink to latest develop
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated .gitmodules
* Merge branch 'board_config' into develop
* Updated tidy and shared
* Updated libnop library (`#344 <https://github.com/luxonis/depthai-core/issues/344>`_)
* Updated XLink library
* Added watchdog protection in core and XLink
* Merge pull request `#335 <https://github.com/luxonis/depthai-core/issues/335>`_ from luxonis/confidence_map_rgb_alignment
  Add support for confidence map RGB alignment; fix bounding box remapping for RGB aligned depth frames
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* PipelineImpl::create() use make_shared, not raw new() (`#341 <https://github.com/luxonis/depthai-core/issues/341>`_)
  - fixes `luxonis/depthai-core#340 <https://github.com/luxonis/depthai-core/issues/340>`_
* Updated FW to sync with shared changes
* Updated shared
* Fix StereoDepth::setDefaultProfilePreset
* Added NN examples to tests, added utility conversion from fp16
* Added NN examples
* Re-enable feature tracker metadata
* Add support for confidence map RGB alignment; fix bounding box remapping for RGB aligned depth frames
* Merge pull request `#333 <https://github.com/luxonis/depthai-core/issues/333>`_ from diablodale/fix284-unreachable
  Remove unreachable code in DataQueue
* remove unreachable code in DataQueue
  - fixes `luxonis/depthai-core#284 <https://github.com/luxonis/depthai-core/issues/284>`_
* add const ADatatype::getRaw(), Buffer::getData(); add copy+move Buffer::setData() (`#331 <https://github.com/luxonis/depthai-core/issues/331>`_)
  - fixes `luxonis/depthai-core#330 <https://github.com/luxonis/depthai-core/issues/330>`_
* Merge pull request `#332 <https://github.com/luxonis/depthai-core/issues/332>`_ from luxonis/typos_fix
  Updated depthai-core with typo fixes
* Updated depthai-shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated multiple devices test
* XLink library fixes for multiple devices case (`#329 <https://github.com/luxonis/depthai-core/issues/329>`_)
* Merge branch 'resources_lazy_load_tsan' into develop
* Optimized condition_variable usage
* Configurable FPS for IMX214: 0.735 .. 35 for 1080p, 1.4 .. 30 (28.5 actually, TODO) for 4K/12MP/13MP
* Update FW with latest apriltag
* Merge remote-tracking branch 'origin/develop' into HEAD
* Moved over to a condition variable to signify end of lazy loading
* Merge remote-tracking branch 'origin/main' into HEAD
* Fix regression for ColorCamera, StereoDepth
* StereoDepth: check if input/output messages are connected
* Fix compilation error w/ clang 13
* Updated XLink
* Merge remote-tracking branch 'diablodale/fix257-move-owner-threads' into develop
* Added initial BoardConfig
* fix stream+packet ownership/move; fix thread crashes
  - fix many thread/ownership issues for start/stop scenarios
  - XLinkStream::readMove() for moving packet ownership
  - fix XLinkStream move semantics
  - removed all use of XLinkStream::readRaw as often leads to
  memory violations and/or memory leaks
  - deprecate all XLinkStream::readRaw...() APIs
  - fixes `luxonis/depthai-core#257 <https://github.com/luxonis/depthai-core/issues/257>`_
* Added missing throw statements
* Add spatialLocationCalculator output message to spatial detection network
* USB bootloader with support for missing 24MHz USB osc (OAK-D Pro-PoE initial flashing)
* Merge 'origin/develop' into oak-d-pro_develop
* Update FW with ipv6 disabled
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add RPC for LM3644 IR projector registers read/write on OAK-D-Pro
* Update FW with xlink thread priority changes
* Update FW: openvino 2021.4.2 support
* Update firmware SDK to r17.5
* Update linking
* Update bootloader and fixing errors
* Update shared
* Merge remote-tracking branch 'origin/develop' into gen2_apriltag
* Add apriltag_rgb example
* Rename, update shared
* Update FW
* Update apriltag example
* Add initial working version
* Contributors: CsabaGergely, Dale Phurrough, Erik, Erol444, Martin Peterlin, OanaMariaVatavu, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.14.1 (2022-01-11)
-------------------
* Release v2.14.1
* Fix regression for ColorCamera, StereoDepth
* StereoDepth: check if input/output messages are connected
* Contributors: SzabolcsGergely

2.14.0 (2022-01-05)
-------------------
* Release v2.14.0
* Fixed script json communication example
* Updated libnop
* Fixed updated Hunter usage
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Bump Hunter to add support for VS2022
* Update shared/FW
* Merge pull request `#312 <https://github.com/luxonis/depthai-core/issues/312>`_ from luxonis/connect_timeout_override
  Override watchdog initial delay and connect/bootup timeout
* Rename env var DEPTHAI_INIT_WATCHDOG -> DEPTHAI_WATCHDOG_INITIAL_DELAY
* clangformat changes,
  <> changed to "" as it was suggesting a new-line between <> and "" includes, and then alphabetically ordered
* `DEPTHAI_INIT_WATCHDOG` env var to set initial delay [ms] for the device watchdog,
  mainly to be set to larger values for Ethernet case with network equipment that takes long to establish the link. Default: 8s for USB, 15s for ETH
* Merge 'origin/develop' into connect_timeout_override
* Fix serialization of spatial img data
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#308 <https://github.com/luxonis/depthai-core/issues/308>`_ from luxonis/json_comm_example
  Added json communication example
* Updated XLink
* Update FW; wakeup driven high
* Set pullup for IMU wakeup pin
* Added json communication example
* FeatureTracker: Add support for 4k/12MP inputs
* Fix typo: assigment -> assignment
* FW fix for Stereo HW desync when extended is enabled
* Adds rgb/depth weight slider to rgb_depth_aligned example
* Merge remote-tracking branch 'origin/main' into HEAD
* Bump Windows SDK to 10.0.18362.0 with conforming preprocessor support (`#306 <https://github.com/luxonis/depthai-core/issues/306>`_)
* Updated FW to match shared
* Added MSVC preprocessor conformance flag
* Merge pull request `#303 <https://github.com/luxonis/depthai-core/issues/303>`_ from luxonis/typos_fix
  Typos fix
* NNData serialize fix (`#305 <https://github.com/luxonis/depthai-core/issues/305>`_)
  * Adds proper TensorInfo to serialized layer
* Merge branch 'develop_refactor' into develop
* Typos fix
* Updated shared
* Updated shared
* Merge branch 'develop' into develop_refactor
* Added incoming message parse timing to trace level debugging
* Merge pull request `#301 <https://github.com/luxonis/depthai-core/issues/301>`_ from diablodale/fix-xlink-local-install-v2
  fix 2 for xlink local cmake
* fix 2 for xlink local cmake
  - fixes `luxonis/depthai-core#272 <https://github.com/luxonis/depthai-core/issues/272>`_
  - replaces PR `#298 <https://github.com/luxonis/depthai-core/issues/298>`_
* Exposed max serialized metadata size
* Merge branch 'develop' into develop_refactor
* Merge pull request `#274 <https://github.com/luxonis/depthai-core/issues/274>`_ from luxonis/stereo_post_processing
  Added stereo post processing filters
* Update FW to latest develop
* Update FW with improved resource allocation for RGB aligment; improved error handling when out of resources
* Update shared w/ stubgen fixes
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'stubs_improvements' into develop
* Fix broken Windows CI
* Fixed XLink dependency in config mode
* Fixed exporting XLink when not using a local version
* Merge pull request `#298 <https://github.com/luxonis/depthai-core/issues/298>`_ from diablodale/fix-xlink-local-install
  fix xlink cmake install for local, shared, and static
* FW: Edge case fix for RGB aligment
* FW update: don't apply threshold filtering on confidence map
* Add depth post processing example
* Change all examples to use setDefaultProfilePreset
* Add default preset mode to StereoDepth constructor
* Add support for runtiem depth aligment mode; improve API
* fix xlink cmake install for local, shared, and static
  - fixes `luxonis/depthai-core#272 <https://github.com/luxonis/depthai-core/issues/272>`_
* Merge pull request `#297 <https://github.com/luxonis/depthai-core/issues/297>`_ from luxonis/tracker_docs
  Added possible tracker types to comment
* Updated shared
* Update FW, fix docs build
* Update FW; add default stereo presets; add configurable HW resources
* Added possible tracker types to comment
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#296 <https://github.com/luxonis/depthai-core/issues/296>`_ from diablodale/fix-264-cmake-shared-vars
  add cmake vars for local depthai-bootloader/shared
* add cmake vars for local depthai-bootloader/shared
  - fixes `luxonis/depthai-core#264 <https://github.com/luxonis/depthai-core/issues/264>`_
* Merge pull request `#295 <https://github.com/luxonis/depthai-core/issues/295>`_ from luxonis/fw_yolov5_and_stability
  FW YoloV5 support and stability updates
* Updated FW with YoloV5 support and stability improvements
* Apply thresholding filter on disparity map if depth is not enabled
* Add configurable decimation filter modes: pixel skipping/non zero median/non zero mean
* Merge branch 'depthai_clock' into develop
* Merge branch 'xlink_mingw_fix' into develop
* Add decimation filter
* Updated XLink with MinGW fixes
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add configurable number of shaves for stereo postprocessing
* Merge remote-tracking branch 'origin/develop' into HEAD
* Added clock
* Add spatial filter
* Clangformat bootloader example
* Add specle filter
* Initial version of temporal + thresholding filter
* Warn if watchdog is disabled, or value overriden.
  Reason for change: env vars might get forgotten set, and not easy to spot with DEPTHAI_LEVEL=debug
* Fix strncpy build warning:
  specified bound 48 equals destination size [-Wstringop-truncation]
* Override XLink wait for bootup/connect timeouts with env vars:
  DEPTHAI_CONNECT_TIMEOUT
  DEPTHAI_BOOTUP_TIMEOUT
  (in ms)
  TODO: add in bootBootloader as well
* Fixed setNumFramesPool for VideoEncoder node
* Fixed a node crtp issue
* Merge branch 'node_crtp' into develop_refactor
* Merge branch 'develop' into neuralnetwork_multiple_inputs
* Added CRTP to Nodes
* Merge branch 'develop' into libnop_serialization
* Refactored Nodes to allow for arbitrary properties and removed issues with multiple copies
* Added libnop dependency and unified serialization
* Merge branch 'develop' into neuralnetwork_multiple_inputs
* Removed deprecated usage and added correct output for DetectionNetwork back
* Updated waitForMessage API and applied across nodes
* Added IO groups and refactored IO references
* Added Node Input options and some tests
* Contributors: Dale Phurrough, Erik, Erol444, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.13.3 (2021-12-01)
-------------------
* Release v2.13.3
* Update FW: zero out uninitialized DDR memory
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Update FW: fix VideoEncoder potential crash (after power-cycle),
  instability introduced in 2.13.0
* Merge pull request `#281 <https://github.com/luxonis/depthai-core/issues/281>`_ from luxonis/manual_white_balance
  Add manual white balance / color temperature camera control
* Updated XLink with a couple of fixes
* Update shared/FW: manual_white_balance merged, other fixes:
  - fixes a crash with more than 4x VideoEncoder instances, now up to around 8 should work
  - StereoDepth fix crash with RGB-depth align and missing RGB calib (calibrated with -drgb)
  - StereoDepth fix RGB alignment when running at calib resolution (OAK-D with 800_P or OAK-D-Lite)
  - an error is thrown if multiple cameras have the same socket assigned
* rgb_camera_control: add manual white balance controls: `[` `]` `B`
* setManualFocus: no need to set OFF mode, auto-handled
* CameraControl: add `setManualWhiteBalance(colorTemperatureK)`
* Contributors: TheMarpe, alex-luxonis

2.13.2 (2021-11-26)
-------------------
* Release v2.13.2
* Merge remote-tracking branch 'origin/main' into HEAD
* FW fix for resource allocation issues when setRuntimeModeSwitch is used
* Contributors: SzabolcsGergely

2.13.1 (2021-11-24)
-------------------
* Applied style
* Merge branch 'develop' into main
* Merge branch 'xlink_regression_fix' into develop
* Updated XLink to fix SIGPIPE regression
* fix initialize() thread/except safety (`#277 <https://github.com/luxonis/depthai-core/issues/277>`_)
  - fixes `luxonis/depthai-core#276 <https://github.com/luxonis/depthai-core/issues/276>`_
* Contributors: Dale Phurrough, TheMarpe

2.13.0 (2021-11-22)
-------------------
* Release v2.13.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Update shared/FW
* Merge pull request `#262 <https://github.com/luxonis/depthai-core/issues/262>`_ from luxonis/oak-d-lite
  Support for OAK-D-Lite
* Remove deprecated VideoEncoder frame size config in examples
* Merge 'origin/develop' into oak-d-lite
* VideoEncoder: maxBitrate following bitrate setting in FW, when 0 (default)
* VideoEncoder: deprecated setting width/height, auto-computed bitrate by default
* Update FW: VideoEncoder source size configured when receiving 1st frame,
  allows to run OAK-D examples (e.g configuring mono cameras to 720_P) on OAK-D-Lite without code changes
* Merge pull request `#268 <https://github.com/luxonis/depthai-core/issues/268>`_ from diablodale/fix248-trunc-2
  Correct float literals, 32/64 trunc, unref vars
* fix errant printf params in examples (`#267 <https://github.com/luxonis/depthai-core/issues/267>`_)
  - fix `luxonis/depthai-core#259 <https://github.com/luxonis/depthai-core/issues/259>`_
* enable build in vscode, custom toolchain+include (`#258 <https://github.com/luxonis/depthai-core/issues/258>`_)
  * enable build in vscode, custom toolchain+include
  - fixes `luxonis/depthai-core#246 <https://github.com/luxonis/depthai-core/issues/246>`_
  * self doc dependency options with set(cache)
* Merge pull request `#269 <https://github.com/luxonis/depthai-core/issues/269>`_ from luxonis/set_ip_example
  Added Poe set IP example
* Added poe_set_ip example
* Updated FW with scripting improvements
* correct float literals, 32/64 trunc, unref vars
  - partial fix `luxonis/depthai-core#248 <https://github.com/luxonis/depthai-core/issues/248>`_
* Fix styling
* Update FW/shared
* Merge branch 'main' into develop
* Merge commit '18c5f8c3d4b4bb3498b515f2cb7a6a61f22db91a' into develop
* Fixed style
* Merge branch 'xlink_macos_fix' into develop
* Adds a timeout for closing an XLink connection
* Add device.getCameraSensorNames RPC call,
  can be used to differentiate between OAK-D and OAK-D-Lite. Should return:
  OAK-D     : RGB: IMX378, LEFT: OV9282, RIGHT: OV9282
  OAK-D-Lite: RGB: IMX214, LEFT: OV7251, RIGHT: OV7251
* Color/MonoCamera: handle more resolutions for OAK-D-Lite cameras:
  IMX214 (13MP) and OV7251 (480P)
* Updated XLink with macOS fix
* Contributors: Dale Phurrough, Erik, Erol444, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.12.1 (2021-11-17)
-------------------
* Merge branch 'win_prebuilt_fix' into main
* Cherry picked XLink macos fix
* Bump version to 2.12.1
* Fixed Windows prebuilt library
* Contributors: TheMarpe

2.12.0 (2021-11-13)
-------------------
* Merge pull request `#261 <https://github.com/luxonis/depthai-core/issues/261>`_ from luxonis/develop
  Release v2.12.0
* Release v2.12.0
* Merge pull request `#256 <https://github.com/luxonis/depthai-core/issues/256>`_ from luxonis/object_tracker_update
  Object tracker fixes, updates: 2 new tracking modes: KCF, short-term imageless.
* Update FW with latest improvements
* Fixes for object tracker; support for KCF and imageless short term tracking algorithms
* Merge pull request `#245 <https://github.com/luxonis/depthai-core/issues/245>`_ from luxonis/non_square_yolo_output
  Add support for non-square YOLO output
* Update FW before merge
* Update FW with error reporting for DetectionNetwork
* Add support for non-square YOLO output
* Update FW with Script node (DynamicPool) related fixes
* Merge pull request `#216 <https://github.com/luxonis/depthai-core/issues/216>`_ from luxonis/stereo_depth_fine_tuning
  Fine tune stereo depth settings
* Increase LR-check threshold to 10; disparity confidence threshold to 245 by default
* Add fine tuned stereo settings, configurable P1/P2 cost aggregation parameters
* Merge remote-tracking branch 'origin/develop' into HEAD
* Revert "Set fine tuned stereo settings"
  This reverts commit 8af5641c0e0d91d89d84bd4de8daa5aceaebc658.
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#240 <https://github.com/luxonis/depthai-core/issues/240>`_ from luxonis/extended_disparity
  Add extended disparity mode
* Update FW before merge
* Add addtional outputs to output list
* Merge remote-tracking branch 'origin/develop' into HEAD
* FW - fixed OpenVINO layer issue
* Spdlog version change (`#239 <https://github.com/luxonis/depthai-core/issues/239>`_)
  * added spdlog fix
* Add extended mode debug outputs
* Merge remote-tracking branch 'origin/develop' into extended_disparity
* StereoDepth: Add extended disparity mode
* Merge pull request `#238 <https://github.com/luxonis/depthai-core/issues/238>`_ from luxonis/disparity_enc
  Added disparity encoding example
* Added disparity encoding example
* Added CMake version into CI and Ubuntu 18.04 fix (`#237 <https://github.com/luxonis/depthai-core/issues/237>`_)
  * Added CMake version into CI
  * Updated ZLIB with fixed ALIAS on imported target
  * CI - Concatenated -D arguments for old CMake version
  * Updated README instructions for CMake version 3.10
  * Fixed Windows build and ZLIB target
  * Removed old CMake build for MSVC
  * Updated -D CMake usage
* Merge pull request `#234 <https://github.com/luxonis/depthai-core/issues/234>`_ from luxonis/script_forward_frames
  Added script forward (demux) example
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'main' into develop
* Merge pull request `#236 <https://github.com/luxonis/depthai-core/issues/236>`_ from luxonis/catch_dependency_fix_new_glibc
  Update catch2 package to 2.13.7
* Update catch2 package to 2.13.7
* Added script forward (demux) example
* Restructured README.md (`#232 <https://github.com/luxonis/depthai-core/issues/232>`_)
  * Restructured README
  * Update README.md
  * Update README.md
* Set fine tuned stereo settings
* Contributors: Erik, Erol444, Sachin Guruswamy, SzabolcsGergely, TheMarpe, szabi-luxonis

2.11.1 (2021-10-19)
-------------------
* Merge pull request `#230 <https://github.com/luxonis/depthai-core/issues/230>`_ from luxonis/develop
  Release v2.11.1
* Bump version to 2.11.1
* Update to latest firmware/depthai-shared
* Change warning to info
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#229 <https://github.com/luxonis/depthai-core/issues/229>`_ from luxonis/fix_build_visual_studio_m_pi
  Fix build with older Visual Studio - M_PI undeclared
* `#define _USE_MATH_DEFINES` at the top of the file
  attempting to fix building with Visual Studio 15 2017:
  `error C2065: 'M_PI': undeclared identifier`
  https://discord.com/channels/790680891252932659/798284448323731456/899110756413489212
* Merge pull request `#227 <https://github.com/luxonis/depthai-core/issues/227>`_ from luxonis/examples_sorting
  Examples sorting
* Merge pull request `#228 <https://github.com/luxonis/depthai-core/issues/228>`_ from luxonis/sipp_fw_bugfixes
  Firmware sdk fixes: for ISP/Sipp filter crashes `#395 <https://github.com/luxonis/depthai-core/issues/395>`_
* Update FW before merge
* Renamed two examples
* Internal firmware sdk fixes: for ISP/Stereo/Sipp filter crashes
* Fixed CMakeLists that should have worked before as well but ok
* Moved examples out of /src folder
* Removed fromPlanarFp16() as it's not needed
* Style fix
* Added script node CPP examples
* Added examples in their corresponding folders
* Grouped tiny yolo3/4 together
* Contributors: Erik, Erol444, SzabolcsGergely, alex-luxonis, szabi-luxonis

2.11.0 (2021-10-13)
-------------------
* Merge branch 'develop' into main
* Updated formatting
* Fixed double promotion warning
* Bumped to v2.11.0
* Merge branch 'backward_issue_fix' into develop
* Backward - Disables use of additional stack unwinding libs
* Update FW: increase ImageManip warp max out height: 1520 -> 2560
* Windows prebuilt libraries (`#220 <https://github.com/luxonis/depthai-core/issues/220>`_)
  * Added CI to build Win64 & Win32 prebuilt libraries and upload along the release
* Merge branch 'spi_improvements' into develop
* Merge branch 'develop' into spi_improvements
* Hotfix for FW message cache coherency
* Merge pull request `#206 <https://github.com/luxonis/depthai-core/issues/206>`_ from luxonis/calib_fov_calculated
  Added getting calculated FOV from intrinsics
* Merge pull request `#212 <https://github.com/luxonis/depthai-core/issues/212>`_ from SpectacularAI/fix-extrinsic-inversions-in-calibration-handler
  Fix the inversion formula for extrinsic matrices in CalibrationHandler
* Fixed for Windows
* Fix inversion formula for extrinsic matrices in CalibrationHandler
* Fix styling
* Merge pull request `#218 <https://github.com/luxonis/depthai-core/issues/218>`_ from luxonis/stereo_confidence_map
  Add confidence map output to stereo node
* Update FW to latest develop
* Update confidence map output docs
* Add confidence map output to stereo node
* Merge pull request `#217 <https://github.com/luxonis/depthai-core/issues/217>`_ from luxonis/ppenc_fixes
  Fix still image output in RGB postprocessing
* Updated FW with SPI improvements
* Update FW to latest develop
* Fix still image output in RGB postprocessing
* Fix bootloader version example
* Merge pull request `#200 <https://github.com/luxonis/depthai-core/issues/200>`_ from luxonis/stereo_fixes
  Stereo improvements, fixes for subpixel, LR-check
* Sync stereo_depth_video example
* Update FW/shared to latest develop
* Replace deprecated getMaxDisparity() function
* Handle disparity companding in getMaxDisparity
* Update FW with runtime disparity range fix
* Add getMaxDisparity() based on subpixel bits
* Add stereo node output config
* Update calibration_reader.cpp
* Add debug outputs to stereo node; expose number of frame pools
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#213 <https://github.com/luxonis/depthai-core/issues/213>`_ from luxonis/spatial_calc_algo_choice
  Add option to pick spatial calculation algorithm : average,min,max of…
* Update FW/shared to latest develop
* Merge pull request `#214 <https://github.com/luxonis/depthai-core/issues/214>`_ from luxonis/flash_bl_example_fix
  flash_bootloader example fix
* Update shared w/ CI fixes
* flash_bootloader: improve user prompts, when booted over USB / recovery mode:
  don't ask for confirmations, as if flashing is interrupted, recovery mode should still be accessible.
  Also it was a bit confusing asking to replace USB bootloader (booted as a flasher helper) with NETWORK
* Update FW to match depthai-shared
* flash_bootloader: fix flashing NETWORK bootloader (when booted over USB),
  or flashing a different bootloader type
* Set bytes per pixel for ImgFrame
* Add option to pick spatial calculation algorithm : average,min,max of selected ROI
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW with subpixel fix
* Refactor stereo depth config structure
* Update FW, enable runtime configuration of Stereo node
* Imu extrinsics (`#211 <https://github.com/luxonis/depthai-core/issues/211>`_)
  * Updated IMU extrinsics
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW with stereo confidence runtime config fix
* Updated Bootloader to 0.0.15
* Update FW with stereo performance improvements
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* FW - Updated ColorCamera 1080P resolution config
* Fixed integration issues
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'develop_embedded' into develop
* Remove rectification flipping on host, it was resolved in firmware
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Updated FW - fixed cache coherency issue
* Update FW, for depthai-shared to match with depthai-core
* Update FW: fix default camera orientation for OAK-1-PoE, was rotated
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Pipeline - number of connections improvement
* Fixed exception rethrow in DeviceBase
* Merge pull request `#207 <https://github.com/luxonis/depthai-core/issues/207>`_ from luxonis/imagemanipcfg_helper_functions
  Add ImageManipConfig helper functions
* Fixed style checks, added FormatConfig
* Added alias
* Add ImageManipConfig helper functions
* Fixed issues for the PR
* Added capability to not install signal handlers
* Added option to calculate FOV based on camera intrinsics. Added this function to calibration_reader and also refactored it so matricies are more readable
* Merge pull request `#205 <https://github.com/luxonis/depthai-core/issues/205>`_ from luxonis/calib_helper_functions
  Calib helper functions
* Fixed typo
* Style check fix
* Updated FW to allow for graceful resets
* Added helper functions to get translation vector and baseline distance
* Merge pull request `#204 <https://github.com/luxonis/depthai-core/issues/204>`_ from luxonis/extrinsics_translation_cm
  Specified that translation is in centimeters
* Specified that translation is in centimeters
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#203 <https://github.com/luxonis/depthai-core/issues/203>`_ from luxonis/overloading_functions
  Added some function overloads
* fix compiling error
* Added some function overloads
* Fixed style
* Added Backward library to print stacktraces on crash
* Updated FW with GPIO and SPI improvements
* Merge branch 'throw.nice' into develop
* Added flash booted state and handling
* Merge branch 'device_config' into develop_embedded
* Merge branch 'bootloader_updates' into develop_embedded
* Fixed incorrect exception message
* Fixed Windows Platform specific code
* Fixed Super Speed mode and added a test
* Updated FW for UsbSpeed handling
* Added versioning to BL requests and refactored
* Updated flash_bootloader example
* Added capability to compress FW and additional BL config helper
* Reduced BL check to 0.0.14 and updated FW and BL
* Update FW with stereo LR-check, subpixel fixes; extended mode is not available
* Apply suggestions by clang-tidy
* Rename vars as requested
* Bring the 3 variable ctor into visibility
* Updated bootloader_configuration example
* Make data members const
* Add pertinent info to XLinkError struct
* Throw XLink specific errors for read/write errors
* WIP: Bootloader configuration
* Merge branch 'develop' into bootloader_updates
* Fixed boot_memory bootloader upgrade routine
* Merge branch 'develop' into bootloader_updates
* Allow to specify which bootloader is overridden by the env var:
  `DEPTHAI_BOOTLOADER_BINARY_USB`
  `DEPTHAI_BOOTLOADER_BINARY_ETH`
  (both can be set)
* Updated flash_bootloader example
* Improved the flash_bootloader example a bit
* Updated flash_bootloader to be a bit more verbose
* Added an explicit flag to allow flashing bootloader
* Moved operator<< overloads to global namespace
* Warn when firmware or bootloader binaries are overriden
  - to confirm it's picked up, or to notice when forgotten exported
* Optional env var DEPTHAI_BOOTLOADER_BINARY to override bootloader FW path,
  mostly for development
* Update bootloader: support for more NOR flash chips,
  fixes issues with flash erasing
* Revert "Removed flash_bootloader"
  This reverts commit f1f03bcefde92b518fe5a1534b83c3fa919e30e6.
* Revert "Removed flash_bootloader example temporarily"
  This reverts commit ee2a04e58b995e1bfa0cb03b91f83a45d446ca7f.
* Updated FW and a catch clause
* Merge branch 'develop' into device_config
* Fixed patching
* Modified watchdog to use a separate stream
* Updated preboot and added watchdog configuration
* Merge branch 'develop' into device_config
* Removed deprecated OpenVINO versions
* Merge branch 'develop' into device_config
* Updated example
* Merge branch 'develop' into device_config
* Refactored and added preboot config
* WIP: Device configuration
* Contributors: Erik, Erol444, Kunal Tyagi, Martin Peterlin, Otto Seiskari, Sachin Guruswamy, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.10.0 (2021-08-24)
-------------------
* Release v2.10.0
* Merge pull request `#201 <https://github.com/luxonis/depthai-core/issues/201>`_ from luxonis/develop
  Release v2.10.0
* Bump version to 2.10.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#199 <https://github.com/luxonis/depthai-core/issues/199>`_ from luxonis/xlink_chunk_size
  Configure XLink chunk size
* Update FW and shared after merge
* DeviceBase/Device: add {set/get}XLinkChunkSize RPC calls
* Merge pull request `#195 <https://github.com/luxonis/depthai-core/issues/195>`_ from luxonis/update_readme
  Update README.md instructions with OpenCV troubleshooting
* Fix naming `setXlinkChunkSize` -> `setXLinkChunkSize`
* Pipeline: add `setXlinkChunkSize`
* Update FW with bilateral fix
* Update README.md
* Merge branch 'main' into develop
* Merge branch 'deviceBase' into develop
* Address review comments
* Merge pull request `#197 <https://github.com/luxonis/depthai-core/issues/197>`_ from luxonis/sysinfo_docs
  Fixed display names
* Fixed display names
* update code template
* Merge pull request `#196 <https://github.com/luxonis/depthai-core/issues/196>`_ from luxonis/stereo_crash_workaround
  Stereo crash workaround
* Add workaround for stereo subpixel/extended mode crash at the expense of system performance
* Update README.md instructions with OpenCV troubleshooting
* Merge pull request `#181 <https://github.com/luxonis/depthai-core/issues/181>`_ from luxonis/feature_tracker
  Feature tracking support
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'main' into develop
* Added default constructor as these are not inherited
* Update FW
* Applied style
* Fixes for MSVC ambiguity with overloaded constructors
* Handle dtor and close without bugs
* Merge branch 'develop' into deviceBase
* Fix build issue
* Rename function arguments to their alias
* Fix docs about feature tracking
* Update shared with type fixes in docs; update FW to latest develop
* Keep same behavior in DeviceBase as Device wrt starting pipeline
* Make ctor API simpler for `DeviceBase` and `Device`
* Merge remote-tracking branch 'origin/develop' into HEAD
* Refactor FeatureTrackerConfig
* Rename feature tracker config fields
* Shutdown gracefully in case of exception in ctor
* Add support for hardware accelerated motion estimation
* Make `connection` as protected
* Move startPipeline from DeviceBase to Device
* Update shared
* Rename FeatureTrackerData to TrackedFeatures
* Sync python-cpp examples
* Add configurable shave/memory resources to feature tracker
* Update FW with memory optimizations
* Update FW and shared
* Add overloaded functions to disable optical flow
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Extend feature tracker configuration
* Add config fields to feature tracker node
* Update FW
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Synchronize python-cpp examples
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Update names, make serialize a public function
* Add note in the documentation of the virtual functions
* Fix reference to base class function in `dai::Device`
* Give more love to StreamPacketParser
* Make the virtual functios protected and public functions non-virtual
* Move items around in startPipeline
* Separate Device and DeviceBase, expose StreamPacketParser
* Separate Queue handling from core API
* Update FW with multi instance support
* Remove leftover code
* Update trackbar naming
* Add FeatureTracker node; add cpp example
* POC: Feature tracker node
* Contributors: Erik, Erol444, Kunal Tyagi, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis, Łukasz Piłatowski

2.9.0 (2021-08-07)
------------------
* Hotfix - temporary prevent flashing apps for PoE models
* Version bump to v2.9.0
* Updated FW
* Merge branch 'develop' into main
* Updated FW
* Removed 'filesystem' include
* Merge branch 'main' into develop
* Added an alias for Script Properties
* Merge pull request `#193 <https://github.com/luxonis/depthai-core/issues/193>`_ from luxonis/image_manip_rotate
  ImageManip tiling and rotating example
* Restarting docs building
* Added }
* Fixed conversion problems
* Fixing compilation error on mac
* Merge pull request `#192 <https://github.com/luxonis/depthai-core/issues/192>`_ from luxonis/distortion_coeff_docs
  Added distortion coefficients representation for the documentation
* Added distortion coefficients representation for the documentation
* Fixed imageManip rotate, added imageManip tiling example
* Added ImageManip example
* Hotfix - updated XLink with a segfault fix
* Merge branch 'xlink_error_221_fix' into develop
* Merge branch 'fp16_no_git_clone' into develop
* Added an XLink 221 fix in FW and a default confidence threshold
* Added a custom fork of FP16 which doesn't use git clone
* Merge pull request `#187 <https://github.com/luxonis/depthai-core/issues/187>`_ from luxonis/update_openvino
  Update OpenVINO version in examples to 2021.4
* Update OpenVINO version in examples to 2021.4
* Merge branch 'gen2_scripting' into develop
* Fixed depth_crop_control example
* Merge branch 'develop' into gen2_scripting
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'develop_spi_in' into develop
* Updated shared and FW
* Added additional options to SPIIn
* Added override to SPIOut::getProperties
* Merge branch 'develop' into develop_spi_in
* Updated FW
* Updated AssetManager::get function documentation
* Increased test timeout to 10s
* Addressed PR comments and updated FW
* Merge branch 'develop' into gen2_scripting
* Fixed a binding issue in FW
* Merge branch 'develop' into gen2_scripting
* Updated FW
* Updated FW
* Updated FW
* Indented example script
* WIP: Merge resolution
* Merge branch 'develop' into gen2_scripting
* Merge remote-tracking branch 'origin/develop' into gen2_scripting
  # Conflicts:
  #	cmake/Depthai/DepthaiDeviceSideConfig.cmake
  #	shared/depthai-shared
  #	src/pipeline/node/NeuralNetwork.cpp
* Merge branch 'gen2_scripting' of github.com:luxonis/depthai-core into gen2_scripting
* Script - added struct and fixed json modules
* Merge FW with latest develop
* Updated Script node with json and ctypes libraries
* Merge branch 'develop' into gen2_scripting
* Fixed NN bug
* Added missing includes
* Updated SPIIn and FW
* Merge branch 'develop' into develop_spi_in
* Merge branch 'develop' into gen2_scripting
* Changes to get SPIIn working (WIP)
* added script camera control example  (as in python)
* added include Script node in depthai.hpp
* Merge branch 'gen2_scripting' of github.com:luxonis/depthai-core into gen2_scripting
* Updating firmware (Fixing datetime on ImgFrame::getTimestamp)
* Fixed scripting 'setCropRect' and added bounds
* Update firmware.
* Updating firmware and adding a check to raw PoBuf parsing.
* Updated FW
* Added DEPTHAI_FW_BINARY_PATH environment variable
* Fixed an incorrect RPC call
* Renamed 'LxScript' to 'Script'
* Updated style
* Updated FW to reduce size
* Applied formatting
* Improved Asset handling
* Merge branch 'develop' into gen2_micropython
* Renaming MicroPython node to LxScript.
* Checking in micropython asset changes.
* Removed unneeded variable
* Added capability to specify additional IO
* Refactored asset loading and capitalized MicroPython
* Merge branch 'gen2_develop' into gen2_micropython
* Adding micropython.
* Contributors: Erik, Erol444, Jon Ngai, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, Łukasz Piłatowski

2.8.0 (2021-07-23)
------------------
* Merge pull request `#185 <https://github.com/luxonis/depthai-core/issues/185>`_ from luxonis/develop
  Release v2.8.0
* Update FW to 2.8.0
* Update shared to 2.8.0
* Bump version to 2.8.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#174 <https://github.com/luxonis/depthai-core/issues/174>`_ from luxonis/cam_sync
  RGB - Mono capture time sync
* Merge remote-tracking branch 'origin/develop' into cam_sync
  Update FW and depthai-shared after merge
* Merge branch 'rpc_issue_fix' into develop
* Applied formatting
* Updated comment on RPC mutex
* WIP: Reenabled RPC mutex lock
* Hide nanorpc client under Device::Impl
* Merge pull request `#179 <https://github.com/luxonis/depthai-core/issues/179>`_ from luxonis/imu-accuracy-name-clash
  Fix imu accuracy name clash
* Update shared/FW
* Rename IMUReportAccuracy enum to Accuracy
* Fix name clash for accuracy field in RotationVector structure
* Merge pull request `#167 <https://github.com/luxonis/depthai-core/issues/167>`_ from luxonis/openvino_2021_4
  Add OpenVino 2021.4 support; remove deprecated 2020.1, 2020.2
* Add openvino 2021.4 blob to tests
* Update FW to latest develop
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge remote-tracking branch 'origin/develop' into HEAD
* Update FW with OpenVino FW fix
* Merge remote-tracking branch 'origin/develop' into HEAD
* Update FW: implement RGB - Mono sync:
  capture time and sequence numbers
* Add ImgFrame::getTimestampDevice() API - mostly for debugging
* ImgFrame.hpp: fix some typos
* Add OpenVino 2021.4 support; remove dperecated 2020.1, 2020.2
* Contributors: Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.7.2 (2021-07-19)
------------------
* Merge pull request `#178 <https://github.com/luxonis/depthai-core/issues/178>`_ from luxonis/develop
  Release v2.7.2
* Bump version to 2.7.2
* Update FW with SDK update
* Merge remote-tracking branch 'origin/main' into HEAD
* Update FW
* Hotfix: fix NN memory allocation regression
* Merge branch 'queue_reference_fix' into develop
* Fixed DataQueue isClosed logic
* Closing the data queue joins the underlying thread
* Close queues when closing the device
* Merge branch 'get-in-out.const' into develop
* Fix style
* Add the EdgeDetector for the CI
* Convert from 2 pointers to a vector
* Fix style
* Add {In,Out}putRef getters
* Fix style, again
* Adding getters for parents
* Adjust visibility of getName, getInput, getOutput
* Mark member functions `dai::Node::get{In,Out}put` as const
* Contributors: Kunal Tyagi, SzabolcsGergely, TheMarpe, szabi-luxonis

2.7.1 (2021-07-16)
------------------
* Merge pull request `#176 <https://github.com/luxonis/depthai-core/issues/176>`_ from luxonis/2.7.0_hotfix
  Release 2.7.1
* Bump version to 2.7.1.0
* Hotfix: fix NN memory allocation regression
* Contributors: SzabolcsGergely, szabi-luxonis

2.7.0 (2021-07-13)
------------------
* Merge branch 'develop' into main
* Updated FW
* Bump to version 2.7.0
* Merge branch 'bootloader_improvements_eth_desync_fix' into develop
* Removed flash_bootloader
* Removed flash_bootloader example temporarily
* Updated bootloader_version example
* Merge branch 'develop' into bootloader_improvements_eth_desync_fix
* Hotfix FW: revert increased memory consumption
* Updated XLink dependency
* Updated resources to handle FW diff
* Naming changes and additional bootloader capabilities
* Merge branch 'develop' into bootloader_improvements_eth_desync_fix
* Updated to develop FW
* Added export of integration options
* Merge pull request `#169 <https://github.com/luxonis/depthai-core/issues/169>`_ from luxonis/3rdparty_integration_docs_fix
  Updated instructions for thirdparty library integration
* Merge pull request `#169 <https://github.com/luxonis/depthai-core/issues/169>`_ from luxonis/3rdparty_integration_docs_fix
  Updated instructions for thirdparty library integration
* Updated instructions for thirdparty library integration
* Added getMxId call for ethernet use case
* Update FW and XLink for desync fix
* Update ETH bootloader/FW: fix some IPv6 related crashes, improve performance
* Fixed MacOS build. Local XLink option skips hunter
* Updated bootloader
* Applied formatting
* Resources: ETH bootloader bug. Added flash_bootloader example
* Updated XLink to tcpip_driver branch
* Improving some BootMemory cases and updated bootloader and FW
* Added temporary ETH specific fixes
* Booting specified bootloader
* Merge branch 'develop' into bootloader_improvements
* Added resource loading for bootloader
* Added backwards compatibility
* Merge branch 'bootloader_boot_memory' into bootloader_improvements
* Updated bootloader and command to boot fw
* Contributors: Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis

2.6.0 (2021-07-06)
------------------
* Release v2.6.0
* Merge pull request `#168 <https://github.com/luxonis/depthai-core/issues/168>`_ from luxonis/develop
  Release 2.6.0
* Bump version to 2.6.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Hotfix: Fix mobilenet detection network
* Merge pull request `#165 <https://github.com/luxonis/depthai-core/issues/165>`_ from luxonis/edge_detector
  Add EdgeDetector node
* Update shared/FW
* Fix BUG in ParsePacket for received SpatialLocationCalculatorConfig
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'host_build_c++14' into develop
* Merge branch 'xlink_desync_fix' into develop
* Merge branch 'nn_dimensions_strides_order_fix' into develop
* Calib fix (`#163 <https://github.com/luxonis/depthai-core/issues/163>`_)
  * Bug fix
  * Fixing negative
  * Updated device side fix for signs
  * Additinal checks
  * Fixed styling
  * updated FW to develop:
* Update FW/shared
* Merge with latest develop; Update FW
* Add edge detector node using HW sobel edge filter
* Hotfix: Update FW with fix for crash w/ depth-rgb aligment
* Replace deprecated function call
* Updated FW and XLink to fix stream desync issue
* Merge pull request `#159 <https://github.com/luxonis/depthai-core/issues/159>`_ from luxonis/bilateral_filter
  Add support for 5x5 bilateral filter in stereo depth; add runtime con…
* Update FW
* Deprecate setConfidenceThreshold; setMedianFilter
* Add config for LR-check threshold
* Update FW with median filter configurability
* Deprecate setEmptyCalibration
* Update FW with resource allocation fix
* Update shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed Seg Fault in getImuToCameraExtrinsics (`#156 <https://github.com/luxonis/depthai-core/issues/156>`_)
  * Fixed Seg Fault in getImuToCameraExtrinsics
  * Added additional check at ComuteExtrinsics
  * Changed error display
* Updated formatting
* Reversed dimension and stride order
* Add support for 5x5 bilateral filter in stereo depth; add runtime configurability for stereo depth
* Updated shared and FW
* Merge pull request `#153 <https://github.com/luxonis/depthai-core/issues/153>`_ from kunaltyagi/headers
  Add convenience headers in depthai/pipeline
* Merge pull request `#157 <https://github.com/luxonis/depthai-core/issues/157>`_ from kunaltyagi/libarchive.cmake
  Change name of libarchive for better compatiblity with Hunter's packages
* Change name of libarchive for better compatiblity with Hunter's packages
* Removed unnedeed standard specification
* Merge pull request `#148 <https://github.com/luxonis/depthai-core/issues/148>`_ from luxonis/depth_align_improvements
  StereoDepth: mesh rectification, disp/depth configurable resolution
* Merge remote-tracking branch 'origin/develop' into depth_align_improvements
* Update FW: depthai-shared PR merged,
  also included FW changes from https://github.com/luxonis/depthai-core/pull/118 :
  fixes for new boards with 0x2 boot mode (not switching back to bootloader after app reset)
* Rename as requested
* Added C++14 as transitive property
* Fixed NN deadlock edge case
* Replaced 'unique_ptr' and 'new' with 'make_unique'
* Fixed some bugs
* Merge pull request `#154 <https://github.com/luxonis/depthai-core/issues/154>`_ from kunaltyagi/parent.public
  Make `Node::getParentPipeline` publically available
* Make `getParentPipeline` publically available
* Adding convenience headers
* Merge pull request `#152 <https://github.com/luxonis/depthai-core/issues/152>`_ from luxonis/synch_calibration
  Synchronize calibration examples w/ python
* Update FW
* Synchronize with python
* Add support for median filter for LR check depth mode
* StereoDepth: add setOutputKeepAspectRatio
* Rename `setOutputResolution` -> `setOutputSize`, for consistency
  with similar API in ColorCamera, etc
* rgb_depth_aligned: increase confidence threshold 200 -> 230,
  as in the python example
* Update depthai-shared: make clangformat
* clangformat
* rgb_depth_aligned: lower L/R res: 720p -> 400p, to fix lag for now
  Also add configurable FPS, to allow quick swap to 720p with a lower FPS
* Merge remote-tracking branch 'origin/develop' into depth_align_improvements
* StereoDepth: add mesh calibration support
* Merge branch 'develop' into host_build_c++14
* Moved C++ standard specification to targets
* Update to C++14,
  remove depthai-shared workaround for unordered_map with enum class
* StereoDepth: add setOutputResolution, currently applicable with
  RGB alignment
* Contributors: CsabaGergely, Kunal Tyagi, Sachin Guruswamy, SzabolcsGergely, TheMarpe, alex-luxonis, csaba-luxonis, szabi-luxonis

2.5.0 (2021-06-08)
------------------
* Release v2.5.0
* Merge pull request `#149 <https://github.com/luxonis/depthai-core/issues/149>`_ from luxonis/develop
  Release v2.5.0
* Bump version to 2.5.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#147 <https://github.com/luxonis/depthai-core/issues/147>`_ from luxonis/update_doc
  Update documentation
* Update shared
* Update FW
* Merge pull request `#143 <https://github.com/luxonis/depthai-core/issues/143>`_ from luxonis/queue_add_callback_cpp
  Added example on how to add a queue callback in cpp
* Update shared
* Merge remote-tracking branch 'origin/develop' into update_doc
* Update 2
* Merge pull request `#119 <https://github.com/luxonis/depthai-core/issues/119>`_ from luxonis/imu_node
  IMU: BNO 085/6 support
* Update FW, shared
* Update documentation
* Rename RAW\_* to *_RAW in ImuSensors
* Update FW; fix high CPU load; enable full speed raw sensors
* Merge remote-tracking branch 'origin/develop' into HEAD
* Rename imu_gyro_accelero example
* Add convenience functions; sync cpp python examples
* Calibration data bug fix (`#146 <https://github.com/luxonis/depthai-core/issues/146>`_)
  Changed double to float in set/get fov.
  Modified Docstring for matrix (C++ only for now)
  FW bug fix in stereo when rgb camera calibration was not available
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#144 <https://github.com/luxonis/depthai-core/issues/144>`_ from luxonis/spatial_calculator_improvements
  Add depthMin, depthMax to spatial calculator
* Update FW
* Merge remote-tracking branch 'origin/develop' into spatial_calculator_improvements
* Add depthMin, depthMax to spatial calculator
* Add comments
* Add example on how to add a queue callback in cpp
* Merge pull request `#141 <https://github.com/luxonis/depthai-core/issues/141>`_ from luxonis/object_tracker_video
  Add object tracker video example
* Merge branch 'blob_version_compatibility' into develop
* Merge branch 'develop' into blob_version_compatibility
* Merge pull request `#101 <https://github.com/luxonis/depthai-core/issues/101>`_ from luxonis/gen2_eeprom_api
  Calibration read/write/load API
* Updated device side
* Updated examples to create backup
* Typo fix
* Example bug fix
* Fixed styling
* Merged with develop
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into HEAD
* Update on revierws
* Added a test for various OpenVINO versions
* Bug fix
* Updated examples
* Updated validation
* Add timestamp to video mobilenet
* Add timestamp
* Fixed docstring
* Updated device side commit
* clangformat
* Synchronize stereo_depth_video example
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* Changes for swap WIP
* Add empty frame check
* Add object tracker video cpp example
* Modified test and example adding function
* Added openvino blob versioning support
* Updated device side
* Updated device side commit id
* Updated device side and shared
* removed -
* fixed test and updated shated
* Updated styling
* Merged with develop:
* addressing PR Requests
* changed measured* to spec*
* removed bootloader test
* fixed tests
* tidy
* modified examples for test:
* merged with develop and added validatecameraArray
* merged with develop
* added more getters
* added headers
* Add RAW accelerometer/gyro sensors
* adressed PR requests
* fixed intrinsics scaling bug
* updated device side commit id
* updated examples with API changes
* updated shared and device side commits
* docstring updates
* Remove function argument from getters
* Rename imu_example to imu_gyro_accelero_example
* Add rotation vector example
* Add configurable IMU report rates for gyro and accelero
* Update FW with fix for timesync
* fixed api function calls style
* fixed rgb measured translation issue:
* refactoring
* fixed extrinsics sign issue
* fixed overloading function issue:
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* added throw
* fixed styling
* fixing style
* added throw to runtime errors
* Update FW with fixes for newer OAK-D
* added lensPosition setter
* Add initial implementation of IMU node: acceleration and gyro at 500hz
* Added lens position to eepromData
* fixed width and height order
* added stereoRectification getters
* changed device side commit id
* updated shared:
* Added device info getter
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* local commit
* Delete calib_data2.json
* changed storeCalibration to flashEepropm
* modified extrinsics setters
* modified device side config
* clang-tidy 2
* clang-tidy
* changed commit id and rebased
* merged with develop
* updated depthai-shared
* updated shared link
* fixed getIntrinsics bug and Added Device commit id
* added docstrings and cameraType
* added more functions test in calibration_reader
* added extrinsics getter functions WIP
* Added eeprom reader and an example
* added calibration_stereo example
* added setters
* WIP calibration store example
* added constructor and fetchers headers
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* updated shared
* updated shared
* Contributors: CsabaGergely, Erik, Sachin, Sachin Guruswamy, SzabolcsGergely, TheMarpe, csaba-luxonis, saching13, szabi-luxonis

2.4.0 (2021-05-24)
------------------
* Merge pull request `#140 <https://github.com/luxonis/depthai-core/issues/140>`_ from luxonis/develop
  Release v2.4.0
* Bump version to 2.4.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#139 <https://github.com/luxonis/depthai-core/issues/139>`_ from luxonis/stereo_fixes_2
  Stereo fixes 2
* Comment out for now ImgFrame excess data warning,
  doesn't build on Windows
* Update FW, update `setRectifyMirrorFrame` functionaliy/description
* Merge remote-tracking branch 'origin/develop' into stereo_fixes_2
* Merge pull request `#135 <https://github.com/luxonis/depthai-core/issues/135>`_ from luxonis/ov9282_over_exposure_fix
  OV9282: fix over-exposure outdoors, in sunlight
* Merge remote-tracking branch 'origin/develop' into ov9282_over_exposure_fix
* Merge pull request `#138 <https://github.com/luxonis/depthai-core/issues/138>`_ from luxonis/usb_crash_mitigation
  Update FW with fix for random crashes (kernel crash on RPI/jetson)
* Revert RPC mutex lock; it's reported that has issues on Windows
* Update FW
* MonoCamera: add `raw` output. Update FW: OV9282 min autoexposure 20us
* Update FW
* Merge remote-tracking branch 'origin/develop' into ov9282_over_exposure_fix
* Update FW with fix for random crashes (kernel crash on RPI/jetson)
* Merge pull request `#110 <https://github.com/luxonis/depthai-core/issues/110>`_ from luxonis/renamed_and_new_examples
  Synchronize cpp examples with python
* ImgFrame CV conversion: more verbose about size mismatch
* Update FW: stereo fixes: LR-check flip, `depth` align to RGB
* Synch
* Sync cpp examples with latest python
* Merge remote-tracking branch 'origin/develop' into HEAD
* Hotfix - Lossless encoding
* OV9282: fix over-exposure outdoors, in sunlight
* Remove unnecessary libraries, improving the code
* Merge branch 'usb_speed' into develop
* Fixed style
* Added printing helpers and UsbSpeed example
* Added getUsbSpeed
* Merge pull request `#133 <https://github.com/luxonis/depthai-core/issues/133>`_ from luxonis/getMaxDisparity
  Fixed getMaxDisparity calculation
* Fixed getMaxDisparity calculation
* Merge pull request `#132 <https://github.com/luxonis/depthai-core/issues/132>`_ from luxonis/getMaxDisparity
  stereo node getMaxDisparity()
* removed void from the function's arguments
* fixes for the PR
* added a function in stereo node that returns the maxDisparity. Also changed stereo_example.cpp to use this new function
* Merge pull request `#131 <https://github.com/luxonis/depthai-core/issues/131>`_ from luxonis/camera_custom_tuning
  Camera custom tuning
* Comments
* Fix createDirectory for windows
* Renamed rgb_depth_aligned_example to rgb_depth_aligned
* Fixing errors
* Remove duplicated example
* Merge remote-tracking branch 'origin/develop' into HEAD
* Rename
* Remove redundant in/out flags from ifstream/ofstream across codebase
* Fix formatting, fix a merge issue
* Merge remote-tracking branch 'origin/develop' into camera_custom_tuning
* Renamed examples
* Comments
* Remove whitespaces
* Disable median filter to avoid warning
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Remove numbers and some optimization
* Resolved some warnings
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Replace RawImgFrame with ImgFrame
* Optimization, comments.
* Pipeline: add custom camera tuning blob option
* Optimization, comments
* Add video mobilenet and fix some others
* Add rgb encoding mobilenet
* Upgraded rgb mobilenet example
* Add rgb mobilenet 4k example
* Add encoding max limit example
* Add rgb encoding mono mobilenet with depth
* Synch
* Add mono depth mobilenet example
* Add mono mobilenet cpp example
* Add rgb and mono full resolution saver
* Add depth crop control
* Add mono camera control and fixed encodings
* Add rgb mono encoding example to cpp
* Changed 04 to match with python
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Add depth preview
* Renamed files to match with python examples and added a new example
* Contributors: CsabaGergely, Erik, Erol444, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, csaba-luxonis, szabi-luxonis

2.3.0 (2021-05-04)
------------------
* Merge branch 'develop' into main
* Bump version to 2.3.0
* Merge pull request `#128 <https://github.com/luxonis/depthai-core/issues/128>`_ from luxonis/fix_lrcheck_spatial
  Fix spatial calculator output with stereo LR-check enabled
* Fix docs build
* Try fixing docs build:
  docstring of depthai.StereoDepth.disparity:6: WARNING: Bullet list ends without a blank line; unexpected unindent.
* Update FW (properly set flipping with LRcheck enabled to spatial calculator),
  update StereoDepth docs
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Update depthai-shared
* Merge branch 'readme_refactor' into develop
* Merge pull request `#120 <https://github.com/luxonis/depthai-core/issues/120>`_ from luxonis/object_tracker_improvements
  Add new field: Removed to object tracker status
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed documentation issue and inconsistencies
* Added build information and config.hpp to remove the need to specify compile definitions
* Merge pull request `#126 <https://github.com/luxonis/depthai-core/issues/126>`_ from luxonis/nn_performance_fix
  Update FW with fix for resource allocation when depth is enabled; fix…
* Move queue init after pipeline start in system information example
* Apply formatting
* Update FW with fix for resource allocation when depth is enabled; fix system_information_example
* Updated README.md
* Applied formatting
* Merge branch 'device_improvements' into develop
* Merge branch 'main' into develop
* Updated device side with 'getConnectedCameras'
* Fixed sanitizers for examples
* Removed deprecated functions from examples
* Deprecated 'startPipeline()'
* Merge pull request `#82 <https://github.com/luxonis/depthai-core/issues/82>`_ from luxonis/stereo_fixes
  Fixes and improvements for StereoDepth, ColorCamera
* Update FW, fix CI build: depthai-shared PR merged
* Update FW: fix ImageManip U16 crop (for depth/subpixel disparity)
  Update shared: stereo_fixes merged
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW: fix still capture with scaling, add FPS capping (with warnings)
* clangformat cleanup
* Address review comments. Note:
  The change in discussion (--parallel 8 in Readme) was already cherry-picked to develop (so no longer appears on this PR)
* Add new field: Removed to object tracker status
* WIP: Decouple pipeline from Device
* fixed getting size of video/still when setting isp scaling
* Added a try catch for callbacks for better error messages
* Merge pull request `#113 <https://github.com/luxonis/depthai-core/issues/113>`_ from luxonis/custom_binary_env
  Capability to specify a custom device binary
* Added capability to specify custom device binary
* spatial_object_tracker example: remove deprecated setOutputDepth
* stereo_example: rectified flip no longer needed with LR-check on,
  don't link depth in pipeline if not used, and other cleanup.
  Update FW: ispScale factors simplification done on device, other bugfixes
* Merge pull request `#105 <https://github.com/luxonis/depthai-core/issues/105>`_ from luxonis/docs_fix_disparity_range
  fixed extended disparity range documentation
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#112 <https://github.com/luxonis/depthai-core/issues/112>`_ from luxonis/datainputqueue_nullptr_check
  DataInputQueue nullptr check
* Added nullptr check to DataInputQueue::send
* Merge pull request `#111 <https://github.com/luxonis/depthai-core/issues/111>`_ from luxonis/hotfix_stereo_confidence_thr
  StereoDepth: fix confidence threshold configuration
* Update FW: hotfix for stereo confidence threshold setting,
  it was overwritten to 200
* Updated FW - MobileNet parsing bugfix
* Merge pull request `#108 <https://github.com/luxonis/depthai-core/issues/108>`_ from luxonis/videnc_fixes
  Video encoder fixes
* Fix wrongly set bitrate
* Merge remote-tracking branch 'diablodale/fix71_various_code_warnings' into develop
* Added default CMAKE_BUILD_TYPE
* fixed extended disparity range documentation
* Update FW: optimize depth align, make it work with subpixel/U16
  (still not optimized)
* Cleanup, remove some unused variables
* README build snippets: limit `cmake --parallel` to 8
* GitHub CI: limit cmake --parallel to 8 threads,
  to prevent an out-of-memory situation due to too many threads created
* Examples: remove deprecated API setOutputDepth/Rectified
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Fix conversion of YUV420p frames to OpenCV BGR,
  the chroma planes were swapped
* Add rgb_depth_aligned example
* CameraControl: add ranges for extra controls,
  remove non-implemented setNoiseReductionStrength.
  Updated FW: all initial controls can be applied for Mono/ColorCamera
  (no longer limited to focus settings for Color)
* Address review comments:
  - add `isp` and `raw` to ColorCamera list of outputs, add docstrings
  - overloaded `setIspScale`, with tuple inputs as options
  - also overloaded `setPreviewSize`, `setVideoSize` and `setStillSize` with tuple inputs
* Update FW: disparity (U8) aligning to RGB works.
  TODO depth and subpixel (U16)
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* fix narrowing, clangformat, mutex lock, VideoEncoder::get/setFrameRate to float
* StereoDepth: remove for now 'setBaselineOverrideCm', 'setFovOverrideDegrees',
  will be refactored when the new calibration structure is integrated
* StereoDepth: add overloaded setDepthAlign(CameraBoardSocket)
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Add API to configure disparity/depth alignment: left/right/center.
  Works with LRcheck or LRcheck+Subpixel for now.
  The updated FW also fixes some crashes with LRcheck mode enabled
* Update FW: stereo fixes, stereo/ColorCamera improvements
* `make clangformat`
* ColorCamera: add API to get 'isp' scaled output size
* StereoDepth: deprecate setOutputDepth/Rectified
* ColorCamera: add setIspScale/setIspScaleFull API
* ColorCamera: add `isp`, `raw` outputs
* Contributors: Dale Phurrough, Erik, Erol444, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.2.1 (2021-04-13)
------------------
* Merge pull request `#107 <https://github.com/luxonis/depthai-core/issues/107>`_ from luxonis/develop
  Release 2.2.1
* Bump version to 2.2.1
* Merge pull request `#106 <https://github.com/luxonis/depthai-core/issues/106>`_ from luxonis/spatial_data_extension
  SpatialCalculator fixes
* Merge remote-tracking branch 'origin/main' into HEAD
* Update FW with bugfixes for spatial calculator
* Merge remote-tracking branch 'origin/develop' into HEAD
* Update FW
* Update SpatialCalculator data output with a new field: depthAveragePixelCount
* Contributors: SzabolcsGergely, szabi-luxonis

2.2.0 (2021-04-12)
------------------
* Merge pull request `#104 <https://github.com/luxonis/depthai-core/issues/104>`_ from luxonis/develop
  Release 2.2.0
* README build snippets: limit `cmake --parallel` to 8
* GitHub CI: limit cmake --parallel to 8 threads,
  to prevent an out-of-memory situation due to too many threads created
* Fix MacOS CI builds
* Bump version to 2.2.0
* Merge pull request `#103 <https://github.com/luxonis/depthai-core/issues/103>`_ from luxonis/develop_main_merge
  Develop-main merge
* Update gitignore with git generated files on merge conflict
* Merge remote-tracking branch 'origin/main' into develop
* Update FW: camera_init_fixes. Changes:
  - IMX378/477: increase reset low time to 20ms
  - OV9282: increase reset low time to 10ms, wait after 5ms
  - OV9282: allow using modules with I2C addr 0x20
  - report errors if color/mono cameras are not detected
* Merge branch 'videoencoder_lossless' into develop
* Added lossless jpeg encoding and some improvements
* Merge pull request `#92 <https://github.com/luxonis/depthai-core/issues/92>`_ from luxonis/object_tracker
  Integrated Intel's object tracker, added spatial object tracker
* Update FW
* Update FW, submodules
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add support for tracker on full frame
* Addresses CMake 3.20 regression in parsing '--parallel' ('-j') option
* Merge branch 'cmake_regression_workaround' into develop
* Merge branch 'opencv_version_requirement' into develop
* Addresses CMake 3.20 regression in parsing '--parallel' ('-j') option
* Explicitly specify OpenCV 4 version
* Add ID assigment policy for object tracker
* Merge remote-tracking branch 'origin/develop' into HEAD
* Style changes
* Revert globbing
* Hotfix for bounding box mapping
* Merge pull request `#93 <https://github.com/luxonis/depthai-core/issues/93>`_ from luxonis/openvino_2021.3_support
  Add OpenVino 2021.3 support
* Chanhe default OpenVino version to 2021.3; update FW
* Set default OpenVino version to 2020.3
* Add OpenVino 2021.3 support
* Add Intel's object tracker + spatial object tracker
* Merge branch 'main' into develop
* Contributors: Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.1.0 (2021-03-23)
------------------
* Hotfix - added setBitrateKbps and fixed function description
* Merge branch 'develop' into main
* Version bump to 2.1.0
* Merge branch 'documentation_improvements' into develop
* Improved some parts of documentation
* Merge branch 'invalid_device_info_fix' into develop
* Merge pull request `#88 <https://github.com/luxonis/depthai-core/issues/88>`_ from luxonis/camera_driver_fix
  Gen2 Camera driver fix
* Fix stability issues WRT camera driver
* Added a fix for passing an invalid deviceInfo
* Merge pull request `#86 <https://github.com/luxonis/depthai-core/issues/86>`_ from luxonis/gen2-spatial-yolo-example
  Add gen2 spatial yolo example
* Merge remote-tracking branch 'origin/develop' into gen2-spatial-yolo-example
* Merge pull request `#87 <https://github.com/luxonis/depthai-core/issues/87>`_ from luxonis/develop-main-merge
  Merge main into develop
* Merge remote-tracking branch 'origin/main' into develop
* Change to getCvFrame
* Add tiny-yolo-v3 and v4 examples
* Updated FW - fixes depth calculator issues on devices without calibration
* Merge branch 'install_and_integration' into develop
* Merge pull request `#83 <https://github.com/luxonis/depthai-core/issues/83>`_ from luxonis/gen2_spatial_detection_network
  Gen2: Add spatial detection network and spatial location calculator
* Update submodules before merge
* Add property aliases
* Update FW with input sanitization for spatial calculator
* Change Rect type to match OpenCV's
* Reverted testing bump of minimum CMake version
* Added EXPORT_NAME property to targets
* Added examples and tests as part of build process
* Added capability to import build directory
* Added clangformat to tests and examples
* Fixed inconsistent usage of CMAKE_INSTALL_LIBDIR
* Fixed BUILD_SHARED_LIBS usage and other variable naming
* Added integration CI tests
* Update FW with rgb-depth sync
* Modify dependency installation and integration test
* modified cmake to fix lib install issues
* Handled warning when not added as a subdirectory
* Add passthrougDepth; rename passthroughRoi to boundingBoxMapping; add missing members to getOutput for NN nodes
* Update FW
* Rename roi to ROI
* Add documentation; rename SpatialLocationCalculatorDataOut to SpatialLocations
* Revert mobilenet_device_side_decoding_example to original
* Merge remote-tracking branch 'origin/develop' into gen2_spatial_detection_network
* Rename DepthCalculator to SpatialLocationCalculator
* Update FW with rectified fixes in stereo
* Merge pull request `#78 <https://github.com/luxonis/depthai-core/issues/78>`_ from luxonis/gen2-tests-fix
  Gen2 tests fix
* Fix gen2 unit tests
* Fix build error from PR `#67 <https://github.com/luxonis/depthai-core/issues/67>`_
* Throw value error if queue size 0 is specified
* Add separate data type for spatial image detections
* Set up DetectionNetwork, SpatialNetwork properties polymorphycally
* Rename DetectionNetworkDepth to SpatialDetectionNetwork
* Code refactorization
* Update cpp examples
* Update FW
* Update FW with fix for stereo cams
* Update FW; improve example for depth calculator node
* Update FW/shared: formal switch to develop branch
* Update FW with fix
* Update FW
* Update formatting
* Fix build error from PR `#67 <https://github.com/luxonis/depthai-core/issues/67>`_
* Merge remote-tracking branch 'origin/develop' into WIP-gen2-detection-3d
* WIP: add first example of 3d detections
* Merge pull request `#67 <https://github.com/luxonis/depthai-core/issues/67>`_ from luxonis/gen2_yolov4_tiny_demo
  add (demo, test) added demo code and the test
* fix (yolo3/4 demo) fixed readability
* fix (yolov3/4 demo) fixed formatting for pr
* add (demo, test) added demo code and the test
* Fixed `#70 <https://github.com/luxonis/depthai-core/issues/70>`_ - Git check not using correct working directory
* Add dynamic ROI config
* Update FW
* Add support for FP16 depth
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Update FW: add OV9282 Strobe output
* Add initial depth calculator node
* Contributors: Erol444, Sachin, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

2.0.0 (2021-03-01)
------------------

1.7.4 (2022-05-20)
------------------
* Updated ament cmake rule
* created a dummy release for noetic test
* Made ament condirtional
* Updated changelog:
* add ament package:
* Cleanup
* Added ament found condition
* CHnaged version for testing
* Added author
* Json fix (`#478 <https://github.com/luxonis/depthai-core/issues/478>`_)
  * Fixed nlohmann json < v3.9.0 compat and toolchain generation
  * turn off clang format
  Co-authored-by: Martin Peterlin <martin.peterlin7@gmail.com>
  Co-authored-by: TheMarpe <martin@luxonis.com>
* Empty-Commit
* Update package.xml
* ROS2 release test commit (`#475 <https://github.com/luxonis/depthai-core/issues/475>`_)
  * Empty-Commit
  * Change libusb versions
* Change libsub to libusb-dev
* Empty-Commit
* Updated sub modules
* Modified json in package.xml and changed the changelog"
* Added Changelog file
* Merge remote-tracking branch 'origin/main' into ros-release
* Release 2.15.4
* Update docs; removed unsupported AprilTag families
* FW: VideoEncoder: fix keyframe rate config, fix resource computations for JPEG
  (e.g: MJPEG for 4K video 30fps + MJPEG for still 12MP ~1fps)
  properly set resources used to allow
* Update FW
* Update FW; change behavior of stereo rectification based on stereo camera FOV
* Merge 'origin/poe_mtu_sysctl' into develop - `#428 <https://github.com/luxonis/depthai-core/issues/428>`_
  Improve PoE throughput and latency for some usecases
* Updatedf package xml, cmake list to include json from system install
* Update XLink to set TCP_NODELAY, reducing latency
* Merge 'origin/develop' into poe_mtu_sysctl
* Merge branch 'eeprom_version_v7' into develop
* Merge branch 'develop' into eeprom_version_v7
* Merge branch 'json_compat' into develop
* Lowered minimum required nlohmann json version to 3.6.0
* WIP package.xml
* Set RGB aligned depth output to match mono camera
* Merge 'ov7251_configurable_fps' into develop - `#455 <https://github.com/luxonis/depthai-core/issues/455>`_
* Update FW: fix overriding useHomographyRectification behaviour specified in docs when custom mesh is provided
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#459 <https://github.com/luxonis/depthai-core/issues/459>`_ from diablodale/fix458-cmaketest-flags
  reduce num conforming tests; add missing _conforming test suffix
* reduce tests for MSVC conforming preprocessor
  - drastically reduce number of tests run for
  MSVC conforming preprocessor
  https://github.com/luxonis/depthai-core/pull/459#issuecomment-1108649206
  - add option to test harness that indicates
  when a test is run with the MSVC conforming preprocessor
* Updated flashing permissions
* Fix RGB alignment remapping when configured color camera resolution is different from calibration one
* Updated Bootloader to v0.0.18
* Updated FW with device EEPROM handling fixes
* strengthen test for device construct+q+frame
* Updated bootloader with PCIe internal clock fixes
* Added capability to create CalibrationHandler from json
* Fixed factory reset functionality and exposed more functions
* Updated BL with more build information and new EEPROM data support
* Updated EEPROM and added another level of permissions
* add missing _conforming suffix to tests cmake
  - fixes `luxonis/depthai-core#458 <https://github.com/luxonis/depthai-core/issues/458>`_
* Merge pull request `#457 <https://github.com/luxonis/depthai-core/issues/457>`_ from luxonis/rgb_alignment
  Enable RGB alignment for spatial detection examples
* Enable RGB alignment for spatial detection examples
* Merge pull request `#454 <https://github.com/luxonis/depthai-core/issues/454>`_ from diablodale/test-device-queues1
  test case for Device constructor not calling tryStartPipeline()
* Add explicit documentation about loadMesh behavior; specify that only the first 8 distortion coefficients are used
* test case for Device constructor not tryStartPipeline()
  - catch bug and prevent regression as discussed
  https://github.com/luxonis/depthai-core/commit/7257b95ecfb8dcb77c075e196ac774cc05cb8bc6#commitcomment-71730879
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#456 <https://github.com/luxonis/depthai-core/issues/456>`_ from luxonis/macos_ci_test
  Fix failing CI for MacOS
* Extend useHomographyRectification documentation with more details
* Update FW: configurable FPS for OV7251: max 99 for 480p, 117 for 400p
* Remove brew update
* Added bindings and support for new EEPROM version
* WIP - modify behavior to be backwards compatible and add checks if calibration is available
* Added additional EEPROM functionality
* Applied formatting
* Merge branch 'main' into develop
* Bump version to 2.15.3
* Merge branch 'release_2.15.3' into main
* Clarify docs for homography rectification default behavior
* Merge pull request `#437 <https://github.com/luxonis/depthai-core/issues/437>`_ from luxonis/warp_mesh_on_device
  Add on-device mesh generator for Stereo
* Disable mesh rectification by default; fix error reporting when RGB alignment is enabled and left-right check disabled
* Fix styling
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'serialization_type' into develop
* Fixed incorrect Device constructors not starting the pipeline and creating queues
* Fixed device Clock.now in Script node to match messages timestamps
* Modifed serializeToJson to create a json object instead
* Added Clock.now bindings on device
* Added capability to serialize pipeline to json
* Merge pull request `#424 <https://github.com/luxonis/depthai-core/issues/424>`_ from luxonis/bmi270_support
  IMU: Bmi270 support
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#449 <https://github.com/luxonis/depthai-core/issues/449>`_ from luxonis/openvino_no_blob
  Openvino: Fix error reporting when blob is not set
* Removed DEPTHAI_NODISCARD for docs generation
* Updated libnop  (`#448 <https://github.com/luxonis/depthai-core/issues/448>`_)
  * Updated libnop with C++20 fixes and added fs test targeting C++20
  * Added a guard for non-existent tests
  * Modified tests to not require higher CMake version
* Fix openvino get version
* Openvino: Fix error reporting when blob is not set
* Removed deprecated StereoDepth API
* new class `dai::Path` for APIs that accept path/filenames (`#384 <https://github.com/luxonis/depthai-core/issues/384>`_)
  * initial dai::Path and test cases
  - fixes `luxonis/depthai-core#352 <https://github.com/luxonis/depthai-core/issues/352>`_
  * move codecvt from header -> cpp
  * add Path::string() and u8string()
  - to enable display/log of Path
  * fmt for dai::Path; NN::setBlobPath(dai::Path)
  * dia::path throws like std::fs::path
  * c++17, pub/pvt header, test cmake c++ std level
  - enable c++17 std::filesystem support and test cases
  - split header into public/private parts
  - cmake for test cases now supports optional
  c++ standard level param
  * verify c++ std compiler support for tests
  - add COMPILER_SUPPORTS_CXX{14,17,20,23} vars
  to Flags.cmake and can be used everywhere
  * add dai::Path::empty()
  * add dai::Path to Device, DeviceBase, Resources
  - simplify Device, DeviceBase constructors by delegating
  - add is_same<> template on constructors with bool param to
  prevent implicit convert of almost everything to bool
  - make two DeviceInfo constructors explicit to prevent their use in
  implicit conversion
  - relevant test cases
  - fix minor throw text bugs
  * fix Device usb2Mode sigs, add test case
  * add dai::Path to CalibrationHandler
  * minor refactor dai::Path
  * enable 2 Calibration+1 Bootloader example
  * add dai::Path to DeviceBootloader, XLinkConnection
  - plus test cases
  * add dai::Path to Pipeline, StereoDepth, AssetManager
  - plus test cases
  * add dai::Path to dai::Script + test cases
  * linux fixes for test cases, and c++14 type_traits
  * add doxygen to dai::Path
  * detect compiler c++ std level and update cmake
  * fix preprocessor flag for tests on MSVC
  - fixes luxonis/`depthai-core/issues#408 <https://github.com/depthai-core/issues/issues/408>`_
  * partial dai::Path support for c++20 utf-8
  - unable to fully test due to bug `#407 <https://github.com/luxonis/depthai-core/issues/407>`_
  * add windows header define WIN32_LEAN_AND_MEAN
  * rename macro to DEPTHAI_NODISCARD
  - review feedback
* Release v2.15.2
* Merge pull request `#439 <https://github.com/luxonis/depthai-core/issues/439>`_ from 0xMihir/main
  Bump Hunter version
* chore: bump Hunter version
  Adds support for MSVC 1931
  Using 0.24.0 doesn't work because there's a duplication error in the nlohmann/json library hunter config file
* std::exchange needs <utility> to be included (`#435 <https://github.com/luxonis/depthai-core/issues/435>`_)
  * std::exchange needs <utility> to be included
  Without <utility> it is gives "error: ‘exchange’ is not a member of ‘std’" errors.
  Ref : https://en.cppreference.com/w/cpp/utility/exchange
  * clang format fix
* Merge branch 'main' into develop
* Fixes `#436 <https://github.com/luxonis/depthai-core/issues/436>`_ - removes temporary warning log in StereoDepth
* Apply style
* Add on-device mesh generator
* Updated XLink - removed dependency on pthread_getname_np
* Merge branch 'device_is_closed_fix' into develop
* Fixed XLink issue with not erroring on write failures
* Openvino: improve error logging for out of memory cases
* Modified to store fisheye Camera model
  * Add getter for distortion model in CalibrationHandler
  * Pad distortion coefficients with 0's if there's less than 14
  * Only return first four distortion coefficients for Fisheye distortion
* Merge pull request `#430 <https://github.com/luxonis/depthai-core/issues/430>`_ from luxonis/custom_depth_unit
  Customizable depth unit
* Change metre to meter
* Change millimetre to depth unit where it's applicable in docs
* Add setter/getter utility function for depth unit
* Add customizable depth unit
* Update FW: improve PoE throughput and latency (set net.inet.tcp.delayed_ack=0),
  add config for MTU (not advised to change for now) and other sysctl params
* Merge pull request `#427 <https://github.com/luxonis/depthai-core/issues/427>`_ from luxonis/warp_improvements
  Warp engine improvements for RGB alignment/stereo rectification
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Warp engine improvements for RGB alignment/stereo rectification
* Merge branch 'release_2.15.1' into main
* Merge pull request `#426 <https://github.com/luxonis/depthai-core/issues/426>`_ from luxonis/focal_length_from_calib
  Use focal length from calibration by default for Stereo node
* Merge pull request `#422 <https://github.com/luxonis/depthai-core/issues/422>`_ from luxonis/fix_calib_rgb_translation
  Calib data RGB spec translation fix for some CM3/CM4 boards
* Set focal length from calibration as default for stereo node
* Update FW: fix StereoDepth crash with missing EEPROM, report error if missing,
  do not rotate RGB (AUTO orientation) on OAK-D(-Lite) if EEPROM is missing
* Merge 'origin/develop' into fix_calib_rgb_translation
* Bump version to 2.15.1
* Merge pull request `#421 <https://github.com/luxonis/depthai-core/issues/421>`_ from luxonis/confidence_map_alignment_opt
  Optimizing the confidence map alignment
* Initial BMI270 support
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk: Merge confidence_map_alignment_opt into develop
* Merge branch 'xlink_race_improvements' into develop
* Update FW: patch EEPROM data for incorrectly programmed RGB spec translation,
  for manually ran calibration on CM3/CM4 boards.
  Was leading to wrong RGB-depth alignment
* Update FW/mdk - Optimizing the confidence map alignment(CleanUp - remove global variables)
* Adding a new StereoDepth example rgb_depth_confidence_aligned.cpp for aligning the rgb, depth and confidence frames
* Updated XLink with only required changes
* Update FW/mdk - Optimizing the confidence map alignment(CleanUp logs)
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk - Optimizing the confidence map alignment
* Updated XLink with some race fixes and other improvements
* Updated FW with thermal protection mechanism
* Merge pull request `#398 <https://github.com/luxonis/depthai-core/issues/398>`_ from diablodale/fix390-callback-moves
  move semantics with DataOutputQueue::addCallback()
* Merge pull request `#417 <https://github.com/luxonis/depthai-core/issues/417>`_ from ibaiGorordo/patch-1
  Fix Readme links
* Fix Readme links
* Merge remote-tracking branch 'origin/develop' into confidence_map_alignment_opt
* Update FW/mdk - Optimizing the confidence map alignment
* Fix default temporal/spatial filter values when subpixel is enabled
* Merge pull request `#403 <https://github.com/luxonis/depthai-core/issues/403>`_ from diablodale/fix314-cmpjson
  verify device json in example script_json_comm
* Merge pull request `#409 <https://github.com/luxonis/depthai-core/issues/409>`_ from diablodale/fix408-conformflag
  correct test cmake to add preproc conform flag
* correct test cmake to add preproc conform flag
  - fixes `luxonis/depthai-core#408 <https://github.com/luxonis/depthai-core/issues/408>`_
* Updated release template
* Merge branch 'main' into develop
* Merge branch 'release_2.15.0' into main
* Bump version to 2.15.0
* Merge pull request `#287 <https://github.com/luxonis/depthai-core/issues/287>`_ from luxonis/oak-d-pro_develop
  Support for OAK-D Pro
* Merge 'origin/develop' into oak-d-pro_develop
* `getIrDrivers` -> vector of tuples (driverName, i2cBus, i2cAddr). Return if setIrDot/Flood succeeded
* Merge pull request `#401 <https://github.com/luxonis/depthai-core/issues/401>`_ from luxonis/openvino_version_deprecation
  Deprecate OpenVINO 2020.4, 2021.1, 2021.2, 2021.3
* Merge 'origin/develop' into oak-d-pro_develop
* Merge pull request `#405 <https://github.com/luxonis/depthai-core/issues/405>`_ from luxonis/cfg_fps_lite
  Configurable RGB FPS on Lite devices, fix RGB orientation
* Updated test suite for new env var naming
* Renamed env variable and updated README regarding testing
* Update FW: RGB orientation fix for OAK-1 (new versions) and OAK-1-Lite
* Merge 'origin/develop' into cfg_fps_lite
* verify device json in example script_json_comm
  - related to https://github.com/luxonis/depthai-core/issues/314#issuecomment-1007463313
* Merge 'origin/develop' into oak-d-pro_develop
* IR driver: remove raw register access API
* IR driver RPC: add `getIrDrivers()`, update description with limits
* Deprecate OpenVINO 2020.4, 2021.1, 2021.2, 2022.3
* Merge pull request `#389 <https://github.com/luxonis/depthai-core/issues/389>`_ from luxonis/imu_device_ts
  Add device monotonic timestamp to IMU reports
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add dot-projector/flood-illuminator brightness control
* Merge 'origin/develop' into oak-d-pro_develop
* Revert "USB bootloader with support for missing 24MHz USB osc (OAK-D Pro-PoE initial flashing)"
  This reverts commit 96691b9a8295c54bea1c04c20bc4ad60091ca536.
* Update FW: ColorCamera memory optimization when `still` is not connected,
  properly handle width-multiple-of-32 (only needed for VideoEncoder), don't enforce it on `video` by default,
  allow `still` size smaller than `video`
* Modify IMU example: GYRO at 400 hz to avoid spikes
* Update shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#387 <https://github.com/luxonis/depthai-core/issues/387>`_ from luxonis/subpixel_after_lr_check
  Do subpixel interpolation once when LR-check is enabled
* Fix formatting
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated shared
* Fixed crash when device watchdog is disabled
* Merge pull request `#394 <https://github.com/luxonis/depthai-core/issues/394>`_ from luxonis/skip_invalid_devices
  Invalid device warnings suppression
* Suppressed redundant warnings for invalid devices
* Merge pull request `#382 <https://github.com/luxonis/depthai-core/issues/382>`_ from diablodale/fix300-errorname
  skip devices named "<error>" in device search
* Merge branch 'testing_ci' into develop
* Merge branch 'fw_stability_fix' into develop
* Fixes a stability issue bug that affected PoE devices mostly
* Added testing CI
* move semantics in DataOutputQueue::addCallback()
  - fixes `luxonis/depthai-core#390 <https://github.com/luxonis/depthai-core/issues/390>`_
  - minor move tweaks using callbacks
* Add on-device python bindings
* Add device monotonic timestamp to IMU reports
* Update FW, fix for custom alinment subpixel interpolation
* Do subpixel interpolation after LR-check; improves performance to 30fps@800p
* Merge pull request `#378 <https://github.com/luxonis/depthai-core/issues/378>`_ from diablodale/fix366-qsize0
  enable queues of size=0 which only do callbacks
* Merge branch 'fix334-interopt-dll' into develop
* Merge pull request `#361 <https://github.com/luxonis/depthai-core/issues/361>`_ from luxonis/rgbd_depth_align_optimize
  Rgbd-depth alignment optimization
* Update mdk - RGB-depth alignment
* Merge remote-tracking branch 'origin/develop' into rgbd_depth_align_optimize
* enable queues of size=0 which only do callbacks
  - minor optimize LockingQueue
  - fixes `luxonis/depthai-core#366 <https://github.com/luxonis/depthai-core/issues/366>`_
* Modified env variable logging verbosity
* log envvar values at TRACE level only (`#381 <https://github.com/luxonis/depthai-core/issues/381>`_)
  - fixes `luxonis/depthai-core#380 <https://github.com/luxonis/depthai-core/issues/380>`_
* Update FW - adding the RGB scaling factor for the RGB-depth center alignment
* skip devices named "<error>" in device search
  - partial fix `luxonis/depthai-core#300 <https://github.com/luxonis/depthai-core/issues/300>`_
* minor cleanup examples and tests
  - most fixes are signed/unsigned comparison corrections
* fix Win MSVC cmake INTERPROCEDURAL_OPTIMIZATION
  - workaround MSVC incompat BUILD_SHARED_LIBS +
  WINDOWS_EXPORT_ALL_SYMBOLS +
  INTERPROCEDURAL_OPTIMIZATION
  - fixes `luxonis/depthai-core#334 <https://github.com/luxonis/depthai-core/issues/334>`_
  - includes pr feedback
* Merge remote-tracking branch 'origin/develop' into rgbd_depth_align_optimize
* Merge pull request `#375 <https://github.com/luxonis/depthai-core/issues/375>`_ from luxonis/swap_imu_raw_accelerometer_axis
  Swap ACCELEROMETER_RAW x and y axis to match ACCELEROMETER
* Swap ACCELEROMETER_RAW x and y axis to match ACCELEROMETER
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#374 <https://github.com/luxonis/depthai-core/issues/374>`_ from luxonis/reenable_feature_tracker_metadata
  Reenable feature tracker metadata; change default CPU to CSS for Script node
* Update shared to match FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed non-cv support ImgFrame header
* Added initial setter chaining for messages
* Merge branch 'nndata_sequence_num'
* Updated style
* Merge branch 'openvino_blob' into develop
* Change bootloader shared submodule to match develop
* Merge remote-tracking branch 'origin/develop' into HEAD
* Modified docs and type of exception being thrown
* Documented Blob fields and added blob file size check
* Merge remote-tracking branch 'jdavidberger/develop'
* Updated libnop with renamed Nil enum
* Merge branch 'msvc_traditional' into develop
* Removed the need for conforming MSVC preprocessor
* Added capability to read blob information
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed env var usage
* Fixed typo
* Added search time env variable and moved querying of env variables to happen once
* Reverted back search timings in USB protocol case
* Added means of downselecting protocol used by XLink
* Merge branch 'watchdog_protection' into develop
* Merge remote-tracking branch 'origin/poe_improvements' into develop
* Update FW with clock related tweaks for feature tracker
* Updated flash_bootloader example
* Update shared/FW w/ build fix
* Improved PoE interaction
* Merge pull request `#359 <https://github.com/luxonis/depthai-core/issues/359>`_ from luxonis/subpixel_docs_fix
  Fix subpixel fractional bits documentation
* Update FW with stereo fixes for instance number; RGB depth alignment
* Change the resolution to 720p for the RGB-depth alignment example
* Update FW: optimized RGB-depth alignment
* Fix subpixel fractional bits documentation
* Rename AprilTagData to AprilTags
* Merge pull request `#166 <https://github.com/luxonis/depthai-core/issues/166>`_ from luxonis/gen2_apriltag
  Add apriltag support
* Update FW to latest develop
* Add example for advanced settings
* Update FW/shared/examples
* Update docs/FW
* Expose all config options for april tag detection algorithm
* Update FW/shared with fixes for TAG_CIR49H12 TAG_CUST48H12 TAG_STAND41H12 TAG_STAND52H13
* Update FW/shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Apply formatting
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#353 <https://github.com/luxonis/depthai-core/issues/353>`_ from luxonis/depth_docs
  Added some clarifications to depth docs
* Added some clarifications to depth docs
* Update FW with object tracker KCF fixes
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Specify minimum nlohmann version
* Merge pull request `#350 <https://github.com/luxonis/depthai-core/issues/350>`_ from luxonis/focal_from_intrinsics
  Use focal length from calibration intrinsics for fisheye cameras
* Update FW with fisheye lens detection and override option: setFocalLengthFromCalibration
* Get focal length from calibration intrinsics
* workaround bootloader-shared var init bug (`#347 <https://github.com/luxonis/depthai-core/issues/347>`_)
  - manually init class vars as workaround for
  https://github.com/luxonis/depthai-bootloader-shared/issues/4
* Update XLink
* Updated .gitmodules
* Update FW/XLink to latest
* Add openvino 2021.4.2 support
* Revert XLink to latest develop
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated .gitmodules
* Merge branch 'board_config' into develop
* Updated tidy and shared
* Updated libnop library (`#344 <https://github.com/luxonis/depthai-core/issues/344>`_)
* Updated XLink library
* Added watchdog protection in core and XLink
* Merge pull request `#335 <https://github.com/luxonis/depthai-core/issues/335>`_ from luxonis/confidence_map_rgb_alignment
  Add support for confidence map RGB alignment; fix bounding box remapping for RGB aligned depth frames
* Update FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* PipelineImpl::create() use make_shared, not raw new() (`#341 <https://github.com/luxonis/depthai-core/issues/341>`_)
  - fixes `luxonis/depthai-core#340 <https://github.com/luxonis/depthai-core/issues/340>`_
* Updated FW to sync with shared changes
* Updated shared
* Fix StereoDepth::setDefaultProfilePreset
* Added NN examples to tests, added utility conversion from fp16
* Added NN examples
* Re-enable feature tracker metadata
* Add support for confidence map RGB alignment; fix bounding box remapping for RGB aligned depth frames
* Merge pull request `#333 <https://github.com/luxonis/depthai-core/issues/333>`_ from diablodale/fix284-unreachable
  Remove unreachable code in DataQueue
* remove unreachable code in DataQueue
  - fixes `luxonis/depthai-core#284 <https://github.com/luxonis/depthai-core/issues/284>`_
* add const ADatatype::getRaw(), Buffer::getData(); add copy+move Buffer::setData() (`#331 <https://github.com/luxonis/depthai-core/issues/331>`_)
  - fixes `luxonis/depthai-core#330 <https://github.com/luxonis/depthai-core/issues/330>`_
* Merge pull request `#332 <https://github.com/luxonis/depthai-core/issues/332>`_ from luxonis/typos_fix
  Updated depthai-core with typo fixes
* Updated depthai-shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated multiple devices test
* XLink library fixes for multiple devices case (`#329 <https://github.com/luxonis/depthai-core/issues/329>`_)
* Merge branch 'resources_lazy_load_tsan' into develop
* Optimized condition_variable usage
* Configurable FPS for IMX214: 0.735 .. 35 for 1080p, 1.4 .. 30 (28.5 actually, TODO) for 4K/12MP/13MP
* Update FW with latest apriltag
* Merge remote-tracking branch 'origin/develop' into HEAD
* Moved over to a condition variable to signify end of lazy loading
* Merge remote-tracking branch 'origin/main' into HEAD
* Release v2.14.1
* Fix regression for ColorCamera, StereoDepth
* StereoDepth: check if input/output messages are connected
* Fix regression for ColorCamera, StereoDepth
* StereoDepth: check if input/output messages are connected
* Fix compilation error w/ clang 13
* Updated XLink
* Merge remote-tracking branch 'diablodale/fix257-move-owner-threads' into develop
* Added initial BoardConfig
* fix stream+packet ownership/move; fix thread crashes
  - fix many thread/ownership issues for start/stop scenarios
  - XLinkStream::readMove() for moving packet ownership
  - fix XLinkStream move semantics
  - removed all use of XLinkStream::readRaw as often leads to
  memory violations and/or memory leaks
  - deprecate all XLinkStream::readRaw...() APIs
  - fixes `luxonis/depthai-core#257 <https://github.com/luxonis/depthai-core/issues/257>`_
* Added missing throw statements
* Add spatialLocationCalculator output message to spatial detection network
* Release v2.14.0
* Fixed script json communication example
* Updated libnop
* Fixed updated Hunter usage
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Bump Hunter to add support for VS2022
* Update shared/FW
* USB bootloader with support for missing 24MHz USB osc (OAK-D Pro-PoE initial flashing)
* Merge 'origin/develop' into oak-d-pro_develop
* Merge pull request `#312 <https://github.com/luxonis/depthai-core/issues/312>`_ from luxonis/connect_timeout_override
  Override watchdog initial delay and connect/bootup timeout
* Rename env var DEPTHAI_INIT_WATCHDOG -> DEPTHAI_WATCHDOG_INITIAL_DELAY
* clangformat changes,
  <> changed to "" as it was suggesting a new-line between <> and "" includes, and then alphabetically ordered
* `DEPTHAI_INIT_WATCHDOG` env var to set initial delay [ms] for the device watchdog,
  mainly to be set to larger values for Ethernet case with network equipment that takes long to establish the link. Default: 8s for USB, 15s for ETH
* Merge 'origin/develop' into connect_timeout_override
* Fix serialization of spatial img data
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#308 <https://github.com/luxonis/depthai-core/issues/308>`_ from luxonis/json_comm_example
  Added json communication example
* Updated XLink
* Update FW; wakeup driven high
* Set pullup for IMU wakeup pin
* Added json communication example
* FeatureTracker: Add support for 4k/12MP inputs
* Fix typo: assigment -> assignment
* FW fix for Stereo HW desync when extended is enabled
* Adds rgb/depth weight slider to rgb_depth_aligned example
* Merge remote-tracking branch 'origin/main' into HEAD
* Bump Windows SDK to 10.0.18362.0 with conforming preprocessor support (`#306 <https://github.com/luxonis/depthai-core/issues/306>`_)
* Updated FW to match shared
* Added MSVC preprocessor conformance flag
* Merge pull request `#303 <https://github.com/luxonis/depthai-core/issues/303>`_ from luxonis/typos_fix
  Typos fix
* NNData serialize fix (`#305 <https://github.com/luxonis/depthai-core/issues/305>`_)
  * Adds proper TensorInfo to serialized layer
* Merge branch 'develop_refactor' into develop
* Typos fix
* Update FW with ipv6 disabled
* Updated shared
* Updated shared
* Merge branch 'develop' into develop_refactor
* Merge remote-tracking branch 'origin/develop' into HEAD
* Added incoming message parse timing to trace level debugging
* Merge pull request `#301 <https://github.com/luxonis/depthai-core/issues/301>`_ from diablodale/fix-xlink-local-install-v2
  fix 2 for xlink local cmake
* fix 2 for xlink local cmake
  - fixes `luxonis/depthai-core#272 <https://github.com/luxonis/depthai-core/issues/272>`_
  - replaces PR `#298 <https://github.com/luxonis/depthai-core/issues/298>`_
* Exposed max serialized metadata size
* Merge branch 'develop' into develop_refactor
* Merge pull request `#274 <https://github.com/luxonis/depthai-core/issues/274>`_ from luxonis/stereo_post_processing
  Added stereo post processing filters
* Update FW to latest develop
* Update FW with improved resource allocation for RGB aligment; improved error handling when out of resources
* Update shared w/ stubgen fixes
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'stubs_improvements' into develop
* Fix broken Windows CI
* Fixed XLink dependency in config mode
* Fixed exporting XLink when not using a local version
* Merge pull request `#298 <https://github.com/luxonis/depthai-core/issues/298>`_ from diablodale/fix-xlink-local-install
  fix xlink cmake install for local, shared, and static
* FW: Edge case fix for RGB aligment
* FW update: don't apply threshold filtering on confidence map
* Add depth post processing example
* Change all examples to use setDefaultProfilePreset
* Add default preset mode to StereoDepth constructor
* Add support for runtiem depth aligment mode; improve API
* fix xlink cmake install for local, shared, and static
  - fixes `luxonis/depthai-core#272 <https://github.com/luxonis/depthai-core/issues/272>`_
* Merge pull request `#297 <https://github.com/luxonis/depthai-core/issues/297>`_ from luxonis/tracker_docs
  Added possible tracker types to comment
* Updated shared
* Update FW, fix docs build
* Update FW; add default stereo presets; add configurable HW resources
* Added possible tracker types to comment
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#296 <https://github.com/luxonis/depthai-core/issues/296>`_ from diablodale/fix-264-cmake-shared-vars
  add cmake vars for local depthai-bootloader/shared
* add cmake vars for local depthai-bootloader/shared
  - fixes `luxonis/depthai-core#264 <https://github.com/luxonis/depthai-core/issues/264>`_
* Merge pull request `#295 <https://github.com/luxonis/depthai-core/issues/295>`_ from luxonis/fw_yolov5_and_stability
  FW YoloV5 support and stability updates
* Updated FW with YoloV5 support and stability improvements
* Apply thresholding filter on disparity map if depth is not enabled
* Add configurable decimation filter modes: pixel skipping/non zero median/non zero mean
* Merge branch 'depthai_clock' into develop
* Merge branch 'xlink_mingw_fix' into develop
* Add decimation filter
* Updated XLink with MinGW fixes
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add configurable number of shaves for stereo postprocessing
* Merge remote-tracking branch 'origin/develop' into HEAD
* Release v2.13.3
* Add RPC for LM3644 IR projector registers read/write on OAK-D-Pro
* Update FW: zero out uninitialized DDR memory
* Added clock
* Add spatial filter
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Update FW: fix VideoEncoder potential crash (after power-cycle),
  instability introduced in 2.13.0
* Merge pull request `#281 <https://github.com/luxonis/depthai-core/issues/281>`_ from luxonis/manual_white_balance
  Add manual white balance / color temperature camera control
* Updated XLink with a couple of fixes
* Update shared/FW: manual_white_balance merged, other fixes:
  - fixes a crash with more than 4x VideoEncoder instances, now up to around 8 should work
  - StereoDepth fix crash with RGB-depth align and missing RGB calib (calibrated with -drgb)
  - StereoDepth fix RGB alignment when running at calib resolution (OAK-D with 800_P or OAK-D-Lite)
  - an error is thrown if multiple cameras have the same socket assigned
* rgb_camera_control: add manual white balance controls: `[` `]` `B`
* setManualFocus: no need to set OFF mode, auto-handled
* CameraControl: add `setManualWhiteBalance(colorTemperatureK)`
* Release v2.13.2
* Merge remote-tracking branch 'origin/main' into HEAD
* FW fix for resource allocation issues when setRuntimeModeSwitch is used
* Applied style
* Merge branch 'develop' into main
* Merge branch 'xlink_regression_fix' into develop
* Clangformat bootloader example
* Add specle filter
* Updated XLink to fix SIGPIPE regression
* fix initialize() thread/except safety (`#277 <https://github.com/luxonis/depthai-core/issues/277>`_)
  - fixes `luxonis/depthai-core#276 <https://github.com/luxonis/depthai-core/issues/276>`_
* Initial version of temporal + thresholding filter
* Release v2.13.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Update shared/FW
* Merge pull request `#262 <https://github.com/luxonis/depthai-core/issues/262>`_ from luxonis/oak-d-lite
  Support for OAK-D-Lite
* Remove deprecated VideoEncoder frame size config in examples
* Merge 'origin/develop' into oak-d-lite
* VideoEncoder: maxBitrate following bitrate setting in FW, when 0 (default)
* VideoEncoder: deprecated setting width/height, auto-computed bitrate by default
* Update FW with xlink thread priority changes
* Update FW: openvino 2021.4.2 support
* Update firmware SDK to r17.5
* Update FW: VideoEncoder source size configured when receiving 1st frame,
  allows to run OAK-D examples (e.g configuring mono cameras to 720_P) on OAK-D-Lite without code changes
* Merge pull request `#268 <https://github.com/luxonis/depthai-core/issues/268>`_ from diablodale/fix248-trunc-2
  Correct float literals, 32/64 trunc, unref vars
* fix errant printf params in examples (`#267 <https://github.com/luxonis/depthai-core/issues/267>`_)
  - fix `luxonis/depthai-core#259 <https://github.com/luxonis/depthai-core/issues/259>`_
* enable build in vscode, custom toolchain+include (`#258 <https://github.com/luxonis/depthai-core/issues/258>`_)
  * enable build in vscode, custom toolchain+include
  - fixes `luxonis/depthai-core#246 <https://github.com/luxonis/depthai-core/issues/246>`_
  * self doc dependency options with set(cache)
* Merge pull request `#269 <https://github.com/luxonis/depthai-core/issues/269>`_ from luxonis/set_ip_example
  Added Poe set IP example
* Added poe_set_ip example
* Updated FW with scripting improvements
* correct float literals, 32/64 trunc, unref vars
  - partial fix `luxonis/depthai-core#248 <https://github.com/luxonis/depthai-core/issues/248>`_
* Fix styling
* Update FW/shared
* Merge branch 'main' into develop
* Merge branch 'win_prebuilt_fix' into main
* Cherry picked XLink macos fix
* Bump version to 2.12.1
* Fixed Windows prebuilt library
* Merge commit '18c5f8c3d4b4bb3498b515f2cb7a6a61f22db91a' into develop
* Fixed style
* Merge branch 'xlink_macos_fix' into develop
* Adds a timeout for closing an XLink connection
* Add device.getCameraSensorNames RPC call,
  can be used to differentiate between OAK-D and OAK-D-Lite. Should return:
  OAK-D     : RGB: IMX378, LEFT: OV9282, RIGHT: OV9282
  OAK-D-Lite: RGB: IMX214, LEFT: OV7251, RIGHT: OV7251
* Color/MonoCamera: handle more resolutions for OAK-D-Lite cameras:
  IMX214 (13MP) and OV7251 (480P)
* Merge pull request `#261 <https://github.com/luxonis/depthai-core/issues/261>`_ from luxonis/develop
  Release v2.12.0
* Release v2.12.0
* Merge pull request `#256 <https://github.com/luxonis/depthai-core/issues/256>`_ from luxonis/object_tracker_update
  Object tracker fixes, updates: 2 new tracking modes: KCF, short-term imageless.
* Update FW with latest improvements
* Fixes for object tracker; support for KCF and imageless short term tracking algorithms
* Merge pull request `#245 <https://github.com/luxonis/depthai-core/issues/245>`_ from luxonis/non_square_yolo_output
  Add support for non-square YOLO output
* Update FW before merge
* Update FW with error reporting for DetectionNetwork
* Add support for non-square YOLO output
* Update FW with Script node (DynamicPool) related fixes
* Merge pull request `#216 <https://github.com/luxonis/depthai-core/issues/216>`_ from luxonis/stereo_depth_fine_tuning
  Fine tune stereo depth settings
* Increase LR-check threshold to 10; disparity confidence threshold to 245 by default
* Add fine tuned stereo settings, configurable P1/P2 cost aggregation parameters
* Merge remote-tracking branch 'origin/develop' into HEAD
* Revert "Set fine tuned stereo settings"
  This reverts commit 8af5641c0e0d91d89d84bd4de8daa5aceaebc658.
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#240 <https://github.com/luxonis/depthai-core/issues/240>`_ from luxonis/extended_disparity
  Add extended disparity mode
* Update FW before merge
* Add addtional outputs to output list
* Merge remote-tracking branch 'origin/develop' into HEAD
* FW - fixed OpenVINO layer issue
* Warn if watchdog is disabled, or value overriden.
  Reason for change: env vars might get forgotten set, and not easy to spot with DEPTHAI_LEVEL=debug
* Fix strncpy build warning:
  specified bound 48 equals destination size [-Wstringop-truncation]
* Override XLink wait for bootup/connect timeouts with env vars:
  DEPTHAI_CONNECT_TIMEOUT
  DEPTHAI_BOOTUP_TIMEOUT
  (in ms)
  TODO: add in bootBootloader as well
* Updated XLink with macOS fix
* Spdlog version change (`#239 <https://github.com/luxonis/depthai-core/issues/239>`_)
  * added spdlog fix
* Add extended mode debug outputs
* Merge remote-tracking branch 'origin/develop' into extended_disparity
* StereoDepth: Add extended disparity mode
* Fixed setNumFramesPool for VideoEncoder node
* Merge pull request `#238 <https://github.com/luxonis/depthai-core/issues/238>`_ from luxonis/disparity_enc
  Added disparity encoding example
* Added disparity encoding example
* Added CMake version into CI and Ubuntu 18.04 fix (`#237 <https://github.com/luxonis/depthai-core/issues/237>`_)
  * Added CMake version into CI
  * Updated ZLIB with fixed ALIAS on imported target
  * CI - Concatenated -D arguments for old CMake version
  * Updated README instructions for CMake version 3.10
  * Fixed Windows build and ZLIB target
  * Removed old CMake build for MSVC
  * Updated -D CMake usage
* Merge pull request `#234 <https://github.com/luxonis/depthai-core/issues/234>`_ from luxonis/script_forward_frames
  Added script forward (demux) example
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'main' into develop
* Merge pull request `#236 <https://github.com/luxonis/depthai-core/issues/236>`_ from luxonis/catch_dependency_fix_new_glibc
  Update catch2 package to 2.13.7
* Update catch2 package to 2.13.7
* Added script forward (demux) example
* Restructured README.md (`#232 <https://github.com/luxonis/depthai-core/issues/232>`_)
  * Restructured README
  * Update README.md
  * Update README.md
* Fixed a node crtp issue
* Merge branch 'node_crtp' into develop_refactor
* Merge branch 'develop' into neuralnetwork_multiple_inputs
* Added CRTP to Nodes
* Merge pull request `#230 <https://github.com/luxonis/depthai-core/issues/230>`_ from luxonis/develop
  Release v2.11.1
* Bump version to 2.11.1
* Update to latest firmware/depthai-shared
* Change warning to info
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#229 <https://github.com/luxonis/depthai-core/issues/229>`_ from luxonis/fix_build_visual_studio_m_pi
  Fix build with older Visual Studio - M_PI undeclared
* `#define _USE_MATH_DEFINES` at the top of the file
  attempting to fix building with Visual Studio 15 2017:
  `error C2065: 'M_PI': undeclared identifier`
  https://discord.com/channels/790680891252932659/798284448323731456/899110756413489212
* Merge branch 'develop' into libnop_serialization
* Refactored Nodes to allow for arbitrary properties and removed issues with multiple copies
* Merge pull request `#227 <https://github.com/luxonis/depthai-core/issues/227>`_ from luxonis/examples_sorting
  Examples sorting
* Merge pull request `#228 <https://github.com/luxonis/depthai-core/issues/228>`_ from luxonis/sipp_fw_bugfixes
  Firmware sdk fixes: for ISP/Sipp filter crashes `#395 <https://github.com/luxonis/depthai-core/issues/395>`_
* Update FW before merge
* Renamed two examples
* Internal firmware sdk fixes: for ISP/Stereo/Sipp filter crashes
* Added libnop dependency and unified serialization
* Fixed CMakeLists that should have worked before as well but ok
* Moved examples out of /src folder
* Removed fromPlanarFp16() as it's not needed
* Style fix
* Added script node CPP examples
* Added examples in their corresponding folders
* Grouped tiny yolo3/4 together
* Merge branch 'develop' into main
* Updated formatting
* Fixed double promotion warning
* Bumped to v2.11.0
* Merge branch 'backward_issue_fix' into develop
* Backward - Disables use of additional stack unwinding libs
* Update FW: increase ImageManip warp max out height: 1520 -> 2560
* Windows prebuilt libraries (`#220 <https://github.com/luxonis/depthai-core/issues/220>`_)
  * Added CI to build Win64 & Win32 prebuilt libraries and upload along the release
* Merge branch 'spi_improvements' into develop
* Merge branch 'develop' into spi_improvements
* Merge branch 'develop' into neuralnetwork_multiple_inputs
* Hotfix for FW message cache coherency
* Merge pull request `#206 <https://github.com/luxonis/depthai-core/issues/206>`_ from luxonis/calib_fov_calculated
  Added getting calculated FOV from intrinsics
* Merge pull request `#212 <https://github.com/luxonis/depthai-core/issues/212>`_ from SpectacularAI/fix-extrinsic-inversions-in-calibration-handler
  Fix the inversion formula for extrinsic matrices in CalibrationHandler
* Fixed for Windows
* Fix inversion formula for extrinsic matrices in CalibrationHandler
* Fix styling
* Merge pull request `#218 <https://github.com/luxonis/depthai-core/issues/218>`_ from luxonis/stereo_confidence_map
  Add confidence map output to stereo node
* Update FW to latest develop
* Update confidence map output docs
* Add confidence map output to stereo node
* Merge pull request `#217 <https://github.com/luxonis/depthai-core/issues/217>`_ from luxonis/ppenc_fixes
  Fix still image output in RGB postprocessing
* Updated FW with SPI improvements
* Update FW to latest develop
* Fix still image output in RGB postprocessing
* Set fine tuned stereo settings
* Removed deprecated usage and added correct output for DetectionNetwork back
* Updated waitForMessage API and applied across nodes
* Fix bootloader version example
* Merge pull request `#200 <https://github.com/luxonis/depthai-core/issues/200>`_ from luxonis/stereo_fixes
  Stereo improvements, fixes for subpixel, LR-check
* Sync stereo_depth_video example
* Update FW/shared to latest develop
* Replace deprecated getMaxDisparity() function
* Handle disparity companding in getMaxDisparity
* Update FW with runtime disparity range fix
* Add getMaxDisparity() based on subpixel bits
* Add stereo node output config
* Update calibration_reader.cpp
* Added IO groups and refactored IO references
* Add debug outputs to stereo node; expose number of frame pools
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#213 <https://github.com/luxonis/depthai-core/issues/213>`_ from luxonis/spatial_calc_algo_choice
  Add option to pick spatial calculation algorithm : average,min,max of…
* Update FW/shared to latest develop
* Merge pull request `#214 <https://github.com/luxonis/depthai-core/issues/214>`_ from luxonis/flash_bl_example_fix
  flash_bootloader example fix
* Added Node Input options and some tests
* Update shared w/ CI fixes
* flash_bootloader: improve user prompts, when booted over USB / recovery mode:
  don't ask for confirmations, as if flashing is interrupted, recovery mode should still be accessible.
  Also it was a bit confusing asking to replace USB bootloader (booted as a flasher helper) with NETWORK
* Update FW to match depthai-shared
* flash_bootloader: fix flashing NETWORK bootloader (when booted over USB),
  or flashing a different bootloader type
* Set bytes per pixel for ImgFrame
* Add option to pick spatial calculation algorithm : average,min,max of selected ROI
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW with subpixel fix
* Refactor stereo depth config structure
* Update FW, enable runtime configuration of Stereo node
* Imu extrinsics (`#211 <https://github.com/luxonis/depthai-core/issues/211>`_)
  * Updated IMU extrinsics
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW with stereo confidence runtime config fix
* Updated Bootloader to 0.0.15
* Update FW with stereo performance improvements
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* FW - Updated ColorCamera 1080P resolution config
* Fixed integration issues
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'develop_embedded' into develop
* Remove rectification flipping on host, it was resolved in firmware
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Updated FW - fixed cache coherency issue
* Update FW, for depthai-shared to match with depthai-core
* Update FW: fix default camera orientation for OAK-1-PoE, was rotated
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Pipeline - number of connections improvement
* Fixed exception rethrow in DeviceBase
* Merge pull request `#207 <https://github.com/luxonis/depthai-core/issues/207>`_ from luxonis/imagemanipcfg_helper_functions
  Add ImageManipConfig helper functions
* Fixed style checks, added FormatConfig
* Added alias
* Add ImageManipConfig helper functions
* Fixed issues for the PR
* Added capability to not install signal handlers
* Added option to calculate FOV based on camera intrinsics. Added this function to calibration_reader and also refactored it so matricies are more readable
* Merge pull request `#205 <https://github.com/luxonis/depthai-core/issues/205>`_ from luxonis/calib_helper_functions
  Calib helper functions
* Fixed typo
* Style check fix
* Updated FW to allow for graceful resets
* Added helper functions to get translation vector and baseline distance
* Merge pull request `#204 <https://github.com/luxonis/depthai-core/issues/204>`_ from luxonis/extrinsics_translation_cm
  Specified that translation is in centimeters
* Specified that translation is in centimeters
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#203 <https://github.com/luxonis/depthai-core/issues/203>`_ from luxonis/overloading_functions
  Added some function overloads
* fix compiling error
* Added some function overloads
* Fixed style
* Added Backward library to print stacktraces on crash
* Updated FW with GPIO and SPI improvements
* Merge branch 'throw.nice' into develop
* Added flash booted state and handling
* Merge branch 'device_config' into develop_embedded
* Merge branch 'bootloader_updates' into develop_embedded
* Release v2.10.0
* Merge pull request `#201 <https://github.com/luxonis/depthai-core/issues/201>`_ from luxonis/develop
  Release v2.10.0
* Bump version to 2.10.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Fixed incorrect exception message
* Merge pull request `#199 <https://github.com/luxonis/depthai-core/issues/199>`_ from luxonis/xlink_chunk_size
  Configure XLink chunk size
* Update FW and shared after merge
* Fixed Windows Platform specific code
* Fixed Super Speed mode and added a test
* Updated FW for UsbSpeed handling
* DeviceBase/Device: add {set/get}XLinkChunkSize RPC calls
* Added versioning to BL requests and refactored
* Updated flash_bootloader example
* Added capability to compress FW and additional BL config helper
* Reduced BL check to 0.0.14 and updated FW and BL
* Update FW with stereo LR-check, subpixel fixes; extended mode is not available
* Merge pull request `#195 <https://github.com/luxonis/depthai-core/issues/195>`_ from luxonis/update_readme
  Update README.md instructions with OpenCV troubleshooting
* Fix naming `setXlinkChunkSize` -> `setXLinkChunkSize`
* Pipeline: add `setXlinkChunkSize`
* Update FW with bilateral fix
* Apply suggestions by clang-tidy
* Rename vars as requested
* Bring the 3 variable ctor into visibility
* Update README.md
* Merge branch 'main' into develop
* Updated bootloader_configuration example
* Make data members const
* Add pertinent info to XLinkError struct
* Throw XLink specific errors for read/write errors
* WIP: Bootloader configuration
* Merge branch 'develop' into bootloader_updates
* Merge branch 'deviceBase' into develop
* Address review comments
* Merge pull request `#197 <https://github.com/luxonis/depthai-core/issues/197>`_ from luxonis/sysinfo_docs
  Fixed display names
* Fixed display names
* update code template
* Merge pull request `#196 <https://github.com/luxonis/depthai-core/issues/196>`_ from luxonis/stereo_crash_workaround
  Stereo crash workaround
* Add workaround for stereo subpixel/extended mode crash at the expense of system performance
* Update README.md instructions with OpenCV troubleshooting
* Merge pull request `#181 <https://github.com/luxonis/depthai-core/issues/181>`_ from luxonis/feature_tracker
  Feature tracking support
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'main' into develop
* Hotfix - temporary prevent flashing apps for PoE models
* Version bump to v2.9.0
* Updated FW
* Merge branch 'develop' into main
* Updated FW
* Removed 'filesystem' include
* Merge branch 'main' into develop
* Added an alias for Script Properties
* Added default constructor as these are not inherited
* Update FW
* Applied style
* Merge pull request `#193 <https://github.com/luxonis/depthai-core/issues/193>`_ from luxonis/image_manip_rotate
  ImageManip tiling and rotating example
* Restarting docs building
* Added }
* Fixes for MSVC ambiguity with overloaded constructors
* Fixed conversion problems
* Fixing compilation error on mac
* Merge pull request `#192 <https://github.com/luxonis/depthai-core/issues/192>`_ from luxonis/distortion_coeff_docs
  Added distortion coefficients representation for the documentation
* Added distortion coefficients representation for the documentation
* Fixed boot_memory bootloader upgrade routine
* Merge branch 'develop' into bootloader_updates
* Fixed imageManip rotate, added imageManip tiling example
* Added ImageManip example
* Handle dtor and close without bugs
* Merge branch 'develop' into deviceBase
* Allow to specify which bootloader is overridden by the env var:
  `DEPTHAI_BOOTLOADER_BINARY_USB`
  `DEPTHAI_BOOTLOADER_BINARY_ETH`
  (both can be set)
* Fix build issue
* Rename function arguments to their alias
* Fix docs about feature tracking
* Update shared with type fixes in docs; update FW to latest develop
* Update linking
* Keep same behavior in DeviceBase as Device wrt starting pipeline
* Make ctor API simpler for `DeviceBase` and `Device`
* Merge remote-tracking branch 'origin/develop' into HEAD
* Refactor FeatureTrackerConfig
* Rename feature tracker config fields
* Shutdown gracefully in case of exception in ctor
* Updated flash_bootloader example
* Hotfix - updated XLink with a segfault fix
* Add support for hardware accelerated motion estimation
* Merge branch 'xlink_error_221_fix' into develop
* Improved the flash_bootloader example a bit
* Updated flash_bootloader to be a bit more verbose
* Make `connection` as protected
* Added an explicit flag to allow flashing bootloader
* Moved operator<< overloads to global namespace
* Move startPipeline from DeviceBase to Device
* Merge branch 'fp16_no_git_clone' into develop
* Warn when firmware or bootloader binaries are overriden
  - to confirm it's picked up, or to notice when forgotten exported
* Optional env var DEPTHAI_BOOTLOADER_BINARY to override bootloader FW path,
  mostly for development
* Update bootloader: support for more NOR flash chips,
  fixes issues with flash erasing
* Revert "Removed flash_bootloader"
  This reverts commit f1f03bcefde92b518fe5a1534b83c3fa919e30e6.
* Revert "Removed flash_bootloader example temporarily"
  This reverts commit ee2a04e58b995e1bfa0cb03b91f83a45d446ca7f.
* Added an XLink 221 fix in FW and a default confidence threshold
* Added a custom fork of FP16 which doesn't use git clone
* Update shared
* Rename FeatureTrackerData to TrackedFeatures
* Update bootloader and fixing errors
* Update shared
* Merge remote-tracking branch 'origin/develop' into gen2_apriltag
* Merge pull request `#187 <https://github.com/luxonis/depthai-core/issues/187>`_ from luxonis/update_openvino
  Update OpenVINO version in examples to 2021.4
* Update OpenVINO version in examples to 2021.4
* Sync python-cpp examples
* Add configurable shave/memory resources to feature tracker
* Update FW with memory optimizations
* Update FW and shared
* Add overloaded functions to disable optical flow
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Extend feature tracker configuration
* Updated FW and a catch clause
* Merge branch 'develop' into device_config
* Merge branch 'gen2_scripting' into develop
* Fixed depth_crop_control example
* Merge branch 'develop' into gen2_scripting
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Merge branch 'develop_spi_in' into develop
* Merge pull request `#185 <https://github.com/luxonis/depthai-core/issues/185>`_ from luxonis/develop
  Release v2.8.0
* Update FW to 2.8.0
* Updated shared and FW
* Update shared to 2.8.0
* Bump version to 2.8.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Added additional options to SPIIn
* Added override to SPIOut::getProperties
* Merge branch 'develop' into develop_spi_in
* Fixed patching
* Modified watchdog to use a separate stream
* Updated preboot and added watchdog configuration
* Add config fields to feature tracker node
* Merge branch 'develop' into device_config
* Merge pull request `#174 <https://github.com/luxonis/depthai-core/issues/174>`_ from luxonis/cam_sync
  RGB - Mono capture time sync
* Merge remote-tracking branch 'origin/develop' into cam_sync
  Update FW and depthai-shared after merge
* Updated FW
* Update FW
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Updated AssetManager::get function documentation
* Increased test timeout to 10s
* Addressed PR comments and updated FW
* Merge branch 'develop' into gen2_scripting
* Merge branch 'rpc_issue_fix' into develop
* Applied formatting
* Updated comment on RPC mutex
* WIP: Reenabled RPC mutex lock
* Hide nanorpc client under Device::Impl
* Synchronize python-cpp examples
* Merge remote-tracking branch 'origin/develop' into feature_tracker
* Fixed a binding issue in FW
* Merge branch 'develop' into gen2_scripting
* Update names, make serialize a public function
* Add note in the documentation of the virtual functions
* Fix reference to base class function in `dai::Device`
* Give more love to StreamPacketParser
* Make the virtual functios protected and public functions non-virtual
* Move items around in startPipeline
* Separate Device and DeviceBase, expose StreamPacketParser
* Merge pull request `#179 <https://github.com/luxonis/depthai-core/issues/179>`_ from luxonis/imu-accuracy-name-clash
  Fix imu accuracy name clash
* Update shared/FW
* Rename IMUReportAccuracy enum to Accuracy
* Fix name clash for accuracy field in RotationVector structure
* Merge pull request `#167 <https://github.com/luxonis/depthai-core/issues/167>`_ from luxonis/openvino_2021_4
  Add OpenVino 2021.4 support; remove deprecated 2020.1, 2020.2
* Add openvino 2021.4 blob to tests
* Update FW to latest develop
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge pull request `#178 <https://github.com/luxonis/depthai-core/issues/178>`_ from luxonis/develop
  Release v2.7.2
* Bump version to 2.7.2
* Merge remote-tracking branch 'origin/develop' into HEAD
* Update FW with OpenVino FW fix
* Update FW with SDK update
* Updated FW
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated FW
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#176 <https://github.com/luxonis/depthai-core/issues/176>`_ from luxonis/2.7.0_hotfix
  Release 2.7.1
* Bump version to 2.7.1.0
* Hotfix: fix NN memory allocation regression
* Update FW
* Hotfix: fix NN memory allocation regression
* Updated FW
* Indented example script
* Separate Queue handling from core API
* Update FW: implement RGB - Mono sync:
  capture time and sequence numbers
* Add ImgFrame::getTimestampDevice() API - mostly for debugging
* ImgFrame.hpp: fix some typos
* WIP: Merge resolution
* Merge branch 'develop' into gen2_scripting
* Merge branch 'queue_reference_fix' into develop
* Fixed DataQueue isClosed logic
* Closing the data queue joins the underlying thread
* Close queues when closing the device
* Merge branch 'get-in-out.const' into develop
* Update FW with multi instance support
* Merge branch 'develop' into main
* Updated FW
* Bump to version 2.7.0
* Add apriltag_rgb example
* Merge branch 'bootloader_improvements_eth_desync_fix' into develop
* Removed flash_bootloader
* Removed flash_bootloader example temporarily
* Updated bootloader_version example
* Merge branch 'develop' into bootloader_improvements_eth_desync_fix
* Hotfix FW: revert increased memory consumption
* Rename, update shared
* Updated XLink dependency
* Updated resources to handle FW diff
* Naming changes and additional bootloader capabilities
* Remove leftover code
* Update trackbar naming
* Add FeatureTracker node; add cpp example
* Merge branch 'develop' into bootloader_improvements_eth_desync_fix
* Updated to develop FW
* Added export of integration options
* Merge pull request `#169 <https://github.com/luxonis/depthai-core/issues/169>`_ from luxonis/3rdparty_integration_docs_fix
  Updated instructions for thirdparty library integration
* Merge pull request `#169 <https://github.com/luxonis/depthai-core/issues/169>`_ from luxonis/3rdparty_integration_docs_fix
  Updated instructions for thirdparty library integration
* POC: Feature tracker node
* Updated instructions for thirdparty library integration
* Release v2.6.0
* Merge pull request `#168 <https://github.com/luxonis/depthai-core/issues/168>`_ from luxonis/develop
  Release 2.6.0
* Bump version to 2.6.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Add OpenVino 2021.4 support; remove dperecated 2020.1, 2020.2
* Hotfix: Fix mobilenet detection network
* Merge remote-tracking branch 'origin/develop' into gen2_scripting
  # Conflicts:
  #	cmake/Depthai/DepthaiDeviceSideConfig.cmake
  #	shared/depthai-shared
  #	src/pipeline/node/NeuralNetwork.cpp
* Fix style
* Add the EdgeDetector for the CI
* Convert from 2 pointers to a vector
* Fix style
* Add {In,Out}putRef getters
* Fix style, again
* Adding getters for parents
* Adjust visibility of getName, getInput, getOutput
* Mark member functions `dai::Node::get{In,Out}put` as const
* Merge pull request `#165 <https://github.com/luxonis/depthai-core/issues/165>`_ from luxonis/edge_detector
  Add EdgeDetector node
* Update shared/FW
* Fix BUG in ParsePacket for received SpatialLocationCalculatorConfig
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'host_build_c++14' into develop
* Merge branch 'xlink_desync_fix' into develop
* Merge branch 'nn_dimensions_strides_order_fix' into develop
* Calib fix (`#163 <https://github.com/luxonis/depthai-core/issues/163>`_)
  * Bug fix
  * Fixing negative
  * Updated device side fix for signs
  * Additinal checks
  * Fixed styling
  * updated FW to develop:
* Update FW
* Update apriltag example
* Add initial working version
* Update FW/shared
* Merge with latest develop; Update FW
* Add edge detector node using HW sobel edge filter
* Merge branch 'gen2_scripting' of github.com:luxonis/depthai-core into gen2_scripting
* Script - added struct and fixed json modules
* Merge FW with latest develop
* Hotfix: Update FW with fix for crash w/ depth-rgb aligment
* Updated Script node with json and ctypes libraries
* Merge branch 'develop' into gen2_scripting
* Replace deprecated function call
* Added getMxId call for ethernet use case
* Update FW and XLink for desync fix
* Updated FW and XLink to fix stream desync issue
* Update ETH bootloader/FW: fix some IPv6 related crashes, improve performance
* Merge pull request `#159 <https://github.com/luxonis/depthai-core/issues/159>`_ from luxonis/bilateral_filter
  Add support for 5x5 bilateral filter in stereo depth; add runtime con…
* Update FW
* Deprecate setConfidenceThreshold; setMedianFilter
* Fixed MacOS build. Local XLink option skips hunter
* Updated bootloader
* Applied formatting
* Resources: ETH bootloader bug. Added flash_bootloader example
* Add config for LR-check threshold
* Update FW with median filter configurability
* Updated XLink to tcpip_driver branch
* Improving some BootMemory cases and updated bootloader and FW
* Deprecate setEmptyCalibration
* Update FW with resource allocation fix
* Update shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Added temporary ETH specific fixes
* Booting specified bootloader
* Fixed Seg Fault in getImuToCameraExtrinsics (`#156 <https://github.com/luxonis/depthai-core/issues/156>`_)
  * Fixed Seg Fault in getImuToCameraExtrinsics
  * Added additional check at ComuteExtrinsics
  * Changed error display
* Merge branch 'develop' into bootloader_improvements
* Updated formatting
* Reversed dimension and stride order
* Add support for 5x5 bilateral filter in stereo depth; add runtime configurability for stereo depth
* Updated shared and FW
* Merge pull request `#153 <https://github.com/luxonis/depthai-core/issues/153>`_ from kunaltyagi/headers
  Add convenience headers in depthai/pipeline
* Added resource loading for bootloader
* Merge pull request `#157 <https://github.com/luxonis/depthai-core/issues/157>`_ from kunaltyagi/libarchive.cmake
  Change name of libarchive for better compatiblity with Hunter's packages
* Added backwards compatibility
* Merge branch 'bootloader_boot_memory' into bootloader_improvements
* Change name of libarchive for better compatiblity with Hunter's packages
* Removed unnedeed standard specification
* Merge pull request `#148 <https://github.com/luxonis/depthai-core/issues/148>`_ from luxonis/depth_align_improvements
  StereoDepth: mesh rectification, disp/depth configurable resolution
* Merge remote-tracking branch 'origin/develop' into depth_align_improvements
* Update FW: depthai-shared PR merged,
  also included FW changes from https://github.com/luxonis/depthai-core/pull/118 :
  fixes for new boards with 0x2 boot mode (not switching back to bootloader after app reset)
* Rename as requested
* Added C++14 as transitive property
* Fixed NN deadlock edge case
* Fixed NN bug
* Replaced 'unique_ptr' and 'new' with 'make_unique'
* Fixed some bugs
* Merge pull request `#154 <https://github.com/luxonis/depthai-core/issues/154>`_ from kunaltyagi/parent.public
  Make `Node::getParentPipeline` publically available
* Make `getParentPipeline` publically available
* Adding convenience headers
* Merge pull request `#152 <https://github.com/luxonis/depthai-core/issues/152>`_ from luxonis/synch_calibration
  Synchronize calibration examples w/ python
* Update FW
* Synchronize with python
* Add support for median filter for LR check depth mode
* StereoDepth: add setOutputKeepAspectRatio
* Rename `setOutputResolution` -> `setOutputSize`, for consistency
  with similar API in ColorCamera, etc
* Release v2.5.0
* Merge pull request `#149 <https://github.com/luxonis/depthai-core/issues/149>`_ from luxonis/develop
  Release v2.5.0
* Bump version to 2.5.0
* Merge remote-tracking branch 'origin/main' into HEAD
* rgb_depth_aligned: increase confidence threshold 200 -> 230,
  as in the python example
* Update depthai-shared: make clangformat
* clangformat
* rgb_depth_aligned: lower L/R res: 720p -> 400p, to fix lag for now
  Also add configurable FPS, to allow quick swap to 720p with a lower FPS
* Merge remote-tracking branch 'origin/develop' into depth_align_improvements
* StereoDepth: add mesh calibration support
* Merge pull request `#147 <https://github.com/luxonis/depthai-core/issues/147>`_ from luxonis/update_doc
  Update documentation
* Update shared
* Update FW
* Merge pull request `#143 <https://github.com/luxonis/depthai-core/issues/143>`_ from luxonis/queue_add_callback_cpp
  Added example on how to add a queue callback in cpp
* Update shared
* Merge remote-tracking branch 'origin/develop' into update_doc
* Update 2
* Merge pull request `#119 <https://github.com/luxonis/depthai-core/issues/119>`_ from luxonis/imu_node
  IMU: BNO 085/6 support
* Update FW, shared
* Update documentation
* Rename RAW\_* to *_RAW in ImuSensors
* Merge branch 'develop' into host_build_c++14
* Update FW; fix high CPU load; enable full speed raw sensors
* Merge remote-tracking branch 'origin/develop' into HEAD
* Rename imu_gyro_accelero example
* Add convenience functions; sync cpp python examples
* Calibration data bug fix (`#146 <https://github.com/luxonis/depthai-core/issues/146>`_)
  Changed double to float in set/get fov.
  Modified Docstring for matrix (C++ only for now)
  FW bug fix in stereo when rgb camera calibration was not available
* Moved C++ standard specification to targets
* Merge remote-tracking branch 'origin/develop' into HEAD
* Updated bootloader and command to boot fw
* Merge pull request `#144 <https://github.com/luxonis/depthai-core/issues/144>`_ from luxonis/spatial_calculator_improvements
  Add depthMin, depthMax to spatial calculator
* Update FW
* Merge remote-tracking branch 'origin/develop' into spatial_calculator_improvements
* Add depthMin, depthMax to spatial calculator
* Removed deprecated OpenVINO versions
* Merge branch 'develop' into device_config
* Add comments
* Add example on how to add a queue callback in cpp
* Update to C++14,
  remove depthai-shared workaround for unordered_map with enum class
* Merge pull request `#141 <https://github.com/luxonis/depthai-core/issues/141>`_ from luxonis/object_tracker_video
  Add object tracker video example
* Merge branch 'blob_version_compatibility' into develop
* Merge branch 'develop' into blob_version_compatibility
* Merge pull request `#101 <https://github.com/luxonis/depthai-core/issues/101>`_ from luxonis/gen2_eeprom_api
  Calibration read/write/load API
* Updated device side
* Updated examples to create backup
* Typo fix
* Example bug fix
* Fixed styling
* Merged with develop
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into HEAD
* Update on revierws
* Updated example
* Merge branch 'develop' into device_config
* Refactored and added preboot config
* Added a test for various OpenVINO versions
* Bug fix
* Updated examples
* Updated validation
* StereoDepth: add setOutputResolution, currently applicable with
  RGB alignment
* Add timestamp to video mobilenet
* Add timestamp
* Fixed docstring
* Updated device side commit
* clangformat
* Synchronize stereo_depth_video example
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* Changes for swap WIP
* Add empty frame check
* Add object tracker video cpp example
* Merge pull request `#140 <https://github.com/luxonis/depthai-core/issues/140>`_ from luxonis/develop
  Release v2.4.0
* Bump version to 2.4.0
* Merge remote-tracking branch 'origin/main' into HEAD
* Merge pull request `#139 <https://github.com/luxonis/depthai-core/issues/139>`_ from luxonis/stereo_fixes_2
  Stereo fixes 2
* Comment out for now ImgFrame excess data warning,
  doesn't build on Windows
* Update FW, update `setRectifyMirrorFrame` functionaliy/description
* Merge remote-tracking branch 'origin/develop' into stereo_fixes_2
* Merge pull request `#135 <https://github.com/luxonis/depthai-core/issues/135>`_ from luxonis/ov9282_over_exposure_fix
  OV9282: fix over-exposure outdoors, in sunlight
* Merge remote-tracking branch 'origin/develop' into ov9282_over_exposure_fix
* Merge pull request `#138 <https://github.com/luxonis/depthai-core/issues/138>`_ from luxonis/usb_crash_mitigation
  Update FW with fix for random crashes (kernel crash on RPI/jetson)
* Revert RPC mutex lock; it's reported that has issues on Windows
* Update FW
* MonoCamera: add `raw` output. Update FW: OV9282 min autoexposure 20us
* Update FW
* Merge remote-tracking branch 'origin/develop' into ov9282_over_exposure_fix
* Modified test and example adding function
* Added openvino blob versioning support
* Update FW with fix for random crashes (kernel crash on RPI/jetson)
* Merge pull request `#110 <https://github.com/luxonis/depthai-core/issues/110>`_ from luxonis/renamed_and_new_examples
  Synchronize cpp examples with python
* ImgFrame CV conversion: more verbose about size mismatch
* Update FW: stereo fixes: LR-check flip, `depth` align to RGB
* Updated device side
* Synch
* Updated device side commit id
* Added missing includes
* Updated device side and shared
* Updated SPIIn and FW
* Sync cpp examples with latest python
* Merge remote-tracking branch 'origin/develop' into HEAD
* Merge branch 'develop' into develop_spi_in
* Hotfix - Lossless encoding
* OV9282: fix over-exposure outdoors, in sunlight
* Remove unnecessary libraries, improving the code
* Merge branch 'usb_speed' into develop
* Fixed style
* Added printing helpers and UsbSpeed example
* Added getUsbSpeed
* WIP: Device configuration
* Merge pull request `#133 <https://github.com/luxonis/depthai-core/issues/133>`_ from luxonis/getMaxDisparity
  Fixed getMaxDisparity calculation
* Fixed getMaxDisparity calculation
* Merge pull request `#132 <https://github.com/luxonis/depthai-core/issues/132>`_ from luxonis/getMaxDisparity
  stereo node getMaxDisparity()
* removed void from the function's arguments
* removed -
* fixed test and updated shated
* Updated styling
* fixes for the PR
* Merged with develop:
* addressing PR Requests
* changed measured* to spec*
* added a function in stereo node that returns the maxDisparity. Also changed stereo_example.cpp to use this new function
* Merge branch 'develop' into gen2_scripting
* Merge pull request `#131 <https://github.com/luxonis/depthai-core/issues/131>`_ from luxonis/camera_custom_tuning
  Camera custom tuning
* removed bootloader test
* fixed tests
* tidy
* modified examples for test:
* merged with develop and added validatecameraArray
* Comments
* Fix createDirectory for windows
* Renamed rgb_depth_aligned_example to rgb_depth_aligned
* Fixing errors
* merged with develop
* Remove duplicated example
* Merge remote-tracking branch 'origin/develop' into HEAD
* Rename
* Remove redundant in/out flags from ifstream/ofstream across codebase
* Fix formatting, fix a merge issue
* Merge remote-tracking branch 'origin/develop' into camera_custom_tuning
* added more getters
* added headers
* Renamed examples
* Comments
* Add RAW accelerometer/gyro sensors
* Merge branch 'develop' into main
* Bump version to 2.3.0
* adressed PR requests
* Remove whitespaces
* Disable median filter to avoid warning
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Merge pull request `#128 <https://github.com/luxonis/depthai-core/issues/128>`_ from luxonis/fix_lrcheck_spatial
  Fix spatial calculator output with stereo LR-check enabled
* Fix docs build
* Try fixing docs build:
  docstring of depthai.StereoDepth.disparity:6: WARNING: Bullet list ends without a blank line; unexpected unindent.
* Update FW (properly set flipping with LRcheck enabled to spatial calculator),
  update StereoDepth docs
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Update depthai-shared
* Merge branch 'readme_refactor' into develop
* Merge pull request `#120 <https://github.com/luxonis/depthai-core/issues/120>`_ from luxonis/object_tracker_improvements
  Add new field: Removed to object tracker status
* Merge remote-tracking branch 'origin/develop' into HEAD
* Fixed documentation issue and inconsistencies
* Remove numbers and some optimization
* Added build information and config.hpp to remove the need to specify compile definitions
* Merge pull request `#126 <https://github.com/luxonis/depthai-core/issues/126>`_ from luxonis/nn_performance_fix
  Update FW with fix for resource allocation when depth is enabled; fix…
* Move queue init after pipeline start in system information example
* Apply formatting
* Update FW with fix for resource allocation when depth is enabled; fix system_information_example
* Updated README.md
* fixed intrinsics scaling bug
* Applied formatting
* updated device side commit id
* Merge branch 'device_improvements' into develop
* Merge branch 'main' into develop
* Resolved some warnings
* updated examples with API changes
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Replace RawImgFrame with ImgFrame
* Optimization, comments.
* Pipeline: add custom camera tuning blob option
* Updated device side with 'getConnectedCameras'
* Fixed sanitizers for examples
* Removed deprecated functions from examples
* updated shared and device side commits
* Deprecated 'startPipeline()'
* docstring updates
* Optimization, comments
* Changes to get SPIIn working (WIP)
* Merge pull request `#82 <https://github.com/luxonis/depthai-core/issues/82>`_ from luxonis/stereo_fixes
  Fixes and improvements for StereoDepth, ColorCamera
* Update FW, fix CI build: depthai-shared PR merged
* Update FW: fix ImageManip U16 crop (for depth/subpixel disparity)
  Update shared: stereo_fixes merged
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Update FW: fix still capture with scaling, add FPS capping (with warnings)
* clangformat cleanup
* Address review comments. Note:
  The change in discussion (--parallel 8 in Readme) was already cherry-picked to develop (so no longer appears on this PR)
* Add new field: Removed to object tracker status
* Add video mobilenet and fix some others
* Remove function argument from getters
* Rename imu_example to imu_gyro_accelero_example
* Add rotation vector example
* Add configurable IMU report rates for gyro and accelero
* WIP: Decouple pipeline from Device
* Add rgb encoding mobilenet
* Upgraded rgb mobilenet example
* Add rgb mobilenet 4k example
* Update FW with fix for timesync
* added script camera control example  (as in python)
* added include Script node in depthai.hpp
* fixed api function calls style
* fixed getting size of video/still when setting isp scaling
* fixed rgb measured translation issue:
* Add encoding max limit example
* Add rgb encoding mono mobilenet with depth
* refactoring
* Added a try catch for callbacks for better error messages
* Synch
* Add mono depth mobilenet example
* Add mono mobilenet cpp example
* Merge pull request `#113 <https://github.com/luxonis/depthai-core/issues/113>`_ from luxonis/custom_binary_env
  Capability to specify a custom device binary
* Added capability to specify custom device binary
* fixed extrinsics sign issue
* spatial_object_tracker example: remove deprecated setOutputDepth
* stereo_example: rectified flip no longer needed with LR-check on,
  don't link depth in pipeline if not used, and other cleanup.
  Update FW: ispScale factors simplification done on device, other bugfixes
* fixed overloading function issue:
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* added throw
* fixed styling
* fixing style
* added throw to runtime errors
* Add rgb and mono full resolution saver
* Update FW with fixes for newer OAK-D
* added lensPosition setter
* Add initial implementation of IMU node: acceleration and gyro at 500hz
* Added lens position to eepromData
* Add depth crop control
* Add mono camera control and fixed encodings
* Merge pull request `#105 <https://github.com/luxonis/depthai-core/issues/105>`_ from luxonis/docs_fix_disparity_range
  fixed extended disparity range documentation
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Merge pull request `#112 <https://github.com/luxonis/depthai-core/issues/112>`_ from luxonis/datainputqueue_nullptr_check
  DataInputQueue nullptr check
* Added nullptr check to DataInputQueue::send
* Merge pull request `#111 <https://github.com/luxonis/depthai-core/issues/111>`_ from luxonis/hotfix_stereo_confidence_thr
  StereoDepth: fix confidence threshold configuration
* Update FW: hotfix for stereo confidence threshold setting,
  it was overwritten to 200
* Add rgb mono encoding example to cpp
* Updated FW - MobileNet parsing bugfix
* Changed 04 to match with python
* fixed width and height order
* added stereoRectification getters
* Merge remote-tracking branch 'origin/develop' into renamed_and_new_examples
* Add depth preview
* Merge branch 'gen2_scripting' of github.com:luxonis/depthai-core into gen2_scripting
* Updating firmware (Fixing datetime on ImgFrame::getTimestamp)
* Renamed files to match with python examples and added a new example
* changed device side commit id
* Fixed scripting 'setCropRect' and added bounds
* Merge pull request `#108 <https://github.com/luxonis/depthai-core/issues/108>`_ from luxonis/videnc_fixes
  Video encoder fixes
* Fix wrongly set bitrate
* updated shared:
* Added device info getter
* Merge remote-tracking branch 'diablodale/fix71_various_code_warnings' into develop
* Added default CMAKE_BUILD_TYPE
* Merge pull request `#107 <https://github.com/luxonis/depthai-core/issues/107>`_ from luxonis/develop
  Release 2.2.1
* Bump version to 2.2.1
* Merge pull request `#106 <https://github.com/luxonis/depthai-core/issues/106>`_ from luxonis/spatial_data_extension
  SpatialCalculator fixes
* Merge remote-tracking branch 'origin/main' into HEAD
* Update FW with bugfixes for spatial calculator
* Merge remote-tracking branch 'origin/develop' into HEAD
* Update FW
* fixed extended disparity range documentation
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* local commit
* Merge pull request `#104 <https://github.com/luxonis/depthai-core/issues/104>`_ from luxonis/develop
  Release 2.2.0
* README build snippets: limit `cmake --parallel` to 8
* GitHub CI: limit cmake --parallel to 8 threads,
  to prevent an out-of-memory situation due to too many threads created
* Fix MacOS CI builds
* Bump version to 2.2.0
* Merge pull request `#103 <https://github.com/luxonis/depthai-core/issues/103>`_ from luxonis/develop_main_merge
  Develop-main merge
* Update gitignore with git generated files on merge conflict
* Merge remote-tracking branch 'origin/main' into develop
* Delete calib_data2.json
* changed storeCalibration to flashEepropm
* Update firmware.
* Update SpatialCalculator data output with a new field: depthAveragePixelCount
* modified extrinsics setters
* modified device side config
* clang-tidy 2
* clang-tidy
* changed commit id and rebased
* Updating firmware and adding a check to raw PoBuf parsing.
* merged with develop
* updated depthai-shared
* updated shared link
* Update FW: camera_init_fixes. Changes:
  - IMX378/477: increase reset low time to 20ms
  - OV9282: increase reset low time to 10ms, wait after 5ms
  - OV9282: allow using modules with I2C addr 0x20
  - report errors if color/mono cameras are not detected
* Merge branch 'videoencoder_lossless' into develop
* Added lossless jpeg encoding and some improvements
* Updated FW
* Added DEPTHAI_FW_BINARY_PATH environment variable
* Fixed an incorrect RPC call
* Merge pull request `#92 <https://github.com/luxonis/depthai-core/issues/92>`_ from luxonis/object_tracker
  Integrated Intel's object tracker, added spatial object tracker
* Update FW
* Update FW, submodules
* fixed getIntrinsics bug and Added Device commit id
* added docstrings and cameraType
* Merge remote-tracking branch 'origin/develop' into HEAD
* Add support for tracker on full frame
* Renamed 'LxScript' to 'Script'
* Updated style
* Updated FW to reduce size
* Applied formatting
* Improved Asset handling
* added more functions test in calibration_reader
* Merge branch 'develop' into gen2_micropython
* added extrinsics getter functions WIP
* Addresses CMake 3.20 regression in parsing '--parallel' ('-j') option
* Update FW: optimize depth align, make it work with subpixel/U16
  (still not optimized)
* Cleanup, remove some unused variables
* README build snippets: limit `cmake --parallel` to 8
* GitHub CI: limit cmake --parallel to 8 threads,
  to prevent an out-of-memory situation due to too many threads created
* Examples: remove deprecated API setOutputDepth/Rectified
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Fix conversion of YUV420p frames to OpenCV BGR,
  the chroma planes were swapped
* Add rgb_depth_aligned example
* Merge branch 'cmake_regression_workaround' into develop
* Merge branch 'opencv_version_requirement' into develop
* Addresses CMake 3.20 regression in parsing '--parallel' ('-j') option
* Renaming MicroPython node to LxScript.
* Explicitly specify OpenCV 4 version
* CameraControl: add ranges for extra controls,
  remove non-implemented setNoiseReductionStrength.
  Updated FW: all initial controls can be applied for Mono/ColorCamera
  (no longer limited to focus settings for Color)
* Address review comments:
  - add `isp` and `raw` to ColorCamera list of outputs, add docstrings
  - overloaded `setIspScale`, with tuple inputs as options
  - also overloaded `setPreviewSize`, `setVideoSize` and `setStillSize` with tuple inputs
* Update FW: disparity (U8) aligning to RGB works.
  TODO depth and subpixel (U16)
* Add ID assigment policy for object tracker
* Checking in micropython asset changes.
* Merge remote-tracking branch 'origin/develop' into HEAD
* Style changes
* Revert globbing
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* Hotfix for bounding box mapping
* Merge pull request `#93 <https://github.com/luxonis/depthai-core/issues/93>`_ from luxonis/openvino_2021.3_support
  Add OpenVino 2021.3 support
* Chanhe default OpenVino version to 2021.3; update FW
* fix narrowing, clangformat, mutex lock, VideoEncoder::get/setFrameRate to float
* Added eeprom reader and an example
* Set default OpenVino version to 2020.3
* Add OpenVino 2021.3 support
* Add Intel's object tracker + spatial object tracker
* StereoDepth: remove for now 'setBaselineOverrideCm', 'setFovOverrideDegrees',
  will be refactored when the new calibration structure is integrated
* StereoDepth: add overloaded setDepthAlign(CameraBoardSocket)
* Merge remote-tracking branch 'origin/develop' into stereo_fixes
* added calibration_stereo example
* added setters
* WIP calibration store example
* Merge branch 'main' into develop
* Hotfix - added setBitrateKbps and fixed function description
* Merge branch 'develop' into main
* Version bump to 2.1.0
* Merge branch 'documentation_improvements' into develop
* Improved some parts of documentation
* Merge branch 'invalid_device_info_fix' into develop
* Merge pull request `#88 <https://github.com/luxonis/depthai-core/issues/88>`_ from luxonis/camera_driver_fix
  Gen2 Camera driver fix
* Fix stability issues WRT camera driver
* Added a fix for passing an invalid deviceInfo
* added constructor and fetchers headers
* Removed unneeded variable
* Merge pull request `#86 <https://github.com/luxonis/depthai-core/issues/86>`_ from luxonis/gen2-spatial-yolo-example
  Add gen2 spatial yolo example
* Merge remote-tracking branch 'origin/develop' into gen2-spatial-yolo-example
* Merge pull request `#87 <https://github.com/luxonis/depthai-core/issues/87>`_ from luxonis/develop-main-merge
  Merge main into develop
* Merge remote-tracking branch 'origin/main' into develop
* Change to getCvFrame
* Add tiny-yolo-v3 and v4 examples
* Updated FW - fixes depth calculator issues on devices without calibration
* Merge branch 'install_and_integration' into develop
* Merge pull request `#83 <https://github.com/luxonis/depthai-core/issues/83>`_ from luxonis/gen2_spatial_detection_network
  Gen2: Add spatial detection network and spatial location calculator
* Update submodules before merge
* Add property aliases
* Update FW with input sanitization for spatial calculator
* Change Rect type to match OpenCV's
* Merge branch 'gen2_eeprom_api' of github.com:luxonis/depthai-core into gen2_eeprom_api
* updated shared
* Reverted testing bump of minimum CMake version
* Added EXPORT_NAME property to targets
* Added examples and tests as part of build process
* Added capability to import build directory
* Added clangformat to tests and examples
* Fixed inconsistent usage of CMAKE_INSTALL_LIBDIR
* Fixed BUILD_SHARED_LIBS usage and other variable naming
* Added integration CI tests
* Update FW with rgb-depth sync
* Modify dependency installation and integration test
* Add API to configure disparity/depth alignment: left/right/center.
  Works with LRcheck or LRcheck+Subpixel for now.
  The updated FW also fixes some crashes with LRcheck mode enabled
* modified cmake to fix lib install issues
* Handled warning when not added as a subdirectory
* Add passthrougDepth; rename passthroughRoi to boundingBoxMapping; add missing members to getOutput for NN nodes
* Update FW
* Rename roi to ROI
* Add documentation; rename SpatialLocationCalculatorDataOut to SpatialLocations
* Revert mobilenet_device_side_decoding_example to original
* Merge remote-tracking branch 'origin/develop' into gen2_spatial_detection_network
* Rename DepthCalculator to SpatialLocationCalculator
* Update FW with rectified fixes in stereo
* Update FW: stereo fixes, stereo/ColorCamera improvements
* `make clangformat`
* ColorCamera: add API to get 'isp' scaled output size
* StereoDepth: deprecate setOutputDepth/Rectified
* ColorCamera: add setIspScale/setIspScaleFull API
* ColorCamera: add `isp`, `raw` outputs
* Merge pull request `#78 <https://github.com/luxonis/depthai-core/issues/78>`_ from luxonis/gen2-tests-fix
  Gen2 tests fix
* Fix gen2 unit tests
* Fix build error from PR `#67 <https://github.com/luxonis/depthai-core/issues/67>`_
* Throw value error if queue size 0 is specified
* Add separate data type for spatial image detections
* Set up DetectionNetwork, SpatialNetwork properties polymorphycally
* Rename DetectionNetworkDepth to SpatialDetectionNetwork
* Code refactorization
* Update cpp examples
* Update FW
* Update FW with fix for stereo cams
* Update FW; improve example for depth calculator node
* Update FW/shared: formal switch to develop branch
* Update FW with fix
* Update FW
* Update formatting
* Fix build error from PR `#67 <https://github.com/luxonis/depthai-core/issues/67>`_
* Merge remote-tracking branch 'origin/develop' into WIP-gen2-detection-3d
* WIP: add first example of 3d detections
* Merge pull request `#67 <https://github.com/luxonis/depthai-core/issues/67>`_ from luxonis/gen2_yolov4_tiny_demo
  add (demo, test) added demo code and the test
* fix (yolo3/4 demo) fixed readability
* fix (yolov3/4 demo) fixed formatting for pr
* add (demo, test) added demo code and the test
* Fixed `#70 <https://github.com/luxonis/depthai-core/issues/70>`_ - Git check not using correct working directory
* Merge branch 'develop' into main
* Merge branch 'gen2_develop' into develop
* Fixed installation steps
* Version bump to 2.0.0
* Refactored sanitizer
* Merge branch 'gen2_release_preparation' into gen2_develop
* Merge branch 'gen2_mkdoc_autodoc' into gen2_develop
* Addressed typo in documentation
* Applied formatting
* Merge branch 'gen2_develop' into gen2_mkdoc_autodoc
* Merge branch 'gen2_detach_removal' into gen2_develop
* Initialized 'found' to false
* Removed unnecessary mutable and commented code
* Added same alias for both add_subdirectory and find_package
* Merge branch 'gen2_develop' into gen2_detach_removal
* Removed timeout from data queues
* Added new close and cleanup to DeviceBootloader, updated XLink
* Fixed segmentation fault, stream opening order and added capability to explicitly close a device
* Add dynamic ROI config
* Refactored CMakeLists.txt to better support installation
* Modified CMake file naming
* Added release steps to workflow
* Gen1: Release 1.0.0.0
* Documented majority of the public API
* Removed sendSync API and added docstrings
* Merge branch 'gen2_develop' into gen2_mkdoc_autodoc
* Merge branch 'gen2_opencv_support' into gen2_develop
* Updated Device destructor to remove callbacks, Created XLinkStream to fix data races and updated spdlog library
* Update FW: some cleanup
* Updated timeout script to work cross-platform
* Merge branch 'gen2_develop' into gen2_detach_removal
* Updated naming and added documentation
* Update depthai-shared,
  as it was left behind...
* Update FW: improve image resize mechanism (less cropping),
  for ColorCamera `preview` output and ImageManip setResize
* Added checks if data doesn't match the metadata
* Merge pull request `#64 <https://github.com/luxonis/depthai-core/issues/64>`_ from luxonis/usb_product_name_change
  Change enumerated product name to Luxonis Device / Luxonis Bootloader
* Change enumerated product name to Luxonis Device / Luxonis Bootloader
* Merge branch 'gen2_develop' into gen2_opencv_support
* Added capability to specify additional IO
* Refactored asset loading and capitalized MicroPython
* Update FW
* Add support for FP16 depth
* Merge branch 'gen2_develop' into gen2_micropython
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Merge pull request `#63 <https://github.com/luxonis/depthai-core/issues/63>`_ from luxonis/usb2_fix
  Add USB2 fix for large blobs; add h264 cpp example
* Add USB2 fix for large blobs; add h264 cpp example
* Improved build system for Windows
* Adding micropython.
* Merge pull request `#61 <https://github.com/luxonis/depthai-core/issues/61>`_ from luxonis/manip_improvements
  ImageManip improvements
* Update FW
* Hotifx: uninitialized metadatOnly field
* Applied formatting
* Merge branch 'gen2_develop' into gen2_mkdoc_autodoc
* Fixed sync example
* Added some sample documentation
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Fix formatting
* Add CMX memory usage repor
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Merge pull request `#60 <https://github.com/luxonis/depthai-core/issues/60>`_ from kunaltyagi/amend.gitmodules-gen2
  Update .gitmodules
* Modified example description
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Merge branch 'gen2_develop' into gen2_input_queue_xlink
* Update .gitmodules
* Merge pull request `#57 <https://github.com/luxonis/depthai-core/issues/57>`_ from luxonis/openvino_2021.2
  Openvino 2021.2 support
* Updated naming
* Changes from review; update FW with venc fix
* Added capability to specify input queue sizes and XLinkOut metadata only
* Update FW
* Update mobilenet/tiny yolo v3 with openvino 2021.2 blobs
* Update FW
* Handle empty path (default) case
* Add checks if path exists for device constructor
* Update FW
* Update FW
* Merge branch 'gen2_develop' into gen2_detach_removal
* Removed unneded shared source files
* Add openvino 2021.2; add configurable NCE per inference thread; add warnings in FW in case the NN config is suboptimal
* Merge pull request `#50 <https://github.com/luxonis/depthai-core/issues/50>`_ from luxonis/detection_nn
  Add detection NN example
* Swithc mobilenet blob to 6 shaves/2 threads
* Update FW
* Add private virtual getter for NN properties to avoid overriding all base class functions; add FPS counter for yolo/mobilenet examples
* Added checks for submodule being initialized
* Update FW
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Merge pull request `#56 <https://github.com/luxonis/depthai-core/issues/56>`_ from kunaltyagi/reduce.mem_alloc
  Reduce memory allocations by using the nodeMap
* Use the nodeMap acccessor, use early return to reduce indent
* Add an accesor in `Pipeline` for the underlying node map
* Fixed timeout loop
* Merge pull request `#38 <https://github.com/luxonis/depthai-core/issues/38>`_ from luxonis/gen2_imagemanip_rotate
  ImageManip: rotated/arbitrary cropping, add camera controls
* Update FW: fix for 4-point warp with normalized = false
* Set running true on construction
* NeuralNetwork: add getNumInferenceThreads()
* Update color_camera_control example with new controls:
  auto/manual exposure/focus, as in the depthai-python example
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate
* CameraControl.hpp: make non-Raw types public
* Update shared/FW after merge to gen2_develop
* Merge pull request `#52 <https://github.com/luxonis/depthai-core/issues/52>`_ from kunaltyagi/add.getConnectionMap
  Allow direct access to the node connections
* Fix formatting
* Fix quit on q; add setNumFramesPool to detectionNetwork too
* Address PR comments
* Add a typedef in `Pipeline` class as well
* Update FW: log warnings for unimplemented `initialControl` commands,
  changes to shared
* `make clangformat`
* Minor cleanup: use imported RawCameraControl types
* Color/MonoCamera: add `initialControl` member,
  similar ImageManip's `initialConfig`
* Removed unneeded queue variables
* ImageManip: add `initialConfig` member and deprecate API
  setting config fields
* WIP: Updated XLink and added timeout API
* Added planar RGB and BGR conversion
* Update FW: add OV9282 Strobe output
* Update FW, update image_manip_warp example
* `make clangformat`
* ImageManipConfig: add warp border handling:
  replicate pixels or solid color fill
* Refactor to not expose Raw.. types in API,
  ImageManipConfig: convert from Rad to Deg
* Tweaked opencv support target
* Removed unneded flag
* Moved opencv support to separate target
* WIP: Initial OpenCV support
* Merge pull request `#51 <https://github.com/luxonis/depthai-core/issues/51>`_ from kunaltyagi/throw.failed_boot
  Add confirmation that the device has booted up
* Add keepAspectRatio to ImageManip; update FW with fixes; support up to 4056 pixel width images in ImageManip
* Allow direct access to the node connections
* Change variable name style to camelCase
* Update FW with usb2 patch
* Removed serialization symbols from documentation
* Add confirmation that the device has booted up
* updated shared
* Propagate version
* Fix formatting
* Add device side tiny yolo v3 decoding example
* Add FW fix for jpeg and NN; set openvino 2020.2 to 2020.3 since they are the same
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate
* Remove deleted function declaration
* Add image_manip_warp example
* Apply `make clangformat` corrections
* ImageManip: add warp transform, rotation API
* Add detection network example
* Merge branch 'gen2_develop' into gen2_mkdoc_autodoc
* Added a change to default dependency build type for MSVC
* Modified sanitizer behaviour
* Added sanitizers to tests and tested examples and doxygen output path
* Added doxygen documenation generation
* Add force set openvino version; add int32 layer utilities
* Merge branch 'gen2_neuralnetwork_error_fix' into gen2_develop
* Added a NN test and updated FW
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate
* Merge branch 'gen2_develop' into gen2_neuralnetwork_error_fix
* Merge branch 'gen2_cleanup' into gen2_develop
* Applied formatting
* Merge branch 'gen2_develop' into gen2_cleanup
* Added additional docstrings to Device
* Updated FW and added extra checks when running examples
* Compilation error fix
* Updated FW - ImageManip rounding fix
* Updated FW and updated depthai-shared references
* Merge branch 'gen2_nndata_fix' into gen2_develop
* Added sanitize undefined behavior
* Update FW: enable OV9282 strobe output,
  pulse high-active and aligned with exposure for now
* Add "inputControl" to MonoCamera node. Update FW
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate
* CameraControl: add initial set of ISP/3A controls. Update FW
* Merge pull request `#48 <https://github.com/luxonis/depthai-core/issues/48>`_ from luxonis/pmolloy/device-error-string
  Remove repetition in error message
* Remove repetition in error message
* Add initial depth calculator node
* ImageManip: remove outSize from setCropQuadrilateral
* ColorCamera: add setInitialLensPosition. Update FW
* Fixed compilation error
* NNData - fixed serialization
* Updated FW - NeuralNetwork hotfix
* Updating firmware. (`#44 <https://github.com/luxonis/depthai-core/issues/44>`_)
  Adding GET_MESSAGE_PART for SPI api..
* Update FW and add API for:
  - ImageManip: 'reusePreviousImage' option
  - ColorCamera: autofocus mode config at init
  - NeuralNetwork: specify 'numThreads', fix a bug for input frame size checks
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate
* Updated naming and FW
* Updated device side
* Merge branch 'gen2_improvements' into gen2_develop
* Changed error flags and fixed a warning
* Merge remote-tracking branch 'origin/gen2_develop' into gen2_imagemanip_rotate,
  and update FW: ImageManip fix for input from host/XLinkIn, fix Warp line artifacts
* Added cpu usage and updated XLink
* Device side bugfix and XLink update
* Updated style
* Updated a comment
* Made event queue maximum size smaller
* Added checks for getQueueEvent and getQueueEvents functions
* Added some functions to Device and updated FW
* Added queue events and callback
* UX improvements
* Added SystemLogger node and ImgDetections message
* Merge branch 'gen2_develop' into gen2_improvements
* ARM host issue hotfix and style
* Pipeline::remove iterator invalidation hotfix
* Added better exception messages
* Handle SPI Idle Watchdog Failure (`#40 <https://github.com/luxonis/depthai-core/issues/40>`_)
  * Update depthai.cmd ref and make BlobAssetInfo protected member of NeuralNetwork.
  * clangformat changes.
* Removing private test/example.
* Gen2 common objdet (`#34 <https://github.com/luxonis/depthai-core/issues/34>`_)
  Adding DetectionNetwork Node.
* ImageManip: add support for rotated/arbitrary cropping
* Modified getTimestamp convinience function to return time_point
* Updated README
* Merge pull request `#37 <https://github.com/luxonis/depthai-core/issues/37>`_ from luxonis/gen2_camera_orientation
  Color/MonoCamera: add API to configure image orientation
* Update README.md
* Update README.md
* Updated examples cmake for new 'ExecuteTestTimeout' script
* Updated 'ExecuteTestTimeout' script
* Color/MonoCamera: add API to configure image orientation
* Merge branch 'gen2_camera_improvements' into gen2_develop
* Added camera control example
* Updated examples to use modified API
* Added checks to VideoEncoder and updated ColorCamera
* Depthai firmware package decompression hotfix
* Added convinience function for saving .dap file
* ColorCamera - added keep aspect ratio
* Fixed style
* Updated device side FW
* API improvements
* Merge pull request `#32 <https://github.com/luxonis/depthai-core/issues/32>`_ from luxonis/gen2_resource_allocator
  Gen2 resource allocator
* Update device side FW with updated mdk
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* move docs to depthai-python
* Update device FW
* Update FW
* Update FW
* Fix formatting
* Fix potential memory leak in bspatch
* Update device side FW
* Merge remote-tracking branch 'origin/gen2_develop' into HEAD
* Fixed ImageManip test
* Updated device with VideoEncoder improvements
* Update device FW
* Update device side FW
* Update depthai shared w/ supressed warnings
* Removed obsolete NN information
* Device - style fix
* Merge branch 'docs_kickoff' into gen2_develop
* Merge branch 'gen2_develop' into docs_kickoff
* add info about building the project
* Exposed device log level option
* change project name
* remove broken dependency
* Initialize the docs
* Added logging from device side
* WIP: device logging
* Merge pull request `#29 <https://github.com/luxonis/depthai-core/issues/29>`_ from luxonis/gen2_fix_build_ub18
  Fix libarchive build with CMake version < 3.12
* Update libarchive: set cmake minimum to 3.2 for Hunter
* Fix build on Ubuntu18.04 (CMake 3.10), undo libarchive CMake min ver
* Resources loading fix
* Updated libarchive dependency
* Updated firmware - Fixed NN bug
* Applied formatting
* Fixed compilation issues and removed -pedantic flag
* Added multi openvino support, split examples and added tests
* Improved install step for library and its dependecies
* Added FP16 mode for ColorCamera and updated firmware
* Merge branch 'gen2_develop' of github.com:luxonis/depthai-core into gen2_develop
* Single mono camera fix, install 3rdparty public headers
* Fix pipeline when rectified out is disabled
* Updated FW
* Added capability for custom IOs and blocking option on inputs
* Fix for NNData getLayerFp16 crash
* Improved shared library building
* Merge branch 'gen2_stereo' into gen2_develop
* Removed namespace identation style and applied
* Merged gen2_imagemanip into gen2_develop
* Updated 'initialize' function with atomic variable
* VideoEncoder default profile fix
* Updated libarchive
* Updated device-side firmware
* Added VideoEncoder settings and applied style
* Resolve merge conflict
* Repository cleanup
* Added resources, openvino detection, initial tar.xz support, pipeline node removal
* Updating depthai-shared reference.
* Adding SPI node for gen2 (`#26 <https://github.com/luxonis/depthai-core/issues/26>`_)
  * Experiemental changes for SPIOut.
  * Upding depthai-shared.
  * Check in missing file.
  * Checking in demo code.
  * Cleanup and update DepthaiDeviceSideConfig.cmake (the depthai bin).
  * Updating depthai.cmd hunter reference and minor cleanup. Also, checking mobilenet into this branch to make example usage a bit easier.
  * Minor changes to the example app for debugging.
  * Updating depthai.cmd commit id.
  * Updating depthai.cmd with nlohmann json.
  * Adding get jpg example. WIP.
  * Updating depthai.cmd reference.
  * Separating metadata and message data.
  It's ultimately to make transfering larger pieces of data off the esp32 easier.
  By separating metadata and message data we don't have to deal with finding the location of metadata and excluding it from the transmission.
  It was fine with smaller message because we could fit the entire message in memory and parse it.
  * Removing duplicate methods.
  * Checking style changes from clang-format.
  * Updating depthai.cmd reference.
  * Cleaning up some review comments and removing mobilenet blob to save space.
* Cherry-pick Node.cpp changes from 6621aac8
  -- will allow XLinkIn to link to nodes with ImgFrame inputs
* Revert "Change StereoDepth inputs from ImgFrame to Buffer, to allow XLinkIn nodes to link to"
  -- will apply next a proper fix taken from gen2_imagemanip
  This reverts commit a232078c0f89ea27918abc4e20cf0b27516dbc54.
* Merge remote-tracking branch 'origin/gen2' into gen2_stereo
* Apply clangformat corrections
* Refactor after removing `tl::optional` from calibration props
* Added ImageManip node
* Update depthai-shared and FW
* Update depthai-shared and device FW after merging
  gen2_updates_from_develop
* Update device FW after merge with latest `gen2`
* C++ example improvements:
  - basic argument parsing
  - "d" / "debug" option to run with debugger
* Merge remote-tracking branch 'origin/gen2' into gen2_stereo
* Add API StereoDepth::setEmptyCalibration,
  for already rectified input images.
  Update device FW, StereoDepth working with images received from host
* Change StereoDepth inputs from ImgFrame to Buffer, to allow
  XLinkIn nodes to link to
* ImgFrame::setWidth: initialize also 'stride'
* Added capability to find a device by MxId
* Updated device, bootloader and XLink
* Update FW: rectified working with subpixel / extended disp
* WIP: getting device by mx id
* Merge branch 'gen2' of github.com:luxonis/depthai-core into gen2
* Added local XLink option
* StereoDepth: update FW
* Add StereoDepth node and example.
  Run: ./myapp depth
  Device FW not yet updated, TODO
* MonoCam example: quit when 'q' is pressed
* Add FPS config for MonoCamera and ColorCamera
* Disabled xlink logging
* Renamed major and minor
* Fixed lambda and style
* Merge branch 'gen2_bootloader' of github.com:luxonis/depthai-core into gen2_bootloader
* Updated XLink
* Fixed lambda capture
* Fixed formatting
* Added bootloader support and flashing
* WIP: Added class for flashing
* WIP: bootloader support
* Fix MonoCamera after last updates
  (the branch was rebased)
* Add MonoCamera node and example (left+right streams):
  build in example/build, run: ./myapp mono
* Added getName to data queues, removed deprecated depencencies
* Added NNData functions and removed some gen1 stuff
* Added layer between raw data and helping functions
* Updated DataQueue
* Formatted with clang-format
* Merge branch 'gen2' of github.com:luxonis/depthai-core into gen2
* Added watchdog and improved disconnect and end of program handling
* increase boot time to 5s
* WIP: XLink connection fix and NNTensor rename
* Updated formatting and device side binary downloading
* DataQueue exception on connection failure
* Added extra example, updated device side
* Added timeout capabilities to queues
* Formatted and added missing headers
* Added parameter to specify clang-tidy bin
* Added clang-tidy to CI
* Updated CI workflow and some changes
* Made clangformat optional
* Added dependencies and submodules
* Added initial CI
* Format using clang-format
* Updated tidy and format and added VideoEncoder node
* Fixed booting
* Added clang-tidy and format
* Created cpp files for nodes
* Added XLinkIn node
* Improved some API aspects
* PoC working
* DataQueues, Callbacks and Assets
* Initial gen2 commit
* Contributors: CsabaGergely, Dale Phurrough, Erik, Erol444, Ibai Gorordo, Jon Ngai, Kunal Tyagi, Martin Peterlin, Mihir Patil, OanaMariaVatavu, Onuralp Sezer, Otto Seiskari, Philip Molloy, Sachin, Sachin Guruswamy, SzabolcsGergely, TheMarpe, alex-luxonis, csaba-luxonis, jonngai, saching13, slitcch, szabi-luxonis, Łukasz Piłatowski

1.0.0 (2021-02-26)
------------------
* Gen1: Release 1.0.0.0
* Merge pull request `#68 <https://github.com/luxonis/depthai-core/issues/68>`_ from luxonis/develop
  Merge latest gen1 changes into main
* Merge pull request `#58 <https://github.com/luxonis/depthai-core/issues/58>`_ from kunaltyagi/amend.gitmodules
  Allow recursive-clone with forks
* Allow recursive-clone with forks
* Update FW: fix -fusb2,
  USB descriptor was improperly set up,
  causing enumeration failure on Windows
* Merge pull request `#45 <https://github.com/luxonis/depthai-core/issues/45>`_ from luxonis/pre_release_041
  Bump core version; include fixes for Xlink
* Merge pull request `#41 <https://github.com/luxonis/depthai-core/issues/41>`_ from luxonis/calibration_info
  Changed calibration file read and console information (gen1)
* Contributors: Kunal Tyagi, Sachin Guruswamy, SzabolcsGergely, TheMarpe, alex-luxonis, szabi-luxonis

0.4.1 (2021-01-22)
------------------
* Merge pull request `#46 <https://github.com/luxonis/depthai-core/issues/46>`_ from luxonis/pre_release_041
  Release 0.4.1.1
* Bump core version; include fixes for Xlink
* changed the debug print
* Merge remote-tracking branch 'origin/develop' into calibration_info_1
* changed debug statements for calibration file
* Update FW: rotate RGB camera view 180deg on OAK-1 (Gen1)
* Update FW: increase_manual_exposure_limits
* Contributors: SzabolcsGergely, alex-luxonis, saching13, szabi-luxonis

0.4.0 (2020-12-02)
------------------
* Merge pull request `#25 <https://github.com/luxonis/depthai-core/issues/25>`_ from luxonis/release_0_4_0
  Release 0.4.0
* Bump version 0.4.0
* Merge pull request `#24 <https://github.com/luxonis/depthai-core/issues/24>`_ from luxonis/rgb_fixes
  Update device FW with fixes for RGB
* Update device FW with fixes for RGB:
  - fix crash with RGB 12MP + depth
  - fix the cropping for 4K (make it centered)
* Merge pull request `#14 <https://github.com/luxonis/depthai-core/issues/14>`_ from luxonis/usb-testing
  Fetching USB speed and Myriad X serial number
* updated xlink and device side commit
* updated device side sha id
* modified is_usb3()
* updated depthai-shared and changed device side commit id
* added wrapper for xlinkConnect to make it threadsafe in depthai-shared
* changed xlink path
* Merge pull request `#22 <https://github.com/luxonis/depthai-core/issues/22>`_ from luxonis/fix_recreate_device_loop
  Fix crash on 2nd device object delete
* Fix crash on 2nd device delete, due to watchdog thread not recreated.
  Now running create+delete in a loop should work fine.
* renamed api to write_eeprom_data
* added api to write to eeprom and fetch existing pipeline
* Update FW, bugfix for config_h2d handling after initial setup
* Separate config_d2h handling in a new function,
  add test code to call it again at the end of create_pipeline.
  Add test code for sending a new config_h2d, TODO create API
  for sending a new calib file/structure.
* Device support for sending/receiving config_d2h/config_h2d
  (in this order, possible multiple times) after the initial setup,
  if the host sets app.enable_reconfig (default:true) in the first config.
  Report EEPROM write status as 'logs' in the "meta_d2h" stream. Possible values:
  "EEPROM cleared"
  "EEPROM write OK"
  "EEPROM write FAILED"
* Do not crash when mat_mul fails, return an empty vector instead.
  Fixes a host app crash when the device calib data is full-zero.
  Was reproducible by writing to EEPROM a calib file generated by:
  dd if=/dev/zero of=resources/depthai.calib bs=1 count=445
* Merge remote-tracking branch 'origin/develop' into usb-testing
* Merge pull request `#21 <https://github.com/luxonis/depthai-core/issues/21>`_ from luxonis/release_0_3_0_0
  Release 0.3.0
* added api to check cameras connection
* added device change check on swapping devices for testing
* added develop merged depthai device side
* updated device side hash
* Merge remote-tracking branch 'origin/develop' into HEAD
* updatedd shared
* merge on updates
* added usb speed fetch
* updatedd shared
* added print statement on calibration file path to verify with mx id
* updated link to xlink
* updated link to xlink
* updated link to xlink
* updated link to xlink
* updated xlink config cmake
* updated device side config
* modified mx_id and is_usb3 api to use link id for multi device
* updated link to Xlink in config.cmake of hunter
* modified hunter cmake config
* changed XLink wrapped init from Host functions to not take speed and mx serial id as args and added getters in XLinkwrapper to fetch usb speed and mx serial id from XLink
* added eeprom loaded check
* updated hash commit to device and XLink
* update depthai-core
* modifed the Xlink wrapper to get myriad x serial id along with usb speed and added an api to return myriad x id for calibration saving of multiple devices
* updated device commit id
* Merge branch 'usb-testing' of https://github.com/luxonis/depthai-core into HEAD
* updated on merge
* added is_usb3() check
* updating submodule linking
* modifed hunter config
* updated wrapper to write usb speed
* updated for usb fetch wip
* added usb speed fetch
* added is_usb3() check
* updating submodule linking
* modifed hunter config
* updated wrapper to write usb speed
* updated for usb fetch wip
* added usb speed fetch
* Contributors: Luxonis-Brandon, Sachin Guruswamy, alex-luxonis, saching13, Łukasz Piłatowski

0.3.0 (2020-10-26)
------------------
* bump core version to 0.3.0
* Merge pull request `#19 <https://github.com/luxonis/depthai-core/issues/19>`_ from luxonis/develop
  Develop merge - release 0.3.0.0
* Merge pull request `#20 <https://github.com/luxonis/depthai-core/issues/20>`_ from luxonis/fix_xyz_calc
  Update device FW: fix X,Y,Z calc on NN bounding box
* Update device FW: fix X,Y,Z calc on NN bounding box
* Update depthai-shared after previous PR merge
* Merge pull request `#17 <https://github.com/luxonis/depthai-core/issues/17>`_ from luxonis/output_format_checks
  Output format checks
* Add check for 2 stage network
* Check if output format matches the one in config
* Merge pull request `#16 <https://github.com/luxonis/depthai-core/issues/16>`_ from luxonis/gen1_updates
  Gen1 updates: FrameMetadata all streams, sync seqNo, IMX477 RPi HQcam
* Add an option 'usb_chunk_KiB', useful to improve throughput
* Update XLink package: fix_boot_multiple_512
* Add experimental option 'sync_sequence_numbers'
* Receive metadata for all streams
* Correct FrameMetadata bytesPP for disparity_color
* Merge pull request `#15 <https://github.com/luxonis/depthai-core/issues/15>`_ from luxonis/fix_confidence_threshold
  Remove depth confidence threshold
* Merge remote-tracking branch 'origin/develop' into fix_confidence_threshold
* Throw error if depth confidence threshold is used
* Updated shared
* Remove depth_confidence threshold
* Merge pull request `#13 <https://github.com/luxonis/depthai-core/issues/13>`_ from luxonis/fix_nn_stereo_rectified
  Fix a scaling issue or crash with NN on rectified frames
* Update FW: fix sporadic scaling issue or crash,
  when rectified streams are used for NN input
* Merge pull request `#12 <https://github.com/luxonis/depthai-core/issues/12>`_ from luxonis/nn_stereo_rectified
  Option to run NN on stereo rectified streams
* Update device commit: support for NN on stereo rectified frames
* Merge pull request `#9 <https://github.com/luxonis/depthai-core/issues/9>`_ from luxonis/isp_3a_controls_develop
  Add ISP 3A camera controls
* Merge remote-tracking branch 'origin/develop' into isp_3a_controls_develop
* Update depthai-shared (and device side with it)
* Merge pull request `#8 <https://github.com/luxonis/depthai-core/issues/8>`_ from luxonis/openvino_tensor_representation
  Openvino tensor representation
* Fix default camera resolution/fps if not specified
* Update device side SHA (depthai-shared changed)
* Slightly refactor CameraControl API.
  Address review comments.
* Update depthai firmware hash
* Update firmware hash
* Update device side firmware hash
* Remove redundant tensor\_ prefix
* Address PR review; use make_shared instead shared_ptr()
* Add ISP 3A camera controls
* Remove leftover file, it was moved to depthai-shared
* Merge remote-tracking branch 'origin/develop' into HEAD
* Use get_dimension instead raw buffer
* Move tensor_info and half.hpp into shared submodule
* Add support to run network without user defined JSON
* Remove unused files
* Move packet data raw data allocation on heap, wrapped in shared_ptr
* Add input,output tensor descriptor getter; improve API
* Merge pull request `#7 <https://github.com/luxonis/depthai-core/issues/7>`_ from luxonis/calibration_fix
  Calibration fix when EEPROM is not programmed.
* clearing calibration variables to avoid init_device from appending when watch_dog is called
* Fix issues with calibration when EEPROM is unprogrammed
* Fix issues with calibration when EEPROM is unprogrammed
* Merge in initial version of openvino tensor representation
* Merge pull request `#6 <https://github.com/luxonis/depthai-core/issues/6>`_ from luxonis/rename_depthraw
  Rename depth_raw to depth
* Fix build error
* Change git submodule url to relative path to inherit ssh/https option
* Rename depth_raw to depth
* Merge pull request `#4 <https://github.com/luxonis/depthai-core/issues/4>`_ from luxonis/enhance_stere_depth
  Enhance stereo depth develop merge
* Update FW: rectified L/R swapping corrected with mirroring disabled
* Merge remote-tracking branch 'origin/develop' into HEAD
* bug fix for version 3
* added version check on calibration adjustments
* Merge branch 'enhance_stere_depth' of https://github.com/luxonis/depthai-core into HEAD
  adding some comments after the push
* added comments
* Update FrameMetadata .frameSize at producer for disparity_color
* added math ops. mesh WIP and disbaled
* Merge branch 'enhance_stere_depth' of https://github.com/luxonis/depthai-core into HEAD
  merge for version 5 EEPROM to store R1, R2 and other calibration data and inverse matrix calculations on the host.
* added LU decomposition
* Update FW internal commit hash
* Distortion coefficients array length changed: 12 -> 14
* For V5, pass calib_R1/R2 from device instead of H1/H2
* tested and fixed some syntax errors. Added version check and abort on EEPROM write.
* replacing homography with R1 and R2 and adding distortion coefficients along with place holders for rgb camera calibrations.
* updated get_translation() and modified get_right_homography() to work on single homography version
* Merge branch 'raw-color' into develop
* Merge with 'develop'
* Updated device side binary
* Updated shared
* Merge remote-tracking branch 'origin/main' into enhance_stere_depth
* Updated device binary
* Updated depthai-shared and fix for packet data size
* Update firmware commit hash
* Merge changes from enhance_stereo_depth into develop
* Contributors: GergelySzabolcs, Martin Peterlin, SzabolcsGergely, TheMarpe, alex-luxonis, iamsg-luxonis, saching13, szabi-luxonis, Łukasz Piłatowski

0.2.0 (2020-09-04)
------------------
* Bump version to 0.2.0
* Added documentation and depthai-shared check
* Fixed XLink hunter config
* Updated XLink to address libusb issue
* Added C language support for bspatch
* Merge pull request `#2 <https://github.com/luxonis/depthai-core/issues/2>`_ from luxonis/develop_merge
  Merge latest master into develop
* remove libcurl; update submodules
* update firmware commit hash
* fix build error after merge
* update w/ latest master
* Merge remote-tracking branch 'origin/develop' into HEAD
* Added build information
* Replace variable size array with std::vector
* update device side commit
* add message if libcurl is^Cnabled/disabled
* temporary disable of libcurl cpp backend
* fix 2 stage NN
* fix watchdog in develop
* fix watchdog
* merge develop with latest master
* Merge branch 'develop' of github.com:luxonis/depthai-core into develop
* Updated Hunter
* Updated Hunter
* Updated XLink and JSON Schema Validator
* Capability to use local boost libraries
* Updated object tracker, RC, depthai and shared repo
* Updated CMakeRC to themarpe fork
* Added version
* Updated device side, fixed some issues and added blocking read
* fix assertion failed on max tracklet size
* Added AF and JPEG requests
* Updated XLink dependency
* Merge branch 'develop' into refactor
* add configurable max tracklet and confidence for object tracker
* Updated structure - public include directory
* Fixed example
* Moved core to its own repository
* Merge remote-tracking branch 'origin/object-tracker' into HEAD
* fix build on mac
* Merge branch 'nn_on_left_right' into develop
* Merge branch 'autofocus_feature' into develop
* simplify tracklet decoder
* return tracklet status as string
* fix std::isnan/isinf dependencies
* Merge remote-tracking branch 'origin/master' into HEAD
* add object traker decode in cpp backend
* Merge remote-tracking branch 'origin/master' into nn_on_left_right
* NNetPacket: add getMetadata routine
* FrameMetadata: add getCameraName routine
* Fixed a naming issue
* Added autofocus mode and trigger
* Merged jpeg_h264_feature into master
* Merged master into jpeg_h264_feature
* Merge remote-tracking branch 'origin/master' into HEAD
* add object tracker stream
* Merge pull request `#25 <https://github.com/luxonis/depthai-core/issues/25>`_ from luxonis/usb-loglevel
  fix std::system_error; set usb_loglevel to 1
* fix std::system_error; set usb_loglevel to 1
* Updated depthai-shared
* Merge pull request `#20 <https://github.com/luxonis/depthai-core/issues/20>`_ from luxonis/configurable-confidence-padding
  add configurable depth confidence threshold; remove limit for padding
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge pull request `#23 <https://github.com/luxonis/depthai-core/issues/23>`_ from ConnorChristie/cpp-compile
  Move python getMetadata function into PY conditional
* Merge pull request `#21 <https://github.com/luxonis/depthai-core/issues/21>`_ from luxonis/low-packet-size-fix
  Don't limit packet sizes to > 16
* Added C++11 standard
* WIP: Windows support
* Updated depthai-shared
* Update README.md
* Initial refactoring commit and patching capability
* Merge pull request `#18 <https://github.com/luxonis/depthai-core/issues/18>`_ from luxonis/sanitize-nn-output
  Sanitize NN output (NaN, Infinite); handle corrupted frames
* Move python getMetadata call into PY conditional
* implement LRU policy for frame queue
* return Null object on failure
* Pass an option 'ai'.'camera_input' to device
* Merge branch 'low-packet-size-fix' into HEAD
* don't limit packet sizes to > 16
* add configurable depth confidence threshold; remove limit for padding
* Merge remote-tracking branch 'origin/master' into sanitize-nn-output
* Merge pull request `#16 <https://github.com/luxonis/depthai-core/issues/16>`_ from luxonis/enhance_roi_depth_mapping
  Pass the NN-to-depth-bbox-mapping from device
* Merge remote-tracking branch 'origin/master' into HEAD
* Pass an option ai.keep_aspect_ratio to device, default true
* Merge remote-tracking branch 'origin/master' into enhance_roi_depth_mapping
* Pass a depth limit for XYZ calculation to device
* Merge pull request `#17 <https://github.com/luxonis/depthai-core/issues/17>`_ from luxonis/wd-segfault-fix
  Watchdog and segfault fix
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge pull request `#19 <https://github.com/luxonis/depthai-core/issues/19>`_ from luxonis/temp-metadata
  add temperature metadata on-demand
* Merge remote-tracking branch 'origin/master' into enhance_roi_depth_mapping
* Merge remote-tracking branch 'origin/temp-metadata' into HEAD
* add temperature metadata on-demand
* Added capability to change encoder settings
* reduce verbosity
* reduce device deinit time
* reduce queue size from 100 to 30
* discard tensorentry if it's nan or infinity
* guard XLinkResetRemote w/ ifdef PC
* Merge remote-tracking branch 'origin/master' into wd-segfault-fix
* fix watchdog with latest Xlink from openvino 2020.1 and segmentation fault at exit
* Merge branch 'jpeg_h264_feature'
* Merge branch 'master' into jpeg_h264_feature
* Pass the NN to depth bbox mapping from device
* Merge pull request `#13 <https://github.com/luxonis/depthai-core/issues/13>`_ from luxonis/source-code-cleanup
  Cleanup source code; add depth_color_h fps limitation
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge pull request `#9 <https://github.com/luxonis/depthai-core/issues/9>`_ from luxonis/eeprom_board_type
  Pass more data in board config: board name, revision, RGB FOV
* Merge pull request `#15 <https://github.com/luxonis/depthai-core/issues/15>`_ from luxonis/eeprom_print__lr_center
  Display EEPROM data, center L/R cameras
* Print EEPROM data
* Add 'stereo_center_crop' option, enabled from calib file or config
* Rename 'override_eeprom_calib' to 'override_eeprom'
* Merge remote-tracking branch 'origin/master' into eeprom_board_type
* Merge pull request `#14 <https://github.com/luxonis/depthai-core/issues/14>`_ from philnelson/master
  Fix (harmless) warning on Darwin systems
* Fix (harmless) warning on Darwin systems
  Use sysctl on mac/darwin systems to remove a totally harmless warning that bugged me.
* cleanup source code; add depth_color_h fps limitation
* Merge branch 'master' into eeprom_board_type
* Merge pull request `#8 <https://github.com/luxonis/depthai-core/issues/8>`_ from luxonis/multi-output-tensor
  Multi output tensor
* Merge remote-tracking branch 'origin/master' into HEAD
* Increased watchdog timeout
* Merge remote-tracking branch 'origin/master' into jpeg_h264_feature
* Merge branch 'frame_metadata'
  Added frame metadata functionality to master
* Added video stream
* JPEG request command
* Merge branch 'master' into jpeg_h264_feature
  Updating current branch to continue development
* Pass more data in board config: board name, revision, RGB FOV
* Implemented metadata for left and right streams
* stop watchdog thread only if it was started
* Merge remote-tracking branch 'origin/master' into HEAD
* Merge remote-tracking branch 'origin/master' into multi-output-tensor
* add parser for multiple output CNN models
* WIP: Frame metadata
* WIP: JPEG still image encoding
* Merge pull request `#7 <https://github.com/luxonis/depthai-core/issues/7>`_ from luxonis/macos_build
  DepthAI-API support for macOS
* Update Readme with support for macOS
* /proc/meminfo doesn't exist on macOS. Just don't limit 'make' jobs
* Avoid lots of warnings during final link on macOS
* Fix py_module build on macOS
* Update dldt submodule to 2020.1
* Merge pull request `#6 <https://github.com/luxonis/depthai-core/issues/6>`_ from luxonis/eeprom_calib
  Pass options to store calibration and board parameters in EEPROM
* Pass options to store calibration and board parameters in EEPROM
* Merge pull request `#5 <https://github.com/luxonis/depthai-core/issues/5>`_ from luxonis/host_watchdog_debug
  Depthai USB watchdog
* add default version info
* Change error code to 9 on init_device error in watchdog
* Merge remote-tracking branch 'origin/master' into HEAD
* reduce the number of retries from 3 to 1
* Merge pull request `#4 <https://github.com/luxonis/depthai-core/issues/4>`_ from luxonis/multi_device_support
  Multi device support
* Merge remote-tracking branch 'origin/multi_device_support' into host_watchdog_debug
* cleanup; remove unused functions
* Pass a USB port number of 'list' to init_device
* debug RPI crash
* implement watchdog in cpp backend
* add deinit function to pipeline with python binding
* Create LICENSE
* Update README.md - reference issue 3
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Refactor XLink init code. Fix for init hang on RPi4 w/USB3
  (use new API that no longer tries to match pre- and post-boot USB device addresses).
  Minor prints cleanup.
* Fix build hang for memory constrained hosts
* install_dependencies.sh: fix check when ran from other dir
  (e.g: ./depthai-api/install_dependencies.sh)
* Initial import of the host source code
* Initial empty commit
* Contributors: Connor Christie, GergelySzabolcs, Luxonis-Brandon, Martin Peterlin, Phil Nelson, TheMarpe, alex-luxonis, iamsg, iamsg-luxonis, szabi-luxonis, Łukasz Piłatowski
