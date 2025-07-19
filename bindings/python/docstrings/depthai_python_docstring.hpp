/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by pybind11_mkdoc.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif


static const char *__doc_DetectionNetworkType = R"doc()doc";

static const char *__doc_DetectionNetworkType_MOBILENET = R"doc()doc";

static const char *__doc_DetectionNetworkType_YOLO = R"doc()doc";

static const char *__doc_ZSTD_CCtx_s = R"doc()doc";

static const char *__doc_dai = R"doc()doc";

static const char *__doc_dai_2 = R"doc()doc";

static const char *__doc_dai_3 = R"doc()doc";

static const char *__doc_dai_4 = R"doc()doc";

static const char *__doc_dai_5 = R"doc()doc";

static const char *__doc_dai_ADatatype = R"doc(Abstract message)doc";

static const char *__doc_dai_ADatatype_ADatatype = R"doc()doc";

static const char *__doc_dai_ADatatype_data = R"doc()doc";

static const char *__doc_dai_ADatatype_serialize = R"doc()doc";

static const char *__doc_dai_Affine = R"doc()doc";

static const char *__doc_dai_Affine_Affine = R"doc()doc";

static const char *__doc_dai_Affine_Affine_2 = R"doc()doc";

static const char *__doc_dai_Affine_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Affine_matrix = R"doc()doc";

static const char *__doc_dai_Affine_str = R"doc()doc";

static const char *__doc_dai_Affine_toStr = R"doc()doc";

static const char *__doc_dai_AprilTag = R"doc(AprilTag structure.)doc";

static const char *__doc_dai_AprilTagConfig = R"doc(AprilTagConfig message.)doc";

static const char *__doc_dai_AprilTagConfig_AprilTagConfig = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family = R"doc(Supported AprilTag families.)doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_16H5 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_25H9 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_36H10 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_36H11 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_CIR21H7 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_Family_TAG_STAND41H12 = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds = R"doc(AprilTag quad threshold parameters.)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_criticalDegree =
R"doc(Reject quads where pairs of edges have angles that are close to straight or
close to 180 degrees. Zero means that no quads are rejected. (In degrees).)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_deglitch = R"doc(Should the thresholded image be deglitched? Only useful for very noisy images)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_maxLineFitMse =
R"doc(When fitting lines to the contours, what is the maximum mean squared error
allowed? This is useful in rejecting contours that are far from being quad
shaped; rejecting these quads "early" saves expensive decoding processing.)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_maxNmaxima =
R"doc(How many corner candidates to consider when segmenting a group of pixels into a
quad.)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_minClusterPixels = R"doc(Reject quads containing too few pixels.)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_minWhiteBlackDiff =
R"doc(When we build our model of black & white pixels, we add an extra check that the
white model must be (overall) brighter than the black model. How much brighter?
(in pixel values: [0,255]).)doc";

static const char *__doc_dai_AprilTagConfig_QuadThresholds_str = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_decodeSharpening =
R"doc(How much sharpening should be done to decoded images? This can help decode small
tags but may or may not help in odd lighting conditions or low light conditions.
The default value is 0.25.)doc";

static const char *__doc_dai_AprilTagConfig_family = R"doc(AprilTag family.)doc";

static const char *__doc_dai_AprilTagConfig_maxHammingDistance =
R"doc(Max number of error bits that should be corrected. Accepting large numbers of
corrected errors leads to greatly increased false positive rates. As of this
implementation, the detector cannot detect tags with a hamming distance greater
than 2.)doc";

static const char *__doc_dai_AprilTagConfig_quadDecimate =
R"doc(Detection of quads can be done on a lower-resolution image, improving speed at a
cost of pose accuracy and a slight decrease in detection rate. Decoding the
binary payload is still done at full resolution.)doc";

static const char *__doc_dai_AprilTagConfig_quadSigma =
R"doc(What Gaussian blur should be applied to the segmented image. Parameter is the
standard deviation in pixels. Very noisy images benefit from non-zero values
(e.g. 0.8).)doc";

static const char *__doc_dai_AprilTagConfig_quadThresholds = R"doc(AprilTag quad threshold parameters.)doc";

static const char *__doc_dai_AprilTagConfig_refineEdges =
R"doc(When non-zero, the edges of the each quad are adjusted to "snap to" strong
gradients nearby. This is useful when decimation is employed, as it can increase
the quality of the initial quad estimate substantially. Generally recommended to
be on. Very computationally inexpensive. Option is ignored if quadDecimate = 1.)doc";

static const char *__doc_dai_AprilTagConfig_serialize = R"doc()doc";

static const char *__doc_dai_AprilTagConfig_setFamily =
R"doc(Parameter ``family``:
    AprilTag family)doc";

static const char *__doc_dai_AprilTagConfig_str = R"doc()doc";

static const char *__doc_dai_AprilTagProperties = R"doc(Specify properties for AprilTag)doc";

static const char *__doc_dai_AprilTagProperties_initialConfig = R"doc()doc";

static const char *__doc_dai_AprilTagProperties_inputConfigSync = R"doc(Whether to wait for config at 'inputConfig' IO)doc";

static const char *__doc_dai_AprilTagProperties_numThreads = R"doc(How many threads to use for AprilTag detection)doc";

static const char *__doc_dai_AprilTag_bottomLeft = R"doc(The detected bottom left coordinates.)doc";

static const char *__doc_dai_AprilTag_bottomRight = R"doc(The detected bottom right coordinates.)doc";

static const char *__doc_dai_AprilTag_decisionMargin =
R"doc(A measure of the quality of the binary decoding process; the average difference
between the intensity of a data bit versus the decision threshold. Higher
numbers roughly indicate better decodes. This is a reasonable measure of
detection accuracy only for very small tags-- not effective for larger tags
(where we could have sampled anywhere within a bit cell and still gotten a good
detection.)doc";

static const char *__doc_dai_AprilTag_hamming =
R"doc(How many error bits were corrected? Note: accepting large numbers of corrected
errors leads to greatly increased false positive rates. As of this
implementation, the detector cannot detect tags with a hamming distance greater
than 2.)doc";

static const char *__doc_dai_AprilTag_id = R"doc(The decoded ID of the tag)doc";

static const char *__doc_dai_AprilTag_topLeft = R"doc(The detected top left coordinates.)doc";

static const char *__doc_dai_AprilTag_topRight = R"doc(The detected top right coordinates.)doc";

static const char *__doc_dai_AprilTags = R"doc(AprilTags message.)doc";

static const char *__doc_dai_AprilTags_AprilTags = R"doc(Construct AprilTags message.)doc";

static const char *__doc_dai_AprilTags_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_AprilTags_aprilTags = R"doc()doc";

static const char *__doc_dai_AprilTags_serialize = R"doc()doc";

static const char *__doc_dai_AprilTags_str = R"doc()doc";

static const char *__doc_dai_Asset = R"doc(Asset is identified with string key and can store arbitrary binary data)doc";

static const char *__doc_dai_AssetManager = R"doc(AssetManager can store assets and serialize)doc";

static const char *__doc_dai_AssetManager_AssetManager = R"doc()doc";

static const char *__doc_dai_AssetManager_AssetManager_2 = R"doc()doc";

static const char *__doc_dai_AssetManager_addExisting =
R"doc(Adds all assets in an array to the AssetManager

Parameter ``assets``:
    Vector of assets to add)doc";

static const char *__doc_dai_AssetManager_assetMap = R"doc()doc";

static const char *__doc_dai_AssetManager_get =
R"doc(Returns:
    Asset assigned to the specified key or a nullptr otherwise)doc";

static const char *__doc_dai_AssetManager_get_2 =
R"doc(Returns:
    Asset assigned to the specified key or a nullptr otherwise)doc";

static const char *__doc_dai_AssetManager_getAll =
R"doc(Returns:
    All asset stored in the AssetManager)doc";

static const char *__doc_dai_AssetManager_getAll_2 =
R"doc(Returns:
    All asset stored in the AssetManager)doc";

static const char *__doc_dai_AssetManager_getRelativeKey = R"doc()doc";

static const char *__doc_dai_AssetManager_getRootPath =
R"doc(Get root path of the asset manager

Returns:
    Root path)doc";

static const char *__doc_dai_AssetManager_remove =
R"doc(Removes asset with key

Parameter ``key``:
    Key of asset to remove)doc";

static const char *__doc_dai_AssetManager_rootPath = R"doc()doc";

static const char *__doc_dai_AssetManager_serialize = R"doc(Serializes)doc";

static const char *__doc_dai_AssetManager_set =
R"doc(Adds or overwrites an asset object to AssetManager.

Parameter ``asset``:
    Asset to add

Returns:
    Shared pointer to asset)doc";

static const char *__doc_dai_AssetManager_set_2 =
R"doc(Adds or overwrites an asset object to AssetManager with a specified key. Key
value will be assigned to an Asset as well

Parameter ``key``:
    Key under which the asset should be stored

Parameter ``asset``:
    Asset to store

Returns:
    Shared pointer to asset)doc";

static const char *__doc_dai_AssetManager_set_3 =
R"doc(Loads file into asset manager under specified key.

Parameter ``key``:
    Key under which the asset should be stored

Parameter ``path``:
    Path to file which to load as asset

Parameter ``alignment``:
    [Optional] alignment of asset data in asset storage. Default is 64B)doc";

static const char *__doc_dai_AssetManager_set_4 =
R"doc(Loads file into asset manager under specified key.

Parameter ``key``:
    Key under which the asset should be stored

Parameter ``data``:
    Asset data

Parameter ``alignment``:
    [Optional] alignment of asset data in asset storage. Default is 64B

Returns:
    Shared pointer to asset)doc";

static const char *__doc_dai_AssetManager_set_5 = R"doc()doc";

static const char *__doc_dai_AssetManager_setRootPath =
R"doc(Set root path of the asset manager

Parameter ``rootPath``:
    Root path)doc";

static const char *__doc_dai_AssetManager_size =
R"doc(Returns:
    Number of asset stored in the AssetManager)doc";

static const char *__doc_dai_AssetView = R"doc()doc";

static const char *__doc_dai_AssetView_AssetView = R"doc()doc";

static const char *__doc_dai_AssetView_alignment = R"doc()doc";

static const char *__doc_dai_AssetView_data = R"doc()doc";

static const char *__doc_dai_AssetView_size = R"doc()doc";

static const char *__doc_dai_Asset_Asset = R"doc()doc";

static const char *__doc_dai_Asset_Asset_2 = R"doc()doc";

static const char *__doc_dai_Asset_alignment = R"doc()doc";

static const char *__doc_dai_Asset_data = R"doc()doc";

static const char *__doc_dai_Asset_getRelativeUri = R"doc()doc";

static const char *__doc_dai_Asset_key = R"doc()doc";

static const char *__doc_dai_Assets = R"doc()doc";

static const char *__doc_dai_AssetsMutable = R"doc()doc";

static const char *__doc_dai_AssetsMutable_set = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal_alignment = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal_offset = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal_size = R"doc()doc";

static const char *__doc_dai_Assets_AssetInternal_str = R"doc()doc";

static const char *__doc_dai_Assets_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Assets_get = R"doc()doc";

static const char *__doc_dai_Assets_getAll = R"doc()doc";

static const char *__doc_dai_Assets_has = R"doc()doc";

static const char *__doc_dai_Assets_map = R"doc()doc";

static const char *__doc_dai_Assets_pStorageStart = R"doc()doc";

static const char *__doc_dai_Assets_setStorage = R"doc()doc";

static const char *__doc_dai_Assets_str = R"doc()doc";

static const char *__doc_dai_AtomicBool = R"doc()doc";

static const char *__doc_dai_AtomicBool_AtomicBool = R"doc()doc";

static const char *__doc_dai_AtomicBool_AtomicBool_2 = R"doc()doc";

static const char *__doc_dai_AtomicBool_AtomicBool_3 = R"doc()doc";

static const char *__doc_dai_AtomicBool_operator_assign = R"doc()doc";

static const char *__doc_dai_AtomicBool_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_BenchmarkInProperties = R"doc(Specify benchmark properties (number of messages to send/receive))doc";

static const char *__doc_dai_BenchmarkInProperties_attachLatencies = R"doc(Specify whether the latenices are attached to the report individually)doc";

static const char *__doc_dai_BenchmarkInProperties_logReportsAsWarnings = R"doc(Send the reports also as logger warnings)doc";

static const char *__doc_dai_BenchmarkInProperties_reportEveryNMessages = R"doc(Specify how many messages to measure for each report)doc";

static const char *__doc_dai_BenchmarkOutProperties = R"doc(Specify benchmark properties (number of messages to send/receive))doc";

static const char *__doc_dai_BenchmarkOutProperties_fps = R"doc(FPS for sending, 0 means as fast as possible)doc";

static const char *__doc_dai_BenchmarkOutProperties_numMessages = R"doc(Number of messages to send)doc";

static const char *__doc_dai_BenchmarkReport = R"doc(BenchmarkReport message.)doc";

static const char *__doc_dai_BenchmarkReport_BenchmarkReport = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_averageLatency = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_fps = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_latencies = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_numMessagesReceived = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_serialize = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_str = R"doc()doc";

static const char *__doc_dai_BenchmarkReport_timeTotal = R"doc()doc";

static const char *__doc_dai_BoardConfig = R"doc()doc";

static const char *__doc_dai_BoardConfig_Camera = R"doc(Camera description)doc";

static const char *__doc_dai_BoardConfig_Camera_name = R"doc()doc";

static const char *__doc_dai_BoardConfig_Camera_orientation = R"doc()doc";

static const char *__doc_dai_BoardConfig_Camera_sensorType = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO = R"doc(GPIO config)doc";

static const char *__doc_dai_BoardConfig_GPIO_Direction = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Direction_INPUT = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Direction_OUTPUT = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Drive = R"doc(Drive strength in mA (2, 4, 8 and 12mA))doc";

static const char *__doc_dai_BoardConfig_GPIO_Drive_MA_12 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Drive_MA_2 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Drive_MA_4 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Drive_MA_8 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO_2 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO_3 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO_4 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO_5 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_GPIO_6 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Level = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Level_HIGH = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Level_LOW = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_0 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_1 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_2 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_3 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_4 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_5 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_ALT_MODE_6 = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Mode_DIRECT = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Pull = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Pull_BUS_KEEPER = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Pull_NO_PULL = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Pull_PULL_DOWN = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_Pull_PULL_UP = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_direction = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_drive = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_level = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_mode = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_pull = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_schmitt = R"doc()doc";

static const char *__doc_dai_BoardConfig_GPIO_slewFast = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_IMU = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_boot = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_bus = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_csGpio = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_interrupt = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_reset = R"doc()doc";

static const char *__doc_dai_BoardConfig_IMU_wake = R"doc()doc";

static const char *__doc_dai_BoardConfig_Network = R"doc(Network configuration)doc";

static const char *__doc_dai_BoardConfig_Network_mtu =
R"doc(Network MTU, 0 is auto (usually 1500 for Ethernet) or forwarded from bootloader
(not yet implemented there). Note: not advised to increase past 1500 for now)doc";

static const char *__doc_dai_BoardConfig_Network_xlinkTcpNoDelay =
R"doc(Sets the `TCP_NODELAY` option for XLink TCP sockets (disable Nagle's algorithm),
reducing latency at the expense of a small hit for max throughput. Default is
`true`)doc";

static const char *__doc_dai_BoardConfig_UART = R"doc(UART instance config)doc";

static const char *__doc_dai_BoardConfig_UART_tmp = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB = R"doc(USB related config)doc";

static const char *__doc_dai_BoardConfig_USB_flashBootedPid = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_flashBootedVid = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_manufacturer = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_maxSpeed = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_pid = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_productName = R"doc()doc";

static const char *__doc_dai_BoardConfig_USB_vid = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC = R"doc(UVC configuration for USB descriptor)doc";

static const char *__doc_dai_BoardConfig_UVC_UVC = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_UVC_2 = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_cameraName = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_enable = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_frameType = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_height = R"doc()doc";

static const char *__doc_dai_BoardConfig_UVC_width = R"doc()doc";

static const char *__doc_dai_BoardConfig_camera = R"doc()doc";

static const char *__doc_dai_BoardConfig_defaultImuExtr = R"doc()doc";

static const char *__doc_dai_BoardConfig_emmc = R"doc(eMMC config)doc";

static const char *__doc_dai_BoardConfig_gpio = R"doc()doc";

static const char *__doc_dai_BoardConfig_imu = R"doc()doc";

static const char *__doc_dai_BoardConfig_logDevicePrints = R"doc(log device prints)doc";

static const char *__doc_dai_BoardConfig_logPath = R"doc(log path)doc";

static const char *__doc_dai_BoardConfig_logSizeMax = R"doc(Max log size)doc";

static const char *__doc_dai_BoardConfig_logVerbosity = R"doc(log verbosity)doc";

static const char *__doc_dai_BoardConfig_mipi4LaneRgb = R"doc(MIPI 4Lane RGB config)doc";

static const char *__doc_dai_BoardConfig_network = R"doc()doc";

static const char *__doc_dai_BoardConfig_nonExclusiveMode = R"doc()doc";

static const char *__doc_dai_BoardConfig_pcieInternalClock = R"doc(PCIe config)doc";

static const char *__doc_dai_BoardConfig_sysctl =
R"doc(Optional list of FreeBSD sysctl parameters to be set (system, network, etc.).
For example: "net.inet.tcp.delayed_ack=0" (this one is also set by default))doc";

static const char *__doc_dai_BoardConfig_uart = R"doc(UART instance map)doc";

static const char *__doc_dai_BoardConfig_usb = R"doc()doc";

static const char *__doc_dai_BoardConfig_usb3PhyInternalClock = R"doc(USB3 phy config)doc";

static const char *__doc_dai_BoardConfig_uvc = R"doc()doc";

static const char *__doc_dai_BoardConfig_watchdogInitialDelayMs = R"doc()doc";

static const char *__doc_dai_BoardConfig_watchdogTimeoutMs = R"doc(Watchdog config)doc";

static const char *__doc_dai_Buffer = R"doc(Base message - buffer of binary data)doc";

static const char *__doc_dai_Buffer_Buffer = R"doc()doc";

static const char *__doc_dai_Buffer_Buffer_2 = R"doc()doc";

static const char *__doc_dai_Buffer_Buffer_3 = R"doc()doc";

static const char *__doc_dai_Buffer_Buffer_4 = R"doc()doc";

static const char *__doc_dai_Buffer_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Buffer_getData =
R"doc(Get non-owning reference to internal buffer

Returns:
    Reference to internal buffer)doc";

static const char *__doc_dai_Buffer_getData_2 = R"doc()doc";

static const char *__doc_dai_Buffer_getRecordData = R"doc()doc";

static const char *__doc_dai_Buffer_getSequenceNum = R"doc(Retrieves image sequence number)doc";

static const char *__doc_dai_Buffer_getTimestamp = R"doc(Retrieves timestamp related to dai::Clock::now())doc";

static const char *__doc_dai_Buffer_getTimestampDevice =
R"doc(Retrieves timestamp directly captured from device's monotonic clock, not
synchronized to host time. Used mostly for debugging)doc";

static const char *__doc_dai_Buffer_getVisualizationMessage =
R"doc(Get visualizable message

Returns:
    Visualizable message, either ImgFrame, ImgAnnotations or std::monostate
    (None))doc";

static const char *__doc_dai_Buffer_sequenceNum = R"doc()doc";

static const char *__doc_dai_Buffer_serialize = R"doc()doc";

static const char *__doc_dai_Buffer_setData =
R"doc(Parameter ``data``:
    Copies data to internal buffer)doc";

static const char *__doc_dai_Buffer_setData_2 = R"doc()doc";

static const char *__doc_dai_Buffer_setData_3 =
R"doc(Parameter ``data``:
    Moves data to internal buffer)doc";

static const char *__doc_dai_Buffer_setSequenceNum = R"doc(Sets image sequence number)doc";

static const char *__doc_dai_Buffer_setTimestamp = R"doc(Sets image timestamp related to dai::Clock::now())doc";

static const char *__doc_dai_Buffer_setTimestampDevice = R"doc(Sets image timestamp related to dai::Clock::now())doc";

static const char *__doc_dai_Buffer_str = R"doc()doc";

static const char *__doc_dai_Buffer_ts = R"doc()doc";

static const char *__doc_dai_Buffer_tsDevice = R"doc()doc";

static const char *__doc_dai_CalibrationHandler =
R"doc(CalibrationHandler is an interface to read/load/write structured calibration and
device data. The following fields are protected and aren't allowed to be
overridden by default: - boardName - boardRev - boardConf - hardwareConf -
batchName - batchTime - boardOptions - productName)doc";

static const char *__doc_dai_CalibrationHandler_CalibrationHandler = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_CalibrationHandler_2 =
R"doc(Construct a new Calibration Handler object using the eeprom json file created
from calibration procedure.

Parameter ``eepromDataPath``:
    takes the full path to the json file containing the calibration and device
    info.)doc";

static const char *__doc_dai_CalibrationHandler_CalibrationHandler_3 =
R"doc(Construct a new Calibration Handler object using the board config json file and
.calib binary files created using gen1 calibration.

Parameter ``calibrationDataPath``:
    Full Path to the .calib binary file from the gen1 calibration. (Supports
    only Version 5)

Parameter ``boardConfigPath``:
    Full Path to the board config json file containing device information.)doc";

static const char *__doc_dai_CalibrationHandler_CalibrationHandler_4 =
R"doc(Construct a new Calibration Handler object from EepromData object.

Parameter ``eepromData``:
    EepromData data structure containing the calibration data.)doc";

static const char *__doc_dai_CalibrationHandler_checkExtrinsicsLink = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_checkSrcLinks = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_computeExtrinsicMatrix = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_dependent_false = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_eepromData = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_eepromToJson =
R"doc(Get JSON representation of calibration data

Returns:
    JSON structure)doc";

static const char *__doc_dai_CalibrationHandler_eepromToJsonFile =
R"doc(Write raw calibration/board data to json file.

Parameter ``destPath``:
    Full path to the json file in which raw calibration data will be stored

Returns:
    True on success, false otherwise)doc";

static const char *__doc_dai_CalibrationHandler_fromJson =
R"doc(Construct a new Calibration Handler object from JSON EepromData.

Parameter ``eepromDataJson``:
    EepromData as JSON)doc";

static const char *__doc_dai_CalibrationHandler_getBaselineDistance =
R"doc(Get the baseline distance between two specified cameras. By default it will get
the baseline between CameraBoardSocket.RIGHT and CameraBoardSocket.LEFT.

Parameter ``cam1``:
    First camera

Parameter ``cam2``:
    Second camera

Parameter ``useSpecTranslation``:
    Enabling this bool uses the translation information from the board design
    data (not the calibration data)

Returns:
    baseline distance in centimeters)doc";

static const char *__doc_dai_CalibrationHandler_getCameraExtrinsics =
R"doc(Get the Camera Extrinsics object between two cameras from the calibration data
if there is a linked connection between any two cameras then the relative
rotation and translation (in centimeters) is returned by this function.

Parameter ``srcCamera``:
    Camera Id of the camera which will be considered as origin.

Parameter ``dstCamera``:
    Camera Id of the destination camera to which we are fetching the rotation
    and translation from the SrcCamera

Parameter ``useSpecTranslation``:
    Enabling this bool uses the translation information from the board design
    data

Returns:
    a transformationMatrix which is 4x4 in homogeneous coordinate system

Matrix representation of transformation matrix \f[ \text{Transformation Matrix}
= \left [ \begin{matrix} r_{00} & r_{01} & r_{02} & T_x \\ r_{10} & r_{11} &
r_{12} & T_y \\ r_{20} & r_{21} & r_{22} & T_z \\ 0 & 0 & 0 & 1 \end{matrix}
\right ] \f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraIntrinsics =
R"doc(Get the Camera Intrinsics object

Parameter ``cameraId``:
    Uses the cameraId to identify which camera intrinsics to return

Parameter ``resizewidth``:
    resized width of the image for which intrinsics is requested. resizewidth =
    -1 represents width is same as default intrinsics

Parameter ``resizeHeight``:
    resized height of the image for which intrinsics is requested. resizeHeight
    = -1 represents height is same as default intrinsics

Parameter ``topLeftPixelId``:
    (x, y) point represents the top left corner coordinates of the cropped image
    which is used to modify the intrinsics for the respective cropped image

Parameter ``bottomRightPixelId``:
    (x, y) point represents the bottom right corner coordinates of the cropped
    image which is used to modify the intrinsics for the respective cropped
    image

Parameter ``keepAspectRatio``:
    Enabling this will scale on width or height depending on which provides the
    max resolution and crops the remaining part of the other side

Returns:
    Represents the 3x3 intrinsics matrix of the respective camera at the
    requested size and crop dimensions.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraIntrinsics_2 =
R"doc(Get the Camera Intrinsics object

Parameter ``cameraId``:
    Uses the cameraId to identify which camera intrinsics to return

Parameter ``destShape``:
    resized width and height of the image for which intrinsics is requested.

Parameter ``topLeftPixelId``:
    (x, y) point represents the top left corner coordinates of the cropped image
    which is used to modify the intrinsics for the respective cropped image

Parameter ``bottomRightPixelId``:
    (x, y) point represents the bottom right corner coordinates of the cropped
    image which is used to modify the intrinsics for the respective cropped
    image

Parameter ``keepAspectRatio``:
    Enabling this will scale on width or height depending on which provides the
    max resolution and crops the remaining part of the other side

Returns:
    Represents the 3x3 intrinsics matrix of the respective camera at the
    requested size and crop dimensions.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraIntrinsics_3 =
R"doc(Get the Camera Intrinsics object

Parameter ``cameraId``:
    Uses the cameraId to identify which camera intrinsics to return

Parameter ``destShape``:
    resized width and height of the image for which intrinsics is requested.

Parameter ``topLeftPixelId``:
    (x, y) point represents the top left corner coordinates of the cropped image
    which is used to modify the intrinsics for the respective cropped image

Parameter ``bottomRightPixelId``:
    (x, y) point represents the bottom right corner coordinates of the cropped
    image which is used to modify the intrinsics for the respective cropped
    image

Parameter ``keepAspectRatio``:
    Enabling this will scale on width or height depending on which provides the
    max resolution and crops the remaining part of the other side

Returns:
    Represents the 3x3 intrinsics matrix of the respective camera at the
    requested size and crop dimensions.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraRotationMatrix =
R"doc(Get the Camera rotation matrix between two cameras from the calibration data.

Parameter ``srcCamera``:
    Camera Id of the camera which will be considered as origin.

Parameter ``dstCamera``:
    Camera Id of the destination camera to which we are fetching the rotation
    vector from the SrcCamera

Returns:
    a 3x3 rotation matrix Matrix representation of rotation matrix \f[
    \text{Rotation Matrix} = \left [ \begin{matrix} r_{00} & r_{01} & r_{02}\\
    r_{10} & r_{11} & r_{12}\\ r_{20} & r_{21} & r_{22}\\ \end{matrix} \right ]
    \f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraToImuExtrinsics =
R"doc(Get the Camera To Imu Extrinsics object From the data loaded if there is a
linked connection between IMU and the given camera then there relative rotation
and translation from the camera to IMU is returned.

Parameter ``cameraId``:
    Camera Id of the camera which will be considered as origin. from which
    Transformation matrix to the IMU will be found

Parameter ``useSpecTranslation``:
    Enabling this bool uses the translation information from the board design
    data

Returns:
    Returns a transformationMatrix which is 4x4 in homogeneous coordinate system

Matrix representation of transformation matrix \f[ \text{Transformation Matrix}
= \left [ \begin{matrix} r_{00} & r_{01} & r_{02} & T_x \\ r_{10} & r_{11} &
r_{12} & T_y \\ r_{20} & r_{21} & r_{22} & T_z \\ 0 & 0 & 0 & 1 \end{matrix}
\right ] \f])doc";

static const char *__doc_dai_CalibrationHandler_getCameraTranslationVector =
R"doc(Get the Camera translation vector between two cameras from the calibration data.

Parameter ``srcCamera``:
    Camera Id of the camera which will be considered as origin.

Parameter ``dstCamera``:
    Camera Id of the destination camera to which we are fetching the translation
    vector from the SrcCamera

Parameter ``useSpecTranslation``:
    Disabling this bool uses the translation information from the calibration
    data (not the board design data)

Returns:
    a translation vector like [x, y, z] in centimeters)doc";

static const char *__doc_dai_CalibrationHandler_getDefaultIntrinsics =
R"doc(Get the Default Intrinsics object

Parameter ``cameraId``:
    Uses the cameraId to identify which camera intrinsics to return

Returns:
    Represents the 3x3 intrinsics matrix of the respective camera along with
    width and height at which it was calibrated.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_getDistortionCoefficients =
R"doc(Get the Distortion Coefficients object

Parameter ``cameraId``:
    Uses the cameraId to identify which distortion Coefficients to return.

Returns:
    the distortion coefficients of the requested camera in this order:
    [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,τx,τy] for CameraModel::Perspective or
    [k1, k2, k3, k4] for CameraModel::Fisheye see
    https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html for Perspective
    model (Rational Polynomial Model) see
    https://docs.opencv.org/4.5.4/db/d58/group__calib3d__fisheye.html for
    Fisheye model)doc";

static const char *__doc_dai_CalibrationHandler_getDistortionModel =
R"doc(Get the distortion model of the given camera

Parameter ``cameraId``:
    of the camera with lens position is requested.

Returns:
    lens position of the camera with given cameraId at which it was calibrated.)doc";

static const char *__doc_dai_CalibrationHandler_getEepromData =
R"doc(Get the Eeprom Data object

Returns:
    EepromData object which contains the raw calibration data)doc";

static const char *__doc_dai_CalibrationHandler_getExtrinsicsToOrigin =
R"doc(Get the Transformation matrix from the given camera to the coordinate system
origin (one without extrinsics and linked to CameraBoardSocket.AUTO)

Parameter ``cameraId``:
    Camera Id of the camera for which the origin matrix is being calculated

Parameter ``useSpecTranslation``:
    Enabling this bool uses the translation information from the board design
    data

Returns:
    a transformationMatrix which is 4x4 in homogeneous coordinate system)doc";

static const char *__doc_dai_CalibrationHandler_getFov =
R"doc(Get the Fov of the camera

Parameter ``cameraId``:
    of the camera of which we are fetching fov.

Parameter ``useSpec``:
    Disabling this bool will calculate the fov based on intrinsics (focal
    length, image width), instead of getting it from the camera specs

Returns:
    field of view of the camera with given cameraId.)doc";

static const char *__doc_dai_CalibrationHandler_getImuToCameraExtrinsics =
R"doc(Get the Imu To Camera Extrinsics object from the data loaded if there is a
linked connection between IMU and the given camera then there relative rotation
and translation from the IMU to Camera is returned.

Parameter ``cameraId``:
    Camera Id of the camera which will be considered as destination. To which
    Transformation matrix from the IMU will be found.

Parameter ``useSpecTranslation``:
    Enabling this bool uses the translation information from the board design
    data

Returns:
    Returns a transformationMatrix which is 4x4 in homogeneous coordinate system

Matrix representation of transformation matrix \f[ \text{Transformation Matrix}
= \left [ \begin{matrix} r_{00} & r_{01} & r_{02} & T_x \\ r_{10} & r_{11} &
r_{12} & T_y \\ r_{20} & r_{21} & r_{22} & T_z \\ 0 & 0 & 0 & 1 \end{matrix}
\right ] \f])doc";

static const char *__doc_dai_CalibrationHandler_getLensPosition =
R"doc(Get the lens position of the given camera

Parameter ``cameraId``:
    of the camera with lens position is requested.

Returns:
    lens position of the camera with given cameraId at which it was calibrated.)doc";

static const char *__doc_dai_CalibrationHandler_getRTABMapCameraModel = R"doc()doc";

static const char *__doc_dai_CalibrationHandler_getStereoLeftCameraId =
R"doc(Get the camera id of the camera which is used as left camera of the stereo setup

Returns:
    cameraID of the camera used as left camera)doc";

static const char *__doc_dai_CalibrationHandler_getStereoLeftRectificationRotation =
R"doc(Get the Stereo Left Rectification Rotation object

Returns:
    returns a 3x3 rectification rotation matrix)doc";

static const char *__doc_dai_CalibrationHandler_getStereoRightCameraId =
R"doc(Get the camera id of the camera which is used as right camera of the stereo
setup

Returns:
    cameraID of the camera used as right camera)doc";

static const char *__doc_dai_CalibrationHandler_getStereoRightRectificationRotation =
R"doc(Get the Stereo Right Rectification Rotation object

Returns:
    returns a 3x3 rectification rotation matrix)doc";

static const char *__doc_dai_CalibrationHandler_setBoardInfo =
R"doc(Set the Board Info object

Parameter ``version``:
    Sets the version of the Calibration data(Current version is 6)

Parameter ``boardName``:
    Sets your board name.

Parameter ``boardRev``:
    set your board revision id.)doc";

static const char *__doc_dai_CalibrationHandler_setBoardInfo_2 =
R"doc(Set the Board Info object. Creates version 7 EEPROM data

Parameter ``productName``:
    Sets product name (alias).

Parameter ``boardName``:
    Sets board name.

Parameter ``boardRev``:
    Sets board revision id.

Parameter ``boardConf``:
    Sets board configuration id.

Parameter ``hardwareConf``:
    Sets hardware configuration id.

Parameter ``batchName``:
    Sets batch name.

Parameter ``batchTime``:
    Sets batch time (unix timestamp).

Parameter ``boardCustom``:
    Sets a custom board (Default empty string).)doc";

static const char *__doc_dai_CalibrationHandler_setBoardInfo_3 =
R"doc(Set the Board Info object. Creates version 7 EEPROM data

Parameter ``deviceName``:
    Sets device name.

Parameter ``productName``:
    Sets product name (alias).

Parameter ``boardName``:
    Sets board name.

Parameter ``boardRev``:
    Sets board revision id.

Parameter ``boardConf``:
    Sets board configuration id.

Parameter ``hardwareConf``:
    Sets hardware configuration id.

Parameter ``batchName``:
    Sets batch name. Not supported anymore

Parameter ``batchTime``:
    Sets batch time (unix timestamp).

Parameter ``boardCustom``:
    Sets a custom board (Default empty string).)doc";

static const char *__doc_dai_CalibrationHandler_setCameraExtrinsics =
R"doc(Set the Camera Extrinsics object

Parameter ``srcCameraId``:
    Camera Id of the camera which will be considered as relative origin.

Parameter ``destCameraId``:
    Camera Id of the camera which will be considered as destination from
    srcCameraId.

Parameter ``rotationMatrix``:
    Rotation between srcCameraId and destCameraId origins.

Parameter ``translation``:
    Translation between srcCameraId and destCameraId origins.

Parameter ``specTranslation``:
    Translation between srcCameraId and destCameraId origins from the design.)doc";

static const char *__doc_dai_CalibrationHandler_setCameraIntrinsics =
R"doc(Set the Camera Intrinsics object

Parameter ``cameraId``:
    CameraId of the camera for which Camera intrinsics are being loaded

Parameter ``intrinsics``:
    3x3 intrinsics matrix

Parameter ``frameSize``:
    Represents the width and height of the image at which intrinsics are
    calculated.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_setCameraIntrinsics_2 =
R"doc(Set the Camera Intrinsics object

Parameter ``cameraId``:
    CameraId of the camera for which Camera intrinsics are being loaded

Parameter ``intrinsics``:
    3x3 intrinsics matrix

Parameter ``width``:
    Represents the width of the image at which intrinsics are calculated.

Parameter ``height``:
    Represents the height of the image at which intrinsics are calculated.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_setCameraIntrinsics_3 =
R"doc(Set the Camera Intrinsics object

Parameter ``cameraId``:
    CameraId of the camera for which Camera intrinsics are being loaded

Parameter ``intrinsics``:
    3x3 intrinsics matrix

Parameter ``frameSize``:
    Represents the width and height of the image at which intrinsics are
    calculated.

Matrix representation of intrinsic matrix \f[ \text{Intrinsic Matrix} = \left [
\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix} \right ]
\f])doc";

static const char *__doc_dai_CalibrationHandler_setCameraType =
R"doc(Set the Camera Type object

Parameter ``cameraId``:
    CameraId of the camera for which cameraModel Type is being updated.

Parameter ``cameraModel``:
    Type of the model the camera represents)doc";

static const char *__doc_dai_CalibrationHandler_setDeviceName =
R"doc(Set the deviceName which responses to getDeviceName of Device

Parameter ``deviceName``:
    Sets device name.)doc";

static const char *__doc_dai_CalibrationHandler_setDistortionCoefficients =
R"doc(Sets the distortion Coefficients obtained from camera calibration

Parameter ``cameraId``:
    Camera Id of the camera for which distortion coefficients are computed

Parameter ``distortionCoefficients``:
    Distortion Coefficients of the respective Camera.)doc";

static const char *__doc_dai_CalibrationHandler_setFov =
R"doc(Set the Fov of the Camera

Parameter ``cameraId``:
    Camera Id of the camera

Parameter ``hfov``:
    Horizontal fov of the camera from Camera Datasheet)doc";

static const char *__doc_dai_CalibrationHandler_setImuExtrinsics =
R"doc(Set the Imu to Camera Extrinsics object

Parameter ``destCameraId``:
    Camera Id of the camera which will be considered as destination from IMU.

Parameter ``rotationMatrix``:
    Rotation between srcCameraId and destCameraId origins.

Parameter ``translation``:
    Translation between IMU and destCameraId origins.

Parameter ``specTranslation``:
    Translation between IMU and destCameraId origins from the design.)doc";

static const char *__doc_dai_CalibrationHandler_setLensPosition =
R"doc(Sets the distortion Coefficients obtained from camera calibration

Parameter ``cameraId``:
    Camera Id of the camera

Parameter ``lensPosition``:
    lens posiotion value of the camera at the time of calibration)doc";

static const char *__doc_dai_CalibrationHandler_setProductName =
R"doc(Set the productName which acts as alisas for users to identify the device

Parameter ``productName``:
    Sets product name (alias).)doc";

static const char *__doc_dai_CalibrationHandler_setStereoLeft =
R"doc(Set the Stereo Left Rectification object

Parameter ``cameraId``:
    CameraId of the camera which will be used as left Camera of stereo Setup

Parameter ``rectifiedRotation``:
    Rectification rotation of the left camera required for feature matching

Homography of the Left Rectification = Intrinsics_right * rectifiedRotation *
inv(Intrinsics_left))doc";

static const char *__doc_dai_CalibrationHandler_setStereoRight =
R"doc(Set the Stereo Right Rectification object

Parameter ``cameraId``:
    CameraId of the camera which will be used as left Camera of stereo Setup

Parameter ``rectifiedRotation``:
    Rectification rotation of the left camera required for feature matching

Homography of the Right Rectification = Intrinsics_right * rectifiedRotation *
inv(Intrinsics_right))doc";

static const char *__doc_dai_CalibrationHandler_validateCameraArray =
R"doc(Using left camera as the head it iterates over the camera extrinsics connection
to check if all the camera extrinsics are connected and no loop exists.

Returns:
    true on proper connection with no loops.)doc";

static const char *__doc_dai_CallbackHandler = R"doc()doc";

static const char *__doc_dai_CallbackHandler_CallbackHandler = R"doc()doc";

static const char *__doc_dai_CallbackHandler_callback = R"doc()doc";

static const char *__doc_dai_CallbackHandler_connection = R"doc()doc";

static const char *__doc_dai_CallbackHandler_running = R"doc()doc";

static const char *__doc_dai_CallbackHandler_setCallback = R"doc()doc";

static const char *__doc_dai_CallbackHandler_t = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket =
R"doc(Which Camera socket to use.

AUTO denotes that the decision will be made by device)doc";

static const char *__doc_dai_CameraBoardSocket_AUTO = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_A = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_B = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_C = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_D = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_E = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_F = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_G = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_H = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_I = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CAM_J = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_CENTER = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_LEFT = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_RGB = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_RIGHT = R"doc()doc";

static const char *__doc_dai_CameraBoardSocket_VERTICAL = R"doc()doc";

static const char *__doc_dai_CameraControl =
R"doc(CameraControl message. Specifies various camera control commands like:

- Still capture

- Auto/manual focus

- Auto/manual white balance

- Auto/manual exposure

- Anti banding

- ...

By default the camera enables 3A, with auto-focus in `CONTINUOUS_VIDEO` mode,
auto-white-balance in `AUTO` mode, and auto-exposure with anti-banding for 50Hz
mains frequency.)doc";

static const char *__doc_dai_CameraControl_AntiBandingMode = R"doc()doc";

static const char *__doc_dai_CameraControl_AntiBandingMode_AUTO =
R"doc(The camera device will automatically adapt its antibanding routine to the
current illumination condition. This is the default mode if AUTO is available on
given camera device.)doc";

static const char *__doc_dai_CameraControl_AntiBandingMode_MAINS_50_HZ =
R"doc(The camera device will adjust exposure duration to avoid banding problems with
50Hz illumination sources.)doc";

static const char *__doc_dai_CameraControl_AntiBandingMode_MAINS_60_HZ =
R"doc(The camera device will adjust exposure duration to avoid banding problems with
60Hz illumination sources.)doc";

static const char *__doc_dai_CameraControl_AntiBandingMode_OFF = R"doc(The camera device will not adjust exposure duration to avoid banding problems.)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode = R"doc()doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_AUTO =
R"doc(Basic automatic focus mode. In this mode, the lens does not move unless the
autofocus trigger action is called.)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_CONTINUOUS_PICTURE =
R"doc(In this mode, the AF algorithm modifies the lens position continually to attempt
to provide a constantly-in-focus image stream. The focusing behavior should be
suitable for still image capture; typically this means focusing as fast as
possible)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_CONTINUOUS_VIDEO =
R"doc(In this mode, the AF algorithm modifies the lens position continually to attempt
to provide a constantly-in-focus image stream. The focusing behavior should be
suitable for good quality video recording; typically this means slower focus
movement and no overshoots.)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_EDOF =
R"doc(Extended depth of field (digital focus) mode. The camera device will produce
images with an extended depth of field automatically. AF triggers are ignored.)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_MACRO =
R"doc(Close-up focusing mode - this mode is optimized for focusing on objects very
close to the camera.)doc";

static const char *__doc_dai_CameraControl_AutoFocusMode_OFF = R"doc(Autofocus disabled. Suitable for manual focus)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode = R"doc()doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_AUTO = R"doc(The camera device's auto-white balance routine is active.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_CLOUDY_DAYLIGHT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses cloudy daylight light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_DAYLIGHT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses daylight light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_FLUORESCENT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses fluorescent light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_INCANDESCENT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses incandescent light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_OFF = R"doc(The camera device's auto-white balance routine is disabled.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_SHADE =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses shade light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_TWILIGHT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses twilight light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_AutoWhiteBalanceMode_WARM_FLUORESCENT =
R"doc(The camera device's auto-white balance routine is disabled; the camera device
uses warm fluorescent light as the assumed scene illumination for white balance.)doc";

static const char *__doc_dai_CameraControl_CameraControl = R"doc()doc";

static const char *__doc_dai_CameraControl_CaptureIntent = R"doc()doc";

static const char *__doc_dai_CameraControl_CaptureIntent_CUSTOM =
R"doc(The goal of this request doesn't fall into the other categories. The camera
device will default to preview-like behavior.)doc";

static const char *__doc_dai_CameraControl_CaptureIntent_PREVIEW = R"doc(This request is for a preview-like use case.)doc";

static const char *__doc_dai_CameraControl_CaptureIntent_STILL_CAPTURE = R"doc(This request is for a still capture-type use case.)doc";

static const char *__doc_dai_CameraControl_CaptureIntent_VIDEO_RECORD = R"doc(This request is for a video recording use case.)doc";

static const char *__doc_dai_CameraControl_CaptureIntent_VIDEO_SNAPSHOT =
R"doc(This request is for a video snapshot (still image while recording video) use
case. The camera device should take the highest-quality image possible (given
the other settings) without disrupting the frame rate of video recording.)doc";

static const char *__doc_dai_CameraControl_CaptureIntent_ZERO_SHUTTER_LAG =
R"doc(This request is for a ZSL usecase; the application will stream full-resolution
images and reprocess one or several later for a final capture.)doc";

static const char *__doc_dai_CameraControl_Command = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AE_AUTO = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AE_LOCK = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AE_MANUAL = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AE_REGION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AE_TARGET_FPS_RANGE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AF_LENS_RANGE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AF_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AF_REGION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AF_TRIGGER = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_ANTIBANDING_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AWB_LOCK = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_AWB_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_BRIGHTNESS = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CAPTURE_INTENT = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CHROMA_DENOISE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CONTRAST = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CONTROL_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CUSTOM_CAPTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CUSTOM_CAPT_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CUSTOM_EXP_BRACKETS = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_CUSTOM_USECASE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_EFFECT_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_EXPOSURE_COMPENSATION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_EXTERNAL_TRIGGER = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_FRAME_DURATION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_FRAME_SYNC = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_HDR = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_LUMA_DENOISE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_MOVE_LENS = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_MOVE_LENS_RAW = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_NOISE_REDUCTION_STRENGTH = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_RESOLUTION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_SATURATION = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_SCENE_MODE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_SENSITIVITY = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_SHARPNESS = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_START_STREAM = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_STILL_CAPTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_STOP_STREAM = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_STREAM_FORMAT = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_STROBE_CONFIG = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_STROBE_TIMINGS = R"doc()doc";

static const char *__doc_dai_CameraControl_Command_WB_COLOR_TEMP = R"doc()doc";

static const char *__doc_dai_CameraControl_ControlMode = R"doc()doc";

static const char *__doc_dai_CameraControl_ControlMode_AUTO =
R"doc(Use settings for each individual 3A routine. Manual control of capture
parameters is disabled.)doc";

static const char *__doc_dai_CameraControl_ControlMode_OFF =
R"doc(Full application control of pipeline. All control by the device's metering and
focusing (3A) routines is disabled.)doc";

static const char *__doc_dai_CameraControl_ControlMode_USE_SCENE_MODE =
R"doc(Use a specific scene mode. Enabling this disables Auto-Exposure, AWB and AF
controls;)doc";

static const char *__doc_dai_CameraControl_EffectMode = R"doc()doc";

static const char *__doc_dai_CameraControl_EffectMode_AQUA = R"doc(An "aqua" effect where a blue hue is added to the image.)doc";

static const char *__doc_dai_CameraControl_EffectMode_BLACKBOARD =
R"doc(A "blackboard" effect where the image is typically displayed as regions of
black, with white or grey details.)doc";

static const char *__doc_dai_CameraControl_EffectMode_MONO =
R"doc(A "monocolor" effect where the image is mapped into a single color. This will
typically be grayscale.)doc";

static const char *__doc_dai_CameraControl_EffectMode_NEGATIVE = R"doc(A "photo-negative" effect where the image's colors are inverted.)doc";

static const char *__doc_dai_CameraControl_EffectMode_OFF = R"doc(No color effect will be applied.)doc";

static const char *__doc_dai_CameraControl_EffectMode_POSTERIZE =
R"doc(A "posterization" effect where the image uses discrete regions of tone rather
than a continuous gradient of tones.)doc";

static const char *__doc_dai_CameraControl_EffectMode_SEPIA = R"doc(A "sepia" effect where the image is mapped into warm gray, red, and brown tones.)doc";

static const char *__doc_dai_CameraControl_EffectMode_SOLARIZE =
R"doc(A "solarisation" effect (Sabattier effect) where the image is wholly or
partially reversed in tone.)doc";

static const char *__doc_dai_CameraControl_EffectMode_WHITEBOARD =
R"doc(A "whiteboard" effect where the image is typically displayed as regions of
white, with black or grey details.)doc";

static const char *__doc_dai_CameraControl_FrameSyncMode = R"doc()doc";

static const char *__doc_dai_CameraControl_FrameSyncMode_INPUT = R"doc()doc";

static const char *__doc_dai_CameraControl_FrameSyncMode_OFF = R"doc()doc";

static const char *__doc_dai_CameraControl_FrameSyncMode_OUTPUT = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams_exposureTimeUs = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams_frameDurationUs = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams_sensitivityIso = R"doc()doc";

static const char *__doc_dai_CameraControl_ManualExposureParams_str = R"doc()doc";

static const char *__doc_dai_CameraControl_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_height = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_priority = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_str = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_width = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_x = R"doc()doc";

static const char *__doc_dai_CameraControl_RegionParams_y = R"doc()doc";

static const char *__doc_dai_CameraControl_SceneMode = R"doc()doc";

static const char *__doc_dai_CameraControl_SceneMode_ACTION = R"doc(Optimized for photos of quickly moving objects. Similar to SPORTS scene mode.)doc";

static const char *__doc_dai_CameraControl_SceneMode_BARCODE =
R"doc(Optimized for accurately capturing a photo of barcode for use by camera
applications that wish to read the barcode value.)doc";

static const char *__doc_dai_CameraControl_SceneMode_BEACH = R"doc(Optimized for bright, outdoor beach settings.)doc";

static const char *__doc_dai_CameraControl_SceneMode_CANDLELIGHT = R"doc(Optimized for dim settings where the main light source is a candle.)doc";

static const char *__doc_dai_CameraControl_SceneMode_FACE_PRIORITY =
R"doc(If face detection support exists, use face detection data for auto-focus, auto-
white balance, and auto-exposure routines.)doc";

static const char *__doc_dai_CameraControl_SceneMode_FIREWORKS = R"doc(Optimized for nighttime photos of fireworks.)doc";

static const char *__doc_dai_CameraControl_SceneMode_LANDSCAPE = R"doc(Optimized for photos of distant macroscopic objects.)doc";

static const char *__doc_dai_CameraControl_SceneMode_NIGHT = R"doc(Optimized for low-light settings.)doc";

static const char *__doc_dai_CameraControl_SceneMode_NIGHT_PORTRAIT = R"doc(Optimized for still photos of people in low-light settings.)doc";

static const char *__doc_dai_CameraControl_SceneMode_PARTY = R"doc(Optimized for dim, indoor settings with multiple moving people.)doc";

static const char *__doc_dai_CameraControl_SceneMode_PORTRAIT = R"doc(Optimized for still photos of people.)doc";

static const char *__doc_dai_CameraControl_SceneMode_SNOW = R"doc(Optimized for bright, outdoor settings containing snow.)doc";

static const char *__doc_dai_CameraControl_SceneMode_SPORTS = R"doc(Optimized for photos of quickly moving people.)doc";

static const char *__doc_dai_CameraControl_SceneMode_STEADYPHOTO =
R"doc(Optimized to avoid blurry photos due to small amounts of device motion (for
example: due to hand shake).)doc";

static const char *__doc_dai_CameraControl_SceneMode_SUNSET = R"doc(Optimized for scenes of the setting sun.)doc";

static const char *__doc_dai_CameraControl_SceneMode_THEATRE = R"doc(Optimized for dim, indoor settings where flash must remain off.)doc";

static const char *__doc_dai_CameraControl_SceneMode_UNSUPPORTED = R"doc(Indicates that no scene modes are set for a given capture request.)doc";

static const char *__doc_dai_CameraControl_StrobeConfig = R"doc()doc";

static const char *__doc_dai_CameraControl_StrobeConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_StrobeConfig_activeLevel = R"doc(1 for normal polarity (high-active), 0 otherwise)doc";

static const char *__doc_dai_CameraControl_StrobeConfig_enable = R"doc(Enable strobe output)doc";

static const char *__doc_dai_CameraControl_StrobeConfig_gpioNumber = R"doc(GPIO number to drive, or -1 if sensor driven)doc";

static const char *__doc_dai_CameraControl_StrobeConfig_str = R"doc()doc";

static const char *__doc_dai_CameraControl_StrobeTimings = R"doc()doc";

static const char *__doc_dai_CameraControl_StrobeTimings_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraControl_StrobeTimings_durationUs =
R"doc(Fixed duration in microseconds. If set (non-zero), overrides
`exposureEndOffsetUs`)doc";

static const char *__doc_dai_CameraControl_StrobeTimings_exposureBeginOffsetUs = R"doc(Start offset in microseconds, relative to exposure window)doc";

static const char *__doc_dai_CameraControl_StrobeTimings_exposureEndOffsetUs = R"doc(End offset in microseconds, relative to exposure window)doc";

static const char *__doc_dai_CameraControl_StrobeTimings_str = R"doc()doc";

static const char *__doc_dai_CameraControl_aeLockMode = R"doc()doc";

static const char *__doc_dai_CameraControl_aeMaxExposureTimeUs = R"doc()doc";

static const char *__doc_dai_CameraControl_aeRegion = R"doc()doc";

static const char *__doc_dai_CameraControl_afRegion = R"doc()doc";

static const char *__doc_dai_CameraControl_antiBandingMode = R"doc()doc";

static const char *__doc_dai_CameraControl_autoFocusMode = R"doc()doc";

static const char *__doc_dai_CameraControl_awbLockMode = R"doc()doc";

static const char *__doc_dai_CameraControl_awbMode = R"doc()doc";

static const char *__doc_dai_CameraControl_brightness = R"doc()doc";

static const char *__doc_dai_CameraControl_captureIntent = R"doc()doc";

static const char *__doc_dai_CameraControl_chromaDenoise = R"doc()doc";

static const char *__doc_dai_CameraControl_clearCommand = R"doc()doc";

static const char *__doc_dai_CameraControl_clearMiscControls = R"doc(Clear the list of miscellaneous controls set by `setControl`)doc";

static const char *__doc_dai_CameraControl_cmdMask = R"doc()doc";

static const char *__doc_dai_CameraControl_contrast = R"doc()doc";

static const char *__doc_dai_CameraControl_controlMode = R"doc()doc";

static const char *__doc_dai_CameraControl_effectMode = R"doc()doc";

static const char *__doc_dai_CameraControl_enableHdr = R"doc()doc";

static const char *__doc_dai_CameraControl_expCompensation = R"doc()doc";

static const char *__doc_dai_CameraControl_expManual = R"doc()doc";

static const char *__doc_dai_CameraControl_frameSyncMode = R"doc()doc";

static const char *__doc_dai_CameraControl_getCaptureStill =
R"doc(Check whether command to capture a still is set

Returns:
    True if capture still command is set)doc";

static const char *__doc_dai_CameraControl_getCommand = R"doc()doc";

static const char *__doc_dai_CameraControl_getExposureTime = R"doc(Retrieves exposure time)doc";

static const char *__doc_dai_CameraControl_getHdr =
R"doc(Whether or not HDR (High Dynamic Range) mode is enabled

Returns:
    True if HDR mode is enabled, false otherwise)doc";

static const char *__doc_dai_CameraControl_getLensPosition = R"doc(Retrieves lens position, range 0..255. Returns -1 if not available)doc";

static const char *__doc_dai_CameraControl_getLensPositionRaw = R"doc(Retrieves lens position, range 0.0f..1.0f.)doc";

static const char *__doc_dai_CameraControl_getMiscControls =
R"doc(Get the list of miscellaneous controls set by `setControl`

Returns:
    A list of <key, value> pairs as strings)doc";

static const char *__doc_dai_CameraControl_getSensitivity = R"doc(Retrieves sensitivity, as an ISO value)doc";

static const char *__doc_dai_CameraControl_lensPosAutoInfinity = R"doc()doc";

static const char *__doc_dai_CameraControl_lensPosAutoMacro = R"doc()doc";

static const char *__doc_dai_CameraControl_lensPosition =
R"doc(Lens/VCM position, range: 0..255. Used with `autoFocusMode = OFF`. With current
IMX378 modules: - max 255: macro focus, at 8cm distance - infinite focus at
about 120..130 (may vary from module to module) - lower values lead to out-of-
focus (lens too close to the sensor array))doc";

static const char *__doc_dai_CameraControl_lensPositionRaw = R"doc()doc";

static const char *__doc_dai_CameraControl_lowPowerNumFramesBurst = R"doc()doc";

static const char *__doc_dai_CameraControl_lowPowerNumFramesDiscard = R"doc()doc";

static const char *__doc_dai_CameraControl_lumaDenoise = R"doc()doc";

static const char *__doc_dai_CameraControl_miscControls = R"doc()doc";

static const char *__doc_dai_CameraControl_saturation = R"doc()doc";

static const char *__doc_dai_CameraControl_sceneMode = R"doc()doc";

static const char *__doc_dai_CameraControl_serialize = R"doc()doc";

static const char *__doc_dai_CameraControl_setAntiBandingMode =
R"doc(Set a command to specify anti-banding mode. Anti-banding / anti-flicker works in
auto-exposure mode, by controlling the exposure time to be applied in multiples
of half the mains period, for example in multiple of 10ms for 50Hz (period 20ms)
AC-powered illumination sources.

If the scene would be too bright for the smallest exposure step (10ms in the
example, with ISO at a minimum of 100), anti-banding is not effective.

Parameter ``mode``:
    Anti-banding mode to use. Default: `MAINS_50_HZ`)doc";

static const char *__doc_dai_CameraControl_setAutoExposureCompensation =
R"doc(Set a command to specify auto exposure compensation

Parameter ``compensation``:
    Compensation value between -9..9, default 0)doc";

static const char *__doc_dai_CameraControl_setAutoExposureEnable = R"doc(Set a command to enable auto exposure)doc";

static const char *__doc_dai_CameraControl_setAutoExposureLimit =
R"doc(Set a command to specify the maximum exposure time limit for auto-exposure. By
default the AE algorithm prioritizes increasing exposure over ISO, up to around
frame-time (subject to further limits imposed by anti-banding)

Parameter ``maxExposureTimeUs``:
    Maximum exposure time in microseconds)doc";

static const char *__doc_dai_CameraControl_setAutoExposureLimit_2 =
R"doc(Set a command to specify the maximum exposure time limit for auto-exposure. By
default the AE algorithm prioritizes increasing exposure over ISO, up to around
frame-time (subject to further limits imposed by anti-banding)

Parameter ``maxExposureTime``:
    Maximum exposure time)doc";

static const char *__doc_dai_CameraControl_setAutoExposureLock =
R"doc(Set a command to specify lock auto exposure

Parameter ``lock``:
    Auto exposure lock mode enabled or disabled)doc";

static const char *__doc_dai_CameraControl_setAutoExposureRegion =
R"doc(Set a command to specify auto exposure region in pixels. Note: the region should
be mapped to the configured sensor resolution, before ISP scaling

Parameter ``startX``:
    X coordinate of top left corner of region

Parameter ``startY``:
    Y coordinate of top left corner of region

Parameter ``width``:
    Region width

Parameter ``height``:
    Region height)doc";

static const char *__doc_dai_CameraControl_setAutoFocusLensRange =
R"doc(Set autofocus lens range, `infinityPosition < macroPosition`, valid values
`0..255`. May help to improve autofocus in case the lens adjustment is not
typical/tuned)doc";

static const char *__doc_dai_CameraControl_setAutoFocusMode = R"doc(Set a command to specify autofocus mode. Default `CONTINUOUS_VIDEO`)doc";

static const char *__doc_dai_CameraControl_setAutoFocusRegion =
R"doc(Set a command to specify focus region in pixels. Note: the region should be
mapped to the configured sensor resolution, before ISP scaling

Parameter ``startX``:
    X coordinate of top left corner of region

Parameter ``startY``:
    Y coordinate of top left corner of region

Parameter ``width``:
    Region width

Parameter ``height``:
    Region height)doc";

static const char *__doc_dai_CameraControl_setAutoFocusTrigger = R"doc(Set a command to trigger autofocus)doc";

static const char *__doc_dai_CameraControl_setAutoWhiteBalanceLock =
R"doc(Set a command to specify auto white balance lock

Parameter ``lock``:
    Auto white balance lock mode enabled or disabled)doc";

static const char *__doc_dai_CameraControl_setAutoWhiteBalanceMode =
R"doc(Set a command to specify auto white balance mode

Parameter ``mode``:
    Auto white balance mode to use. Default `AUTO`)doc";

static const char *__doc_dai_CameraControl_setBrightness =
R"doc(Set a command to adjust image brightness

Parameter ``value``:
    Brightness, range -10..10, default 0)doc";

static const char *__doc_dai_CameraControl_setCaptureIntent =
R"doc(Set a command to specify capture intent mode

Parameter ``mode``:
    Capture intent mode)doc";

static const char *__doc_dai_CameraControl_setCaptureStill = R"doc(Set a command to capture a still image)doc";

static const char *__doc_dai_CameraControl_setChromaDenoise =
R"doc(Set a command to adjust chroma denoise amount

Parameter ``value``:
    Chroma denoise amount, range 0..4, default 1)doc";

static const char *__doc_dai_CameraControl_setCommand = R"doc()doc";

static const char *__doc_dai_CameraControl_setContrast =
R"doc(Set a command to adjust image contrast

Parameter ``value``:
    Contrast, range -10..10, default 0)doc";

static const char *__doc_dai_CameraControl_setControlMode =
R"doc(Set a command to specify control mode

Parameter ``mode``:
    Control mode)doc";

static const char *__doc_dai_CameraControl_setEffectMode =
R"doc(Set a command to specify effect mode

Parameter ``mode``:
    Effect mode)doc";

static const char *__doc_dai_CameraControl_setExternalTrigger =
R"doc(Set a command to enable external trigger snapshot mode

A rising edge on the sensor FSIN pin will make it capture a sequence of
`numFramesBurst` frames. First `numFramesDiscard` will be skipped as configured
(can be set to 0 as well), as they may have degraded quality)doc";

static const char *__doc_dai_CameraControl_setFrameSyncMode =
R"doc(Set the frame sync mode for continuous streaming operation mode, translating to
how the camera pin FSIN/FSYNC is used: input/output/disabled)doc";

static const char *__doc_dai_CameraControl_setHdr =
R"doc(Whether or not to enable HDR (High Dynamic Range) mode

Parameter ``enable``:
    True to enable HDR mode, false to disable)doc";

static const char *__doc_dai_CameraControl_setLumaDenoise =
R"doc(Set a command to adjust luma denoise amount

Parameter ``value``:
    Luma denoise amount, range 0..4, default 1)doc";

static const char *__doc_dai_CameraControl_setManualExposure =
R"doc(Set a command to manually specify exposure

Parameter ``exposureTimeUs``:
    Exposure time in microseconds

Parameter ``sensitivityIso``:
    Sensitivity as ISO value, usual range 100..1600)doc";

static const char *__doc_dai_CameraControl_setManualExposure_2 =
R"doc(Set a command to manually specify exposure

Parameter ``exposureTime``:
    Exposure time

Parameter ``sensitivityIso``:
    Sensitivity as ISO value, usual range 100..1600)doc";

static const char *__doc_dai_CameraControl_setManualFocus =
R"doc(Set a command to specify manual focus position

Parameter ``lensPosition``:
    specify lens position 0..255)doc";

static const char *__doc_dai_CameraControl_setManualFocusRaw =
R"doc(Set a command to specify manual focus position (more precise control).

Parameter ``lensPositionRaw``:
    specify lens position 0.0f .. 1.0f

Returns:
    CameraControl&)doc";

static const char *__doc_dai_CameraControl_setManualWhiteBalance =
R"doc(Set a command to manually specify white-balance color correction

Parameter ``colorTemperatureK``:
    Light source color temperature in kelvins, range 1000..12000)doc";

static const char *__doc_dai_CameraControl_setMisc =
R"doc(Set a miscellaneous control. The controls set by this function get appended to a
list, processed after the standard controls

Parameter ``control``:
    Control name

Parameter ``value``:
    Value as a string)doc";

static const char *__doc_dai_CameraControl_setMisc_2 =
R"doc(Set a miscellaneous control. The controls set by this function get appended to a
list, processed after the standard controls

Parameter ``control``:
    Control name

Parameter ``value``:
    Value as an integer number)doc";

static const char *__doc_dai_CameraControl_setMisc_3 =
R"doc(Set a miscellaneous control. The controls set by this function get appended to a
list, processed after the standard controls

Parameter ``control``:
    Control name

Parameter ``value``:
    Value as a floating point number)doc";

static const char *__doc_dai_CameraControl_setSaturation =
R"doc(Set a command to adjust image saturation

Parameter ``value``:
    Saturation, range -10..10, default 0)doc";

static const char *__doc_dai_CameraControl_setSceneMode =
R"doc(Set a command to specify scene mode

Parameter ``mode``:
    Scene mode)doc";

static const char *__doc_dai_CameraControl_setSharpness =
R"doc(Set a command to adjust image sharpness

Parameter ``value``:
    Sharpness, range 0..4, default 1)doc";

static const char *__doc_dai_CameraControl_setStartStreaming = R"doc(Set a command to start streaming)doc";

static const char *__doc_dai_CameraControl_setStopStreaming = R"doc(Set a command to stop streaming)doc";

static const char *__doc_dai_CameraControl_setStrobeDisable = R"doc(Disable STROBE output)doc";

static const char *__doc_dai_CameraControl_setStrobeExternal =
R"doc(Enable STROBE output driven by a MyriadX GPIO, optionally configuring the
polarity This normally requires a FSIN/FSYNC/trigger input for MyriadX (usually
GPIO 41), to generate timings)doc";

static const char *__doc_dai_CameraControl_setStrobeSensor =
R"doc(Enable STROBE output on sensor pin, optionally configuring the polarity. Note:
for many sensors the polarity is high-active and not configurable)doc";

static const char *__doc_dai_CameraControl_sharpness = R"doc()doc";

static const char *__doc_dai_CameraControl_str = R"doc()doc";

static const char *__doc_dai_CameraControl_strobeConfig = R"doc()doc";

static const char *__doc_dai_CameraControl_strobeTimings = R"doc()doc";

static const char *__doc_dai_CameraControl_wbColorTemp = R"doc()doc";

static const char *__doc_dai_CameraExposureOffset = R"doc(Describe possible exposure offsets)doc";

static const char *__doc_dai_CameraExposureOffset_END = R"doc()doc";

static const char *__doc_dai_CameraExposureOffset_MIDDLE = R"doc()doc";

static const char *__doc_dai_CameraExposureOffset_START = R"doc()doc";

static const char *__doc_dai_CameraFeatures =
R"doc(CameraFeatures structure

Characterizes detected cameras on board)doc";

static const char *__doc_dai_CameraFeatures_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraFeatures_additionalNames = R"doc(Additional camera names or aliases)doc";

static const char *__doc_dai_CameraFeatures_calibrationResolution = R"doc()doc";

static const char *__doc_dai_CameraFeatures_configs = R"doc(Available sensor configs)doc";

static const char *__doc_dai_CameraFeatures_hasAutofocus = R"doc(Whether camera has auto focus capabilities, or is a fixed focus lens)doc";

static const char *__doc_dai_CameraFeatures_hasAutofocusIC = R"doc(Whether an autofocus VCM IC was detected)doc";

static const char *__doc_dai_CameraFeatures_height = R"doc(Maximum sensor resolution)doc";

static const char *__doc_dai_CameraFeatures_name = R"doc(Camera name or alias)doc";

static const char *__doc_dai_CameraFeatures_orientation = R"doc(Default camera orientation, board dependent)doc";

static const char *__doc_dai_CameraFeatures_sensorName = R"doc(Camera sensor name, e.g: "IMX378", "OV9282")doc";

static const char *__doc_dai_CameraFeatures_socket = R"doc(Board socket where the camera was detected)doc";

static const char *__doc_dai_CameraFeatures_str = R"doc()doc";

static const char *__doc_dai_CameraFeatures_supportedTypes =
R"doc(List of supported types of processing for the given camera.

For some sensors it's not possible to determine if they are color or mono (e.g.
OV9782 and OV9282), so this could return more than one entry)doc";

static const char *__doc_dai_CameraFeatures_width = R"doc(Maximum sensor resolution)doc";

static const char *__doc_dai_CameraImageOrientation =
R"doc(Camera sensor image orientation / pixel readout. This exposes direct sensor
settings. 90 or 270 degrees rotation is not available.

AUTO denotes that the decision will be made by device (e.g. on OAK-1/megaAI:
ROTATE_180_DEG).)doc";

static const char *__doc_dai_CameraImageOrientation_AUTO = R"doc()doc";

static const char *__doc_dai_CameraImageOrientation_HORIZONTAL_MIRROR = R"doc()doc";

static const char *__doc_dai_CameraImageOrientation_NORMAL = R"doc()doc";

static const char *__doc_dai_CameraImageOrientation_ROTATE_180_DEG = R"doc()doc";

static const char *__doc_dai_CameraImageOrientation_VERTICAL_FLIP = R"doc()doc";

static const char *__doc_dai_CameraInfo = R"doc(CameraInfo structure)doc";

static const char *__doc_dai_CameraInfo_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CameraInfo_cameraType = R"doc()doc";

static const char *__doc_dai_CameraInfo_distortionCoeff = R"doc()doc";

static const char *__doc_dai_CameraInfo_extrinsics = R"doc()doc";

static const char *__doc_dai_CameraInfo_height = R"doc()doc";

static const char *__doc_dai_CameraInfo_intrinsicMatrix = R"doc()doc";

static const char *__doc_dai_CameraInfo_lensPosition = R"doc()doc";

static const char *__doc_dai_CameraInfo_specHfovDeg = R"doc()doc";

static const char *__doc_dai_CameraInfo_str = R"doc()doc";

static const char *__doc_dai_CameraInfo_width = R"doc()doc";

static const char *__doc_dai_CameraModel = R"doc(Which CameraModel to initialize the calibration with.)doc";

static const char *__doc_dai_CameraModel_Equirectangular = R"doc()doc";

static const char *__doc_dai_CameraModel_Fisheye = R"doc()doc";

static const char *__doc_dai_CameraModel_Perspective = R"doc()doc";

static const char *__doc_dai_CameraModel_RadialDivision = R"doc()doc";

static const char *__doc_dai_CameraProperties = R"doc(Specify properties for ColorCamera such as camera ID, ...)doc";

static const char *__doc_dai_CameraProperties_boardSocket = R"doc(Which socket will color camera use)doc";

static const char *__doc_dai_CameraProperties_cameraName = R"doc(Which camera name will color camera use)doc";

static const char *__doc_dai_CameraProperties_fps = R"doc(Camera sensor FPS)doc";

static const char *__doc_dai_CameraProperties_imageOrientation = R"doc(Camera sensor image orientation / pixel readout)doc";

static const char *__doc_dai_CameraProperties_initialControl = R"doc(Initial controls applied to ColorCamera node)doc";

static const char *__doc_dai_CameraProperties_isp3aFps =
R"doc(Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls
etc.). Default (0) matches the camera FPS, meaning that 3A is running on each
frame. Reducing the rate of 3A reduces the CPU usage on CSS, but also increases
the convergence rate of 3A. Note that camera controls will be processed at this
rate. E.g. if camera is running at 30 fps, and camera control is sent at every
frame, but 3A fps is set to 15, the camera control messages will be processed at
15 fps rate, which will lead to queueing.)doc";

static const char *__doc_dai_CameraProperties_mockIspFps = R"doc(Select the mock isp fps. Overrides fps if mockIsp is connected.)doc";

static const char *__doc_dai_CameraProperties_mockIspHeight =
R"doc(Select the mock isp height. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_CameraProperties_mockIspWidth =
R"doc(Select the mock isp width. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_CameraProperties_numFramesPoolIsp = R"doc()doc";

static const char *__doc_dai_CameraProperties_numFramesPoolPreview = R"doc()doc";

static const char *__doc_dai_CameraProperties_numFramesPoolRaw = R"doc(Pool sizes)doc";

static const char *__doc_dai_CameraProperties_numFramesPoolStill = R"doc()doc";

static const char *__doc_dai_CameraProperties_numFramesPoolVideo = R"doc()doc";

static const char *__doc_dai_CameraProperties_outputRequests = R"doc()doc";

static const char *__doc_dai_CameraProperties_resolutionHeight = R"doc(Select the camera sensor height)doc";

static const char *__doc_dai_CameraProperties_resolutionWidth = R"doc(Select the camera sensor width)doc";

static const char *__doc_dai_CameraSensorConfig = R"doc(Sensor config)doc";

static const char *__doc_dai_CameraSensorConfig_fov = R"doc(Sensor active view area in physical area [pixels])doc";

static const char *__doc_dai_CameraSensorConfig_height = R"doc()doc";

static const char *__doc_dai_CameraSensorConfig_maxFps = R"doc()doc";

static const char *__doc_dai_CameraSensorConfig_minFps = R"doc()doc";

static const char *__doc_dai_CameraSensorConfig_type = R"doc()doc";

static const char *__doc_dai_CameraSensorConfig_width = R"doc()doc";

static const char *__doc_dai_CameraSensorType = R"doc(Camera sensor type)doc";

static const char *__doc_dai_CameraSensorType_AUTO = R"doc()doc";

static const char *__doc_dai_CameraSensorType_COLOR = R"doc()doc";

static const char *__doc_dai_CameraSensorType_MONO = R"doc()doc";

static const char *__doc_dai_CameraSensorType_THERMAL = R"doc()doc";

static const char *__doc_dai_CameraSensorType_TOF = R"doc()doc";

static const char *__doc_dai_Capability = R"doc()doc";

static const char *__doc_dai_CapabilityCRTP = R"doc()doc";

static const char *__doc_dai_CapabilityCRTP_get = R"doc()doc";

static const char *__doc_dai_CapabilityCRTP_getName = R"doc()doc";

static const char *__doc_dai_CapabilityRange = R"doc()doc";

static const char *__doc_dai_CapabilityRange_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CapabilityRange_discrete = R"doc()doc";

static const char *__doc_dai_CapabilityRange_fixed = R"doc()doc";

static const char *__doc_dai_CapabilityRange_minMax = R"doc()doc";

static const char *__doc_dai_CapabilityRange_minMax_2 = R"doc()doc";

static const char *__doc_dai_CapabilityRange_minMax_3 = R"doc()doc";

static const char *__doc_dai_CapabilityRange_str = R"doc()doc";

static const char *__doc_dai_CapabilityRange_value = R"doc()doc";

static const char *__doc_dai_Capability_getName = R"doc()doc";

static const char *__doc_dai_CastProperties = R"doc(Specify properties for Cast)doc";

static const char *__doc_dai_CastProperties_numFramesPool = R"doc()doc";

static const char *__doc_dai_CastProperties_offset = R"doc()doc";

static const char *__doc_dai_CastProperties_outputType = R"doc()doc";

static const char *__doc_dai_CastProperties_scale = R"doc()doc";

static const char *__doc_dai_ChipTemperature =
R"doc(Chip temperature information.

Multiple temperature measurement points and their average)doc";

static const char *__doc_dai_ChipTemperatureS3 =
R"doc(Chip temperature information.

Multiple temperature measurement points and their average)doc";

static const char *__doc_dai_ChipTemperatureS3_average = R"doc(Average of measurements)doc";

static const char *__doc_dai_ChipTemperatureS3_css = R"doc(CPU Subsystem)doc";

static const char *__doc_dai_ChipTemperatureS3_mss = R"doc(Media Subsystem)doc";

static const char *__doc_dai_ChipTemperatureS3_nce = R"doc(TODO: What does nce stand for?)doc";

static const char *__doc_dai_ChipTemperatureS3_soc = R"doc(SoC)doc";

static const char *__doc_dai_ChipTemperature_average = R"doc(Average of measurements)doc";

static const char *__doc_dai_ChipTemperature_css = R"doc(CPU Subsystem)doc";

static const char *__doc_dai_ChipTemperature_dss = R"doc(DRAM Subsystem)doc";

static const char *__doc_dai_ChipTemperature_mss = R"doc(Media Subsystem)doc";

static const char *__doc_dai_ChipTemperature_upa = R"doc(Shave Array)doc";

static const char *__doc_dai_CircleAnnotation = R"doc()doc";

static const char *__doc_dai_CircleAnnotation_diameter = R"doc()doc";

static const char *__doc_dai_CircleAnnotation_fillColor = R"doc()doc";

static const char *__doc_dai_CircleAnnotation_outlineColor = R"doc()doc";

static const char *__doc_dai_CircleAnnotation_position = R"doc()doc";

static const char *__doc_dai_CircleAnnotation_thickness = R"doc()doc";

static const char *__doc_dai_Color =
R"doc(Color structure

r,g,b,a color values with values in range [0.0, 1.0])doc";

static const char *__doc_dai_ColorCameraProperties = R"doc(Specify properties for ColorCamera such as camera ID, ...)doc";

static const char *__doc_dai_ColorCameraProperties_ColorOrder = R"doc(For 24 bit color these can be either RGB or BGR)doc";

static const char *__doc_dai_ColorCameraProperties_ColorOrder_BGR = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_ColorOrder_RGB = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_horizDenominator = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_horizNumerator = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_str = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_vertDenominator = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_IspScale_vertNumerator = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution = R"doc(Select the camera sensor resolution)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_1080_P = R"doc(1920 × 1080)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_1200_P = R"doc(1920 × 1200)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_1280X962 = R"doc(1280 x 962)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_12_MP = R"doc(4056 × 3040)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_1352X1012 = R"doc(1352 × 1012)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_13_MP = R"doc(4208 × 3120)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_1440X1080 = R"doc(1440 × 1080)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_2000X1500 = R"doc(2000 × 1500)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_2024X1520 = R"doc(2024 × 1520)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_2028X1520 = R"doc(2028 × 1520)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_2104X1560 = R"doc(2104 × 1560)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_240X180 = R"doc(240 x 180)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_4000X3000 = R"doc(4000 × 3000)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_48_MP = R"doc(8000 × 6000)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_4_K = R"doc(3840 × 2160)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_5312X6000 = R"doc(5312 × 6000)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_5_MP = R"doc(2592 × 1944)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_720_P = R"doc(1280 × 720)doc";

static const char *__doc_dai_ColorCameraProperties_SensorResolution_THE_800_P = R"doc(1280 × 800)doc";

static const char *__doc_dai_ColorCameraProperties_WarpMeshSource = R"doc(Warp mesh source)doc";

static const char *__doc_dai_ColorCameraProperties_WarpMeshSource_AUTO = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_WarpMeshSource_CALIBRATION = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_WarpMeshSource_NONE = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_WarpMeshSource_URI = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_boardSocket = R"doc(Which socket will color camera use)doc";

static const char *__doc_dai_ColorCameraProperties_calibAlpha = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_cameraName = R"doc(Which camera name will color camera use)doc";

static const char *__doc_dai_ColorCameraProperties_eventFilter = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_fps = R"doc(Camera sensor FPS)doc";

static const char *__doc_dai_ColorCameraProperties_imageOrientation = R"doc(Camera sensor image orientation / pixel readout)doc";

static const char *__doc_dai_ColorCameraProperties_initialControl = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_isp3aFps =
R"doc(Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls
etc.). Default (0) matches the camera FPS, meaning that 3A is running on each
frame. Reducing the rate of 3A reduces the CPU usage on CSS, but also increases
the convergence rate of 3A. Note that camera controls will be processed at this
rate. E.g. if camera is running at 30 fps, and camera control is sent at every
frame, but 3A fps is set to 15, the camera control messages will be processed at
15 fps rate, which will lead to queueing.)doc";

static const char *__doc_dai_ColorCameraProperties_ispScale = R"doc(Configure scaling for `isp` output.)doc";

static const char *__doc_dai_ColorCameraProperties_mockIspHeight =
R"doc(Select the mock isp height. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_ColorCameraProperties_mockIspWidth =
R"doc(Select the mock isp width. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_ColorCameraProperties_numFramesPoolIsp = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_numFramesPoolPreview = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_numFramesPoolRaw = R"doc(Pool sizes)doc";

static const char *__doc_dai_ColorCameraProperties_numFramesPoolStill = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_numFramesPoolVideo = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_previewHeight = R"doc(Preview frame output height)doc";

static const char *__doc_dai_ColorCameraProperties_previewKeepAspectRatio = R"doc(Whether to keep aspect ratio of input (video size) or not)doc";

static const char *__doc_dai_ColorCameraProperties_previewType = R"doc(Frame type)doc";

static const char *__doc_dai_ColorCameraProperties_previewWidth = R"doc(Preview frame output width)doc";

static const char *__doc_dai_ColorCameraProperties_rawPacked =
R"doc(Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
The packed format is more efficient, consuming less memory on device, and less
data to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on
3 bytes. When packing is disabled (`false`), data is saved lsb-aligned, e.g. a
RAW10 pixel will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
Default is auto: enabled for standard color/monochrome cameras where ISP can
work with both packed/unpacked, but disabled for other cameras like ToF.)doc";

static const char *__doc_dai_ColorCameraProperties_resolution = R"doc(Select the camera sensor resolution)doc";

static const char *__doc_dai_ColorCameraProperties_sensorCropX = R"doc(Initial sensor crop, -1 signifies center crop)doc";

static const char *__doc_dai_ColorCameraProperties_sensorCropY = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_stillHeight = R"doc(Preview frame output height)doc";

static const char *__doc_dai_ColorCameraProperties_stillWidth = R"doc(Preview frame output width)doc";

static const char *__doc_dai_ColorCameraProperties_videoHeight = R"doc(Preview frame output height)doc";

static const char *__doc_dai_ColorCameraProperties_videoWidth = R"doc(Preview frame output width)doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshHeight = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshSource = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshStepHeight = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshStepWidth = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshUri = R"doc()doc";

static const char *__doc_dai_ColorCameraProperties_warpMeshWidth = R"doc()doc";

static const char *__doc_dai_Color_Color = R"doc()doc";

static const char *__doc_dai_Color_Color_2 =
R"doc(Parameter ``r``:
    Red value

Parameter ``g``:
    Green value

Parameter ``b``:
    Blue value

Parameter ``a``:
    Alpha value

Throws:
    std::invalid_argument if r,g,b,a values are not in range [0.0, 1.0])doc";

static const char *__doc_dai_Color_a = R"doc()doc";

static const char *__doc_dai_Color_b = R"doc()doc";

static const char *__doc_dai_Color_g = R"doc()doc";

static const char *__doc_dai_Color_r = R"doc()doc";

static const char *__doc_dai_Colormap = R"doc(Camera sensor type)doc";

static const char *__doc_dai_Colormap_JET = R"doc()doc";

static const char *__doc_dai_Colormap_NONE = R"doc()doc";

static const char *__doc_dai_Colormap_STEREO_JET = R"doc()doc";

static const char *__doc_dai_Colormap_STEREO_TURBO = R"doc()doc";

static const char *__doc_dai_Colormap_TURBO = R"doc()doc";

static const char *__doc_dai_ConnectionInterface = R"doc()doc";

static const char *__doc_dai_ConnectionInterface_ETHERNET = R"doc()doc";

static const char *__doc_dai_ConnectionInterface_USB = R"doc()doc";

static const char *__doc_dai_ConnectionInterface_WIFI = R"doc()doc";

static const char *__doc_dai_CpuUsage =
R"doc(CpuUsage structure

Average usage in percent and time span of the average (since last query))doc";

static const char *__doc_dai_CpuUsage_average = R"doc(Average CPU usage, expressed with a normalized value (0-1))doc";

static const char *__doc_dai_CpuUsage_msTime = R"doc(Time span in which the average was calculated in milliseconds)doc";

static const char *__doc_dai_CrashDump = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext_fileName = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext_functionName = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext_line = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_AssertContext_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext_trapAddress = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext_trapName = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_TrapContext_trapNumber = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_assertContext = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_errorId = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ErrorSourceInfo_trapContext = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_callSite = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_calledTarget = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_context = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_framePointer = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_CallstackContext_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_callStack = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_instructionPointer = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_stackBottom = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_stackPointer = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_stackTop = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_threadId = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_threadName = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_ThreadCallstack_threadStatus = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_crashedThreadId = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_errorSource = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_errorSourceInfo = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_processor = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_str = R"doc()doc";

static const char *__doc_dai_CrashDump_CrashReport_threadCallstack = R"doc()doc";

static const char *__doc_dai_CrashDump_crashReports = R"doc()doc";

static const char *__doc_dai_CrashDump_depthaiCommitHash = R"doc()doc";

static const char *__doc_dai_CrashDump_deviceId = R"doc()doc";

static const char *__doc_dai_CrashDump_serializeToJson = R"doc()doc";

static const char *__doc_dai_Crop = R"doc()doc";

static const char *__doc_dai_Crop_Crop = R"doc()doc";

static const char *__doc_dai_Crop_Crop_2 = R"doc()doc";

static const char *__doc_dai_Crop_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Crop_center = R"doc()doc";

static const char *__doc_dai_Crop_clone = R"doc()doc";

static const char *__doc_dai_Crop_height = R"doc()doc";

static const char *__doc_dai_Crop_normalized = R"doc()doc";

static const char *__doc_dai_Crop_str = R"doc()doc";

static const char *__doc_dai_Crop_toStr = R"doc()doc";

static const char *__doc_dai_Crop_width = R"doc()doc";

static const char *__doc_dai_DatatypeEnum = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ADatatype = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_AprilTagConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_AprilTags = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_BenchmarkReport = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_Buffer = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_CameraControl = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_EdgeDetectorConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_EncodedFrame = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_FeatureTrackerConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_IMUData = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ImageAlignConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ImageManipConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ImgAnnotations = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ImgDetections = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ImgFrame = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_Landmarks = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_MessageGroup = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_NNData = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ObjectTrackerConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_PointCloudConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_PointCloudData = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_RGBDData = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_SpatialImgDetections = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_SpatialLocationCalculatorConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_SpatialLocationCalculatorData = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_StereoDepthConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_SystemInformation = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_SystemInformationS3 = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ThermalConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_ToFConfig = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_TrackedFeatures = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_Tracklets = R"doc()doc";

static const char *__doc_dai_DatatypeEnum_TransformData = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions =
R"doc(DetectionParserOptions

Specifies how to parse output of detection networks)doc";

static const char *__doc_dai_DetectionParserOptions_anchorMasks = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_anchors = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_anchorsV2 = R"doc(see YoloDetectionNetwork::setAnchors() for format)doc";

static const char *__doc_dai_DetectionParserOptions_classNames = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_classes = R"doc(YOLO specific network properties)doc";

static const char *__doc_dai_DetectionParserOptions_confidenceThreshold = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_coordinates = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_iouThreshold = R"doc()doc";

static const char *__doc_dai_DetectionParserOptions_nnFamily = R"doc(Generic Neural Network properties)doc";

static const char *__doc_dai_DetectionParserOptions_subtype = R"doc()doc";

static const char *__doc_dai_DetectionParserProperties = R"doc(Specify properties for DetectionParser)doc";

static const char *__doc_dai_DetectionParserProperties_networkInputs = R"doc(Network inputs)doc";

static const char *__doc_dai_DetectionParserProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_DetectionParserProperties_parser = R"doc(Options for parser)doc";

static const char *__doc_dai_Device =
R"doc(Represents the DepthAI device with the methods to interact with it. Implements
the host-side queues to connect with XLinkIn and XLinkOut nodes)doc";

static const char *__doc_dai_DeviceBase =
R"doc(The core of depthai device for RAII, connects to device and maintains watchdog,
timesync, ...)doc";

static const char *__doc_dai_DeviceBase_Config = R"doc(Device specific configuration)doc";

static const char *__doc_dai_DeviceBase_Config_board = R"doc()doc";

static const char *__doc_dai_DeviceBase_Config_logLevel = R"doc()doc";

static const char *__doc_dai_DeviceBase_Config_nonExclusiveMode = R"doc()doc";

static const char *__doc_dai_DeviceBase_Config_outputLogLevel = R"doc()doc";

static const char *__doc_dai_DeviceBase_Config_version = R"doc()doc";

static const char *__doc_dai_DeviceBase_DeviceBase =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout. Uses
OpenVINO version OpenVINO::VERSION_UNIVERSAL)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_2 =
R"doc(Connects to device

Parameter ``maxUsbSpeed``:
    Maximum allowed USB speed)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_3 =
R"doc(Connects to device specified by devInfo.

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to

Parameter ``maxUsbSpeed``:
    Maximum allowed USB speed)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_4 =
R"doc(Connects to device specified by devInfo.

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to

Parameter ``pathToCmd``:
    Path to custom device firmware)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_5 =
R"doc(Connects to any available device with custom config.

Parameter ``config``:
    Device custom configuration to boot with)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_6 =
R"doc(Connects to device 'devInfo' with custom config.

Parameter ``config``:
    Device custom configuration to boot with

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_7 =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout. Uses
OpenVINO version OpenVINO::VERSION_UNIVERSAL

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_8 =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout. Uses
OpenVINO version OpenVINO::VERSION_UNIVERSAL

Parameter ``nameOrDeviceId``:
    Creates DeviceInfo with nameOrDeviceId to connect to)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_9 =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout. Uses
OpenVINO version OpenVINO::VERSION_UNIVERSAL

Parameter ``nameOrDeviceId``:
    Creates DeviceInfo with nameOrDeviceId to connect to

Parameter ``maxUsbSpeed``:
    Maximum allowed USB speed)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_10 =
R"doc(Connects to device specified by devInfo.

Parameter ``config``:
    Config with which the device will be booted with

Parameter ``maxUsbSpeed``:
    Maximum allowed USB speed)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_11 =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout.

Parameter ``config``:
    Config with which the device will be booted with

Parameter ``pathToCmd``:
    Path to custom device firmware)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_12 =
R"doc(Connects to device specified by devInfo.

Parameter ``config``:
    Config with which the device will be booted with

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to

Parameter ``maxUsbSpeed``:
    Maximum allowed USB speed)doc";

static const char *__doc_dai_DeviceBase_DeviceBase_13 =
R"doc(Connects to device specified by devInfo.

Parameter ``config``:
    Config with which the device will be booted with

Parameter ``devInfo``:
    DeviceInfo which specifies which device to connect to

Parameter ``pathToCmd``:
    Path to custom device firmware

Parameter ``dumpOnly``:
    If true only the minimal connection is established to retrieve the crash
    dump)doc";

static const char *__doc_dai_DeviceBase_Impl = R"doc()doc";

static const char *__doc_dai_DeviceBase_PrevInfo = R"doc()doc";

static const char *__doc_dai_DeviceBase_PrevInfo_cfg = R"doc()doc";

static const char *__doc_dai_DeviceBase_PrevInfo_deviceInfo = R"doc()doc";

static const char *__doc_dai_DeviceBase_PrevInfo_hasPipeline = R"doc()doc";

static const char *__doc_dai_DeviceBase_PrevInfo_pathToMvcmd = R"doc()doc";

static const char *__doc_dai_DeviceBase_ReconnectionStatus = R"doc()doc";

static const char *__doc_dai_DeviceBase_ReconnectionStatus_RECONNECTED = R"doc()doc";

static const char *__doc_dai_DeviceBase_ReconnectionStatus_RECONNECTING = R"doc()doc";

static const char *__doc_dai_DeviceBase_ReconnectionStatus_RECONNECT_FAILED = R"doc()doc";

static const char *__doc_dai_DeviceBase_addLogCallback =
R"doc(Add a callback for device logging. The callback will be called from a separate
thread with the LogMessage being passed.

Parameter ``callback``:
    Callback to call whenever a log message arrives

Returns:
    Id which can be used to later remove the callback)doc";

static const char *__doc_dai_DeviceBase_bootloaderVersion = R"doc()doc";

static const char *__doc_dai_DeviceBase_close =
R"doc(Explicitly closes connection to device. @note This function does not need to be
explicitly called as destructor closes the device automatically)doc";

static const char *__doc_dai_DeviceBase_closeImpl =
R"doc(Allows the derived classes to handle custom setup for gracefully stopping the
pipeline

@note Remember to call this function in the overload to setup the communication
properly)doc";

static const char *__doc_dai_DeviceBase_closed = R"doc()doc";

static const char *__doc_dai_DeviceBase_closedMtx = R"doc()doc";

static const char *__doc_dai_DeviceBase_config = R"doc()doc";

static const char *__doc_dai_DeviceBase_connection = R"doc()doc";

static const char *__doc_dai_DeviceBase_crashDevice =
R"doc(Crashes the device $.. warning::

ONLY FOR TESTING PURPOSES, it causes an unrecoverable crash on the device)doc";

static const char *__doc_dai_DeviceBase_deviceInfo = R"doc()doc";

static const char *__doc_dai_DeviceBase_dumpOnly = R"doc()doc";

static const char *__doc_dai_DeviceBase_factoryResetCalibration =
R"doc(Factory reset EEPROM data if factory backup is available.

Throws:
    std::runtime_exception If factory reset was unsuccessful)doc";

static const char *__doc_dai_DeviceBase_firmwarePath = R"doc()doc";

static const char *__doc_dai_DeviceBase_flashCalibration =
R"doc(Stores the Calibration and Device information to the Device EEPROM

Throws:
    std::runtime_exception if failed to flash the calibration

Parameter ``calibrationObj``:
    CalibrationHandler object which is loaded with calibration information.)doc";

static const char *__doc_dai_DeviceBase_flashEepromClear =
R"doc(Destructive action, deletes User area EEPROM contents Requires PROTECTED
permissions

Throws:
    std::runtime_exception if failed to flash the calibration

Returns:
    True on successful flash, false on failure)doc";

static const char *__doc_dai_DeviceBase_flashFactoryCalibration =
R"doc(Stores the Calibration and Device information to the Device EEPROM in Factory
area To perform this action, correct env variable must be set

Throws:
    std::runtime_exception if failed to flash the calibration

Returns:
    True on successful flash, false on failure)doc";

static const char *__doc_dai_DeviceBase_flashFactoryEepromClear =
R"doc(Destructive action, deletes Factory area EEPROM contents Requires FACTORY
PROTECTED permissions

Throws:
    std::runtime_exception if failed to flash the calibration

Returns:
    True on successful flash, false on failure)doc";

static const char *__doc_dai_DeviceBase_gate = R"doc()doc";

static const char *__doc_dai_DeviceBase_getAllAvailableDevices =
R"doc(Returns all available devices

Returns:
    Vector of available devices)doc";

static const char *__doc_dai_DeviceBase_getAllConnectedDevices =
R"doc(Returns information of all connected devices. The devices could be both
connectable as well as already connected to devices.

Returns:
    Vector of connected device information)doc";

static const char *__doc_dai_DeviceBase_getAnyAvailableDevice =
R"doc(Waits for any available device with a timeout

Parameter ``timeout``:
    duration of time to wait for the any device

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_DeviceBase_getAnyAvailableDevice_2 =
R"doc(Gets any available device

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_DeviceBase_getAnyAvailableDevice_3 =
R"doc(Waits for any available device with a timeout

Parameter ``timeout``:
    duration of time to wait for the any device

Parameter ``cb``:
    callback function called between pooling intervals

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_DeviceBase_getAvailableStereoPairs =
R"doc(Get stereo pairs taking into account the calibration and connected cameras.

@note This method will always return a subset of `getStereoPairs`.

Returns:
    Vector of stereo pairs)doc";

static const char *__doc_dai_DeviceBase_getBootloaderVersion =
R"doc(Gets Bootloader version if it was booted through Bootloader

Returns:
    DeviceBootloader::Version if booted through Bootloader or none otherwise)doc";

static const char *__doc_dai_DeviceBase_getCalibration =
R"doc(Retrieves the CalibrationHandler object containing the non-persistent
calibration

Throws:
    std::runtime_exception if failed to get the calibration

Returns:
    The CalibrationHandler object containing the non-persistent calibration)doc";

static const char *__doc_dai_DeviceBase_getCameraSensorNames =
R"doc(Get sensor names for cameras that are connected to the device

Returns:
    Map/dictionary with camera sensor names, indexed by socket)doc";

static const char *__doc_dai_DeviceBase_getChipTemperature =
R"doc(Retrieves current chip temperature as measured by device

Returns:
    Temperature of various onboard sensors)doc";

static const char *__doc_dai_DeviceBase_getCmxMemoryUsage =
R"doc(Retrieves current CMX memory information from device

Returns:
    Used, remaining and total cmx memory)doc";

static const char *__doc_dai_DeviceBase_getConnectedCameraFeatures =
R"doc(Get cameras that are connected to the device with their features/properties

Returns:
    Vector of connected camera features)doc";

static const char *__doc_dai_DeviceBase_getConnectedCameras =
R"doc(Get cameras that are connected to the device

Returns:
    Vector of connected cameras)doc";

static const char *__doc_dai_DeviceBase_getConnectedIMU =
R"doc(Get connected IMU type

Returns:
    IMU type)doc";

static const char *__doc_dai_DeviceBase_getConnection = R"doc(Returns underlying XLinkConnection)doc";

static const char *__doc_dai_DeviceBase_getConnection_2 = R"doc(Returns underlying XLinkConnection)doc";

static const char *__doc_dai_DeviceBase_getConnectionInterfaces =
R"doc(Get connection interfaces for device

Returns:
    Vector of connection type)doc";

static const char *__doc_dai_DeviceBase_getCrashDump = R"doc(Retrieves crash dump for debugging.)doc";

static const char *__doc_dai_DeviceBase_getDdrMemoryUsage =
R"doc(Retrieves current DDR memory information from device

Returns:
    Used, remaining and total ddr memory)doc";

static const char *__doc_dai_DeviceBase_getDefaultSearchTime =
R"doc(Get the Default Search Time for finding devices

Returns:
    Default search time in milliseconds)doc";

static const char *__doc_dai_DeviceBase_getDeviceById =
R"doc(Finds a device by Device ID. Example: 14442C10D13EABCE00

Parameter ``deviceId``:
    Device ID which uniquely specifies a device

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_DeviceBase_getDeviceId =
R"doc(Get DeviceId of device

Returns:
    DeviceId of connected device)doc";

static const char *__doc_dai_DeviceBase_getDeviceInfo =
R"doc(Get the Device Info object o the device which is currently running

Returns:
    DeviceInfo of the current device in execution)doc";

static const char *__doc_dai_DeviceBase_getDeviceName =
R"doc(Get device name if available

Returns:
    device name or empty string if not available)doc";

static const char *__doc_dai_DeviceBase_getEmbeddedDeviceBinary =
R"doc(Gets device firmware binary for a specific OpenVINO version

Parameter ``usb2Mode``:
    USB2 mode firmware

Parameter ``version``:
    Version of OpenVINO which firmware will support

Returns:
    Firmware binary)doc";

static const char *__doc_dai_DeviceBase_getEmbeddedDeviceBinary_2 =
R"doc(Gets device firmware binary for a specific configuration

Parameter ``config``:
    FW with applied configuration

Returns:
    Firmware binary)doc";

static const char *__doc_dai_DeviceBase_getEmbeddedIMUFirmwareVersion =
R"doc(Get embedded IMU firmware version to which IMU can be upgraded

Returns:
    Get embedded IMU firmware version to which IMU can be upgraded.)doc";

static const char *__doc_dai_DeviceBase_getFirstAvailableDevice =
R"doc(Gets first available device. Device can be either in XLINK_UNBOOTED or
XLINK_BOOTLOADER state

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_DeviceBase_getGlobalProfilingData =
R"doc(Get current global accumulated profiling data

Returns:
    ProfilingData from all devices)doc";

static const char *__doc_dai_DeviceBase_getIMUFirmwareUpdateStatus =
R"doc(Get IMU firmware update status

Returns:
    Whether IMU firmware update is done and last firmware update progress as
    percentage. return value true and 100 means that the update was successful
    return value true and other than 100 means that the update failed)doc";

static const char *__doc_dai_DeviceBase_getIMUFirmwareVersion =
R"doc(Get connected IMU firmware version

Returns:
    IMU firmware version)doc";

static const char *__doc_dai_DeviceBase_getIrDrivers =
R"doc(Retrieves detected IR laser/LED drivers.

Returns:
    Vector of tuples containing: driver name, I2C bus, I2C address. For OAK-D-
    Pro it should be `[{"LM3644", 2, 0x63}]`)doc";

static const char *__doc_dai_DeviceBase_getLeonCssCpuUsage =
R"doc(Retrieves average CSS Leon CPU usage

Returns:
    Average CPU usage and sampling duration)doc";

static const char *__doc_dai_DeviceBase_getLeonCssHeapUsage =
R"doc(Retrieves current CSS Leon CPU heap information from device

Returns:
    Used, remaining and total heap memory)doc";

static const char *__doc_dai_DeviceBase_getLeonMssCpuUsage =
R"doc(Retrieves average MSS Leon CPU usage

Returns:
    Average CPU usage and sampling duration)doc";

static const char *__doc_dai_DeviceBase_getLeonMssHeapUsage =
R"doc(Retrieves current MSS Leon CPU heap information from device

Returns:
    Used, remaining and total heap memory)doc";

static const char *__doc_dai_DeviceBase_getLogLevel =
R"doc(Gets current logging severity level of the device.

Returns:
    Logging severity level)doc";

static const char *__doc_dai_DeviceBase_getLogOutputLevel =
R"doc(Gets logging level which decides printing level to standard output.

Returns:
    Standard output printing severity)doc";

static const char *__doc_dai_DeviceBase_getMxId =
R"doc(Get MxId of device

Returns:
    MxId of connected device)doc";

static const char *__doc_dai_DeviceBase_getNodeLogLevel =
R"doc(Gets the logging severity level for a specific node with a given ID.

Parameter ``id``:
    Node ID

Returns:
    Logging severity level)doc";

static const char *__doc_dai_DeviceBase_getProductName =
R"doc(Get product name if available

Returns:
    product name or empty string if not available)doc";

static const char *__doc_dai_DeviceBase_getProfilingData =
R"doc(Get current accumulated profiling data

Returns:
    ProfilingData from the specific device)doc";

static const char *__doc_dai_DeviceBase_getStereoPairs =
R"doc(Get stereo pairs based on the device type.

Returns:
    Vector of stereo pairs)doc";

static const char *__doc_dai_DeviceBase_getSystemInformationLoggingRate =
R"doc(Gets current rate of system information logging ("info" severity) in Hz.

Returns:
    Logging rate in Hz)doc";

static const char *__doc_dai_DeviceBase_getUsbSpeed =
R"doc(Retrieves USB connection speed

Returns:
    USB connection speed of connected device if applicable. Unknown otherwise.)doc";

static const char *__doc_dai_DeviceBase_getXLinkChunkSize =
R"doc(Gets current XLink chunk size.

Returns:
    XLink chunk size in bytes)doc";

static const char *__doc_dai_DeviceBase_hasCrashDump = R"doc(Retrieves whether the is crash dump stored on device or not.)doc";

static const char *__doc_dai_DeviceBase_init = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_2 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_3 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_4 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_5 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_6 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_7 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_8 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_9 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_10 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_11 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_12 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_13 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_14 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_15 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_16 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init_17 = R"doc()doc";

static const char *__doc_dai_DeviceBase_init2 = R"doc()doc";

static const char *__doc_dai_DeviceBase_isClosed =
R"doc(Is the device already closed (or disconnected)

.. warning::
    This function is thread-unsafe and may return outdated incorrect values. It
    is only meant for use in simple single-threaded code. Well written code
    should handle exceptions when calling any DepthAI apis to handle hardware
    events and multithreaded use.)doc";

static const char *__doc_dai_DeviceBase_isClosing = R"doc()doc";

static const char *__doc_dai_DeviceBase_isEepromAvailable =
R"doc(Check if EEPROM is available

Returns:
    True if EEPROM is present on board, false otherwise)doc";

static const char *__doc_dai_DeviceBase_isPipelineRunning =
R"doc(Checks if devices pipeline is already running

Returns:
    True if running, false otherwise)doc";

static const char *__doc_dai_DeviceBase_lastWatchdogPingTime = R"doc()doc";

static const char *__doc_dai_DeviceBase_lastWatchdogPingTimeMtx = R"doc()doc";

static const char *__doc_dai_DeviceBase_logCallbackMap = R"doc()doc";

static const char *__doc_dai_DeviceBase_logCallbackMapMtx = R"doc()doc";

static const char *__doc_dai_DeviceBase_loggingRunning = R"doc()doc";

static const char *__doc_dai_DeviceBase_loggingThread = R"doc()doc";

static const char *__doc_dai_DeviceBase_maxReconnectionAttempts = R"doc()doc";

static const char *__doc_dai_DeviceBase_monitorCallback = R"doc()doc";

static const char *__doc_dai_DeviceBase_monitorThread = R"doc()doc";

static const char *__doc_dai_DeviceBase_pimpl = R"doc()doc";

static const char *__doc_dai_DeviceBase_pipelinePtr = R"doc()doc";

static const char *__doc_dai_DeviceBase_pipelineSchema = R"doc()doc";

static const char *__doc_dai_DeviceBase_profilingRunning = R"doc()doc";

static const char *__doc_dai_DeviceBase_profilingThread = R"doc()doc";

static const char *__doc_dai_DeviceBase_readCalibration =
R"doc(Fetches the EEPROM data from the device and loads it into CalibrationHandler
object If no calibration is flashed, it returns default

Returns:
    The CalibrationHandler object containing the calibration currently flashed
    on device EEPROM)doc";

static const char *__doc_dai_DeviceBase_readCalibration2 =
R"doc(Fetches the EEPROM data from the device and loads it into CalibrationHandler
object

Throws:
    std::runtime_exception if no calibration is flashed

Returns:
    The CalibrationHandler object containing the calibration currently flashed
    on device EEPROM)doc";

static const char *__doc_dai_DeviceBase_readCalibrationOrDefault =
R"doc(Fetches the EEPROM data from the device and loads it into CalibrationHandler
object If no calibration is flashed, it returns default

Returns:
    The CalibrationHandler object containing the calibration currently flashed
    on device EEPROM)doc";

static const char *__doc_dai_DeviceBase_readCalibrationRaw =
R"doc(Fetches the raw EEPROM data from User area

Throws:
    std::runtime_exception if any error occurred

Returns:
    Binary dump of User area EEPROM data)doc";

static const char *__doc_dai_DeviceBase_readFactoryCalibration =
R"doc(Fetches the EEPROM data from Factory area and loads it into CalibrationHandler
object

Throws:
    std::runtime_exception if no calibration is flashed

Returns:
    The CalibrationHandler object containing the calibration currently flashed
    on device EEPROM in Factory Area)doc";

static const char *__doc_dai_DeviceBase_readFactoryCalibrationOrDefault =
R"doc(Fetches the EEPROM data from Factory area and loads it into CalibrationHandler
object If no calibration is flashed, it returns default

Returns:
    The CalibrationHandler object containing the calibration currently flashed
    on device EEPROM in Factory Area)doc";

static const char *__doc_dai_DeviceBase_readFactoryCalibrationRaw =
R"doc(Fetches the raw EEPROM data from Factory area

Throws:
    std::runtime_exception if any error occurred

Returns:
    Binary dump of Factory area EEPROM data)doc";

static const char *__doc_dai_DeviceBase_reconnectionCallback = R"doc()doc";

static const char *__doc_dai_DeviceBase_removeLogCallback =
R"doc(Removes a callback

Parameter ``callbackId``:
    Id of callback to be removed

Returns:
    True if callback was removed, false otherwise)doc";

static const char *__doc_dai_DeviceBase_rpcStream = R"doc()doc";

static const char *__doc_dai_DeviceBase_setCalibration =
R"doc(Sets the Calibration at runtime. This is not persistent and will be lost after
device reset.

Throws:
    std::runtime_error if failed to set the calibration

Parameter ``calibrationObj``:
    CalibrationHandler object which is loaded with calibration information.)doc";

static const char *__doc_dai_DeviceBase_setIrFloodLightIntensity =
R"doc(Sets the intensity of the IR Flood Light. Limits: Intensity is directly
normalized to 0 - 1500mA current. The duty cycle is 30% when exposure time is
longer than 30% frame time. Otherwise, duty cycle is 100% of exposure time. The
duty cycle is controlled by the `left` camera STROBE, aligned to start of
exposure. The emitter is turned off by default

Parameter ``intensity``:
    Intensity on range 0 to 1, that will determine brightness, 0 or negative to
    turn off

Parameter ``mask``:
    Optional mask to modify only Left (0x1) or Right (0x2) sides on OAK-D-Pro-W-
    DEV

Returns:
    True on success, false if not found or other failure)doc";

static const char *__doc_dai_DeviceBase_setIrLaserDotProjectorIntensity =
R"doc(Sets the intensity of the IR Laser Dot Projector. Limits: up to 765mA at 30%
frame time duty cycle when exposure time is longer than 30% frame time.
Otherwise, duty cycle is 100% of exposure time, with current increased up to max
1200mA to make up for shorter duty cycle. The duty cycle is controlled by `left`
camera STROBE, aligned to start of exposure. The emitter is turned off by
default

Parameter ``intensity``:
    Intensity on range 0 to 1, that will determine brightness. 0 or negative to
    turn off

Parameter ``mask``:
    Optional mask to modify only Left (0x1) or Right (0x2) sides on OAK-D-Pro-W-
    DEV

Returns:
    True on success, false if not found or other failure)doc";

static const char *__doc_dai_DeviceBase_setLogLevel =
R"doc(Sets the devices logging severity level. This level affects which logs are
transferred from device to host.

Parameter ``level``:
    Logging severity)doc";

static const char *__doc_dai_DeviceBase_setLogOutputLevel =
R"doc(Sets logging level which decides printing level to standard output. If lower
than setLogLevel, no messages will be printed

Parameter ``level``:
    Standard output printing severity)doc";

static const char *__doc_dai_DeviceBase_setMaxReconnectionAttempts =
R"doc(Sets max number of automatic reconnection attempts

Parameter ``maxAttempts``:
    Maximum number of reconnection attempts, 0 to disable reconnection

Parameter ``callBack``:
    Callback to be called when reconnection is attempted)doc";

static const char *__doc_dai_DeviceBase_setNodeLogLevel =
R"doc(Sets the logging severity level for a specific node with a given ID.

Parameter ``id``:
    Node ID

Parameter ``level``:
    Logging severity)doc";

static const char *__doc_dai_DeviceBase_setSystemInformationLoggingRate =
R"doc(Sets rate of system information logging ("info" severity). Default 1Hz If
parameter is less or equal to zero, then system information logging will be
disabled

Parameter ``rateHz``:
    Logging rate in Hz)doc";

static const char *__doc_dai_DeviceBase_setTimesync =
R"doc(Configures Timesync service on device. It keeps host and device clocks in sync
First time timesync is started it waits until the initial sync is completed
Afterwards the function changes the following parameters

Parameter ``period``:
    Interval between timesync runs

Parameter ``numSamples``:
    Number of timesync samples per run which are used to compute a better value.
    Set to zero to disable timesync

Parameter ``random``:
    If true partial timesync requests will be performed at random intervals,
    otherwise at fixed intervals)doc";

static const char *__doc_dai_DeviceBase_setTimesync_2 =
R"doc(Enables or disables Timesync service on device. It keeps host and device clocks
in sync.

Parameter ``enable``:
    Enables or disables consistent timesyncing)doc";

static const char *__doc_dai_DeviceBase_setXLinkChunkSize =
R"doc(Sets the chunk size for splitting device-sent XLink packets. A larger value
could increase performance, and 0 disables chunking. A negative value is
ignored. Device defaults are configured per protocol, currently 64*1024 for both
USB and Ethernet.

Parameter ``sizeBytes``:
    XLink chunk size in bytes)doc";

static const char *__doc_dai_DeviceBase_startIMUFirmwareUpdate =
R"doc(Starts IMU firmware update asynchronously only if IMU node is not running. If
current firmware version is the same as embedded firmware version then it's no-
op. Can be overridden by forceUpdate parameter. State of firmware update can be
monitored using getIMUFirmwareUpdateStatus API.

Parameter ``forceUpdate``:
    Force firmware update or not. Will perform FW update regardless of current
    version and embedded firmware version.

Returns:
    Returns whether firmware update can be started. Returns false if IMU node is
    started.)doc";

static const char *__doc_dai_DeviceBase_startPipeline =
R"doc(Starts the execution of a given pipeline

Parameter ``pipeline``:
    OpenVINO version of the pipeline must match the one which the device was
    booted with.

Returns:
    True if pipeline started, false otherwise)doc";

static const char *__doc_dai_DeviceBase_startPipelineImpl =
R"doc(Allows the derived classes to handle custom setup for starting the pipeline

Parameter ``pipeline``:
    OpenVINO version of the pipeline must match the one which the device was
    booted with

See also:
    startPipeline @note Remember to call this function in the overload to setup
    the communication properly

Returns:
    True if pipeline started, false otherwise)doc";

static const char *__doc_dai_DeviceBase_timesyncRunning = R"doc()doc";

static const char *__doc_dai_DeviceBase_timesyncThread = R"doc()doc";

static const char *__doc_dai_DeviceBase_tryFlashCalibration =
R"doc(Stores the Calibration and Device information to the Device EEPROM

Parameter ``calibrationObj``:
    CalibrationHandler object which is loaded with calibration information.

Returns:
    true on successful flash, false on failure)doc";

static const char *__doc_dai_DeviceBase_tryGetDevice = R"doc()doc";

static const char *__doc_dai_DeviceBase_tryStartPipeline = R"doc(a safe way to start a pipeline, which is closed if any exception occurs)doc";

static const char *__doc_dai_DeviceBase_uniqueCallbackId = R"doc()doc";

static const char *__doc_dai_DeviceBase_watchdogCondVar = R"doc()doc";

static const char *__doc_dai_DeviceBase_watchdogMtx = R"doc()doc";

static const char *__doc_dai_DeviceBase_watchdogRunning = R"doc()doc";

static const char *__doc_dai_DeviceBase_watchdogThread = R"doc()doc";

static const char *__doc_dai_DeviceBootloader = R"doc(Represents the DepthAI bootloader with the methods to interact with it.)doc";

static const char *__doc_dai_DeviceBootloader_ApplicationInfo = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_ApplicationInfo_applicationName = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_ApplicationInfo_firmwareVersion = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_ApplicationInfo_hasApplication = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_ApplicationInfo_memory = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_Config = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_Config_data = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_Config_fromJson = R"doc(from JSON)doc";

static const char *__doc_dai_DeviceBootloader_Config_getDnsAltIPv4 = R"doc(Get alternate IPv4 DNS server)doc";

static const char *__doc_dai_DeviceBootloader_Config_getDnsIPv4 = R"doc(Get primary IPv4 DNS server)doc";

static const char *__doc_dai_DeviceBootloader_Config_getIPv4 = R"doc(Get IPv4)doc";

static const char *__doc_dai_DeviceBootloader_Config_getIPv4Gateway = R"doc(Get IPv4 gateway)doc";

static const char *__doc_dai_DeviceBootloader_Config_getIPv4Mask = R"doc(Get IPv4 mask)doc";

static const char *__doc_dai_DeviceBootloader_Config_getMacAddress = R"doc(Get MAC address if not flashed on controller)doc";

static const char *__doc_dai_DeviceBootloader_Config_getNetworkTimeout = R"doc(Get NETWORK timeout)doc";

static const char *__doc_dai_DeviceBootloader_Config_getUsbMaxSpeed = R"doc(Get maxUsbSpeed)doc";

static const char *__doc_dai_DeviceBootloader_Config_getUsbTimeout = R"doc(Get USB timeout)doc";

static const char *__doc_dai_DeviceBootloader_Config_isStaticIPV4 = R"doc(Get if static IPv4 configuration is set)doc";

static const char *__doc_dai_DeviceBootloader_Config_setDnsIPv4 = R"doc(Set IPv4 DNS options)doc";

static const char *__doc_dai_DeviceBootloader_Config_setDynamicIPv4 = R"doc(Setting a dynamic IPv4 will set that IP as well as start DHCP client)doc";

static const char *__doc_dai_DeviceBootloader_Config_setMacAddress = R"doc(Set MAC address if not flashed on controller)doc";

static const char *__doc_dai_DeviceBootloader_Config_setNetworkTimeout = R"doc(Set NETWOR timeout)doc";

static const char *__doc_dai_DeviceBootloader_Config_setStaticIPv4 = R"doc(Setting a static IPv4 won't start DHCP client)doc";

static const char *__doc_dai_DeviceBootloader_Config_setUsbMaxSpeed = R"doc(Set maxUsbSpeed)doc";

static const char *__doc_dai_DeviceBootloader_Config_setUsbTimeout = R"doc(Set USB timeout)doc";

static const char *__doc_dai_DeviceBootloader_Config_toJson = R"doc(To JSON)doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader_2 =
R"doc(Connects to or boots device in bootloader mode depending on devInfo state;
flashing not allowed

Parameter ``devInfo``:
    DeviceInfo of which to boot or connect to)doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader_3 =
R"doc(Connects to or boots device in bootloader mode depending on devInfo state.

Parameter ``devInfo``:
    DeviceInfo of which to boot or connect to

Parameter ``allowFlashingBootloader``:
    (bool) Set to true to allow flashing the devices bootloader)doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader_4 =
R"doc(Connects to device in bootloader of specified type. Throws if it wasn't
possible. This constructor will automatically boot into specified bootloader
type if not already running

Parameter ``devInfo``:
    DeviceInfo of which to boot or connect to

Parameter ``type``:
    Type of bootloader to boot/connect to.

Parameter ``allowFlashingBootloader``:
    Set to true to allow flashing the devices bootloader. Defaults to false)doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader_5 =
R"doc(Connects to or boots device in bootloader mode depending on devInfo state with a
custom bootloader firmware.

Parameter ``devInfo``:
    DeviceInfo of which to boot or connect to

Parameter ``pathToBootloader``:
    Custom bootloader firmware to boot

Parameter ``allowFlashingBootloader``:
    Set to true to allow flashing the devices bootloader. Defaults to false)doc";

static const char *__doc_dai_DeviceBootloader_DeviceBootloader_6 =
R"doc(Connects to device with specified name/device id

Parameter ``nameOrDeviceId``:
    Creates DeviceInfo with nameOrDeviceId to connect to

Parameter ``allowFlashingBootloader``:
    Set to true to allow flashing the devices bootloader. Defaults to false)doc";

static const char *__doc_dai_DeviceBootloader_MemoryInfo = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_MemoryInfo_available = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_MemoryInfo_info = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_MemoryInfo_size = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_allowFlashingBootloader = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_bootMemory =
R"doc(Boots a custom FW in memory

Parameter ``fw``:
    $Throws:

A runtime exception if there are any communication issues)doc";

static const char *__doc_dai_DeviceBootloader_bootUsbRomBootloader =
R"doc(Boots into integrated ROM bootloader in USB mode

Throws:
    A runtime exception if there are any communication issues)doc";

static const char *__doc_dai_DeviceBootloader_bootloaderType = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_close =
R"doc(Explicitly closes connection to device. @note This function does not need to be
explicitly called as destructor closes the device automatically)doc";

static const char *__doc_dai_DeviceBootloader_closed = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_connection = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_createDepthaiApplicationPackage =
R"doc(Creates application package which can be flashed to depthai device.

Parameter ``pipeline``:
    Pipeline from which to create the application package

Parameter ``pathToCmd``:
    Optional path to custom device firmware

Parameter ``compress``:
    Optional boolean which specifies if contents should be compressed

Parameter ``applicationName``:
    Optional name the application that is flashed

Returns:
    Depthai application package)doc";

static const char *__doc_dai_DeviceBootloader_createDepthaiApplicationPackage_2 =
R"doc(Creates application package which can be flashed to depthai device.

Parameter ``pipeline``:
    Pipeline from which to create the application package

Parameter ``compress``:
    Specifies if contents should be compressed

Parameter ``applicationName``:
    Name the application that is flashed

Returns:
    Depthai application package)doc";

static const char *__doc_dai_DeviceBootloader_createWatchdog = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_destroyWatchdog = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_deviceInfo = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_flash =
R"doc(Flashes a given pipeline to the device.

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``pipeline``:
    Pipeline to flash to the board

Parameter ``compress``:
    Compresses application to reduce needed memory size

Parameter ``applicationName``:
    Name the application that is flashed)doc";

static const char *__doc_dai_DeviceBootloader_flash_2 =
R"doc(Flashes a given pipeline to the device.

Parameter ``pipeline``:
    Pipeline to flash to the board

Parameter ``compress``:
    Compresses application to reduce needed memory size

Parameter ``applicationName``:
    Optional name the application that is flashed)doc";

static const char *__doc_dai_DeviceBootloader_flashBootHeader =
R"doc(Flash optimized boot header

Parameter ``memory``:
    Which memory to flasht the header to

Parameter ``frequency``:
    SPI specific parameter, frequency in MHz

Parameter ``location``:
    Target location the header should boot to. Default to location of bootloader

Parameter ``dummyCycles``:
    SPI specific parameter

Parameter ``offset``:
    Offset in memory to flash the header to. Defaults to offset of boot header

Returns:
    status as std::tuple<bool, std::string>)doc";

static const char *__doc_dai_DeviceBootloader_flashBootloader =
R"doc(Flashes bootloader to the current board

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``path``:
    Optional parameter to custom bootloader to flash)doc";

static const char *__doc_dai_DeviceBootloader_flashBootloader_2 =
R"doc(Flash selected bootloader to the current board

Parameter ``memory``:
    Memory to flash

Parameter ``type``:
    Bootloader type to flash

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``path``:
    Optional parameter to custom bootloader to flash)doc";

static const char *__doc_dai_DeviceBootloader_flashClear =
R"doc(Clears flashed application on the device, by removing SBR boot structure Doesn't
remove fast boot header capability to still boot the application)doc";

static const char *__doc_dai_DeviceBootloader_flashConfig =
R"doc(Flashes configuration to bootloader

Parameter ``configData``:
    Configuration structure

Parameter ``memory``:
    Optional - to which memory flash configuration

Parameter ``type``:
    Optional - for which type of bootloader to flash configuration)doc";

static const char *__doc_dai_DeviceBootloader_flashConfigClear =
R"doc(Clears configuration data

Parameter ``memory``:
    Optional - on which memory to clear configuration data

Parameter ``type``:
    Optional - for which type of bootloader to clear configuration data)doc";

static const char *__doc_dai_DeviceBootloader_flashConfigData =
R"doc(Flashes configuration data to bootloader

Parameter ``configData``:
    Unstructured configuration data

Parameter ``memory``:
    Optional - to which memory flash configuration

Parameter ``type``:
    Optional - for which type of bootloader to flash configuration)doc";

static const char *__doc_dai_DeviceBootloader_flashConfigFile =
R"doc(Flashes configuration data to bootloader

Parameter ``configPath``:
    Unstructured configuration data

Parameter ``memory``:
    Optional - to which memory flash configuration

Parameter ``type``:
    Optional - for which type of bootloader to flash configuration)doc";

static const char *__doc_dai_DeviceBootloader_flashCustom =
R"doc(Flash arbitrary data at custom offset in specified memory

Parameter ``memory``:
    Memory to flash

Parameter ``offset``:
    Offset at which to flash the given data in bytes

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``data``:
    Data to flash)doc";

static const char *__doc_dai_DeviceBootloader_flashCustom_2 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_flashCustom_3 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_flashCustom_4 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_flashDepthaiApplicationPackage =
R"doc(Flashes a specific depthai application package that was generated using
createDepthaiApplicationPackage or saveDepthaiApplicationPackage

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``package``:
    Depthai application package to flash to the board)doc";

static const char *__doc_dai_DeviceBootloader_flashDepthaiApplicationPackage_2 =
R"doc(Flashes a specific depthai application package that was generated using
createDepthaiApplicationPackage or saveDepthaiApplicationPackage

Parameter ``package``:
    Depthai application package to flash to the board)doc";

static const char *__doc_dai_DeviceBootloader_flashFastBootHeader =
R"doc(Flash fast boot header. Application must already be present in flash, or
location must be specified manually. Note - Can soft brick your device if
firmware location changes.

Parameter ``memory``:
    Which memory to flash the header to

Parameter ``frequency``:
    SPI specific parameter, frequency in MHz

Parameter ``location``:
    Target location the header should boot to. Default to location of bootloader

Parameter ``dummyCycles``:
    SPI specific parameter

Parameter ``offset``:
    Offset in memory to flash the header to. Defaults to offset of boot header

Returns:
    status as std::tuple<bool, std::string>)doc";

static const char *__doc_dai_DeviceBootloader_flashGpioModeBootHeader =
R"doc(Flash boot header which boots same as equivalent GPIO mode would

Parameter ``gpioMode``:
    GPIO mode equivalent)doc";

static const char *__doc_dai_DeviceBootloader_flashUsbRecoveryBootHeader =
R"doc(Flash USB recovery boot header. Switches to USB ROM Bootloader

Parameter ``memory``:
    Which memory to flash the header to)doc";

static const char *__doc_dai_DeviceBootloader_flashUserBootloader =
R"doc(Flashes user bootloader to the current board. Available for NETWORK bootloader
type

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    flashing progress

Parameter ``path``:
    Optional parameter to custom bootloader to flash)doc";

static const char *__doc_dai_DeviceBootloader_getAllAvailableDevices =
R"doc(Searches for connected devices in either UNBOOTED or BOOTLOADER states.

Returns:
    Vector of all found devices)doc";

static const char *__doc_dai_DeviceBootloader_getEmbeddedBootloaderBinary =
R"doc(Returns:
    Embedded bootloader binary)doc";

static const char *__doc_dai_DeviceBootloader_getEmbeddedBootloaderVersion =
R"doc(Returns:
    Embedded bootloader version)doc";

static const char *__doc_dai_DeviceBootloader_getFirstAvailableDevice =
R"doc(Searches for connected devices in either UNBOOTED or BOOTLOADER states and
returns first available.

Returns:
    Tuple of boolean and DeviceInfo. If found boolean is true and DeviceInfo
    describes the device. Otherwise false)doc";

static const char *__doc_dai_DeviceBootloader_getMemoryInfo =
R"doc(Retrieves information about specified memory

Parameter ``memory``:
    Specifies which memory to query)doc";

static const char *__doc_dai_DeviceBootloader_getType =
R"doc(Returns:
    Type of currently connected bootloader)doc";

static const char *__doc_dai_DeviceBootloader_getVersion =
R"doc(Returns:
    Version of current running bootloader)doc";

static const char *__doc_dai_DeviceBootloader_init = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_isAllowedFlashingBootloader =
R"doc(Returns:
    True if allowed to flash bootloader)doc";

static const char *__doc_dai_DeviceBootloader_isClosed =
R"doc(Is the device already closed (or disconnected)

.. warning::
    This function is thread-unsafe and may return outdated incorrect values. It
    is only meant for use in simple single-threaded code. Well written code
    should handle exceptions when calling any DepthAI apis to handle hardware
    events and multithreaded use.)doc";

static const char *__doc_dai_DeviceBootloader_isEmbedded = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_isEmbeddedVersion =
R"doc(Returns:
    True when bootloader was booted using latest bootloader integrated in the
    library. False when bootloader is already running on the device and just
    connected to.)doc";

static const char *__doc_dai_DeviceBootloader_isUserBootloader =
R"doc(Retrieves whether current bootloader is User Bootloader (B out of A/B
configuration))doc";

static const char *__doc_dai_DeviceBootloader_isUserBootloaderSupported =
R"doc(Checks whether User Bootloader is supported with current bootloader

Returns:
    true of User Bootloader is supported, false otherwise)doc";

static const char *__doc_dai_DeviceBootloader_lastWatchdogPingTime = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_lastWatchdogPingTimeMtx = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_monitorThread = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_parseResponse = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_readApplicationInfo =
R"doc(Reads information about flashed application in specified memory from device

Parameter ``memory``:
    Specifies which memory to query)doc";

static const char *__doc_dai_DeviceBootloader_readConfig =
R"doc(Reads configuration from bootloader

Parameter ``memory``:
    Optional - from which memory to read configuration

Parameter ``type``:
    Optional - from which type of bootloader to read configuration

Returns:
    Configuration structure)doc";

static const char *__doc_dai_DeviceBootloader_readConfigData =
R"doc(Reads configuration data from bootloader

Returns:
    Unstructured configuration data

Parameter ``memory``:
    Optional - from which memory to read configuration data

Parameter ``type``:
    Optional - from which type of bootloader to read configuration data)doc";

static const char *__doc_dai_DeviceBootloader_readCustom =
R"doc(Reads arbitrary data at custom offset in specified memory

Parameter ``memory``:
    Memory to read

Parameter ``offset``:
    Offset at which to read the specified bytes

Parameter ``size``:
    Number of bytes to read

Parameter ``data``:
    Data to read to. Must be at least 'size' number of bytes big

Parameter ``progressCallback``:
    Callback that sends back a value between 0..1 which signifies current
    reading progress)doc";

static const char *__doc_dai_DeviceBootloader_readCustom_2 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_readCustom_3 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_readCustom_4 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_readCustom_5 = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_receiveResponse = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_receiveResponseData = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_receiveResponseThrow = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_requestVersion = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_saveDepthaiApplicationPackage =
R"doc(Saves application package to a file which can be flashed to depthai device.

Parameter ``path``:
    Path where to save the application package

Parameter ``pipeline``:
    Pipeline from which to create the application package

Parameter ``pathToCmd``:
    Optional path to custom device firmware

Parameter ``compress``:
    Optional boolean which specifies if contents should be compressed

Parameter ``applicationName``:
    Optional name the application that is flashed)doc";

static const char *__doc_dai_DeviceBootloader_saveDepthaiApplicationPackage_2 =
R"doc(Saves application package to a file which can be flashed to depthai device.

Parameter ``path``:
    Path where to save the application package

Parameter ``pipeline``:
    Pipeline from which to create the application package

Parameter ``compress``:
    Specifies if contents should be compressed

Parameter ``applicationName``:
    Optional name the application that is flashed)doc";

static const char *__doc_dai_DeviceBootloader_sendRequest = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_sendRequestThrow = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_stream = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_version = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_watchdogRunning = R"doc()doc";

static const char *__doc_dai_DeviceBootloader_watchdogThread = R"doc()doc";

static const char *__doc_dai_DeviceGate = R"doc(Represents the DepthAI Gate with the methods to interact with it.)doc";

static const char *__doc_dai_DeviceGate_CrashDump = R"doc()doc";

static const char *__doc_dai_DeviceGate_CrashDump_data = R"doc()doc";

static const char *__doc_dai_DeviceGate_CrashDump_filename = R"doc()doc";

static const char *__doc_dai_DeviceGate_DeviceGate =
R"doc(Connects to DepthAI Gate

Parameter ``deviceInfo``:
    Device to connect to)doc";

static const char *__doc_dai_DeviceGate_Impl = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_CRASHED = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_CREATED = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_DESTROYED = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_ERROR_STATE = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_NOT_CREATED = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_RUNNING = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_STOPPED = R"doc()doc";

static const char *__doc_dai_DeviceGate_SessionState_STOPPING = R"doc()doc";

static const char *__doc_dai_DeviceGate_VersionInfo = R"doc()doc";

static const char *__doc_dai_DeviceGate_VersionInfo_gate = R"doc()doc";

static const char *__doc_dai_DeviceGate_VersionInfo_os = R"doc()doc";

static const char *__doc_dai_DeviceGate_createSession = R"doc()doc";

static const char *__doc_dai_DeviceGate_deleteSession = R"doc()doc";

static const char *__doc_dai_DeviceGate_destroySession = R"doc()doc";

static const char *__doc_dai_DeviceGate_deviceInfo = R"doc()doc";

static const char *__doc_dai_DeviceGate_getAllVersion = R"doc()doc";

static const char *__doc_dai_DeviceGate_getCrashDump = R"doc()doc";

static const char *__doc_dai_DeviceGate_getFile = R"doc()doc";

static const char *__doc_dai_DeviceGate_getState = R"doc()doc";

static const char *__doc_dai_DeviceGate_getVersion = R"doc()doc";

static const char *__doc_dai_DeviceGate_isBootedNonExclusive = R"doc()doc";

static const char *__doc_dai_DeviceGate_isOkay = R"doc()doc";

static const char *__doc_dai_DeviceGate_pimpl = R"doc()doc";

static const char *__doc_dai_DeviceGate_platform = R"doc()doc";

static const char *__doc_dai_DeviceGate_sessionCreated = R"doc()doc";

static const char *__doc_dai_DeviceGate_sessionId = R"doc()doc";

static const char *__doc_dai_DeviceGate_startSession = R"doc()doc";

static const char *__doc_dai_DeviceGate_stateMonitoringThread = R"doc()doc";

static const char *__doc_dai_DeviceGate_stopSession = R"doc()doc";

static const char *__doc_dai_DeviceGate_version = R"doc()doc";

static const char *__doc_dai_DeviceGate_waitForSessionEnd = R"doc()doc";

static const char *__doc_dai_DeviceInfo = R"doc(Describes a connected device)doc";

static const char *__doc_dai_DeviceInfo_DeviceInfo = R"doc()doc";

static const char *__doc_dai_DeviceInfo_DeviceInfo_2 = R"doc()doc";

static const char *__doc_dai_DeviceInfo_DeviceInfo_3 =
R"doc(Creates a DeviceInfo by checking whether supplied parameter is a DeviceID or
IP/USB name

Parameter ``deviceIdOrName``:
    Either DeviceId, IP Address or USB port name)doc";

static const char *__doc_dai_DeviceInfo_DeviceInfo_4 = R"doc()doc";

static const char *__doc_dai_DeviceInfo_deviceId = R"doc()doc";

static const char *__doc_dai_DeviceInfo_getDeviceId = R"doc()doc";

static const char *__doc_dai_DeviceInfo_getMxId = R"doc()doc";

static const char *__doc_dai_DeviceInfo_getXLinkDeviceDesc = R"doc()doc";

static const char *__doc_dai_DeviceInfo_name = R"doc()doc";

static const char *__doc_dai_DeviceInfo_platform = R"doc()doc";

static const char *__doc_dai_DeviceInfo_protocol = R"doc()doc";

static const char *__doc_dai_DeviceInfo_state = R"doc()doc";

static const char *__doc_dai_DeviceInfo_status = R"doc()doc";

static const char *__doc_dai_DeviceInfo_toString = R"doc()doc";

static const char *__doc_dai_DeviceNode = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_DeviceNodeCRTP = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_DeviceNodeCRTP_2 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_DeviceNodeCRTP_3 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_DeviceNodeCRTP_4 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_DeviceNodeCRTP_5 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_create = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_create_2 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_create_3 = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_getName = R"doc()doc";

static const char *__doc_dai_DeviceNodeCRTP_properties = R"doc(Underlying properties)doc";

static const char *__doc_dai_DeviceNodeGroup = R"doc()doc";

static const char *__doc_dai_DeviceNodeGroupProperties =
R"doc(A dummy property struct for the DeviceNodeGroup node to comply with the
DeviceNode API)doc";

static const char *__doc_dai_DeviceNodeGroupProperties_dummy = R"doc()doc";

static const char *__doc_dai_DeviceNodeGroup_DeviceNodeGroup = R"doc()doc";

static const char *__doc_dai_DeviceNodeGroup_getLogLevel = R"doc()doc";

static const char *__doc_dai_DeviceNodeGroup_getName = R"doc()doc";

static const char *__doc_dai_DeviceNodeGroup_setLogLevel = R"doc()doc";

static const char *__doc_dai_DeviceNode_DeviceNode = R"doc()doc";

static const char *__doc_dai_DeviceNode_DeviceNode_2 = R"doc()doc";

static const char *__doc_dai_DeviceNode_DeviceNode_3 = R"doc()doc";

static const char *__doc_dai_DeviceNode_device = R"doc()doc";

static const char *__doc_dai_DeviceNode_getDevice =
R"doc(Get device for this node

Returns:
    shared pointer to device)doc";

static const char *__doc_dai_DeviceNode_getLogLevel = R"doc()doc";

static const char *__doc_dai_DeviceNode_getProperties = R"doc()doc";

static const char *__doc_dai_DeviceNode_propertiesHolder = R"doc()doc";

static const char *__doc_dai_DeviceNode_run = R"doc()doc";

static const char *__doc_dai_DeviceNode_runOnHost = R"doc()doc";

static const char *__doc_dai_DeviceNode_setDevice =
R"doc(Set device for this node

Parameter ``device:``:
    shared pointer to device)doc";

static const char *__doc_dai_DeviceNode_setLogLevel = R"doc()doc";

static const char *__doc_dai_Device_Device =
R"doc(Connects to any available device with a DEFAULT_SEARCH_TIME timeout. Uses
OpenVINO version OpenVINO::VERSION_UNIVERSAL)doc";

static const char *__doc_dai_Device_closeImpl = R"doc()doc";

static const char *__doc_dai_Device_getPlatform =
R"doc(Get the platform of the connected device

Returns:
    Platform Platform enum)doc";

static const char *__doc_dai_Device_getPlatformAsString =
R"doc(Get the platform of the connected device as string

Returns:
    std::string String representation of Platform)doc";

static const char *__doc_dai_EdgeDetectorConfig = R"doc(EdgeDetectorConfig message. Carries sobel edge filter config.)doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfig = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfigData = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfigData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfigData_sobelFilterHorizontalKernel =
R"doc(Used for horizontal gradient computation in 3x3 Sobel filter Format - 3x3
matrix, 2nd column must be 0 Default - +1 0 -1; +2 0 -2; +1 0 -1)doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfigData_sobelFilterVerticalKernel =
R"doc(Used for vertical gradient computation in 3x3 Sobel filter Format - 3x3 matrix,
2nd row must be 0 Default - +1 +2 +1; 0 0 0; -1 -2 -1)doc";

static const char *__doc_dai_EdgeDetectorConfig_EdgeDetectorConfigData_str = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_config = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_getConfigData =
R"doc(Retrieve configuration data for EdgeDetector

Returns:
    EdgeDetectorConfigData: sobel filter horizontal and vertical 3x3 kernels)doc";

static const char *__doc_dai_EdgeDetectorConfig_serialize = R"doc()doc";

static const char *__doc_dai_EdgeDetectorConfig_setSobelFilterKernels =
R"doc(Set sobel filter horizontal and vertical 3x3 kernels

Parameter ``horizontalKernel``:
    Used for horizontal gradient computation in 3x3 Sobel filter

Parameter ``verticalKernel``:
    Used for vertical gradient computation in 3x3 Sobel filter)doc";

static const char *__doc_dai_EdgeDetectorConfig_str = R"doc()doc";

static const char *__doc_dai_EdgeDetectorProperties = R"doc(Specify properties for EdgeDetector)doc";

static const char *__doc_dai_EdgeDetectorProperties_initialConfig = R"doc(Initial edge detector config)doc";

static const char *__doc_dai_EdgeDetectorProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_EdgeDetectorProperties_outputFrameSize = R"doc(Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes))doc";

static const char *__doc_dai_EepromData =
R"doc(EepromData structure

Contains the Calibration and Board data stored on device)doc";

static const char *__doc_dai_EepromData_batchName = R"doc()doc";

static const char *__doc_dai_EepromData_batchTime = R"doc(Deprecated, not used or stored)doc";

static const char *__doc_dai_EepromData_boardConf = R"doc()doc";

static const char *__doc_dai_EepromData_boardCustom = R"doc()doc";

static const char *__doc_dai_EepromData_boardName = R"doc()doc";

static const char *__doc_dai_EepromData_boardOptions = R"doc()doc";

static const char *__doc_dai_EepromData_boardRev = R"doc()doc";

static const char *__doc_dai_EepromData_cameraData = R"doc()doc";

static const char *__doc_dai_EepromData_deviceName = R"doc()doc";

static const char *__doc_dai_EepromData_hardwareConf = R"doc()doc";

static const char *__doc_dai_EepromData_housingExtrinsics = R"doc()doc";

static const char *__doc_dai_EepromData_imuExtrinsics = R"doc()doc";

static const char *__doc_dai_EepromData_miscellaneousData = R"doc()doc";

static const char *__doc_dai_EepromData_productName = R"doc()doc";

static const char *__doc_dai_EepromData_stereoEnableDistortionCorrection = R"doc()doc";

static const char *__doc_dai_EepromData_stereoRectificationData = R"doc()doc";

static const char *__doc_dai_EepromData_stereoUseSpecTranslation = R"doc()doc";

static const char *__doc_dai_EepromData_version = R"doc()doc";

static const char *__doc_dai_EepromData_verticalCameraSocket = R"doc()doc";

static const char *__doc_dai_EepromError = R"doc()doc";

static const char *__doc_dai_EncodedFrame = R"doc()doc";

static const char *__doc_dai_EncodedFrame_FrameType = R"doc()doc";

static const char *__doc_dai_EncodedFrame_FrameType_B = R"doc()doc";

static const char *__doc_dai_EncodedFrame_FrameType_I = R"doc()doc";

static const char *__doc_dai_EncodedFrame_FrameType_P = R"doc()doc";

static const char *__doc_dai_EncodedFrame_FrameType_Unknown = R"doc()doc";

static const char *__doc_dai_EncodedFrame_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_EncodedFrame_Profile = R"doc()doc";

static const char *__doc_dai_EncodedFrame_Profile_AVC = R"doc()doc";

static const char *__doc_dai_EncodedFrame_Profile_HEVC = R"doc()doc";

static const char *__doc_dai_EncodedFrame_Profile_JPEG = R"doc()doc";

static const char *__doc_dai_EncodedFrame_bitrate = R"doc()doc";

static const char *__doc_dai_EncodedFrame_cam = R"doc()doc";

static const char *__doc_dai_EncodedFrame_frameOffset = R"doc()doc";

static const char *__doc_dai_EncodedFrame_frameSize = R"doc()doc";

static const char *__doc_dai_EncodedFrame_getBitrate = R"doc(Retrieves the encoding bitrate)doc";

static const char *__doc_dai_EncodedFrame_getColorTemperature = R"doc(Retrieves white-balance color temperature of the light source, in kelvins)doc";

static const char *__doc_dai_EncodedFrame_getExposureTime = R"doc(Retrieves exposure time)doc";

static const char *__doc_dai_EncodedFrame_getFrameType = R"doc(Retrieves frame type (H26x only))doc";

static const char *__doc_dai_EncodedFrame_getHeight = R"doc(Retrieves image height in pixels)doc";

static const char *__doc_dai_EncodedFrame_getImgFrameMeta = R"doc()doc";

static const char *__doc_dai_EncodedFrame_getInstanceNum = R"doc(Retrieves instance number)doc";

static const char *__doc_dai_EncodedFrame_getLensPosition = R"doc(Retrieves lens position, range 0..255. Returns -1 if not available)doc";

static const char *__doc_dai_EncodedFrame_getLensPositionRaw = R"doc(Retrieves lens position, range 0.0f..1.0f. Returns -1 if not available)doc";

static const char *__doc_dai_EncodedFrame_getLossless = R"doc(Returns true if encoding is lossless (JPEG only))doc";

static const char *__doc_dai_EncodedFrame_getProfile = R"doc(Retrieves the encoding profile (JPEG, AVC or HEVC))doc";

static const char *__doc_dai_EncodedFrame_getQuality = R"doc(Retrieves the encoding quality)doc";

static const char *__doc_dai_EncodedFrame_getSensitivity = R"doc(Retrieves sensitivity, as an ISO value)doc";

static const char *__doc_dai_EncodedFrame_getWidth = R"doc(Retrieves image width in pixels)doc";

static const char *__doc_dai_EncodedFrame_height = R"doc()doc";

static const char *__doc_dai_EncodedFrame_instanceNum = R"doc()doc";

static const char *__doc_dai_EncodedFrame_lossless = R"doc()doc";

static const char *__doc_dai_EncodedFrame_profile = R"doc()doc";

static const char *__doc_dai_EncodedFrame_quality = R"doc()doc";

static const char *__doc_dai_EncodedFrame_serialize = R"doc()doc";

static const char *__doc_dai_EncodedFrame_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_EncodedFrame_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_EncodedFrame_setBitrate =
R"doc(Specifies the encoding quality

Parameter ``quality``:
    Encoding quality)doc";

static const char *__doc_dai_EncodedFrame_setFrameType =
R"doc(Specifies the frame type (H26x only)

Parameter ``type``:
    Type of h26x frame (I, P, B))doc";

static const char *__doc_dai_EncodedFrame_setHeight =
R"doc(Specifies frame height

Parameter ``height``:
    frame height)doc";

static const char *__doc_dai_EncodedFrame_setInstanceNum =
R"doc(Instance number relates to the origin of the frame (which camera)

Parameter ``instance``:
    Instance number)doc";

static const char *__doc_dai_EncodedFrame_setLossless =
R"doc(Specifies if encoding is lossless (JPEG only)

Parameter ``lossless``:
    True if lossless)doc";

static const char *__doc_dai_EncodedFrame_setProfile =
R"doc(Specifies the encoding profile

Parameter ``profile``:
    Encoding profile)doc";

static const char *__doc_dai_EncodedFrame_setQuality =
R"doc(Specifies the encoding quality

Parameter ``quality``:
    Encoding quality)doc";

static const char *__doc_dai_EncodedFrame_setSize =
R"doc(Specifies frame size

Parameter ``height``:
    frame height

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_EncodedFrame_setSize_2 =
R"doc(Specifies frame size

Parameter ``size``:
    frame size)doc";

static const char *__doc_dai_EncodedFrame_setWidth =
R"doc(Specifies frame width

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_EncodedFrame_str = R"doc()doc";

static const char *__doc_dai_EncodedFrame_transformation = R"doc()doc";

static const char *__doc_dai_EncodedFrame_type = R"doc()doc";

static const char *__doc_dai_EncodedFrame_width = R"doc()doc";

static const char *__doc_dai_Extrinsics = R"doc(Extrinsics structure)doc";

static const char *__doc_dai_Extrinsics_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Extrinsics_rotationMatrix = R"doc()doc";

static const char *__doc_dai_Extrinsics_specTranslation =
R"doc((x, y, z) pose of destCameraSocket w.r.t currentCameraSocket measured through
CAD design)doc";

static const char *__doc_dai_Extrinsics_str = R"doc()doc";

static const char *__doc_dai_Extrinsics_toCameraSocket = R"doc()doc";

static const char *__doc_dai_Extrinsics_translation =
R"doc((x, y, z) pose of destCameraSocket w.r.t currentCameraSocket obtained through
calibration)doc";

static const char *__doc_dai_FeatureTrackerConfig = R"doc(FeatureTrackerConfig message. Carries config for feature tracking algorithm)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector = R"doc(Corner detector configuration structure.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds = R"doc(Threshold settings structure for corner detector.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_decreaseFactor =
R"doc(When detected number of features exceeds the maximum in a cell threshold is
lowered by multiplying its value with this factor.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_increaseFactor =
R"doc(When detected number of features doesn't exceed the maximum in a cell, threshold
is increased by multiplying its value with this factor.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_initialValue =
R"doc(Minimum strength of a feature which will be detected. 0 means automatic
threshold update. Recommended so the tracker can adapt to different
scenes/textures. Each cell has its own threshold. Empirical value.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_max =
R"doc(Maximum limit for threshold. Applicable when automatic threshold update is
enabled. 0 means auto. Empirical value.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_min =
R"doc(Minimum limit for threshold. Applicable when automatic threshold update is
enabled. 0 means auto, 6000000 for HARRIS, 1200 for SHI_THOMASI. Empirical
value.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Thresholds_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Type = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Type_HARRIS = R"doc(Harris corner detector.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_Type_SHI_THOMASI = R"doc(Shi-Thomasi corner detector.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_cellGridDimension =
R"doc(Ensures distributed feature detection across the image. Image is divided into
horizontal and vertical cells, each cell has a target feature count =
numTargetFeatures / cellGridDimension. Each cell has its own feature threshold.
A value of 4 means that the image is divided into 4x4 cells of equal
width/height. Maximum 4, minimum 1.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_enableSobel =
R"doc(Enable 3x3 Sobel operator to smoothen the image whose gradient is to be
computed. If disabled, a simple 1D row/column differentiator is used for
gradient.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_enableSorting = R"doc(Enable sorting detected features based on their score or not.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_numMaxFeatures =
R"doc(Hard limit for the maximum number of features that can be detected. 0 means
auto, will be set to the maximum value based on memory constraints.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_numTargetFeatures =
R"doc(Target number of features to detect. Maximum number of features is determined at
runtime based on algorithm type.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_thresholds =
R"doc(Threshold settings. These are advanced settings, suitable for debugging/special
cases.)doc";

static const char *__doc_dai_FeatureTrackerConfig_CornerDetector_type = R"doc(Corner detector algorithm type.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer = R"doc(FeatureMaintainer configuration structure.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_enable = R"doc(Enable feature maintaining or not.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_lostFeatureErrorThreshold =
R"doc(Optical flow measures the tracking error for every feature. If the point can’t
be tracked or it’s out of the image it will set this error to a maximum value.
This threshold defines the level where the tracking accuracy is considered too
bad to keep the point.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_minimumDistanceBetweenFeatures =
R"doc(Used to filter out detected feature points that are too close. Requires sorting
enabled in detector. Unit of measurement is squared euclidean distance in
pixels.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureMaintainer_trackedFeatureThreshold =
R"doc(Once a feature was detected and we started tracking it, we need to update its
Harris score on each image. This is needed because a feature point can
disappear, or it can become too weak to be tracked. This threshold defines the
point where such a feature must be dropped. As the goal of the algorithm is to
provide longer tracks, we try to add strong points and track them until they are
absolutely untrackable. This is why, this value is usually smaller than the
detection threshold.)doc";

static const char *__doc_dai_FeatureTrackerConfig_FeatureTrackerConfig = R"doc(Construct FeatureTrackerConfig message.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator = R"doc(Used for feature reidentification between current and previous features.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow = R"doc(Optical flow configuration structure.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_epsilon =
R"doc(Feature tracking termination criteria. Optical flow will refine the feature
position on each pyramid level until the displacement between two refinements is
smaller than this value. Decreasing this number increases runtime.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_maxIterations =
R"doc(Feature tracking termination criteria. Optical flow will refine the feature
position maximum this many times on each pyramid level. If the Epsilon criteria
described in the previous chapter is not met after this number of iterations,
the algorithm will continue with the current calculated value. Increasing this
number increases runtime.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_pyramidLevels =
R"doc(Number of pyramid levels, only for optical flow. AUTO means it's decided based
on input resolution: 3 if image width <= 640, else 4. Valid values are either
3/4 for VGA, 4 for 720p and above.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_searchWindowHeight =
R"doc(Image patch height used to track features. Must be an odd number, maximum 9. N
means the algorithm will be able to track motion at most (N-1)/2 pixels in a
direction per pyramid level. Increasing this number increases runtime)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_searchWindowWidth =
R"doc(Image patch width used to track features. Must be an odd number, maximum 9. N
means the algorithm will be able to track motion at most (N-1)/2 pixels in a
direction per pyramid level. Increasing this number increases runtime)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_OpticalFlow_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_Type = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_Type_HW_MOTION_ESTIMATION = R"doc(Using a dense motion estimation hardware block (Block matcher).)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_Type_LUCAS_KANADE_OPTICAL_FLOW = R"doc(Using the pyramidal Lucas-Kanade optical flow method.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_enable = R"doc(Enable motion estimation or not.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_opticalFlow =
R"doc(Optical flow configuration. Takes effect only if MotionEstimator algorithm type
set to LUCAS_KANADE_OPTICAL_FLOW.)doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_MotionEstimator_type = R"doc(Motion estimator algorithm type.)doc";

static const char *__doc_dai_FeatureTrackerConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_cornerDetector = R"doc(Corner detector configuration. Used for feature detection.)doc";

static const char *__doc_dai_FeatureTrackerConfig_featureMaintainer = R"doc(FeatureMaintainer configuration. Used for feature maintaining.)doc";

static const char *__doc_dai_FeatureTrackerConfig_motionEstimator =
R"doc(Motion estimator configuration. Used for feature reidentification between
current and previous features.)doc";

static const char *__doc_dai_FeatureTrackerConfig_serialize = R"doc()doc";

static const char *__doc_dai_FeatureTrackerConfig_setCornerDetector =
R"doc(Set corner detector algorithm type.

Parameter ``cornerDetector``:
    Corner detector type, HARRIS or SHI_THOMASI)doc";

static const char *__doc_dai_FeatureTrackerConfig_setCornerDetector_2 =
R"doc(Set corner detector full configuration.

Parameter ``config``:
    Corner detector configuration)doc";

static const char *__doc_dai_FeatureTrackerConfig_setFeatureMaintainer =
R"doc(Enable or disable feature maintainer.

Parameter ``enable``:)doc";

static const char *__doc_dai_FeatureTrackerConfig_setFeatureMaintainer_2 =
R"doc(Set feature maintainer full configuration.

Parameter ``config``:
    feature maintainer configuration)doc";

static const char *__doc_dai_FeatureTrackerConfig_setHwMotionEstimation =
R"doc(Set hardware accelerated motion estimation using block matching. Faster than
optical flow (software implementation) but might not be as accurate.)doc";

static const char *__doc_dai_FeatureTrackerConfig_setMotionEstimator =
R"doc(Enable or disable motion estimator.

Parameter ``enable``:)doc";

static const char *__doc_dai_FeatureTrackerConfig_setMotionEstimator_2 =
R"doc(Set motion estimator full configuration.

Parameter ``config``:
    Motion estimator configuration)doc";

static const char *__doc_dai_FeatureTrackerConfig_setNumTargetFeatures =
R"doc(Set number of target features to detect.

Parameter ``numTargetFeatures``:
    Number of features)doc";

static const char *__doc_dai_FeatureTrackerConfig_setOpticalFlow = R"doc(Set optical flow as motion estimation algorithm type.)doc";

static const char *__doc_dai_FeatureTrackerConfig_setOpticalFlow_2 =
R"doc(Set optical flow full configuration.

Parameter ``config``:
    Optical flow configuration)doc";

static const char *__doc_dai_FeatureTrackerConfig_str = R"doc()doc";

static const char *__doc_dai_FeatureTrackerProperties = R"doc(Specify properties for FeatureTracker)doc";

static const char *__doc_dai_FeatureTrackerProperties_initialConfig = R"doc(Initial feature tracker config)doc";

static const char *__doc_dai_FeatureTrackerProperties_numMemorySlices =
R"doc(Number of memory slices reserved for feature tracking. Optical flow can use 1 or
2 memory slices, while for corner detection only 1 is enough. Maximum number of
features depends on the number of allocated memory slices. Hardware motion
estimation doesn't require memory slices. Maximum 2, minimum 1.)doc";

static const char *__doc_dai_FeatureTrackerProperties_numShaves =
R"doc(Number of shaves reserved for feature tracking. Optical flow can use 1 or 2
shaves, while for corner detection only 1 is enough. Hardware motion estimation
doesn't require shaves. Maximum 2, minimum 1.)doc";

static const char *__doc_dai_Flip = R"doc()doc";

static const char *__doc_dai_Flip_Direction = R"doc()doc";

static const char *__doc_dai_Flip_Direction_HORIZONTAL = R"doc()doc";

static const char *__doc_dai_Flip_Direction_VERTICAL = R"doc()doc";

static const char *__doc_dai_Flip_Flip = R"doc()doc";

static const char *__doc_dai_Flip_Flip_2 = R"doc()doc";

static const char *__doc_dai_Flip_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Flip_center = R"doc()doc";

static const char *__doc_dai_Flip_direction = R"doc()doc";

static const char *__doc_dai_Flip_str = R"doc()doc";

static const char *__doc_dai_Flip_toStr = R"doc()doc";

static const char *__doc_dai_FourPoints = R"doc()doc";

static const char *__doc_dai_FourPoints_FourPoints = R"doc()doc";

static const char *__doc_dai_FourPoints_FourPoints_2 = R"doc()doc";

static const char *__doc_dai_FourPoints_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_FourPoints_dst = R"doc()doc";

static const char *__doc_dai_FourPoints_normalized = R"doc()doc";

static const char *__doc_dai_FourPoints_src = R"doc()doc";

static const char *__doc_dai_FourPoints_str = R"doc()doc";

static const char *__doc_dai_FourPoints_toStr = R"doc()doc";

static const char *__doc_dai_FrameEvent = R"doc()doc";

static const char *__doc_dai_FrameEvent_NONE = R"doc()doc";

static const char *__doc_dai_FrameEvent_READOUT_END = R"doc()doc";

static const char *__doc_dai_FrameEvent_READOUT_START = R"doc()doc";

static const char *__doc_dai_GlobalProperties = R"doc(Specify properties which apply for whole pipeline)doc";

static const char *__doc_dai_GlobalProperties_calibData = R"doc(Calibration data sent through pipeline)doc";

static const char *__doc_dai_GlobalProperties_cameraTuningBlobSize = R"doc(Camera tuning blob size in bytes)doc";

static const char *__doc_dai_GlobalProperties_cameraTuningBlobUri = R"doc(Uri which points to camera tuning blob)doc";

static const char *__doc_dai_GlobalProperties_leonCssFrequencyHz =
R"doc(Set frequency of Leon OS - Increasing can improve performance, at the cost of
higher power draw)doc";

static const char *__doc_dai_GlobalProperties_leonMssFrequencyHz =
R"doc(Set frequency of Leon RT - Increasing can improve performance, at the cost of
higher power draw)doc";

static const char *__doc_dai_GlobalProperties_pipelineName = R"doc()doc";

static const char *__doc_dai_GlobalProperties_pipelineVersion = R"doc()doc";

static const char *__doc_dai_GlobalProperties_sippBufferSize =
R"doc(SIPP (Signal Image Processing Pipeline) internal memory pool. SIPP is a
framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
Changing the size of this pool is meant for advanced use cases, pushing the
limits of the HW. By default memory is allocated in high speed CMX memory.
Setting to 0 will allocate in DDR 256 kilobytes. Units are bytes.)doc";

static const char *__doc_dai_GlobalProperties_sippDmaBufferSize =
R"doc(SIPP (Signal Image Processing Pipeline) internal DMA memory pool. SIPP is a
framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
Changing the size of this pool is meant for advanced use cases, pushing the
limits of the HW. Memory is allocated in high speed CMX memory Units are bytes.)doc";

static const char *__doc_dai_GlobalProperties_xlinkChunkSize =
R"doc(Chunk size for splitting device-sent XLink packets, in bytes. A larger value
could increase performance, with 0 disabling chunking. A negative value won't
modify the device defaults - configured per protocol, currently 64*1024 for both
USB and Ethernet.)doc";

static const char *__doc_dai_HostRunnable = R"doc()doc";

static const char *__doc_dai_IMUData = R"doc(IMUData message. Carries normalized detection results)doc";

static const char *__doc_dai_IMUData_IMUData = R"doc()doc";

static const char *__doc_dai_IMUData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_IMUData_packets = R"doc(Detections)doc";

static const char *__doc_dai_IMUData_serialize = R"doc()doc";

static const char *__doc_dai_IMUData_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_IMUData_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_IMUData_str = R"doc()doc";

static const char *__doc_dai_IMUPacket =
R"doc(IMU output

Contains combined output for all possible modes. Only the enabled outputs are
populated.)doc";

static const char *__doc_dai_IMUPacket_acceleroMeter = R"doc()doc";

static const char *__doc_dai_IMUPacket_gyroscope = R"doc()doc";

static const char *__doc_dai_IMUPacket_magneticField = R"doc()doc";

static const char *__doc_dai_IMUPacket_rotationVector = R"doc()doc";

static const char *__doc_dai_IMUProperties = R"doc()doc";

static const char *__doc_dai_IMUProperties_batchReportThreshold = R"doc()doc";

static const char *__doc_dai_IMUProperties_enableFirmwareUpdate = R"doc()doc";

static const char *__doc_dai_IMUProperties_imuSensors = R"doc()doc";

static const char *__doc_dai_IMUProperties_maxBatchReports = R"doc()doc";

static const char *__doc_dai_IMUReport = R"doc()doc";

static const char *__doc_dai_IMUReportAccelerometer =
R"doc(Accelerometer

Units are [m/s^2])doc";

static const char *__doc_dai_IMUReportAccelerometer_x = R"doc()doc";

static const char *__doc_dai_IMUReportAccelerometer_y = R"doc()doc";

static const char *__doc_dai_IMUReportAccelerometer_z = R"doc()doc";

static const char *__doc_dai_IMUReportGyroscope =
R"doc(Gyroscope

Units are [rad/s])doc";

static const char *__doc_dai_IMUReportGyroscope_x = R"doc()doc";

static const char *__doc_dai_IMUReportGyroscope_y = R"doc()doc";

static const char *__doc_dai_IMUReportGyroscope_z = R"doc()doc";

static const char *__doc_dai_IMUReportMagneticField =
R"doc(Magnetic field

Units are [uTesla])doc";

static const char *__doc_dai_IMUReportMagneticField_x = R"doc()doc";

static const char *__doc_dai_IMUReportMagneticField_y = R"doc()doc";

static const char *__doc_dai_IMUReportMagneticField_z = R"doc()doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc =
R"doc(Rotation Vector with Accuracy

Contains quaternion components: i,j,k,real)doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc_i = R"doc(Quaternion component i)doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc_j = R"doc(Quaternion component j)doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc_k = R"doc(Quaternion component k)doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc_real = R"doc(Quaternion component, real)doc";

static const char *__doc_dai_IMUReportRotationVectorWAcc_rotationVectorAccuracy = R"doc(Accuracy estimate [radians], 0 means no estimate)doc";

static const char *__doc_dai_IMUReport_Accuracy = R"doc()doc";

static const char *__doc_dai_IMUReport_Accuracy_HIGH = R"doc()doc";

static const char *__doc_dai_IMUReport_Accuracy_LOW = R"doc()doc";

static const char *__doc_dai_IMUReport_Accuracy_MEDIUM = R"doc()doc";

static const char *__doc_dai_IMUReport_Accuracy_UNRELIABLE = R"doc()doc";

static const char *__doc_dai_IMUReport_accuracy = R"doc(Accuracy of sensor)doc";

static const char *__doc_dai_IMUReport_getSequenceNum = R"doc(Retrieves IMU report sequence number)doc";

static const char *__doc_dai_IMUReport_getTimestamp = R"doc(Retrieves timestamp related to dai::Clock::now())doc";

static const char *__doc_dai_IMUReport_getTimestampDevice =
R"doc(Retrieves timestamp directly captured from device's monotonic clock, not
synchronized to host time. Used mostly for debugging)doc";

static const char *__doc_dai_IMUReport_sequence =
R"doc(The sequence number increments once for each report sent. Gaps in the sequence
numbers indicate missing or dropped reports. Max value 2^32 after which resets
to 0.)doc";

static const char *__doc_dai_IMUReport_timestamp = R"doc(Generation timestamp, synced to host time)doc";

static const char *__doc_dai_IMUReport_tsDevice = R"doc(Generation timestamp, direct device monotonic clock)doc";

static const char *__doc_dai_IMUSensor =
R"doc(Available IMU sensors. More details about each sensor can be found in the
datasheet:

https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)doc";

static const char *__doc_dai_IMUSensorConfig = R"doc()doc";

static const char *__doc_dai_IMUSensorConfig_changeSensitivity = R"doc(Report-on-change threshold)doc";

static const char *__doc_dai_IMUSensorConfig_reportRate = R"doc()doc";

static const char *__doc_dai_IMUSensorConfig_sensitivityEnabled = R"doc()doc";

static const char *__doc_dai_IMUSensorConfig_sensitivityRelative = R"doc(Change reports relative (vs absolute))doc";

static const char *__doc_dai_IMUSensorConfig_sensorId = R"doc()doc";

static const char *__doc_dai_IMUSensor_ACCELEROMETER =
R"doc(Section 2.1.1

Acceleration of the device including gravity. Units are [m/s^2])doc";

static const char *__doc_dai_IMUSensor_ACCELEROMETER_RAW =
R"doc(Section 2.1.1

Acceleration of the device without any postprocessing, straight from the sensor.
Units are [m/s^2])doc";

static const char *__doc_dai_IMUSensor_ARVR_STABILIZED_GAME_ROTATION_VECTOR =
R"doc(Section 2.2

While the magnetometer is removed from the calculation of the game rotation
vector, the accelerometer itself can create a potential correction in the
rotation vector produced (i.e. the estimate of gravity changes). For
applications (typically augmented or virtual reality applications) where a
sudden jump can be disturbing, the output is adjusted to prevent these jumps in
a manner that takes account of the velocity of the sensor system. This process
is called AR/VR stabilization.)doc";

static const char *__doc_dai_IMUSensor_ARVR_STABILIZED_ROTATION_VECTOR =
R"doc(Section 2.2

Estimates of the magnetic field and the roll/pitch of the device can create a
potential correction in the rotation vector produced. For applications
(typically augmented or virtual reality applications) where a sudden jump can be
disturbing, the output is adjusted to prevent these jumps in a manner that takes
account of the velocity of the sensor system.)doc";

static const char *__doc_dai_IMUSensor_GAME_ROTATION_VECTOR =
R"doc(Section 2.2

The game rotation vector is an orientation output that is expressed as a
quaternion with no specific reference for heading, while roll and pitch are
referenced against gravity. It is produced by fusing the outputs of the
accelerometer and the gyroscope (i.e. no magnetometer). The game rotation vector
does not use the magnetometer to correct the gyroscopes drift in yaw. This is a
deliberate omission (as specified by Google) to allow gaming applications to use
a smoother representation of the orientation without the jumps that an
instantaneous correction provided by a magnetic field update could provide. Long
term the output will likely drift in yaw due to the characteristics of
gyroscopes, but this is seen as preferable for this output versus a corrected
output.)doc";

static const char *__doc_dai_IMUSensor_GEOMAGNETIC_ROTATION_VECTOR =
R"doc(Section 2.2

The geomagnetic rotation vector is an orientation output that is expressed as a
quaternion referenced to magnetic north and gravity. It is produced by fusing
the outputs of the accelerometer and magnetometer. The gyroscope is specifically
excluded in order to produce a rotation vector output using less power than is
required to produce the rotation vector of section 2.2.4. The consequences of
removing the gyroscope are: Less responsive output since the highly dynamic
outputs of the gyroscope are not used More errors in the presence of varying
magnetic fields.)doc";

static const char *__doc_dai_IMUSensor_GRAVITY =
R"doc(Section 2.1.1

Gravity. Units are [m/s^2])doc";

static const char *__doc_dai_IMUSensor_GYROSCOPE_CALIBRATED =
R"doc(Section 2.1.2

The angular velocity of the device. Units are [rad/s])doc";

static const char *__doc_dai_IMUSensor_GYROSCOPE_RAW =
R"doc(Section 2.1.2

The angular velocity of the device without any postprocessing, straight from the
sensor. Units are [rad/s])doc";

static const char *__doc_dai_IMUSensor_GYROSCOPE_UNCALIBRATED =
R"doc(Section 2.1.2

Angular velocity without bias compensation. Units are [rad/s])doc";

static const char *__doc_dai_IMUSensor_LINEAR_ACCELERATION =
R"doc(Section 2.1.1

Acceleration of the device with gravity removed. Units are [m/s^2])doc";

static const char *__doc_dai_IMUSensor_MAGNETOMETER_CALIBRATED =
R"doc(Section 2.1.3

The fully calibrated magnetic field measurement. Units are [uTesla])doc";

static const char *__doc_dai_IMUSensor_MAGNETOMETER_RAW =
R"doc(Section 2.1.3

Magnetic field measurement without any postprocessing, straight from the sensor.
Units are [uTesla])doc";

static const char *__doc_dai_IMUSensor_MAGNETOMETER_UNCALIBRATED =
R"doc(Section 2.1.3

The magnetic field measurement without hard-iron offset applied. Units are
[uTesla])doc";

static const char *__doc_dai_IMUSensor_ROTATION_VECTOR =
R"doc(Section 2.2

The rotation vector provides an orientation output that is expressed as a
quaternion referenced to magnetic north and gravity. It is produced by fusing
the outputs of the accelerometer, gyroscope and magnetometer. The rotation
vector is the most accurate orientation estimate available. The magnetometer
provides correction in yaw to reduce drift and the gyroscope enables the most
responsive performance.)doc";

static const char *__doc_dai_ImageAlignConfig = R"doc(ImageAlignConfig configuration structure)doc";

static const char *__doc_dai_ImageAlignConfig_ImageAlignConfig = R"doc()doc";

static const char *__doc_dai_ImageAlignConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImageAlignConfig_serialize = R"doc()doc";

static const char *__doc_dai_ImageAlignConfig_staticDepthPlane = R"doc(Optional static depth plane to align to, in depth units, by default millimeters)doc";

static const char *__doc_dai_ImageAlignConfig_str = R"doc()doc";

static const char *__doc_dai_ImageAlignProperties = R"doc(Specify properties for ImageAlign)doc";

static const char *__doc_dai_ImageAlignProperties_alignHeight = R"doc(Optional output height)doc";

static const char *__doc_dai_ImageAlignProperties_alignWidth = R"doc(Optional output width)doc";

static const char *__doc_dai_ImageAlignProperties_initialConfig = R"doc()doc";

static const char *__doc_dai_ImageAlignProperties_interpolation = R"doc(Interpolation type to use)doc";

static const char *__doc_dai_ImageAlignProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_ImageAlignProperties_numShaves = R"doc(Number of shaves reserved.)doc";

static const char *__doc_dai_ImageAlignProperties_outKeepAspectRatio = R"doc(Whether to keep aspect ratio of the input or not)doc";

static const char *__doc_dai_ImageAlignProperties_warpHwIds = R"doc(Warp HW IDs to use, if empty, use auto/default)doc";

static const char *__doc_dai_ImageManipConfig =
R"doc(ImageManipConfig message. Specifies image manipulation options like:

- Crop

- Resize

- Warp

- ...)doc";

static const char *__doc_dai_ImageManipConfig_ImageManipConfig = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_addCrop =
R"doc(Crops the image to the specified rectangle

Parameter ``x``:
    X coordinate of the top-left corner

Parameter ``y``:
    Y coordinate of the top-left corner

Parameter ``w``:
    Width of the rectangle

Parameter ``h``:
    Height of the rectangle)doc";

static const char *__doc_dai_ImageManipConfig_addCrop_2 =
R"doc(Crops the image to the specified rectangle

Parameter ``rect``:
    Rect to crop

Parameter ``normalizedCoords``:
    If true, the coordinates are normalized to range [0, 1] where 1 maps to the
    width/height of the image)doc";

static const char *__doc_dai_ImageManipConfig_addCropRotatedRect =
R"doc(Crops the image to the specified (rotated) rectangle

Parameter ``rect``:
    RotatedRect to crop

Parameter ``normalizedCoords``:
    If true, the coordinates are normalized to range [0, 1] where 1 maps to the
    width/height of the image)doc";

static const char *__doc_dai_ImageManipConfig_addFlipHorizontal = R"doc(Flips the image horizontally)doc";

static const char *__doc_dai_ImageManipConfig_addFlipVertical = R"doc(Flips the image vertically)doc";

static const char *__doc_dai_ImageManipConfig_addRotateDeg =
R"doc(Rotates the image around its center by the specified angle in degrees

Parameter ``angle``:
    Angle in radians)doc";

static const char *__doc_dai_ImageManipConfig_addRotateDeg_2 =
R"doc(Rotates the image around the specified point by the specified angle in degrees

Parameter ``angle``:
    Angle in radians

Parameter ``center``:
    Center of the rotation using normalized coordinates)doc";

static const char *__doc_dai_ImageManipConfig_addScale =
R"doc(Rescales the image using the specified factors

Parameter ``scaleX``:
    Scale factor for the X axis

Parameter ``scaleY``:
    Scale factor for the Y axis. If not specified, scaleY is set to the same
    value as scaleX)doc";

static const char *__doc_dai_ImageManipConfig_addTransformAffine =
R"doc(Applies an affine transformation to the image

Parameter ``matrix``:
    an array containing a 2x2 matrix representing the affine transformation)doc";

static const char *__doc_dai_ImageManipConfig_addTransformFourPoints =
R"doc(Applies a perspective transformation to the image

Parameter ``src``:
    Source points

Parameter ``dst``:
    Destination points

Parameter ``normalizedCoords``:
    If true, the coordinates are normalized to range [0, 1] where 1 maps to the
    width/height of the image)doc";

static const char *__doc_dai_ImageManipConfig_addTransformPerspective =
R"doc(Applies a perspective transformation to the image

Parameter ``matrix``:
    an array containing a 3x3 matrix representing the perspective transformation)doc";

static const char *__doc_dai_ImageManipConfig_base = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_clearOps = R"doc(Removes all operations from the list (does not affect output configuration))doc";

static const char *__doc_dai_ImageManipConfig_getReusePreviousImage =
R"doc(Instruct ImageManip to not remove current image from its queue and use the same
for next message.

Returns:
    True to enable reuse, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_getSkipCurrentImage =
R"doc(Instructs ImageManip to skip current image and wait for next in queue.

Returns:
    True to skip current image, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_getUndistort =
R"doc(Gets the undistort flag

Returns:
    True if undistort is enabled, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_outputFrameType = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_reusePreviousImage = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_serialize = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_setBackgroundColor =
R"doc(Sets the rgb background color of the output image

Parameter ``red``:
    Red component of the background color

Parameter ``green``:
    Green component of the background color

Parameter ``blue``:
    Blue component of the background color)doc";

static const char *__doc_dai_ImageManipConfig_setBackgroundColor_2 =
R"doc(Sets the grayscale background color of the output image

Parameter ``val``:
    Grayscale value of the background color)doc";

static const char *__doc_dai_ImageManipConfig_setColormap =
R"doc(Sets the colormap to be applied to a grayscale image

Parameter ``colormap``:
    Colormap type to be applied)doc";

static const char *__doc_dai_ImageManipConfig_setFrameType =
R"doc(Sets the frame type of the output image

Parameter ``frameType``:
    Frame type of the output image)doc";

static const char *__doc_dai_ImageManipConfig_setOutputCenter =
R"doc(Centers the content in the output image without resizing

Parameter ``c``:
    True to center the content, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_setOutputSize =
R"doc(Sets the output size of the image

Parameter ``w``:
    Width of the output image

Parameter ``h``:
    Height of the output image

Parameter ``mode``:
    Resize mode. NONE - no resize, STRETCH - stretch to fit, LETTERBOX - keep
    aspect ratio and pad with background color, CENTER_CROP - keep aspect ratio
    and crop)doc";

static const char *__doc_dai_ImageManipConfig_setReusePreviousImage =
R"doc(Instruct ImageManip to not remove current image from its queue and use the same
for next message.

Parameter ``reuse``:
    True to enable reuse, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_setSkipCurrentImage =
R"doc(Instructs ImageManip to skip current image and wait for next in queue.

Parameter ``skip``:
    True to skip current image, false otherwise)doc";

static const char *__doc_dai_ImageManipConfig_setUndistort = R"doc(Sets the undistort flag)doc";

static const char *__doc_dai_ImageManipConfig_skipCurrentImage = R"doc()doc";

static const char *__doc_dai_ImageManipConfig_str = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_addOp = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_background = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_backgroundB = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_backgroundG = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_backgroundR = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_center = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_clear = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_cloneTo = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_colormap = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_crop = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_flipHorizontal = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_flipVertical = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_getOperations = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_getUndistort = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_hasWarp = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_operations = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_outputHeight = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_outputWidth = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resize = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resizeFill = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resizeFit = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resizeHeightKeepAspectRatio = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resizeMode = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_resizeWidthKeepAspectRatio = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_rotateDegrees = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_rotateRadians = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setBackgroundColor = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setBackgroundColor_2 = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setColormap = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setOutputCenter = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setOutputResize = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setOutputSize = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_setUndistort = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_str = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_transformAffine = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_transformFourPoints = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_transformPerspective = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_translate = R"doc()doc";

static const char *__doc_dai_ImageManipOpsBase_undistort = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_Background = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_Background_COLOR = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_ResizeMode = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_ResizeMode_CENTER_CROP = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_ResizeMode_LETTERBOX = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_ResizeMode_NONE = R"doc()doc";

static const char *__doc_dai_ImageManipOpsEnums_ResizeMode_STRETCH = R"doc()doc";

static const char *__doc_dai_ImageManipProperties = R"doc(Specify properties for ImageManip)doc";

static const char *__doc_dai_ImageManipProperties_Backend =
R"doc(Enable hardware accelerated image manipulation if set to HW. Only applied on
RVC4. This can cause some unexpected behavior when using multiple ImageManip
nodes in series. Currently, the only operation affected is downscaling.)doc";

static const char *__doc_dai_ImageManipProperties_Backend_CPU = R"doc()doc";

static const char *__doc_dai_ImageManipProperties_Backend_HW = R"doc()doc";

static const char *__doc_dai_ImageManipProperties_PerformanceMode =
R"doc(Set performance mode for ImageManip with a tradeoff between performance and
power consumption. Only applied on RVC4. This only affects scaling NV12 and GRAY
images. - PERFORMANCE: High performance, high power consumption. Uses the OpenCV
backend. - BALANCED: Balanced performance and power consumption. Uses the FastCV
backend configured for high performance where possible with a fallback to
OpenCV. - LOW_POWER: Low performance, low power consumption. Uses the FastCV
backend configured for low power where possible with a fallback to OpenCV.)doc";

static const char *__doc_dai_ImageManipProperties_PerformanceMode_BALANCED = R"doc()doc";

static const char *__doc_dai_ImageManipProperties_PerformanceMode_LOW_POWER = R"doc()doc";

static const char *__doc_dai_ImageManipProperties_PerformanceMode_PERFORMANCE = R"doc()doc";

static const char *__doc_dai_ImageManipProperties_backend =
R"doc(Using HW backend can cause some unexpected behavior when using multiple
ImageManip nodes in series)doc";

static const char *__doc_dai_ImageManipProperties_initialConfig = R"doc(Initial configuration for ImageManip node)doc";

static const char *__doc_dai_ImageManipProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_ImageManipProperties_outputFrameSize = R"doc(Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes))doc";

static const char *__doc_dai_ImageManipProperties_performanceMode = R"doc()doc";

static const char *__doc_dai_ImgAnnotation = R"doc()doc";

static const char *__doc_dai_ImgAnnotation_circles = R"doc()doc";

static const char *__doc_dai_ImgAnnotation_points = R"doc()doc";

static const char *__doc_dai_ImgAnnotation_texts = R"doc()doc";

static const char *__doc_dai_ImgAnnotations = R"doc()doc";

static const char *__doc_dai_ImgAnnotations_2 = R"doc(ImgAnnotations message. Carries annotations for an image.)doc";

static const char *__doc_dai_ImgAnnotations_ImgAnnotations = R"doc(Construct ImgAnnotations message.)doc";

static const char *__doc_dai_ImgAnnotations_ImgAnnotations_2 = R"doc()doc";

static const char *__doc_dai_ImgAnnotations_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgAnnotations_annotations = R"doc(Transform)doc";

static const char *__doc_dai_ImgAnnotations_serialize = R"doc()doc";

static const char *__doc_dai_ImgAnnotations_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_ImgAnnotations_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_ImgAnnotations_str = R"doc()doc";

static const char *__doc_dai_ImgDetection = R"doc()doc";

static const char *__doc_dai_ImgDetection_confidence = R"doc()doc";

static const char *__doc_dai_ImgDetection_label = R"doc()doc";

static const char *__doc_dai_ImgDetection_labelName = R"doc()doc";

static const char *__doc_dai_ImgDetection_xmax = R"doc()doc";

static const char *__doc_dai_ImgDetection_xmin = R"doc()doc";

static const char *__doc_dai_ImgDetection_ymax = R"doc()doc";

static const char *__doc_dai_ImgDetection_ymin = R"doc()doc";

static const char *__doc_dai_ImgDetections = R"doc(ImgDetections message. Carries normalized detection results)doc";

static const char *__doc_dai_ImgDetections_ImgDetections = R"doc(Construct ImgDetections message.)doc";

static const char *__doc_dai_ImgDetections_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgDetections_detections = R"doc(Detections)doc";

static const char *__doc_dai_ImgDetections_serialize = R"doc()doc";

static const char *__doc_dai_ImgDetections_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_ImgDetections_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_ImgDetections_str = R"doc()doc";

static const char *__doc_dai_ImgDetections_transformation = R"doc()doc";

static const char *__doc_dai_ImgFrame = R"doc()doc";

static const char *__doc_dai_ImgFrame_2 = R"doc(ImgFrame message. Carries image data and metadata.)doc";

static const char *__doc_dai_ImgFrameCapability = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_Impl = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_enableUndistortion = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_fps = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_pimpl = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_resizeMode = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_size = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_str = R"doc()doc";

static const char *__doc_dai_ImgFrameCapability_type = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_exposureTimeUs = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_lensPosition = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_lensPositionRaw = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_sensitivityIso = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_str = R"doc()doc";

static const char *__doc_dai_ImgFrame_CameraSettings_wbColorTemp = R"doc()doc";

static const char *__doc_dai_ImgFrame_ImgFrame = R"doc(Construct ImgFrame message. Timestamp is set to now)doc";

static const char *__doc_dai_ImgFrame_ImgFrame_2 = R"doc()doc";

static const char *__doc_dai_ImgFrame_ImgFrame_3 = R"doc()doc";

static const char *__doc_dai_ImgFrame_ImgFrame_4 = R"doc()doc";

static const char *__doc_dai_ImgFrame_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_bytesPP = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_height = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_p1Offset = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_p2Offset = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_p3Offset = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_str = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_stride = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_type = R"doc()doc";

static const char *__doc_dai_ImgFrame_Specs_width = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_BGR888i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_BGR888p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_BGRF16F16F16i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_BGRF16F16F16p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_BITSTREAM = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_GRAY8 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_GRAYF16 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_HDR = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_LUT16 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_LUT2 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_LUT4 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_NONE = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_NV12 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_NV21 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_PACK10 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_PACK12 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW10 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW12 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW14 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW16 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW32 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RAW8 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGB161616 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGB888i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGB888p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGBA8888 = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGBF16F16F16i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_RGBF16F16F16p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV400p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV420p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV422i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV422p = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV444i = R"doc()doc";

static const char *__doc_dai_ImgFrame_Type_YUV444p = R"doc()doc";

static const char *__doc_dai_ImgFrame_cam = R"doc()doc";

static const char *__doc_dai_ImgFrame_category = R"doc()doc";

static const char *__doc_dai_ImgFrame_event = R"doc()doc";

static const char *__doc_dai_ImgFrame_fb = R"doc()doc";

static const char *__doc_dai_ImgFrame_getBytesPerPixel = R"doc(Retrieves image bytes per pixel)doc";

static const char *__doc_dai_ImgFrame_getCategory = R"doc(Retrieves image category)doc";

static const char *__doc_dai_ImgFrame_getColorTemperature = R"doc(Retrieves white-balance color temperature of the light source, in kelvins)doc";

static const char *__doc_dai_ImgFrame_getCvFrame =
R"doc(@note This API only available if OpenCV support is enabled

Retrieves cv::Mat suitable for use in common opencv functions. ImgFrame is
converted to color BGR interleaved or grayscale depending on type.

A copy is always made

Returns:
    cv::Mat for use in opencv functions)doc";

static const char *__doc_dai_ImgFrame_getExposureTime = R"doc(Retrieves exposure time)doc";

static const char *__doc_dai_ImgFrame_getFrame =
R"doc(@note This API only available if OpenCV support is enabled

Retrieves data as cv::Mat with specified width, height and type

Parameter ``copy``:
    If false only a reference to data is made, otherwise a copy

Returns:
    cv::Mat with corresponding to ImgFrame parameters)doc";

static const char *__doc_dai_ImgFrame_getHeight = R"doc(Retrieves image height in pixels)doc";

static const char *__doc_dai_ImgFrame_getInstanceNum = R"doc(Retrieves instance number)doc";

static const char *__doc_dai_ImgFrame_getLensPosition = R"doc(Retrieves lens position, range 0..255. Returns -1 if not available)doc";

static const char *__doc_dai_ImgFrame_getLensPositionRaw = R"doc(Retrieves lens position, range 0.0f..1.0f. Returns -1 if not available)doc";

static const char *__doc_dai_ImgFrame_getPlaneHeight = R"doc(Retrieves image plane height in lines)doc";

static const char *__doc_dai_ImgFrame_getPlaneStride =
R"doc(Retrieves image plane stride (offset to next plane) in bytes

Parameter ``current``:
    plane index, 0 or 1)doc";

static const char *__doc_dai_ImgFrame_getSensitivity = R"doc(Retrieves sensitivity, as an ISO value)doc";

static const char *__doc_dai_ImgFrame_getSourceDFov =
R"doc(@note Fov API works correctly only on rectilinear frames Get the source diagonal
field of view in degrees

Returns:
    field of view in degrees)doc";

static const char *__doc_dai_ImgFrame_getSourceHFov =
R"doc(@note Fov API works correctly only on rectilinear frames Get the source
horizontal field of view

Parameter ``degrees``:
    field of view in degrees)doc";

static const char *__doc_dai_ImgFrame_getSourceHeight = R"doc(Retrieves source image height in pixels)doc";

static const char *__doc_dai_ImgFrame_getSourceVFov =
R"doc(@note Fov API works correctly only on rectilinear frames Get the source vertical
field of view

Parameter ``degrees``:
    field of view in degrees)doc";

static const char *__doc_dai_ImgFrame_getSourceWidth = R"doc(Retrieves source image width in pixels)doc";

static const char *__doc_dai_ImgFrame_getStride = R"doc(Retrieves image line stride in bytes)doc";

static const char *__doc_dai_ImgFrame_getTimestamp =
R"doc(Retrieves image timestamp (at the specified offset of exposure) related to
dai::Clock::now())doc";

static const char *__doc_dai_ImgFrame_getTimestampDevice =
R"doc(Retrieves image timestamp (at the specified offset of exposure) directly
captured from device's monotonic clock, not synchronized to host time. Used when
monotonicity is required.)doc";

static const char *__doc_dai_ImgFrame_getType = R"doc(Retrieves image type)doc";

static const char *__doc_dai_ImgFrame_getWidth = R"doc(Retrieves image width in pixels)doc";

static const char *__doc_dai_ImgFrame_instanceNum = R"doc()doc";

static const char *__doc_dai_ImgFrame_isInterleaved = R"doc()doc";

static const char *__doc_dai_ImgFrame_remapPointBetweenFrames =
R"doc(Remap point between two frames

Parameter ``originPoint``:
    point to remap

Parameter ``originFrame``:
    origin frame

Parameter ``destFrame``:
    destination frame

Returns:
    remapped point)doc";

static const char *__doc_dai_ImgFrame_remapPointFromSource =
R"doc(Remap a point from the current frame to the source frame

Parameter ``point``:
    point to remap

Returns:
    remapped point)doc";

static const char *__doc_dai_ImgFrame_remapPointToSource =
R"doc(Remap a point from the source frame to the current frame

Parameter ``point``:
    point to remap

Returns:
    remapped point)doc";

static const char *__doc_dai_ImgFrame_remapRectBetweenFrames =
R"doc(Remap rectangle between two frames

Parameter ``originRect``:
    rectangle to remap

Parameter ``originFrame``:
    origin frame

Parameter ``destFrame``:
    destination frame

Returns:
    remapped rectangle)doc";

static const char *__doc_dai_ImgFrame_remapRectFromSource =
R"doc(Remap a rectangle from the source frame to the current frame

Parameter ``rect``:
    rectangle to remap

Returns:
    remapped rectangle)doc";

static const char *__doc_dai_ImgFrame_remapRectToSource =
R"doc(Remap a rectangle from the current frame to the source frame

Parameter ``rect``:
    rectangle to remap

Returns:
    remapped rectangle)doc";

static const char *__doc_dai_ImgFrame_serialize = R"doc()doc";

static const char *__doc_dai_ImgFrame_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_ImgFrame_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_ImgFrame_setCategory =
R"doc(Parameter ``category``:
    Image category)doc";

static const char *__doc_dai_ImgFrame_setCvFrame =
R"doc(@note This API only available if OpenCV support is enabled

Copies cv::Mat data to the ImgFrame buffer and converts to a specific type.

Parameter ``frame``:
    Input cv::Mat BGR frame from which to copy the data)doc";

static const char *__doc_dai_ImgFrame_setFrame =
R"doc(@note This API only available if OpenCV support is enabled

Copies cv::Mat data to ImgFrame buffer

Parameter ``frame``:
    Input cv::Mat frame from which to copy the data)doc";

static const char *__doc_dai_ImgFrame_setHeight =
R"doc(Specifies frame height

Parameter ``height``:
    frame height)doc";

static const char *__doc_dai_ImgFrame_setInstanceNum =
R"doc(Instance number relates to the origin of the frame (which camera)

Parameter ``instance``:
    Instance number)doc";

static const char *__doc_dai_ImgFrame_setMetadata =
R"doc(Convience function to initialize meta data from another frame Copies over
timestamps, transformations done on the image, etc.

Parameter ``sourceFrame``:
    source frame from which the metadata is taken from)doc";

static const char *__doc_dai_ImgFrame_setMetadata_2 =
R"doc(Convience function to initialize meta data from another frame Copies over
timestamps, transformations done on the image, etc.

Parameter ``sourceFrame``:
    shared pointer to source frame from which the metadata is taken from)doc";

static const char *__doc_dai_ImgFrame_setSize =
R"doc(Specifies frame size

Parameter ``height``:
    frame height

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_ImgFrame_setSize_2 =
R"doc(Specifies frame size

Parameter ``size``:
    frame size)doc";

static const char *__doc_dai_ImgFrame_setSourceSize =
R"doc(Specifies source frame size

Parameter ``height``:
    frame height

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_ImgFrame_setSourceSize_2 =
R"doc(Specifies source frame size

Parameter ``size``:
    frame size)doc";

static const char *__doc_dai_ImgFrame_setStride =
R"doc(Specifies frame stride

Parameter ``stride``:
    frame stride)doc";

static const char *__doc_dai_ImgFrame_setType =
R"doc(Specifies frame type, RGB, BGR, ...

Parameter ``type``:
    Type of image)doc";

static const char *__doc_dai_ImgFrame_setWidth =
R"doc(Specifies frame width

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_ImgFrame_sourceFb = R"doc()doc";

static const char *__doc_dai_ImgFrame_str = R"doc()doc";

static const char *__doc_dai_ImgFrame_toInterleaved = R"doc()doc";

static const char *__doc_dai_ImgFrame_toPlanar = R"doc()doc";

static const char *__doc_dai_ImgFrame_transformation = R"doc()doc";

static const char *__doc_dai_ImgFrame_typeToBpp = R"doc()doc";

static const char *__doc_dai_ImgFrame_validateTransformations =
R"doc(Check that the image transformation match the image size

Returns:
    true if the transformations are valid)doc";

static const char *__doc_dai_ImgResizeMode = R"doc()doc";

static const char *__doc_dai_ImgResizeMode_CROP =
R"doc(Keeps aspect ratio. Crops the image to get the correct output aspect ratio.
Crops some FOV to match the required FOV, then scale. No potential NN accuracy
decrease.)doc";

static const char *__doc_dai_ImgResizeMode_LETTERBOX =
R"doc(Keeps aspect ratio. Envelop the image with a background color to get the corect
output aspect ratio. Preserves full FOV by padding/letterboxing, but smaller
frame means less features which might decrease NN accuracy.)doc";

static const char *__doc_dai_ImgResizeMode_STRETCH =
R"doc(Doesn't keep aspect ratio. Squishes or streches the image to fill the required
pixel area. Preserves full FOV, but frames are stretched to match the FOV, which
might decrease NN accuracy.)doc";

static const char *__doc_dai_ImgTransformation =
R"doc(ImgTransformation struct. Holds information of how a ImgFrame or related message
was transformed from their source. Useful for remapping from one ImgFrame to
another.)doc";

static const char *__doc_dai_ImgTransformation_ImgTransformation = R"doc()doc";

static const char *__doc_dai_ImgTransformation_ImgTransformation_2 = R"doc()doc";

static const char *__doc_dai_ImgTransformation_ImgTransformation_3 = R"doc()doc";

static const char *__doc_dai_ImgTransformation_ImgTransformation_4 = R"doc()doc";

static const char *__doc_dai_ImgTransformation_ImgTransformation_5 = R"doc()doc";

static const char *__doc_dai_ImgTransformation_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ImgTransformation_addCrop =
R"doc(Add a crop transformation.

Parameter ``x``:
    X coordinate of the top-left corner of the crop

Parameter ``y``:
    Y coordinate of the top-left corner of the crop

Parameter ``width``:
    Width of the crop

Parameter ``height``:
    Height of the crop)doc";

static const char *__doc_dai_ImgTransformation_addFlipHorizontal = R"doc(Add a horizontal flip transformation.)doc";

static const char *__doc_dai_ImgTransformation_addFlipVertical = R"doc(Add a vertical flip transformation.)doc";

static const char *__doc_dai_ImgTransformation_addPadding =
R"doc(Add a pad transformation. Works like crop, but in reverse.

Parameter ``top``:
    Padding on the top

Parameter ``bottom``:
    Padding on the bottom

Parameter ``left``:
    Padding on the left

Parameter ``right``:
    Padding on the right)doc";

static const char *__doc_dai_ImgTransformation_addRotation =
R"doc(Add a rotation transformation.

Parameter ``angle``:
    Angle in degrees

Parameter ``rotationPoint``:
    Point around which to rotate)doc";

static const char *__doc_dai_ImgTransformation_addScale =
R"doc(Add a scale transformation.

Parameter ``scaleX``:
    Scale factor in the horizontal direction

Parameter ``scaleY``:
    Scale factor in the vertical direction)doc";

static const char *__doc_dai_ImgTransformation_addSrcCrops = R"doc()doc";

static const char *__doc_dai_ImgTransformation_addTransformation =
R"doc(Add a new transformation.

Parameter ``matrix``:
    Transformation matrix)doc";

static const char *__doc_dai_ImgTransformation_calcCrops = R"doc()doc";

static const char *__doc_dai_ImgTransformation_cropsValid = R"doc()doc";

static const char *__doc_dai_ImgTransformation_distortionCoefficients = R"doc()doc";

static const char *__doc_dai_ImgTransformation_distortionModel = R"doc()doc";

static const char *__doc_dai_ImgTransformation_dstCrop = R"doc()doc";

static const char *__doc_dai_ImgTransformation_getDFov =
R"doc(Retrieve the diagonal field of view of the image.

Parameter ``source``:
    If true, the source field of view will be returned. Otherwise, the current
    field of view will be returned.

Returns:
    Diagonal field of view in degrees)doc";

static const char *__doc_dai_ImgTransformation_getDistortionCoefficients =
R"doc(Retrieve the distortion coefficients of the source sensor

Returns:
    vector of distortion coefficients)doc";

static const char *__doc_dai_ImgTransformation_getDistortionModel =
R"doc(Retrieve the distortion model of the source sensor

Returns:
    Distortion model)doc";

static const char *__doc_dai_ImgTransformation_getDstMaskPt =
R"doc(Returns true if the point is inside the image region (not in the background
region).)doc";

static const char *__doc_dai_ImgTransformation_getHFov =
R"doc(Retrieve the horizontal field of view of the image.

Parameter ``source``:
    If true, the source field of view will be returned. Otherwise, the current
    field of view will be returned.

Returns:
    Horizontal field of view in degrees)doc";

static const char *__doc_dai_ImgTransformation_getIntrinsicMatrix =
R"doc(Retrieve the total intrinsic matrix calculated from intrinsic * transform.

Returns:
    total intrinsic matrix)doc";

static const char *__doc_dai_ImgTransformation_getIntrinsicMatrixInv =
R"doc(Retrieve the inverse of the total intrinsic matrix calculated from intrinsic *
transform.

Returns:
    inverse total intrinsic matrix)doc";

static const char *__doc_dai_ImgTransformation_getMatrix =
R"doc(Retrieve the transformation matrix from the source frame to the current frame.

Returns:
    Transformation matrix)doc";

static const char *__doc_dai_ImgTransformation_getMatrixInv =
R"doc(Retrieve the inverse transformation matrix from the current frame to the source
frame.

Returns:
    Inverse transformation matrix)doc";

static const char *__doc_dai_ImgTransformation_getSize =
R"doc(Retrieve the size of the frame. Should be equal to the size of the corresponding
ImgFrame message.

Returns:
    Size of the frame)doc";

static const char *__doc_dai_ImgTransformation_getSourceIntrinsicMatrix =
R"doc(Retrieve the intrinsic matrix of the source sensor.

Returns:
    Intrinsic matrix)doc";

static const char *__doc_dai_ImgTransformation_getSourceIntrinsicMatrixInv =
R"doc(Retrieve the inverse intrinsic matrix of the source sensor.

Returns:
    Inverse intrinsic matrix)doc";

static const char *__doc_dai_ImgTransformation_getSourceSize =
R"doc(Retrieve the size of the source frame from which this frame was derived.

Returns:
    Size of the frame)doc";

static const char *__doc_dai_ImgTransformation_getSrcCrops = R"doc()doc";

static const char *__doc_dai_ImgTransformation_getSrcMaskPt =
R"doc(Returns true if the point is inside the transformed region of interest
(determined by crops used).)doc";

static const char *__doc_dai_ImgTransformation_getVFov =
R"doc(Retrieve the vertical field of view of the image.

Parameter ``source``:
    If true, the source field of view will be returned. Otherwise, the current
    field of view will be returned.

Returns:
    Vertical field of view in degrees)doc";

static const char *__doc_dai_ImgTransformation_height = R"doc()doc";

static const char *__doc_dai_ImgTransformation_invTransformPoint =
R"doc(Transform a point from the current frame to the source frame.

Parameter ``point``:
    Point to transform

Returns:
    Transformed point)doc";

static const char *__doc_dai_ImgTransformation_invTransformRect =
R"doc(Transform a rotated rect from the current frame to the source frame.

Parameter ``rect``:
    Rectangle to transform

Returns:
    Transformed rectangle)doc";

static const char *__doc_dai_ImgTransformation_isValid =
R"doc(Check if the transformations are valid. The transformations are valid if the
source frame size and the current frame size are set.)doc";

static const char *__doc_dai_ImgTransformation_remapPointFrom =
R"doc(Remap a point to this transformation from another. If the intrinsics are
different (e.g. different camera), the function will also use the intrinsics to
remap the point.

Parameter ``from``:
    Transformation to remap from

Parameter ``point``:
    Point to remap)doc";

static const char *__doc_dai_ImgTransformation_remapPointTo =
R"doc(Remap a point from this transformation to another. If the intrinsics are
different (e.g. different camera), the function will also use the intrinsics to
remap the point.

Parameter ``to``:
    Transformation to remap to

Parameter ``point``:
    Point to remap)doc";

static const char *__doc_dai_ImgTransformation_remapRectFrom =
R"doc(Remap a rotated rect to this transformation from another. If the intrinsics are
different (e.g. different camera), the function will also use the intrinsics to
remap the rect.

Parameter ``from``:
    Transformation to remap from

Parameter ``point``:
    RotatedRect to remap)doc";

static const char *__doc_dai_ImgTransformation_remapRectTo =
R"doc(Remap a rotated rect from this transformation to another. If the intrinsics are
different (e.g. different camera), the function will also use the intrinsics to
remap the rect.

Parameter ``to``:
    Transformation to remap to

Parameter ``rect``:
    RotatedRect to remap)doc";

static const char *__doc_dai_ImgTransformation_setDistortionCoefficients = R"doc()doc";

static const char *__doc_dai_ImgTransformation_setDistortionModel = R"doc()doc";

static const char *__doc_dai_ImgTransformation_setIntrinsicMatrix = R"doc()doc";

static const char *__doc_dai_ImgTransformation_setSize = R"doc()doc";

static const char *__doc_dai_ImgTransformation_setSourceSize = R"doc()doc";

static const char *__doc_dai_ImgTransformation_sourceIntrinsicMatrix = R"doc()doc";

static const char *__doc_dai_ImgTransformation_sourceIntrinsicMatrixInv = R"doc()doc";

static const char *__doc_dai_ImgTransformation_srcCrop = R"doc()doc";

static const char *__doc_dai_ImgTransformation_srcCrops = R"doc()doc";

static const char *__doc_dai_ImgTransformation_srcHeight = R"doc()doc";

static const char *__doc_dai_ImgTransformation_srcWidth = R"doc()doc";

static const char *__doc_dai_ImgTransformation_str = R"doc()doc";

static const char *__doc_dai_ImgTransformation_transformPoint =
R"doc(Transform a point from the source frame to the current frame.

Parameter ``point``:
    Point to transform

Returns:
    Transformed point)doc";

static const char *__doc_dai_ImgTransformation_transformRect =
R"doc(Transform a rotated rect from the source frame to the current frame.

Parameter ``rect``:
    Rectangle to transform

Returns:
    Transformed rectangle)doc";

static const char *__doc_dai_ImgTransformation_transformationMatrix = R"doc()doc";

static const char *__doc_dai_ImgTransformation_transformationMatrixInv = R"doc()doc";

static const char *__doc_dai_ImgTransformation_width = R"doc()doc";

static const char *__doc_dai_InputQueue = R"doc()doc";

static const char *__doc_dai_InputQueue_2 = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueue =
R"doc(Construct a new Input Queue object. The constructor is private as we only want
to expose the relevant methods - only send for now

Parameter ``maxSize:``:
    Maximum size of the input queue

Parameter ``blocking:``:
    Whether the input queue should block when full)doc";

static const char *__doc_dai_InputQueue_InputQueueNode = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueueNode_InputQueueNode = R"doc(Constructor)doc";

static const char *__doc_dai_InputQueue_InputQueueNode_getName = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueueNode_input = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueueNode_output = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueueNode_run = R"doc()doc";

static const char *__doc_dai_InputQueue_InputQueueNode_send = R"doc(Send message from host)doc";

static const char *__doc_dai_InputQueue_getNode = R"doc()doc";

static const char *__doc_dai_InputQueue_getNodeOutput = R"doc()doc";

static const char *__doc_dai_InputQueue_inputQueueNode =
R"doc(Pointer to InputQueueNode that does the actual communication between host and
device)doc";

static const char *__doc_dai_InputQueue_send =
R"doc(Send a message to the connected input

Parameter ``msg:``:
    Message to send)doc";

static const char *__doc_dai_Interpolation = R"doc(Interpolation type)doc";

static const char *__doc_dai_Interpolation_AUTO = R"doc()doc";

static const char *__doc_dai_Interpolation_BICUBIC = R"doc()doc";

static const char *__doc_dai_Interpolation_BILINEAR = R"doc()doc";

static const char *__doc_dai_Interpolation_BYPASS = R"doc()doc";

static const char *__doc_dai_Interpolation_DEFAULT = R"doc()doc";

static const char *__doc_dai_Interpolation_DEFAULT_DISPARITY_DEPTH = R"doc()doc";

static const char *__doc_dai_Interpolation_NEAREST_NEIGHBOR = R"doc()doc";

static const char *__doc_dai_JoiningThread = R"doc()doc";

static const char *__doc_dai_JoiningThread_JoiningThread = R"doc()doc";

static const char *__doc_dai_JoiningThread_JoiningThread_2 = R"doc()doc";

static const char *__doc_dai_JoiningThread_JoiningThread_3 = R"doc()doc";

static const char *__doc_dai_JoiningThread_JoiningThread_4 = R"doc()doc";

static const char *__doc_dai_JoiningThread_operator_assign = R"doc()doc";

static const char *__doc_dai_JoiningThread_swap = R"doc()doc";

static const char *__doc_dai_Landmark = R"doc()doc";

static const char *__doc_dai_Landmark_id = R"doc(The ID of the Landmark)doc";

static const char *__doc_dai_Landmark_quaternion = R"doc(The orientation of the landmark relative to base_frame (the camera))doc";

static const char *__doc_dai_Landmark_size = R"doc(The size of the landmark in meters)doc";

static const char *__doc_dai_Landmark_translation = R"doc(The translation of the landmark reletive to base_frame (the camera))doc";

static const char *__doc_dai_Landmarks = R"doc()doc";

static const char *__doc_dai_Landmarks_Landmarks = R"doc()doc";

static const char *__doc_dai_Landmarks_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Landmarks_landmarks = R"doc()doc";

static const char *__doc_dai_Landmarks_serialize = R"doc()doc";

static const char *__doc_dai_Landmarks_str = R"doc()doc";

static const char *__doc_dai_LockingQueue = R"doc()doc";

static const char *__doc_dai_LockingQueue_LockingQueue = R"doc()doc";

static const char *__doc_dai_LockingQueue_LockingQueue_2 = R"doc()doc";

static const char *__doc_dai_LockingQueue_LockingQueue_3 = R"doc()doc";

static const char *__doc_dai_LockingQueue_LockingQueue_4 = R"doc()doc";

static const char *__doc_dai_LockingQueue_blocking = R"doc()doc";

static const char *__doc_dai_LockingQueue_consumeAll = R"doc()doc";

static const char *__doc_dai_LockingQueue_destruct = R"doc()doc";

static const char *__doc_dai_LockingQueue_destructed = R"doc()doc";

static const char *__doc_dai_LockingQueue_empty = R"doc()doc";

static const char *__doc_dai_LockingQueue_front = R"doc()doc";

static const char *__doc_dai_LockingQueue_getBlocking = R"doc()doc";

static const char *__doc_dai_LockingQueue_getMaxSize = R"doc()doc";

static const char *__doc_dai_LockingQueue_getSize = R"doc()doc";

static const char *__doc_dai_LockingQueue_guard = R"doc()doc";

static const char *__doc_dai_LockingQueue_isDestroyed = R"doc()doc";

static const char *__doc_dai_LockingQueue_isFull = R"doc()doc";

static const char *__doc_dai_LockingQueue_maxSize = R"doc()doc";

static const char *__doc_dai_LockingQueue_operator_assign = R"doc()doc";

static const char *__doc_dai_LockingQueue_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_LockingQueue_push = R"doc()doc";

static const char *__doc_dai_LockingQueue_push_2 = R"doc()doc";

static const char *__doc_dai_LockingQueue_queue = R"doc()doc";

static const char *__doc_dai_LockingQueue_setBlocking = R"doc()doc";

static const char *__doc_dai_LockingQueue_setMaxSize = R"doc()doc";

static const char *__doc_dai_LockingQueue_signalPop = R"doc()doc";

static const char *__doc_dai_LockingQueue_signalPush = R"doc()doc";

static const char *__doc_dai_LockingQueue_tryPop = R"doc()doc";

static const char *__doc_dai_LockingQueue_tryWaitAndPop = R"doc()doc";

static const char *__doc_dai_LockingQueue_tryWaitAndPush = R"doc()doc";

static const char *__doc_dai_LockingQueue_tryWaitAndPush_2 = R"doc()doc";

static const char *__doc_dai_LockingQueue_waitAndConsumeAll = R"doc()doc";

static const char *__doc_dai_LockingQueue_waitAndConsumeAll_2 = R"doc()doc";

static const char *__doc_dai_LockingQueue_waitAndPop = R"doc()doc";

static const char *__doc_dai_LockingQueue_waitEmpty = R"doc()doc";

static const char *__doc_dai_LogLevel = R"doc()doc";

static const char *__doc_dai_LogLevel_CRITICAL = R"doc()doc";

static const char *__doc_dai_LogLevel_DEBUG = R"doc()doc";

static const char *__doc_dai_LogLevel_ERR = R"doc()doc";

static const char *__doc_dai_LogLevel_INFO = R"doc()doc";

static const char *__doc_dai_LogLevel_OFF = R"doc()doc";

static const char *__doc_dai_LogLevel_TRACE = R"doc()doc";

static const char *__doc_dai_LogLevel_WARN = R"doc()doc";

static const char *__doc_dai_LogMessage = R"doc()doc";

static const char *__doc_dai_LogMessage_colorRangeEnd = R"doc()doc";

static const char *__doc_dai_LogMessage_colorRangeStart = R"doc()doc";

static const char *__doc_dai_LogMessage_level = R"doc()doc";

static const char *__doc_dai_LogMessage_nodeIdName = R"doc()doc";

static const char *__doc_dai_LogMessage_payload = R"doc()doc";

static const char *__doc_dai_LogMessage_time = R"doc()doc";

static const char *__doc_dai_ManipOp = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_2 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_3 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_4 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_5 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_6 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_7 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_8 = R"doc()doc";

static const char *__doc_dai_ManipOp_ManipOp_9 = R"doc()doc";

static const char *__doc_dai_ManipOp_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ManipOp_op = R"doc()doc";

static const char *__doc_dai_ManipOp_str = R"doc()doc";

static const char *__doc_dai_MedianFilter = R"doc(Median filter config)doc";

static const char *__doc_dai_MedianFilter_KERNEL_3x3 = R"doc()doc";

static const char *__doc_dai_MedianFilter_KERNEL_5x5 = R"doc()doc";

static const char *__doc_dai_MedianFilter_KERNEL_7x7 = R"doc()doc";

static const char *__doc_dai_MedianFilter_MEDIAN_OFF = R"doc()doc";

static const char *__doc_dai_Memory = R"doc()doc";

static const char *__doc_dai_MemoryInfo =
R"doc(MemoryInfo structure

Free, remaining and total memory stats)doc";

static const char *__doc_dai_MemoryInfo_remaining = R"doc()doc";

static const char *__doc_dai_MemoryInfo_total = R"doc()doc";

static const char *__doc_dai_MemoryInfo_used = R"doc()doc";

static const char *__doc_dai_Memory_getData = R"doc()doc";

static const char *__doc_dai_Memory_getData_2 = R"doc()doc";

static const char *__doc_dai_Memory_getMaxSize = R"doc()doc";

static const char *__doc_dai_Memory_getOffset = R"doc()doc";

static const char *__doc_dai_Memory_getSize = R"doc()doc";

static const char *__doc_dai_Memory_setSize = R"doc()doc";

static const char *__doc_dai_MessageDemuxProperties = R"doc(MessageDemux does not have any properties to set)doc";

static const char *__doc_dai_MessageDemuxProperties_dummy = R"doc()doc";

static const char *__doc_dai_MessageGroup = R"doc(MessageGroup message. Carries multiple messages in one.)doc";

static const char *__doc_dai_MessageGroup_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_MessageGroup_add = R"doc()doc";

static const char *__doc_dai_MessageGroup_begin = R"doc()doc";

static const char *__doc_dai_MessageGroup_end = R"doc()doc";

static const char *__doc_dai_MessageGroup_get = R"doc()doc";

static const char *__doc_dai_MessageGroup_get_2 = R"doc()doc";

static const char *__doc_dai_MessageGroup_getIntervalNs = R"doc(Retrieves interval between the first and the last message in the group.)doc";

static const char *__doc_dai_MessageGroup_getMessageNames = R"doc(Gets the names of messages in the group)doc";

static const char *__doc_dai_MessageGroup_getNumMessages = R"doc()doc";

static const char *__doc_dai_MessageGroup_group = R"doc()doc";

static const char *__doc_dai_MessageGroup_isSynced =
R"doc(True if all messages in the group are in the interval

Parameter ``thresholdNs``:
    Maximal interval between messages)doc";

static const char *__doc_dai_MessageGroup_operator_array = R"doc(Group)doc";

static const char *__doc_dai_MessageGroup_serialize = R"doc()doc";

static const char *__doc_dai_MessageGroup_str = R"doc()doc";

static const char *__doc_dai_MessageQueue = R"doc(Thread safe queue to send messages between nodes)doc";

static const char *__doc_dai_MessageQueue_MessageQueue = R"doc()doc";

static const char *__doc_dai_MessageQueue_MessageQueue_2 = R"doc()doc";

static const char *__doc_dai_MessageQueue_MessageQueue_3 = R"doc()doc";

static const char *__doc_dai_MessageQueue_MessageQueue_4 = R"doc()doc";

static const char *__doc_dai_MessageQueue_QueueException = R"doc()doc";

static const char *__doc_dai_MessageQueue_QueueException_QueueException = R"doc()doc";

static const char *__doc_dai_MessageQueue_addCallback =
R"doc(Adds a callback on message received

Parameter ``callback``:
    Callback function with queue name and message pointer

Returns:
    Callback id)doc";

static const char *__doc_dai_MessageQueue_addCallback_2 =
R"doc(Adds a callback on message received

Parameter ``callback``:
    Callback function with message pointer

Returns:
    Callback id)doc";

static const char *__doc_dai_MessageQueue_addCallback_3 =
R"doc(Adds a callback on message received

Parameter ``callback``:
    Callback function without any parameters

Returns:
    Callback id)doc";

static const char *__doc_dai_MessageQueue_callCallbacks = R"doc()doc";

static const char *__doc_dai_MessageQueue_callbacks = R"doc()doc";

static const char *__doc_dai_MessageQueue_callbacksMtx = R"doc()doc";

static const char *__doc_dai_MessageQueue_close = R"doc(Closes the queue and unblocks any waiting consumers or producers)doc";

static const char *__doc_dai_MessageQueue_front =
R"doc(Gets first message in the queue.

Returns:
    Message of type T or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_front_2 =
R"doc(Gets first message in the queue.

Returns:
    Message or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_get =
R"doc(Block until a message is available.

Returns:
    Message of type T or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_get_2 =
R"doc(Block until a message is available.

Returns:
    Message or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_get_3 =
R"doc(Block until a message is available with a timeout.

Parameter ``timeout``:
    Duration for which the function should block

Parameter ``hasTimedout``:
    Outputs true if timeout occurred, false otherwise

Returns:
    Message of type T otherwise nullptr if message isn't type T or timeout
    occurred)doc";

static const char *__doc_dai_MessageQueue_get_4 =
R"doc(Block until a message is available with a timeout.

Parameter ``timeout``:
    Duration for which the function should block

Parameter ``hasTimedout``:
    Outputs true if timeout occurred, false otherwise

Returns:
    Message of type T otherwise nullptr if message isn't type T or timeout
    occurred)doc";

static const char *__doc_dai_MessageQueue_getAll =
R"doc(Block until at least one message in the queue. Then return all messages from the
queue.

Returns:
    Vector of messages which can either be of type T or nullptr)doc";

static const char *__doc_dai_MessageQueue_getAll_2 =
R"doc(Block until at least one message in the queue. Then return all messages from the
queue.

Returns:
    Vector of messages)doc";

static const char *__doc_dai_MessageQueue_getAll_3 =
R"doc(Block for maximum timeout duration. Then return all messages from the queue.

Parameter ``timeout``:
    Maximum duration to block

Parameter ``hasTimedout``:
    Outputs true if timeout occurred, false otherwise

Returns:
    Vector of messages which can either be of type T or nullptr)doc";

static const char *__doc_dai_MessageQueue_getAll_4 =
R"doc(Block for maximum timeout duration. Then return all messages from the queue.

Parameter ``timeout``:
    Maximum duration to block

Parameter ``hasTimedout``:
    Outputs true if timeout occurred, false otherwise

Returns:
    Vector of messages)doc";

static const char *__doc_dai_MessageQueue_getBlocking =
R"doc(Gets current queue behavior when full (maxSize)

Returns:
    True if blocking, false otherwise)doc";

static const char *__doc_dai_MessageQueue_getMaxSize =
R"doc(Gets queue maximum size

Returns:
    Maximum queue size)doc";

static const char *__doc_dai_MessageQueue_getName = R"doc(Get name of the queue)doc";

static const char *__doc_dai_MessageQueue_getSize =
R"doc(Gets queue current size

Returns:
    Current queue size)doc";

static const char *__doc_dai_MessageQueue_has =
R"doc(Check whether front of the queue has message of type T

Returns:
    True if queue isn't empty and the first element is of type T, false
    otherwise)doc";

static const char *__doc_dai_MessageQueue_has_2 =
R"doc(Check whether front of the queue has a message (isn't empty)

Returns:
    True if queue isn't empty, false otherwise)doc";

static const char *__doc_dai_MessageQueue_isClosed = R"doc(Check whether queue is closed)doc";

static const char *__doc_dai_MessageQueue_isFull =
R"doc(Gets whether queue is full

Returns:
    True if queue is full, false otherwise)doc";

static const char *__doc_dai_MessageQueue_name = R"doc()doc";

static const char *__doc_dai_MessageQueue_operator_assign = R"doc()doc";

static const char *__doc_dai_MessageQueue_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_MessageQueue_queue = R"doc()doc";

static const char *__doc_dai_MessageQueue_removeCallback =
R"doc(Removes a callback

Parameter ``callbackId``:
    Id of callback to be removed

Returns:
    True if callback was removed, false otherwise)doc";

static const char *__doc_dai_MessageQueue_send =
R"doc(Adds a message to the queue, which will be picked up and sent to the device. Can
either block if 'blocking' behavior is true or overwrite oldest

Parameter ``msg``:
    Message to add to the queue)doc";

static const char *__doc_dai_MessageQueue_send_2 =
R"doc(Adds message to the queue, which will be picked up and sent to the device. Can
either block until timeout if 'blocking' behavior is true or overwrite oldest

Parameter ``msg``:
    Message to add to the queue

Parameter ``timeout``:
    Maximum duration to block in milliseconds)doc";

static const char *__doc_dai_MessageQueue_send_3 =
R"doc(Adds message to the queue, which will be picked up and sent to the device. Can
either block until timeout if 'blocking' behavior is true or overwrite oldest

Parameter ``msg``:
    Message to add to the queue

Parameter ``timeout``:
    Maximum duration to block in milliseconds)doc";

static const char *__doc_dai_MessageQueue_setBlocking =
R"doc(Sets queue behavior when full (maxSize)

Parameter ``blocking``:
    Specifies if block or overwrite the oldest message in the queue)doc";

static const char *__doc_dai_MessageQueue_setMaxSize =
R"doc(Sets queue maximum size

Parameter ``maxSize``:
    Specifies maximum number of messages in the queue @note If maxSize is
    smaller than size, queue will not be truncated immediately, only after
    messages are popped)doc";

static const char *__doc_dai_MessageQueue_setName = R"doc(Set the name of the queue)doc";

static const char *__doc_dai_MessageQueue_tryGet =
R"doc(Try to retrieve message T from queue. If message isn't of type T it returns
nullptr

Returns:
    Message of type T or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_tryGet_2 =
R"doc(Try to retrieve message from queue. If no message available, return immediately
with nullptr

Returns:
    Message or nullptr if no message available)doc";

static const char *__doc_dai_MessageQueue_tryGetAll =
R"doc(Try to retrieve all messages in the queue.

Returns:
    Vector of messages which can either be of type T or nullptr)doc";

static const char *__doc_dai_MessageQueue_tryGetAll_2 =
R"doc(Try to retrieve all messages in the queue.

Returns:
    Vector of messages)doc";

static const char *__doc_dai_MessageQueue_trySend =
R"doc(Tries sending a message

Parameter ``msg``:
    message to send)doc";

static const char *__doc_dai_MessageQueue_uniqueCallbackId = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties = R"doc(Specify properties for MonoCamera such as camera ID, ...)doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution =
R"doc(Select the camera sensor resolution: 1280×720, 1280×800, 640×400, 640×480,
1920×1200, ...)doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_1200_P = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_4000X3000 = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_400_P = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_4224X3136 = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_480_P = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_720_P = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_SensorResolution_THE_800_P = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_boardSocket = R"doc(Which socket will mono camera use)doc";

static const char *__doc_dai_MonoCameraProperties_cameraName = R"doc(Which camera name will mono camera use)doc";

static const char *__doc_dai_MonoCameraProperties_eventFilter = R"doc(List of events to receive, the rest will be ignored)doc";

static const char *__doc_dai_MonoCameraProperties_fps = R"doc(Camera sensor FPS)doc";

static const char *__doc_dai_MonoCameraProperties_imageOrientation = R"doc(Camera sensor image orientation / pixel readout)doc";

static const char *__doc_dai_MonoCameraProperties_initialControl = R"doc()doc";

static const char *__doc_dai_MonoCameraProperties_isp3aFps =
R"doc(Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls
etc.). Default (0) matches the camera FPS, meaning that 3A is running on each
frame. Reducing the rate of 3A reduces the CPU usage on CSS, but also increases
the convergence rate of 3A. Note that camera controls will be processed at this
rate. E.g. if camera is running at 30 fps, and camera control is sent at every
frame, but 3A fps is set to 15, the camera control messages will be processed at
15 fps rate, which will lead to queueing.)doc";

static const char *__doc_dai_MonoCameraProperties_mockIspHeight =
R"doc(Select the mock isp height. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_MonoCameraProperties_mockIspWidth =
R"doc(Select the mock isp width. Overrides resolutionWidth/height if mockIsp is
connected.)doc";

static const char *__doc_dai_MonoCameraProperties_numFramesPool = R"doc(Frame pool size for the main output, ISP processed)doc";

static const char *__doc_dai_MonoCameraProperties_numFramesPoolRaw = R"doc(Frame pool size for the `raw` output)doc";

static const char *__doc_dai_MonoCameraProperties_rawPacked =
R"doc(Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
The packed format is more efficient, consuming less memory on device, and less
data to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on
3 bytes. When packing is disabled (`false`), data is saved lsb-aligned, e.g. a
RAW10 pixel will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
Default is auto: enabled for standard color/monochrome cameras where ISP can
work with both packed/unpacked, but disabled for other cameras like ToF.)doc";

static const char *__doc_dai_MonoCameraProperties_resolution = R"doc(Select the camera sensor resolution)doc";

static const char *__doc_dai_NNArchive = R"doc()doc";

static const char *__doc_dai_NNArchiveConfigVersion = R"doc()doc";

static const char *__doc_dai_NNArchiveConfigVersion_V1 = R"doc()doc";

static const char *__doc_dai_NNArchiveEntry = R"doc()doc";

static const char *__doc_dai_NNArchiveEntry_Compression = R"doc()doc";

static const char *__doc_dai_NNArchiveEntry_Compression_AUTO =
R"doc(Try to guess the file format from the file extension. .json -> RAW_FS everything
else use libarchive to guess the format supported formats are:
https://github.com/libarchive/libarchive?tab=readme-ov-file#supported-formats)doc";

static const char *__doc_dai_NNArchiveEntry_Compression_RAW_FS = R"doc(The entry isn't compressed. Access it directly on the filesystem.)doc";

static const char *__doc_dai_NNArchiveEntry_Compression_TAR = R"doc(Force libarchive to treat the file as .tar)doc";

static const char *__doc_dai_NNArchiveEntry_Compression_TAR_GZ = R"doc(Force libarchive to treat the file as .tar.gz)doc";

static const char *__doc_dai_NNArchiveEntry_Compression_TAR_XZ = R"doc(Force libarchive to treat the file as .tar.xz)doc";

static const char *__doc_dai_NNArchiveEntry_Seek = R"doc(Check stdio.h SEEK_SET, SEEK_CUR, SEEK_END for meaning.)doc";

static const char *__doc_dai_NNArchiveEntry_Seek_CUR = R"doc()doc";

static const char *__doc_dai_NNArchiveEntry_Seek_END = R"doc()doc";

static const char *__doc_dai_NNArchiveEntry_Seek_SET = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_NNArchiveOptions = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_compression = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_compression_2 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_compression_3 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_compression_4 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_compression_5 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_extractFolder = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_extractFolder_2 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_extractFolder_3 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_extractFolder_4 = R"doc()doc";

static const char *__doc_dai_NNArchiveOptions_extractFolder_5 = R"doc()doc";

static const char *__doc_dai_NNArchiveVersionedConfig = R"doc()doc";

static const char *__doc_dai_NNArchiveVersionedConfig_NNArchiveVersionedConfig =
R"doc(@data Should point to a whole compressed NNArchive read to memory if compression
is not set to RAW_FS. If compression is set to RAW_FS, then this should point to
just the config.json file read to memory.)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_NNArchiveVersionedConfig_2 =
R"doc(@path Should point to: 1) if compression is set to RAW_FS: to just the
config.json file. 2) if compression is set to AUTO: to whole compressed
NNArchive or just the config.json file which must end in .json . 3) else: to
whole compressed NNArchive. see NNArchive class for parameter explanation)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_NNArchiveVersionedConfig_3 =
R"doc(Returned data should be just the config.json if compression == RAW_FS or the
whole NNArchive otherwise see NNArchive class for parameter explanation)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_NNArchiveVersionedConfig_4 =
R"doc(Construct NNArchiveVersionedConfig from a specific version of a config.

Parameter ``version:``:
    Version of the config.)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_config = R"doc()doc";

static const char *__doc_dai_NNArchiveVersionedConfig_getConfig =
R"doc(Get stored config cast to a specific version.

Template parameter ``T:``:
    Config type to cast to.)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_getVersion = R"doc(Get version of the underlying config.)doc";

static const char *__doc_dai_NNArchiveVersionedConfig_initConfig = R"doc()doc";

static const char *__doc_dai_NNArchiveVersionedConfig_version = R"doc()doc";

static const char *__doc_dai_NNArchive_NNArchive =
R"doc(Construct a new NNArchive object - a container holding a model and its
configuration

Parameter ``archivePath:``:
    Path to the archive file

Parameter ``options:``:
    Archive options such as compression, number of shaves, etc. See
    NNArchiveOptions.)doc";

static const char *__doc_dai_NNArchive_archiveOptions = R"doc()doc";

static const char *__doc_dai_NNArchive_archiveVersionedConfigPtr = R"doc()doc";

static const char *__doc_dai_NNArchive_blobPtr = R"doc()doc";

static const char *__doc_dai_NNArchive_getBlob =
R"doc(Return a SuperVINO::Blob from the archive if getModelType() returns BLOB,
nothing otherwise

Returns:
    std::optional<OpenVINO::Blob>: Model blob)doc";

static const char *__doc_dai_NNArchive_getConfig =
R"doc(Get NNArchive config.

Template parameter ``T:``:
    Type of config to get

Returns:
    const T&: Config)doc";

static const char *__doc_dai_NNArchive_getInputHeight =
R"doc(Get inputHeight of the model

Parameter ``index:``:
    Index of input

Returns:
    int: inputHeight)doc";

static const char *__doc_dai_NNArchive_getInputSize =
R"doc(Get inputSize of the model

Parameter ``index:``:
    Index of input @note this function is only valid for models with NCHW and
    NHWC input formats

Returns:
    std::vector<std::pair<int, int>>: inputSize)doc";

static const char *__doc_dai_NNArchive_getInputWidth =
R"doc(Get inputWidth of the model

Parameter ``index:``:
    Index of input

Returns:
    int: inputWidth)doc";

static const char *__doc_dai_NNArchive_getModelPath =
R"doc(Return a path to the model inside the archive if getModelType() returns OTHER or
DLC, nothing otherwise

Returns:
    std::optional<Path>: Model path)doc";

static const char *__doc_dai_NNArchive_getModelType =
R"doc(Get type of model contained in NNArchive

Returns:
    model::ModelType: type of model in archive)doc";

static const char *__doc_dai_NNArchive_getSuperBlob =
R"doc(Return a SuperVINO::SuperBlob from the archive if getModelType() returns
SUPERBLOB, nothing otherwise

Returns:
    std::optional<OpenVINO::SuperBlob>: Model superblob)doc";

static const char *__doc_dai_NNArchive_getSupportedPlatforms =
R"doc(Get supported platforms

Returns:
    std::vector<dai::Platform>: Supported platforms)doc";

static const char *__doc_dai_NNArchive_getVersionedConfig =
R"doc(Get NNArchive config wrapper

Returns:
    NNArchiveVersionedConfig: Archive config)doc";

static const char *__doc_dai_NNArchive_modelType = R"doc()doc";

static const char *__doc_dai_NNArchive_readModelFromArchive = R"doc()doc";

static const char *__doc_dai_NNArchive_superblobPtr = R"doc()doc";

static const char *__doc_dai_NNArchive_unpackArchiveInDirectory = R"doc()doc";

static const char *__doc_dai_NNArchive_unpackedModelPath = R"doc()doc";

static const char *__doc_dai_NNData = R"doc(NNData message. Carries tensors and their metadata)doc";

static const char *__doc_dai_NNData_NNData = R"doc(Construct NNData message.)doc";

static const char *__doc_dai_NNData_NNData_2 = R"doc()doc";

static const char *__doc_dai_NNData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_NNData_addTensor =
R"doc(Add a tensor to this NNData object. The provided array is stored as a 1xN tensor
where N is the length of the array.

Parameter ``name:``:
    Name of the tensor

Parameter ``data:``:
    array

Returns:
    NNData&: reference to this object)doc";

static const char *__doc_dai_NNData_addTensor_2 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_3 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_4 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_5 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_6 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_7 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_8 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_9 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_10 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_11 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_12 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_13 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_14 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_15 = R"doc()doc";

static const char *__doc_dai_NNData_addTensor_16 =
R"doc(Add a tensor to this NNData object. The provided array is stored as a 1xN tensor
where N is the length of the array.

Parameter ``name:``:
    Name of the tensor

Parameter ``data:``:
    array

Parameter ``order:``:
    Storage order of the tensor

Returns:
    NNData&: reference to this object)doc";

static const char *__doc_dai_NNData_addTensor_17 =
R"doc(Add a tensor to this NNData object. Implicitly adds a TensorInfo::DataType

Parameter ``name:``:
    Name of the tensor

Parameter ``data:``:
    array

Parameter ``order:``:
    Storage order of the tensor

Returns:
    NNData&: reference to this object)doc";

static const char *__doc_dai_NNData_addTensor_18 =
R"doc(Add a tensor to this NNData object. The storage order is picked based on the
number of dimensions of the tensor. Float values are converted to FP16 and
integers are cast to bytes.

Parameter ``name:``:
    Name of the tensor

Parameter ``tensor:``:
    tensor

Returns:
    NNData&: reference to this object)doc";

static const char *__doc_dai_NNData_addTensor_19 =
R"doc(Add a tensor to this NNData object. The storage order is picked based on the
number of dimensions of the tensor. Float values are converted to FP16 and
integers are cast to bytes.

Parameter ``name:``:
    Name of the tensor

Parameter ``tensor:``:
    tensor

Parameter ``order:``:
    Storage order of the tensor

Returns:
    NNData&: reference to this object)doc";

static const char *__doc_dai_NNData_batchSize = R"doc()doc";

static const char *__doc_dai_NNData_changeStorageOrder = R"doc()doc";

static const char *__doc_dai_NNData_emplaceTensor =
R"doc(Emplace a tensor This function allocates memory for the tensor and return over
the said memory. It is up to the caller to fill the memory out with meaningful
data.

Returns:
    Span over the allocated memory)doc";

static const char *__doc_dai_NNData_fp16_to_fp32 = R"doc()doc";

static const char *__doc_dai_NNData_fp32_to_fp16 = R"doc()doc";

static const char *__doc_dai_NNData_getAllLayerNames =
R"doc(Returns:
    Names of all layers added)doc";

static const char *__doc_dai_NNData_getAllLayers =
R"doc(Returns:
    All layers and their information)doc";

static const char *__doc_dai_NNData_getFirstTensor =
R"doc(Convenience function to retrieve values from the first tensor

Returns:
    xt::xarray<_Ty> tensor)doc";

static const char *__doc_dai_NNData_getFirstTensor_2 =
R"doc(Convenience function to retrieve values from the first tensor

Returns:
    xt::xarray<_Ty> tensor)doc";

static const char *__doc_dai_NNData_getFirstTensorDatatype =
R"doc(Get the datatype of the first tensor

Returns:
    TensorInfo::DataType tensor datatype)doc";

static const char *__doc_dai_NNData_getLayer =
R"doc(Retrieve layers tensor information

Parameter ``name``:
    Name of the layer

Parameter ``tensor``:
    Outputs tensor information of that layer

Returns:
    True if layer exists, false otherwise)doc";

static const char *__doc_dai_NNData_getLayerDatatype =
R"doc(Retrieve datatype of a layers tensor

Parameter ``name``:
    Name of the layer

Parameter ``datatype``:
    Datatype of layers tensor

Returns:
    True if layer exists, false otherwise)doc";

static const char *__doc_dai_NNData_getTensor =
R"doc(Convenience function to retrieve values from a tensor

Returns:
    xt::xarray<_Ty> tensor)doc";

static const char *__doc_dai_NNData_getTensor_2 =
R"doc(Convenience function to retrieve values from a tensor

Returns:
    xt::xarray<_Ty> tensor)doc";

static const char *__doc_dai_NNData_getTensorDatatype =
R"doc(Get the datatype of a given tensor

Returns:
    TensorInfo::DataType tensor datatype)doc";

static const char *__doc_dai_NNData_getTensorInfo =
R"doc(Retrieve tensor information

Parameter ``name``:
    Name of the tensor

Returns:
    Tensor information)doc";

static const char *__doc_dai_NNData_hasLayer =
R"doc(Checks if given layer exists

Parameter ``name``:
    Name of the layer

Returns:
    True if layer exists, false otherwise)doc";

static const char *__doc_dai_NNData_serialize = R"doc()doc";

static const char *__doc_dai_NNData_str = R"doc()doc";

static const char *__doc_dai_NNData_tensors = R"doc()doc";

static const char *__doc_dai_NNData_transformation = R"doc()doc";

static const char *__doc_dai_NNModelDescription = R"doc()doc";

static const char *__doc_dai_NNModelDescription_check =
R"doc(Check if the model description is valid (contains all required fields)

Returns:
    bool: True if the model description is valid, false otherwise)doc";

static const char *__doc_dai_NNModelDescription_compressionLevel = R"doc(Compression level = OPTIONAL parameter)doc";

static const char *__doc_dai_NNModelDescription_fromYamlFile =
R"doc(Initialize NNModelDescription from yaml file If modelName is a relative path
(e.g. ./yolo.yaml), it is used as is. If modelName is a full path (e.g.
/home/user/models/yolo.yaml), it is used as is. If modelName is a model name
(e.g. yolo) or a model yaml file (e.g. yolo.yaml), the function will use
modelsPath if provided or the DEPTHAI_ZOO_MODELS_PATH environment variable and
use a path made by combining the modelsPath and the model name to the yaml file.
For instance, yolo -> ./depthai_models/yolo.yaml (if modelsPath or
DEPTHAI_ZOO_MODELS_PATH are ./depthai_models)

Parameter ``modelName:``:
    model name or yaml file path (string is implicitly converted to Path)

Parameter ``modelsPath:``:
    Path to the models folder, use environment variable DEPTHAI_ZOO_MODELS_PATH
    if not provided

Returns:
    NNModelDescription)doc";

static const char *__doc_dai_NNModelDescription_globalMetadataEntryName = R"doc(Name of the entry in the global metadata file)doc";

static const char *__doc_dai_NNModelDescription_model = R"doc(Model slug = REQUIRED parameter)doc";

static const char *__doc_dai_NNModelDescription_modelPrecisionType = R"doc(modelPrecisionType = OPTIONAL parameter)doc";

static const char *__doc_dai_NNModelDescription_optimizationLevel = R"doc(Optimization level = OPTIONAL parameter)doc";

static const char *__doc_dai_NNModelDescription_platform = R"doc(Hardware platform - RVC2, RVC3, RVC4, ... = REQUIRED parameter)doc";

static const char *__doc_dai_NNModelDescription_saveToYamlFile =
R"doc(Save NNModelDescription to yaml file

Parameter ``yamlPath:``:
    Path to yaml file)doc";

static const char *__doc_dai_NNModelDescription_snpeVersion = R"doc(SNPE version = OPTIONAL parameter)doc";

static const char *__doc_dai_NNModelDescription_toString =
R"doc(Convert NNModelDescription to string for printing purposes. This can be used for
debugging.

Returns:
    std::string: String representation)doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_2 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_3 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_4 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_5 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_6 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_7 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_8 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_9 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_10 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_11 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_12 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_13 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_14 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_15 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_16 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_17 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_18 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_19 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_20 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_21 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_22 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_23 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_24 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_25 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_26 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_27 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_28 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_29 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_30 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_31 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_32 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_33 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_34 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_35 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_36 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_37 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_38 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_39 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_40 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_41 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_42 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_43 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_44 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_45 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_46 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_47 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_48 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_49 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_50 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_51 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_52 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_53 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_54 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_55 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_56 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_57 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_58 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_59 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_60 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_61 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_62 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_63 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_64 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_65 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_66 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_67 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_68 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_69 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_70 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_71 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_72 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_73 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_74 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_75 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_76 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_77 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_78 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_79 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_80 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_81 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_82 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_83 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_84 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_85 = R"doc()doc";

static const char *__doc_dai_NOP_EXTERNAL_STRUCTURE_86 = R"doc()doc";

static const char *__doc_dai_NeuralNetworkProperties = R"doc(Specify properties for NeuralNetwork such as blob path, ...)doc";

static const char *__doc_dai_NeuralNetworkProperties_ModelSource = R"doc(Specify where the node should source the model)doc";

static const char *__doc_dai_NeuralNetworkProperties_ModelSource_BLOB = R"doc()doc";

static const char *__doc_dai_NeuralNetworkProperties_ModelSource_CUSTOM_MODEL = R"doc()doc";

static const char *__doc_dai_NeuralNetworkProperties_backend = R"doc(Specify which backend is used. "" = auto)doc";

static const char *__doc_dai_NeuralNetworkProperties_backendProperties = R"doc(Specify backend properties)doc";

static const char *__doc_dai_NeuralNetworkProperties_blobSize = R"doc(Blob binary size in bytes)doc";

static const char *__doc_dai_NeuralNetworkProperties_blobUri = R"doc(Uri which points to blob)doc";

static const char *__doc_dai_NeuralNetworkProperties_modelSource = R"doc()doc";

static const char *__doc_dai_NeuralNetworkProperties_modelUri = R"doc(Uri which points to the model description)doc";

static const char *__doc_dai_NeuralNetworkProperties_numFrames = R"doc(Number of available output tensors in pool)doc";

static const char *__doc_dai_NeuralNetworkProperties_numNCEPerThread = R"doc(Number of NCE (Neural Compute Engine) per inference thread. 0 = auto)doc";

static const char *__doc_dai_NeuralNetworkProperties_numShavesPerThread = R"doc(Number of Shaves per inference thread. 0 = auto)doc";

static const char *__doc_dai_NeuralNetworkProperties_numThreads = R"doc(Number of threads to create for running inference. 0 = auto)doc";

static const char *__doc_dai_Node = R"doc(Abstract Node)doc";

static const char *__doc_dai_NodeCRTP = R"doc()doc";

static const char *__doc_dai_NodeCRTP_create = R"doc()doc";

static const char *__doc_dai_NodeCRTP_create_2 = R"doc()doc";

static const char *__doc_dai_NodeCRTP_getName = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema = R"doc(Specifies a connection between nodes IOs)doc";

static const char *__doc_dai_NodeConnectionSchema_node1Id = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_node1Output = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_node1OutputGroup = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_node2Id = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_node2Input = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_node2InputGroup = R"doc()doc";

static const char *__doc_dai_NodeConnectionSchema_operator_eq = R"doc()doc";

static const char *__doc_dai_NodeGroup = R"doc()doc";

static const char *__doc_dai_NodeGroup_NodeGroup = R"doc()doc";

static const char *__doc_dai_NodeGroup_getName = R"doc()doc";

static const char *__doc_dai_NodeIoInfo = R"doc(NodeIo informations such as name, type, ...)doc";

static const char *__doc_dai_NodeIoInfo_Type = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_Type_MReceiver = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_Type_MSender = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_Type_SReceiver = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_Type_SSender = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_blocking = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_group = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_id = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_name = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_queueSize = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_type = R"doc()doc";

static const char *__doc_dai_NodeIoInfo_waitForMessage = R"doc()doc";

static const char *__doc_dai_NodeObjInfo = R"doc(NodeObj information structure)doc";

static const char *__doc_dai_NodeObjInfo_IoInfoKey = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_IoInfoKey_operator_call = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_alias = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_id = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_ioInfo = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_logLevel = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_name = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_parentId = R"doc()doc";

static const char *__doc_dai_NodeObjInfo_properties = R"doc()doc";

static const char *__doc_dai_NodeRecordParams = R"doc()doc";

static const char *__doc_dai_NodeRecordParams_name = R"doc()doc";

static const char *__doc_dai_NodeRecordParams_video = R"doc()doc";

static const char *__doc_dai_Node_Connection = R"doc(Connection between an Input and Output)doc";

static const char *__doc_dai_Node_Connection_2 = R"doc(Connection between an Input and Output)doc";

static const char *__doc_dai_Node_ConnectionInternal = R"doc(Connection between an Input and Output internal)doc";

static const char *__doc_dai_Node_ConnectionInternal_2 = R"doc(Connection between an Input and Output internal)doc";

static const char *__doc_dai_Node_ConnectionInternal_ConnectionInternal = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_Hash = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_Hash_operator_call = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_in = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_inputGroup = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_inputName = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_inputNode = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_operator_eq = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_out = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_outputGroup = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_outputName = R"doc()doc";

static const char *__doc_dai_Node_ConnectionInternal_outputNode = R"doc()doc";

static const char *__doc_dai_Node_Connection_Connection = R"doc()doc";

static const char *__doc_dai_Node_Connection_Connection_2 = R"doc()doc";

static const char *__doc_dai_Node_Connection_inputGroup = R"doc()doc";

static const char *__doc_dai_Node_Connection_inputId = R"doc()doc";

static const char *__doc_dai_Node_Connection_inputName = R"doc()doc";

static const char *__doc_dai_Node_Connection_operator_eq = R"doc()doc";

static const char *__doc_dai_Node_Connection_outputGroup = R"doc()doc";

static const char *__doc_dai_Node_Connection_outputId = R"doc()doc";

static const char *__doc_dai_Node_Connection_outputName = R"doc()doc";

static const char *__doc_dai_Node_DatatypeHierarchy = R"doc()doc";

static const char *__doc_dai_Node_DatatypeHierarchy_DatatypeHierarchy = R"doc()doc";

static const char *__doc_dai_Node_DatatypeHierarchy_datatype = R"doc()doc";

static const char *__doc_dai_Node_DatatypeHierarchy_descendants = R"doc()doc";

static const char *__doc_dai_Node_Input = R"doc()doc";

static const char *__doc_dai_Node_Input_2 = R"doc()doc";

static const char *__doc_dai_Node_InputDescription = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_blocking = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_group = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_name = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_queueSize = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_types = R"doc()doc";

static const char *__doc_dai_Node_InputDescription_waitForMessage = R"doc()doc";

static const char *__doc_dai_Node_InputMap =
R"doc(Input map which keeps track of inputs assigned to a node Extends
std::unordered_map<std::string, dai::Node::Input>)doc";

static const char *__doc_dai_Node_InputMap_2 =
R"doc(Input map which keeps track of inputs assigned to a node Extends
std::unordered_map<std::string, dai::Node::Input>)doc";

static const char *__doc_dai_Node_InputMap_InputMap = R"doc()doc";

static const char *__doc_dai_Node_InputMap_InputMap_2 = R"doc()doc";

static const char *__doc_dai_Node_InputMap_defaultInput = R"doc()doc";

static const char *__doc_dai_Node_InputMap_has = R"doc()doc";

static const char *__doc_dai_Node_InputMap_name = R"doc()doc";

static const char *__doc_dai_Node_InputMap_operator_array = R"doc(Create or modify an input)doc";

static const char *__doc_dai_Node_InputMap_operator_array_2 = R"doc(Create or modify an input with specified group)doc";

static const char *__doc_dai_Node_InputMap_parent = R"doc()doc";

static const char *__doc_dai_Node_Input_Input = R"doc()doc";

static const char *__doc_dai_Node_Input_Type = R"doc()doc";

static const char *__doc_dai_Node_Input_Type_MReceiver = R"doc()doc";

static const char *__doc_dai_Node_Input_Type_SReceiver = R"doc()doc";

static const char *__doc_dai_Node_Input_connectedOutputs = R"doc()doc";

static const char *__doc_dai_Node_Input_createInputQueue =
R"doc(Create an shared pointer to an input queue that can be used to send messages to
this input from onhost

Parameter ``maxSize:``:
    Maximum size of the input queue

Parameter ``blocking:``:
    Whether the input queue should block when full

Returns:
    std::shared_ptr<InputQueue>: shared pointer to an input queue)doc";

static const char *__doc_dai_Node_Input_getGroup = R"doc(Get group name for this input)doc";

static const char *__doc_dai_Node_Input_getParent = R"doc(Get the parent node)doc";

static const char *__doc_dai_Node_Input_getParent_2 = R"doc(Get the parent node)doc";

static const char *__doc_dai_Node_Input_getPossibleDatatypes = R"doc(Get possible datatypes that can be received)doc";

static const char *__doc_dai_Node_Input_getReusePreviousMessage = R"doc(Equivalent to getWaitForMessage but with inverted logic.)doc";

static const char *__doc_dai_Node_Input_getType = R"doc(Get type)doc";

static const char *__doc_dai_Node_Input_getWaitForMessage =
R"doc(Get behavior whether to wait for this input when a Node processes certain data
or not

Returns:
    Whether to wait for message to arrive to this input or not)doc";

static const char *__doc_dai_Node_Input_group = R"doc()doc";

static const char *__doc_dai_Node_Input_isConnected = R"doc(Check if this input is connected)doc";

static const char *__doc_dai_Node_Input_parent = R"doc()doc";

static const char *__doc_dai_Node_Input_possibleDatatypes = R"doc()doc";

static const char *__doc_dai_Node_Input_setGroup = R"doc(Set group name for this input)doc";

static const char *__doc_dai_Node_Input_setPossibleDatatypes = R"doc(Set possible datatypes that can be received)doc";

static const char *__doc_dai_Node_Input_setReusePreviousMessage = R"doc(Equivalent to setWaitForMessage but with inverted logic.)doc";

static const char *__doc_dai_Node_Input_setWaitForMessage =
R"doc(Overrides default wait for message behavior. Applicable for nodes with multiple
inputs. Specifies behavior whether to wait for this input when a Node processes
certain data or not.

Parameter ``waitForMessage``:
    Whether to wait for message to arrive to this input or not)doc";

static const char *__doc_dai_Node_Input_toString = R"doc(Input to string representation)doc";

static const char *__doc_dai_Node_Input_type = R"doc()doc";

static const char *__doc_dai_Node_Input_waitForMessage = R"doc()doc";

static const char *__doc_dai_Node_Node = R"doc()doc";

static const char *__doc_dai_Node_Node_2 = R"doc()doc";

static const char *__doc_dai_Node_Node_3 = R"doc()doc";

static const char *__doc_dai_Node_Node_4 = R"doc()doc";

static const char *__doc_dai_Node_Output = R"doc()doc";

static const char *__doc_dai_Node_Output_2 = R"doc()doc";

static const char *__doc_dai_Node_OutputDescription = R"doc()doc";

static const char *__doc_dai_Node_OutputDescription_group = R"doc()doc";

static const char *__doc_dai_Node_OutputDescription_name = R"doc()doc";

static const char *__doc_dai_Node_OutputDescription_types = R"doc()doc";

static const char *__doc_dai_Node_OutputMap =
R"doc(Output map which keeps track of extra outputs assigned to a node Extends
std::unordered_map<std::string, dai::Node::Output>)doc";

static const char *__doc_dai_Node_OutputMap_2 =
R"doc(Output map which keeps track of extra outputs assigned to a node Extends
std::unordered_map<std::string, dai::Node::Output>)doc";

static const char *__doc_dai_Node_OutputMap_OutputMap = R"doc()doc";

static const char *__doc_dai_Node_OutputMap_OutputMap_2 = R"doc()doc";

static const char *__doc_dai_Node_OutputMap_defaultOutput = R"doc()doc";

static const char *__doc_dai_Node_OutputMap_name = R"doc()doc";

static const char *__doc_dai_Node_OutputMap_operator_array = R"doc(Create or modify an output)doc";

static const char *__doc_dai_Node_OutputMap_operator_array_2 = R"doc(Create or modify an output with specified group)doc";

static const char *__doc_dai_Node_OutputMap_parent = R"doc()doc";

static const char *__doc_dai_Node_Output_Output = R"doc()doc";

static const char *__doc_dai_Node_Output_QueueConnection = R"doc()doc";

static const char *__doc_dai_Node_Output_QueueConnection_operator_eq = R"doc()doc";

static const char *__doc_dai_Node_Output_QueueConnection_output = R"doc()doc";

static const char *__doc_dai_Node_Output_QueueConnection_queue = R"doc()doc";

static const char *__doc_dai_Node_Output_Type = R"doc()doc";

static const char *__doc_dai_Node_Output_Type_MSender = R"doc()doc";

static const char *__doc_dai_Node_Output_Type_SSender = R"doc()doc";

static const char *__doc_dai_Node_Output_canConnect =
R"doc(Check if connection is possible

Parameter ``in``:
    Input to connect to

Returns:
    True if connection is possible, false otherwise)doc";

static const char *__doc_dai_Node_Output_connectedInputs = R"doc()doc";

static const char *__doc_dai_Node_Output_createOutputQueue =
R"doc(Construct and return a shared pointer to an output message queue

Parameter ``maxSize:``:
    Maximum size of the output queue

Parameter ``blocking:``:
    Whether the output queue should block when full

Returns:
    std::shared_ptr<dai::MessageQueue>: shared pointer to an output queue)doc";

static const char *__doc_dai_Node_Output_desc = R"doc()doc";

static const char *__doc_dai_Node_Output_getConnections =
R"doc(Retrieve all connections from this output

Returns:
    Vector of connections)doc";

static const char *__doc_dai_Node_Output_getGroup = R"doc(Get group of the output)doc";

static const char *__doc_dai_Node_Output_getName = R"doc(Get name of the output)doc";

static const char *__doc_dai_Node_Output_getParent = R"doc()doc";

static const char *__doc_dai_Node_Output_getParent_2 = R"doc()doc";

static const char *__doc_dai_Node_Output_getPossibleDatatypes = R"doc(Get possible datatypes that can be sent)doc";

static const char *__doc_dai_Node_Output_getQueueConnections =
R"doc(Retrieve all queue connections from this output

Returns:
    Vector of queue connections)doc";

static const char *__doc_dai_Node_Output_getType = R"doc(Get type of the output)doc";

static const char *__doc_dai_Node_Output_isSamePipeline =
R"doc(Check if this output and given input are on the same pipeline.

See also:
    canConnect for checking if connection is possible

Returns:
    True if output and input are on the same pipeline)doc";

static const char *__doc_dai_Node_Output_link = R"doc()doc";

static const char *__doc_dai_Node_Output_link_2 =
R"doc(Link current output to input.

Throws an error if this output cannot be linked to given input, or if they are
already linked

Parameter ``in``:
    Input to link to)doc";

static const char *__doc_dai_Node_Output_link_3 = R"doc()doc";

static const char *__doc_dai_Node_Output_parent = R"doc()doc";

static const char *__doc_dai_Node_Output_queueConnections = R"doc()doc";

static const char *__doc_dai_Node_Output_send =
R"doc(Sends a Message to all connected inputs

Parameter ``msg``:
    Message to send to all connected inputs)doc";

static const char *__doc_dai_Node_Output_setGroup = R"doc(Set group name for this output)doc";

static const char *__doc_dai_Node_Output_setName = R"doc(Set name for this output)doc";

static const char *__doc_dai_Node_Output_setPossibleDatatypes = R"doc(Set possible datatypes that can be sent)doc";

static const char *__doc_dai_Node_Output_toString = R"doc(Output to string representation)doc";

static const char *__doc_dai_Node_Output_trySend =
R"doc(Try sending a message to all connected inputs

Parameter ``msg``:
    Message to send to all connected inputs

Returns:
    True if ALL connected inputs got the message, false otherwise)doc";

static const char *__doc_dai_Node_Output_type = R"doc()doc";

static const char *__doc_dai_Node_Output_unlink = R"doc()doc";

static const char *__doc_dai_Node_Output_unlink_2 =
R"doc(Unlink a previously linked connection

Throws an error if not linked.

Parameter ``in``:
    Input from which to unlink from)doc";

static const char *__doc_dai_Node_PairHash = R"doc()doc";

static const char *__doc_dai_Node_PairHash_operator_call = R"doc()doc";

static const char *__doc_dai_Node_add = R"doc(Add existing node to nodeMap)doc";

static const char *__doc_dai_Node_alias = R"doc(alias or name)doc";

static const char *__doc_dai_Node_assetManager = R"doc()doc";

static const char *__doc_dai_Node_buildInternal =
R"doc(Function called from within the `create` function to build the node. This
function is useful for initialization, setting up inputs and outputs = stuff
that cannot be perform in the constuctor.)doc";

static const char *__doc_dai_Node_buildStage1 = R"doc(Build stages;)doc";

static const char *__doc_dai_Node_buildStage2 = R"doc()doc";

static const char *__doc_dai_Node_buildStage3 = R"doc()doc";

static const char *__doc_dai_Node_configureMode = R"doc()doc";

static const char *__doc_dai_Node_connections = R"doc()doc";

static const char *__doc_dai_Node_create = R"doc(Create and place Node to this Node)doc";

static const char *__doc_dai_Node_createUniqueInputName = R"doc()doc";

static const char *__doc_dai_Node_createUniqueOutputName = R"doc()doc";

static const char *__doc_dai_Node_getAlias = R"doc(Get alias)doc";

static const char *__doc_dai_Node_getAllNodes = R"doc()doc";

static const char *__doc_dai_Node_getAssetManager = R"doc(Get node AssetManager as a const reference)doc";

static const char *__doc_dai_Node_getAssetManager_2 = R"doc(Get node AssetManager as a reference)doc";

static const char *__doc_dai_Node_getConnectionMap = R"doc()doc";

static const char *__doc_dai_Node_getInputMapRef = R"doc(Retrieves reference to specific input map)doc";

static const char *__doc_dai_Node_getInputMapRefs = R"doc(Retrieves reference to node inputs)doc";

static const char *__doc_dai_Node_getInputRef = R"doc(Retrieves reference to specific input)doc";

static const char *__doc_dai_Node_getInputRef_2 = R"doc()doc";

static const char *__doc_dai_Node_getInputRefs = R"doc(Retrieves reference to node inputs)doc";

static const char *__doc_dai_Node_getInputRefs_2 = R"doc(Retrieves reference to node inputs)doc";

static const char *__doc_dai_Node_getInputs = R"doc(Retrieves all nodes inputs)doc";

static const char *__doc_dai_Node_getName = R"doc(Retrieves nodes name)doc";

static const char *__doc_dai_Node_getNode = R"doc()doc";

static const char *__doc_dai_Node_getNode_2 = R"doc()doc";

static const char *__doc_dai_Node_getNodeMap = R"doc()doc";

static const char *__doc_dai_Node_getOutputMapRef = R"doc(Retrieves reference to specific output map)doc";

static const char *__doc_dai_Node_getOutputMapRefs = R"doc(Retrieves reference to node outputs)doc";

static const char *__doc_dai_Node_getOutputRef = R"doc(Retrieves reference to specific output)doc";

static const char *__doc_dai_Node_getOutputRef_2 = R"doc()doc";

static const char *__doc_dai_Node_getOutputRefs = R"doc(Retrieves reference to node outputs)doc";

static const char *__doc_dai_Node_getOutputRefs_2 = R"doc(Retrieves reference to node outputs)doc";

static const char *__doc_dai_Node_getOutputs = R"doc(Retrieves all nodes outputs)doc";

static const char *__doc_dai_Node_getParentPipeline = R"doc()doc";

static const char *__doc_dai_Node_getParentPipeline_2 = R"doc()doc";

static const char *__doc_dai_Node_getRequiredInputs = R"doc()doc";

static const char *__doc_dai_Node_id = R"doc(Id of node. Assigned after being placed on the pipeline)doc";

static const char *__doc_dai_Node_inputId = R"doc()doc";

static const char *__doc_dai_Node_inputMapRefs = R"doc()doc";

static const char *__doc_dai_Node_inputRefs = R"doc()doc";

static const char *__doc_dai_Node_isSourceNode = R"doc()doc";

static const char *__doc_dai_Node_link = R"doc()doc";

static const char *__doc_dai_Node_link_2 = R"doc(Get a reference to internal node map)doc";

static const char *__doc_dai_Node_loadResource = R"doc(Loads resource specified by URI and returns its data)doc";

static const char *__doc_dai_Node_moveResource = R"doc(Moves the resource out)doc";

static const char *__doc_dai_Node_needsBuild = R"doc()doc";

static const char *__doc_dai_Node_nodeMap = R"doc()doc";

static const char *__doc_dai_Node_nodeRefs = R"doc()doc";

static const char *__doc_dai_Node_operator_assign = R"doc()doc";

static const char *__doc_dai_Node_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_Node_outputId = R"doc()doc";

static const char *__doc_dai_Node_outputMapRefs = R"doc()doc";

static const char *__doc_dai_Node_outputRefs = R"doc()doc";

static const char *__doc_dai_Node_parent = R"doc()doc";

static const char *__doc_dai_Node_parentId = R"doc()doc";

static const char *__doc_dai_Node_remove = R"doc()doc";

static const char *__doc_dai_Node_removeConnectionToNode = R"doc()doc";

static const char *__doc_dai_Node_requestOutput = R"doc()doc";

static const char *__doc_dai_Node_runOnHost = R"doc(Returns true or false whether the node should be run on host or not)doc";

static const char *__doc_dai_Node_setAlias = R"doc(Set alias)doc";

static const char *__doc_dai_Node_setInputMapRefs = R"doc()doc";

static const char *__doc_dai_Node_setInputMapRefs_2 = R"doc()doc";

static const char *__doc_dai_Node_setInputRefs = R"doc()doc";

static const char *__doc_dai_Node_setInputRefs_2 = R"doc()doc";

static const char *__doc_dai_Node_setNodeRefs = R"doc()doc";

static const char *__doc_dai_Node_setNodeRefs_2 = R"doc()doc";

static const char *__doc_dai_Node_setNodeRefs_3 = R"doc()doc";

static const char *__doc_dai_Node_setOutputMapRefs = R"doc()doc";

static const char *__doc_dai_Node_setOutputMapRefs_2 = R"doc()doc";

static const char *__doc_dai_Node_setOutputRefs = R"doc()doc";

static const char *__doc_dai_Node_setOutputRefs_2 = R"doc()doc";

static const char *__doc_dai_Node_start = R"doc(Start node execution)doc";

static const char *__doc_dai_Node_stop = R"doc(Stop node execution)doc";

static const char *__doc_dai_Node_stopPipeline = R"doc()doc";

static const char *__doc_dai_Node_uniqueNames = R"doc()doc";

static const char *__doc_dai_Node_unlink = R"doc()doc";

static const char *__doc_dai_Node_wait = R"doc(Wait for node to finish execution)doc";

static const char *__doc_dai_ObjectTrackerConfig =
R"doc(ObjectTrackerConfig message. Carries ROI (region of interest) and threshold for
depth calculation)doc";

static const char *__doc_dai_ObjectTrackerConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ObjectTrackerConfig_ObjectTrackerConfig = R"doc(Construct ObjectTrackerConfig message.)doc";

static const char *__doc_dai_ObjectTrackerConfig_forceRemoveID = R"doc(Force remove a tracklet with specified ID.)doc";

static const char *__doc_dai_ObjectTrackerConfig_forceRemoveIDs = R"doc(Force remove tracklets with specified IDs.)doc";

static const char *__doc_dai_ObjectTrackerConfig_serialize = R"doc()doc";

static const char *__doc_dai_ObjectTrackerConfig_str = R"doc()doc";

static const char *__doc_dai_ObjectTrackerConfig_trackletIdsToRemove = R"doc(Tracklet IDs to remove from tracking. Tracklet will transition to REMOVED state.)doc";

static const char *__doc_dai_ObjectTrackerProperties = R"doc(Specify properties for ObjectTracker)doc";

static const char *__doc_dai_ObjectTrackerProperties_detectionLabelsToTrack = R"doc(Which detections labels to track. Default all labels are tracked.)doc";

static const char *__doc_dai_ObjectTrackerProperties_maxObjectsToTrack =
R"doc(Maximum number of objects to track. Maximum 60 for SHORT_TERM_KCF, maximum 1000
for other tracking methods. Default 60.)doc";

static const char *__doc_dai_ObjectTrackerProperties_occlusionRatioThreshold = R"doc(Occlusion ratio threshold. Used to filter out overlapping tracklets.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackerIdAssignmentPolicy = R"doc(New ID assignment policy.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackerThreshold =
R"doc(Confidence threshold for tracklets. Above this threshold detections will be
tracked. Default 0, all detections are tracked.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackerType = R"doc(Tracking method.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackingPerClass = R"doc(Whether tracker should take into consideration class label for tracking.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackletBirthThreshold =
R"doc(Tracklet birth threshold. Minimum consecutive tracked frames required to
consider a tracklet as a new instance.)doc";

static const char *__doc_dai_ObjectTrackerProperties_trackletMaxLifespan =
R"doc(Tracklet lifespan in number of frames. Number of frames after which a LOST
tracklet is removed.)doc";

static const char *__doc_dai_OpBase = R"doc()doc";

static const char *__doc_dai_OpBase_toStr = R"doc()doc";

static const char *__doc_dai_OpenVINO =
R"doc(Support for basic OpenVINO related actions like version identification of neural
network blobs,...)doc";

static const char *__doc_dai_OpenVINO_Blob = R"doc(OpenVINO Blob)doc";

static const char *__doc_dai_OpenVINO_Blob_Blob =
R"doc(Construct a new Blob from data in memory

Parameter ``data``:
    In memory blob)doc";

static const char *__doc_dai_OpenVINO_Blob_Blob_2 =
R"doc(Construct a new Blob by loading from a filesystem path

Parameter ``path``:
    Filesystem path to the blob)doc";

static const char *__doc_dai_OpenVINO_Blob_data = R"doc(Blob data)doc";

static const char *__doc_dai_OpenVINO_Blob_device = R"doc(Device for which the blob is compiled for)doc";

static const char *__doc_dai_OpenVINO_Blob_networkInputs = R"doc(Map of input names to additional information)doc";

static const char *__doc_dai_OpenVINO_Blob_networkOutputs = R"doc(Map of output names to additional information)doc";

static const char *__doc_dai_OpenVINO_Blob_numShaves = R"doc(Number of shaves the blob was compiled for)doc";

static const char *__doc_dai_OpenVINO_Blob_numSlices = R"doc(Number of CMX slices the blob was compiled for)doc";

static const char *__doc_dai_OpenVINO_Blob_stageCount = R"doc(Number of network stages)doc";

static const char *__doc_dai_OpenVINO_Blob_version = R"doc(OpenVINO version)doc";

static const char *__doc_dai_OpenVINO_Device = R"doc()doc";

static const char *__doc_dai_OpenVINO_Device_VPU = R"doc()doc";

static const char *__doc_dai_OpenVINO_Device_VPUX = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob =
R"doc(A superblob is an efficient way of storing generated blobs for all different
number of shaves.)doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlob =
R"doc(Construct a new SuperBlob object

Parameter ``data:``:
    In memory superblob data)doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlob_2 =
R"doc(Construct a new SuperBlob object

Parameter ``pathToSuperBlobFile:``:
    Path to the superblob file (.superblob suffix))doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlobHeader = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlobHeader_blobSize = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlobHeader_fromData = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_SuperBlobHeader_patchSizes = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_data = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_getBlobDataPointer = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_getBlobDataSize = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_getBlobWithNumShaves =
R"doc(Generate a blob with a specific number of shaves

Parameter ``numShaves:``:
    Number of shaves to generate the blob for. Must be between 1 and
    NUMBER_OF_PATCHES.

Returns:
    dai::OpenVINO::Blob: Blob compiled for the specified number of shaves)doc";

static const char *__doc_dai_OpenVINO_SuperBlob_getPatchDataPointer = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_getPatchDataSize = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_header = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_loadAndCheckHeader = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_readSuperBlobFile = R"doc()doc";

static const char *__doc_dai_OpenVINO_SuperBlob_validateSuperblob = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version = R"doc(OpenVINO Version supported version information)doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2020_3 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2020_4 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2021_1 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2021_2 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2021_3 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2021_4 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_2022_1 = R"doc()doc";

static const char *__doc_dai_OpenVINO_Version_VERSION_UNIVERSAL = R"doc()doc";

static const char *__doc_dai_OpenVINO_areVersionsBlobCompatible = R"doc(Checks whether two blob versions are compatible)doc";

static const char *__doc_dai_OpenVINO_getBlobLatestSupportedVersion =
R"doc(Returns latest potentially supported version by a given blob version.

Parameter ``majorVersion``:
    Major version from OpenVINO blob

Parameter ``minorVersion``:
    Minor version from OpenVINO blob

Returns:
    Latest potentially supported version)doc";

static const char *__doc_dai_OpenVINO_getBlobSupportedVersions =
R"doc(Returns a list of potentially supported versions for a specified blob major and
minor versions.

Parameter ``majorVersion``:
    Major version from OpenVINO blob

Parameter ``minorVersion``:
    Minor version from OpenVINO blob

Returns:
    Vector of potentially supported versions)doc";

static const char *__doc_dai_OpenVINO_getBlobVersion =
R"doc(Returns OpenVINO version of a given blob minor/major revision.

Parameter ``majorVersion``:
    Major version from OpenVINO blob

Parameter ``minorVersion``:
    Minor version from OpenVINO blob

Returns:
    Latest potentially supported version)doc";

static const char *__doc_dai_OpenVINO_getVersionName =
R"doc(Returns string representation of a given version

Parameter ``version``:
    OpenVINO version

Returns:
    Name of a given version)doc";

static const char *__doc_dai_OpenVINO_getVersions =
R"doc(Returns:
    Supported versions)doc";

static const char *__doc_dai_OpenVINO_parseVersionName =
R"doc(Creates Version from string representation. Throws if not possible.

Parameter ``versionString``:
    Version as string

Returns:
    Version object if successful)doc";

static const char *__doc_dai_Perspective = R"doc()doc";

static const char *__doc_dai_Perspective_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Perspective_Perspective = R"doc()doc";

static const char *__doc_dai_Perspective_Perspective_2 = R"doc()doc";

static const char *__doc_dai_Perspective_matrix = R"doc()doc";

static const char *__doc_dai_Perspective_str = R"doc()doc";

static const char *__doc_dai_Perspective_toStr = R"doc()doc";

static const char *__doc_dai_Pimpl = R"doc()doc";

static const char *__doc_dai_Pimpl_Pimpl = R"doc()doc";

static const char *__doc_dai_Pimpl_Pimpl_2 = R"doc()doc";

static const char *__doc_dai_Pimpl_m = R"doc()doc";

static const char *__doc_dai_Pimpl_operator_mul = R"doc()doc";

static const char *__doc_dai_Pimpl_operator_sub = R"doc()doc";

static const char *__doc_dai_Pipeline = R"doc()doc";

static const char *__doc_dai_Pipeline_2 = R"doc()doc";

static const char *__doc_dai_Pipeline_3 = R"doc(Represents the pipeline, set of nodes and connections between them)doc";

static const char *__doc_dai_PipelineImpl = R"doc()doc";

static const char *__doc_dai_PipelineImpl_2 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_3 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_PipelineImpl = R"doc()doc";

static const char *__doc_dai_PipelineImpl_PipelineImpl_2 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_PipelineImpl_3 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_PipelineImpl_4 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_add = R"doc()doc";

static const char *__doc_dai_PipelineImpl_addTask = R"doc()doc";

static const char *__doc_dai_PipelineImpl_assetManager = R"doc()doc";

static const char *__doc_dai_PipelineImpl_board = R"doc()doc";

static const char *__doc_dai_PipelineImpl_build = R"doc()doc";

static const char *__doc_dai_PipelineImpl_canConnect = R"doc()doc";

static const char *__doc_dai_PipelineImpl_create = R"doc()doc";

static const char *__doc_dai_PipelineImpl_createNode = R"doc()doc";

static const char *__doc_dai_PipelineImpl_createNode_2 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_createNode_3 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_defaultDevice = R"doc()doc";

static const char *__doc_dai_PipelineImpl_defaultDeviceId = R"doc()doc";

static const char *__doc_dai_PipelineImpl_disconnectXLinkHosts = R"doc()doc";

static const char *__doc_dai_PipelineImpl_enableHolisticRecordReplay = R"doc()doc";

static const char *__doc_dai_PipelineImpl_forceRequiredOpenVINOVersion = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getAllNodes = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getBoardConfig = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getCalibrationData = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getConnectionMap = R"doc(Get a reference to internal connection representation)doc";

static const char *__doc_dai_PipelineImpl_getConnections = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getConnectionsInternal = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getDeviceConfig = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getEepromData = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getGlobalProperties = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getNextUniqueId = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getNode = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getPipelineSchema = R"doc()doc";

static const char *__doc_dai_PipelineImpl_getSourceNodes = R"doc()doc";

static const char *__doc_dai_PipelineImpl_globalProperties = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isBuild = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isBuilt = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isCalibrationDataAvailable = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isDeviceOnly = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isHostOnly = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isRunning = R"doc()doc";

static const char *__doc_dai_PipelineImpl_isSamePipeline = R"doc()doc";

static const char *__doc_dai_PipelineImpl_latestId = R"doc()doc";

static const char *__doc_dai_PipelineImpl_link = R"doc()doc";

static const char *__doc_dai_PipelineImpl_loadResource = R"doc()doc";

static const char *__doc_dai_PipelineImpl_loadResourceCwd = R"doc()doc";

static const char *__doc_dai_PipelineImpl_nodes = R"doc()doc";

static const char *__doc_dai_PipelineImpl_operator_assign = R"doc()doc";

static const char *__doc_dai_PipelineImpl_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_PipelineImpl_outputQueues = R"doc()doc";

static const char *__doc_dai_PipelineImpl_parent = R"doc()doc";

static const char *__doc_dai_PipelineImpl_processTasks = R"doc()doc";

static const char *__doc_dai_PipelineImpl_recordConfig = R"doc()doc";

static const char *__doc_dai_PipelineImpl_recordReplayFilenames = R"doc()doc";

static const char *__doc_dai_PipelineImpl_remove = R"doc()doc";

static const char *__doc_dai_PipelineImpl_removeRecordReplayFiles = R"doc()doc";

static const char *__doc_dai_PipelineImpl_resetConnections = R"doc()doc";

static const char *__doc_dai_PipelineImpl_run = R"doc()doc";

static const char *__doc_dai_PipelineImpl_running = R"doc()doc";

static const char *__doc_dai_PipelineImpl_serialize = R"doc()doc";

static const char *__doc_dai_PipelineImpl_serializeToJson = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setBoardConfig = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setCalibrationData = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setCameraTuningBlobPath = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setEepromData = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setGlobalProperties = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setSippBufferSize = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setSippDmaBufferSize = R"doc()doc";

static const char *__doc_dai_PipelineImpl_setXLinkChunkSize = R"doc()doc";

static const char *__doc_dai_PipelineImpl_start = R"doc()doc";

static const char *__doc_dai_PipelineImpl_stateMtx = R"doc()doc";

static const char *__doc_dai_PipelineImpl_stop = R"doc()doc";

static const char *__doc_dai_PipelineImpl_tasks = R"doc()doc";

static const char *__doc_dai_PipelineImpl_unlink = R"doc()doc";

static const char *__doc_dai_PipelineImpl_wait = R"doc()doc";

static const char *__doc_dai_PipelineSchema = R"doc(Specifies whole pipeline, nodes, properties and connections between nodes IOs)doc";

static const char *__doc_dai_PipelineSchema_connections = R"doc()doc";

static const char *__doc_dai_PipelineSchema_globalProperties = R"doc()doc";

static const char *__doc_dai_PipelineSchema_nodes = R"doc()doc";

static const char *__doc_dai_Pipeline_Pipeline =
R"doc(Creates a pipeline

Parameter ``createImplicitDevice``:
    If true, creates a default device (default = true))doc";

static const char *__doc_dai_Pipeline_Pipeline_2 = R"doc(Creates a pipeline with specified device)doc";

static const char *__doc_dai_Pipeline_Pipeline_3 = R"doc(Creates a pipeline with specified device)doc";

static const char *__doc_dai_Pipeline_add = R"doc(Adds an existing node to the pipeline)doc";

static const char *__doc_dai_Pipeline_addTask = R"doc()doc";

static const char *__doc_dai_Pipeline_build = R"doc()doc";

static const char *__doc_dai_Pipeline_create =
R"doc(Creates and adds a node to the pipeline.

Node is specified by template argument N)doc";

static const char *__doc_dai_Pipeline_enableHolisticRecord = R"doc(Record and Replay)doc";

static const char *__doc_dai_Pipeline_enableHolisticReplay = R"doc()doc";

static const char *__doc_dai_Pipeline_getAllNodes = R"doc(Get a vector of all nodes)doc";

static const char *__doc_dai_Pipeline_getAssetManager = R"doc(Get pipelines AssetManager as reference)doc";

static const char *__doc_dai_Pipeline_getAssetManager_2 = R"doc(Get pipelines AssetManager as reference)doc";

static const char *__doc_dai_Pipeline_getBoardConfig = R"doc(Gets board configuration)doc";

static const char *__doc_dai_Pipeline_getCalibrationData =
R"doc(gets the calibration data which is set through pipeline

Returns:
    the calibrationHandler with calib data in the pipeline)doc";

static const char *__doc_dai_Pipeline_getConnectionMap = R"doc()doc";

static const char *__doc_dai_Pipeline_getConnections = R"doc(Get all connections)doc";

static const char *__doc_dai_Pipeline_getDefaultDevice = R"doc()doc";

static const char *__doc_dai_Pipeline_getDeviceConfig = R"doc(Get device configuration needed for this pipeline)doc";

static const char *__doc_dai_Pipeline_getEepromData =
R"doc(gets the eeprom data from the pipeline

Returns:
    eepromData from the the pipeline)doc";

static const char *__doc_dai_Pipeline_getGlobalProperties =
R"doc(Returns:
    Global properties of current pipeline)doc";

static const char *__doc_dai_Pipeline_getNode = R"doc(Get node with id if it exists, nullptr otherwise)doc";

static const char *__doc_dai_Pipeline_getNode_2 = R"doc(Get node with id if it exists, nullptr otherwise)doc";

static const char *__doc_dai_Pipeline_getPipelineSchema =
R"doc(Returns:
    Pipeline schema)doc";

static const char *__doc_dai_Pipeline_getSourceNodes = R"doc()doc";

static const char *__doc_dai_Pipeline_impl = R"doc()doc";

static const char *__doc_dai_Pipeline_impl_2 = R"doc()doc";

static const char *__doc_dai_Pipeline_isBuilt = R"doc()doc";

static const char *__doc_dai_Pipeline_isCalibrationDataAvailable =
R"doc(check if calib data has been set or the default will be returned

Returns:
    true - calib data has been set

Returns:
    false - calib data has not been set - default will be returned)doc";

static const char *__doc_dai_Pipeline_isRunning = R"doc()doc";

static const char *__doc_dai_Pipeline_pimpl = R"doc()doc";

static const char *__doc_dai_Pipeline_processTasks = R"doc()doc";

static const char *__doc_dai_Pipeline_remove = R"doc(Removes a node from pipeline)doc";

static const char *__doc_dai_Pipeline_run = R"doc()doc";

static const char *__doc_dai_Pipeline_serialize = R"doc()doc";

static const char *__doc_dai_Pipeline_serializeToJson = R"doc(Returns whole pipeline represented as JSON)doc";

static const char *__doc_dai_Pipeline_setBoardConfig = R"doc(Sets board configuration)doc";

static const char *__doc_dai_Pipeline_setCalibrationData =
R"doc(Sets the calibration in pipeline which overrides the calibration data in eeprom

Parameter ``calibrationDataHandler``:
    CalibrationHandler object which is loaded with calibration information.)doc";

static const char *__doc_dai_Pipeline_setCameraTuningBlobPath = R"doc(Set a camera IQ (Image Quality) tuning blob, used for all cameras)doc";

static const char *__doc_dai_Pipeline_setEepromData =
R"doc(Sets the eeprom data in pipeline

Parameter ``eepromData``:
    EepromData object that is loaded in the pipeline.)doc";

static const char *__doc_dai_Pipeline_setGlobalProperties = R"doc(Sets global properties of pipeline)doc";

static const char *__doc_dai_Pipeline_setOpenVINOVersion = R"doc(Set a specific OpenVINO version to use with this pipeline)doc";

static const char *__doc_dai_Pipeline_setSippBufferSize =
R"doc(SIPP (Signal Image Processing Pipeline) internal memory pool. SIPP is a
framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
Changing the size of this pool is meant for advanced use cases, pushing the
limits of the HW. By default memory is allocated in high speed CMX memory.
Setting to 0 will allocate in DDR 256 kilobytes. Units are bytes.)doc";

static const char *__doc_dai_Pipeline_setSippDmaBufferSize =
R"doc(SIPP (Signal Image Processing Pipeline) internal DMA memory pool. SIPP is a
framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
Changing the size of this pool is meant for advanced use cases, pushing the
limits of the HW. Memory is allocated in high speed CMX memory Units are bytes.)doc";

static const char *__doc_dai_Pipeline_setXLinkChunkSize =
R"doc(Set chunk size for splitting device-sent XLink packets, in bytes. A larger value
could increase performance, with 0 disabling chunking. A negative value won't
modify the device defaults - configured per protocol, currently 64*1024 for both
USB and Ethernet.)doc";

static const char *__doc_dai_Pipeline_start = R"doc()doc";

static const char *__doc_dai_Pipeline_stop = R"doc()doc";

static const char *__doc_dai_Pipeline_wait = R"doc()doc";

static const char *__doc_dai_Platform = R"doc(Hardware platform type)doc";

static const char *__doc_dai_Platform_RVC2 = R"doc()doc";

static const char *__doc_dai_Platform_RVC3 = R"doc()doc";

static const char *__doc_dai_Platform_RVC4 = R"doc()doc";

static const char *__doc_dai_Point2f =
R"doc(Point2f structure

x and y coordinates that define a 2D point.)doc";

static const char *__doc_dai_Point2f_Point2f = R"doc()doc";

static const char *__doc_dai_Point2f_Point2f_2 = R"doc()doc";

static const char *__doc_dai_Point2f_Point2f_3 = R"doc()doc";

static const char *__doc_dai_Point2f_hasNormalized = R"doc()doc";

static const char *__doc_dai_Point2f_isNormalized = R"doc()doc";

static const char *__doc_dai_Point2f_normalized = R"doc()doc";

static const char *__doc_dai_Point2f_x = R"doc()doc";

static const char *__doc_dai_Point2f_y = R"doc()doc";

static const char *__doc_dai_Point3d =
R"doc(Point3d structure

x,y,z coordinates that define a 3D point.)doc";

static const char *__doc_dai_Point3d_Point3d = R"doc()doc";

static const char *__doc_dai_Point3d_Point3d_2 = R"doc()doc";

static const char *__doc_dai_Point3d_x = R"doc()doc";

static const char *__doc_dai_Point3d_y = R"doc()doc";

static const char *__doc_dai_Point3d_z = R"doc()doc";

static const char *__doc_dai_Point3f =
R"doc(Point3f structure

x,y,z coordinates that define a 3D point.)doc";

static const char *__doc_dai_Point3fRGBA =
R"doc(Point3fRGBA structure

x,y,z coordinates and RGB color values that define a 3D point with color.)doc";

static const char *__doc_dai_Point3fRGBA_Point3fRGBA = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_Point3fRGBA_2 = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_a = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_b = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_g = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_r = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_x = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_y = R"doc()doc";

static const char *__doc_dai_Point3fRGBA_z = R"doc()doc";

static const char *__doc_dai_Point3f_Point3f = R"doc()doc";

static const char *__doc_dai_Point3f_Point3f_2 = R"doc()doc";

static const char *__doc_dai_Point3f_x = R"doc()doc";

static const char *__doc_dai_Point3f_y = R"doc()doc";

static const char *__doc_dai_Point3f_z = R"doc()doc";

static const char *__doc_dai_PointCloudConfig =
R"doc(PointCloudConfig message. Carries ROI (region of interest) and threshold for
depth calculation)doc";

static const char *__doc_dai_PointCloudConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_PointCloudConfig_PointCloudConfig = R"doc(Construct PointCloudConfig message.)doc";

static const char *__doc_dai_PointCloudConfig_getSparse =
R"doc(Retrieve sparse point cloud calculation status.

Returns:
    true if sparse point cloud calculation is enabled, false otherwise)doc";

static const char *__doc_dai_PointCloudConfig_getTransformationMatrix =
R"doc(Retrieve transformation matrix for point cloud calculation.

Returns:
    4x4 transformation matrix)doc";

static const char *__doc_dai_PointCloudConfig_serialize = R"doc()doc";

static const char *__doc_dai_PointCloudConfig_setSparse =
R"doc(Enable or disable sparse point cloud calculation.

Parameter ``enable``:)doc";

static const char *__doc_dai_PointCloudConfig_setTransformationMatrix =
R"doc(Set 4x4 transformation matrix for point cloud calculation. Default is an
identity matrix.

Parameter ``transformationMatrix``:)doc";

static const char *__doc_dai_PointCloudConfig_setTransformationMatrix_2 =
R"doc(Set 3x3 transformation matrix for point cloud calculation. Default is an
identity matrix.

Parameter ``transformationMatrix``:)doc";

static const char *__doc_dai_PointCloudConfig_sparse = R"doc()doc";

static const char *__doc_dai_PointCloudConfig_str = R"doc()doc";

static const char *__doc_dai_PointCloudConfig_transformationMatrix = R"doc()doc";

static const char *__doc_dai_PointCloudData = R"doc(PointCloudData message. Carries point cloud data.)doc";

static const char *__doc_dai_PointCloudData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_PointCloudData_PointCloudData = R"doc(Construct PointCloudData message.)doc";

static const char *__doc_dai_PointCloudData_Ptr = R"doc(Converts PointCloudData to pcl::PointCloud<pcl::PointXYZ>)doc";

static const char *__doc_dai_PointCloudData_color = R"doc()doc";

static const char *__doc_dai_PointCloudData_getHeight =
R"doc(Retrieves the height in pixels - in case of a sparse point cloud, this
represents the hight of the frame which was used to generate the point cloud)doc";

static const char *__doc_dai_PointCloudData_getInstanceNum = R"doc(Retrieves instance number)doc";

static const char *__doc_dai_PointCloudData_getMaxX = R"doc(Retrieves maximal x coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getMaxY = R"doc(Retrieves maximal y coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getMaxZ = R"doc(Retrieves maximal z coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getMinX = R"doc(Retrieves minimal x coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getMinY = R"doc(Retrieves minimal y coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getMinZ = R"doc(Retrieves minimal z coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_getPoints = R"doc()doc";

static const char *__doc_dai_PointCloudData_getPointsRGB = R"doc()doc";

static const char *__doc_dai_PointCloudData_getWidth =
R"doc(Retrieves the height in pixels - in case of a sparse point cloud, this
represents the hight of the frame which was used to generate the point cloud)doc";

static const char *__doc_dai_PointCloudData_height = R"doc()doc";

static const char *__doc_dai_PointCloudData_instanceNum = R"doc()doc";

static const char *__doc_dai_PointCloudData_isColor = R"doc(Retrieves whether point cloud is color)doc";

static const char *__doc_dai_PointCloudData_isSparse = R"doc(Retrieves whether point cloud is sparse)doc";

static const char *__doc_dai_PointCloudData_maxx = R"doc()doc";

static const char *__doc_dai_PointCloudData_maxy = R"doc()doc";

static const char *__doc_dai_PointCloudData_maxz = R"doc()doc";

static const char *__doc_dai_PointCloudData_minx = R"doc()doc";

static const char *__doc_dai_PointCloudData_miny = R"doc()doc";

static const char *__doc_dai_PointCloudData_minz = R"doc()doc";

static const char *__doc_dai_PointCloudData_serialize = R"doc()doc";

static const char *__doc_dai_PointCloudData_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_PointCloudData_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_PointCloudData_setColor =
R"doc(Specifies whether point cloud is color

Parameter ``val``:
    whether point cloud is color)doc";

static const char *__doc_dai_PointCloudData_setHeight =
R"doc(Specifies frame height

Parameter ``height``:
    frame height)doc";

static const char *__doc_dai_PointCloudData_setInstanceNum =
R"doc(Specifies instance number

Parameter ``instanceNum``:
    instance number)doc";

static const char *__doc_dai_PointCloudData_setMaxX =
R"doc(Specifies maximal x coordinate in depth units (millimeter by default)

Parameter ``val``:
    maximal x coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setMaxY =
R"doc(Specifies maximal y coordinate in depth units (millimeter by default)

Parameter ``val``:
    maximal y coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setMaxZ =
R"doc(Specifies maximal z coordinate in depth units (millimeter by default)

Parameter ``val``:
    maximal z coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setMinX =
R"doc(Specifies minimal x coordinate in depth units (millimeter by default)

Parameter ``val``:
    minimal x coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setMinY =
R"doc(Specifies minimal y coordinate in depth units (millimeter by default)

Parameter ``val``:
    minimal y coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setMinZ =
R"doc(Specifies minimal z coordinate in depth units (millimeter by default)

Parameter ``val``:
    minimal z coordinate in depth units (millimeter by default))doc";

static const char *__doc_dai_PointCloudData_setPclData = R"doc()doc";

static const char *__doc_dai_PointCloudData_setPclData_2 = R"doc()doc";

static const char *__doc_dai_PointCloudData_setPclDataRGB = R"doc()doc";

static const char *__doc_dai_PointCloudData_setPoints = R"doc()doc";

static const char *__doc_dai_PointCloudData_setPointsRGB = R"doc()doc";

static const char *__doc_dai_PointCloudData_setSize =
R"doc(Specifies frame size

Parameter ``height``:
    frame height

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_PointCloudData_setSize_2 =
R"doc(Specifies frame size

Parameter ``size``:
    frame size)doc";

static const char *__doc_dai_PointCloudData_setSparse =
R"doc(Specifies whether point cloud is sparse

Parameter ``val``:
    whether point cloud is sparse)doc";

static const char *__doc_dai_PointCloudData_setWidth =
R"doc(Specifies frame width

Parameter ``width``:
    frame width)doc";

static const char *__doc_dai_PointCloudData_sparse = R"doc()doc";

static const char *__doc_dai_PointCloudData_str = R"doc()doc";

static const char *__doc_dai_PointCloudData_width = R"doc()doc";

static const char *__doc_dai_PointCloudProperties = R"doc(Specify properties for PointCloud)doc";

static const char *__doc_dai_PointCloudProperties_initialConfig = R"doc()doc";

static const char *__doc_dai_PointCloudProperties_numFramesPool = R"doc()doc";

static const char *__doc_dai_PointsAnnotation = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType_LINE_LIST = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType_LINE_LOOP = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType_LINE_STRIP = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType_POINTS = R"doc()doc";

static const char *__doc_dai_PointsAnnotationType_UNKNOWN = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_fillColor = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_outlineColor = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_outlineColors = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_points = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_thickness = R"doc()doc";

static const char *__doc_dai_PointsAnnotation_type = R"doc()doc";

static const char *__doc_dai_PoolProperties = R"doc(Specify PoolProperties options such as pool uri, pool name, ...)doc";

static const char *__doc_dai_PoolProperties_datatype = R"doc(Optional datatype of messages in the pool)doc";

static const char *__doc_dai_PoolProperties_maxMessageSize = R"doc(Size of data allocated for each message)doc";

static const char *__doc_dai_PoolProperties_numMessages = R"doc(Number of messages in pool)doc";

static const char *__doc_dai_PoolProperties_processor = R"doc(Which processor should hold the pool)doc";

static const char *__doc_dai_ProcessorType =
R"doc(On which processor the node will be placed

Enum specifying processor)doc";

static const char *__doc_dai_ProcessorType_CPU = R"doc()doc";

static const char *__doc_dai_ProcessorType_DSP = R"doc()doc";

static const char *__doc_dai_ProcessorType_LEON_CSS = R"doc()doc";

static const char *__doc_dai_ProcessorType_LEON_MSS = R"doc()doc";

static const char *__doc_dai_ProfilingData = R"doc()doc";

static const char *__doc_dai_ProfilingData_numBytesRead = R"doc()doc";

static const char *__doc_dai_ProfilingData_numBytesWritten = R"doc()doc";

static const char *__doc_dai_Properties = R"doc(Base Properties structure)doc";

static const char *__doc_dai_PropertiesSerializable = R"doc(Serializable properties)doc";

static const char *__doc_dai_PropertiesSerializable_clone = R"doc()doc";

static const char *__doc_dai_PropertiesSerializable_serialize = R"doc()doc";

static const char *__doc_dai_Properties_clone = R"doc()doc";

static const char *__doc_dai_Properties_serialize = R"doc()doc";

static const char *__doc_dai_ProtoSerializable = R"doc()doc";

static const char *__doc_dai_ProtoSerializable_SchemaPair = R"doc()doc";

static const char *__doc_dai_ProtoSerializable_SchemaPair_schema = R"doc()doc";

static const char *__doc_dai_ProtoSerializable_SchemaPair_schemaName = R"doc()doc";

static const char *__doc_dai_ProtoSerializable_serializeProto =
R"doc(Serialize the protobuf message of this object

Returns:
    serialized protobuf message)doc";

static const char *__doc_dai_ProtoSerializable_serializeSchema =
R"doc(Serialize the schema of this object

Returns:
    schemaPair)doc";

static const char *__doc_dai_Quaterniond =
R"doc(Quaterniond structure

qx,qy,qz,qw coordinates that define a 3D point orientation.)doc";

static const char *__doc_dai_Quaterniond_Quaterniond = R"doc()doc";

static const char *__doc_dai_Quaterniond_Quaterniond_2 = R"doc()doc";

static const char *__doc_dai_Quaterniond_qw = R"doc()doc";

static const char *__doc_dai_Quaterniond_qx = R"doc()doc";

static const char *__doc_dai_Quaterniond_qy = R"doc()doc";

static const char *__doc_dai_Quaterniond_qz = R"doc()doc";

static const char *__doc_dai_RGBDData = R"doc(RGBD message. Carries RGB and Depth frames.)doc";

static const char *__doc_dai_RGBDData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_RGBDData_RGBDData = R"doc(Construct RGBD message.)doc";

static const char *__doc_dai_RGBDData_frames = R"doc()doc";

static const char *__doc_dai_RGBDData_getDepthFrame = R"doc()doc";

static const char *__doc_dai_RGBDData_getRGBFrame = R"doc()doc";

static const char *__doc_dai_RGBDData_serialize = R"doc()doc";

static const char *__doc_dai_RGBDData_setDepthFrame = R"doc()doc";

static const char *__doc_dai_RGBDData_setRGBFrame = R"doc()doc";

static const char *__doc_dai_RGBDData_str = R"doc()doc";

static const char *__doc_dai_RecordConfig = R"doc(Configuration for recording and replaying messages)doc";

static const char *__doc_dai_RecordConfig_CompressionLevel = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_DEFAULT = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_FAST = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_FASTEST = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_NONE = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_SLOW = R"doc()doc";

static const char *__doc_dai_RecordConfig_CompressionLevel_SLOWEST = R"doc()doc";

static const char *__doc_dai_RecordConfig_RecordReplayState = R"doc()doc";

static const char *__doc_dai_RecordConfig_RecordReplayState_NONE = R"doc()doc";

static const char *__doc_dai_RecordConfig_RecordReplayState_RECORD = R"doc()doc";

static const char *__doc_dai_RecordConfig_RecordReplayState_REPLAY = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding_bitrate = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding_enabled = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding_lossless = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding_profile = R"doc()doc";

static const char *__doc_dai_RecordConfig_VideoEncoding_quality = R"doc()doc";

static const char *__doc_dai_RecordConfig_compressionLevel = R"doc()doc";

static const char *__doc_dai_RecordConfig_outputDir = R"doc()doc";

static const char *__doc_dai_RecordConfig_state = R"doc()doc";

static const char *__doc_dai_RecordConfig_videoEncoding = R"doc()doc";

static const char *__doc_dai_Rect =
R"doc(Rect structure

x,y coordinates together with width and height that define a rectangle. Can be
either normalized [0,1] or absolute representation.)doc";

static const char *__doc_dai_Rect_Rect = R"doc()doc";

static const char *__doc_dai_Rect_Rect_2 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_3 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_4 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_5 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_6 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_7 = R"doc()doc";

static const char *__doc_dai_Rect_Rect_8 = R"doc()doc";

static const char *__doc_dai_Rect_area = R"doc(Area (width*height) of the rectangle)doc";

static const char *__doc_dai_Rect_bottomRight = R"doc(The bottom-right corner)doc";

static const char *__doc_dai_Rect_contains = R"doc(Checks whether the rectangle contains the point.)doc";

static const char *__doc_dai_Rect_denormalize =
R"doc(Denormalize rectangle.

Parameter ``destWidth``:
    Destination frame width.

Parameter ``destHeight``:
    Destination frame height.)doc";

static const char *__doc_dai_Rect_empty = R"doc(True if rectangle is empty.)doc";

static const char *__doc_dai_Rect_hasNormalized = R"doc()doc";

static const char *__doc_dai_Rect_height = R"doc()doc";

static const char *__doc_dai_Rect_isNormalized = R"doc(Whether rectangle is normalized (coordinates in [0,1] range) or not.)doc";

static const char *__doc_dai_Rect_normalize =
R"doc(Normalize rectangle.

Parameter ``srcWidth``:
    Source frame width.

Parameter ``srcHeight``:
    Source frame height.)doc";

static const char *__doc_dai_Rect_normalized = R"doc()doc";

static const char *__doc_dai_Rect_operator_assign = R"doc()doc";

static const char *__doc_dai_Rect_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_Rect_size = R"doc(Size (width, height) of the rectangle)doc";

static const char *__doc_dai_Rect_topLeft = R"doc(The top-left corner.)doc";

static const char *__doc_dai_Rect_width = R"doc()doc";

static const char *__doc_dai_Rect_x = R"doc()doc";

static const char *__doc_dai_Rect_y = R"doc()doc";

static const char *__doc_dai_RemoteConnection =
R"doc(Runs a websocket server exposing DepthAI messages as well as a static frontend
UI)doc";

static const char *__doc_dai_RemoteConnectionImpl = R"doc()doc";

static const char *__doc_dai_RemoteConnection_RemoteConnection =
R"doc(Constructs a RemoteConnection instance.

Parameter ``address``:
    The address to bind the connection to.

Parameter ``webSocketPort``:
    The port for WebSocket communication.

Parameter ``serveFrontend``:
    Whether to serve a frontend UI.

Parameter ``httpPort``:
    The port for HTTP communication.)doc";

static const char *__doc_dai_RemoteConnection_addTopic =
R"doc(Adds a topic to the remote connection.

Parameter ``topicName``:
    The name of the topic.

Parameter ``output``:
    The output to link to the topic.

Parameter ``group``:
    An optional group name for the topic.

Parameter ``useVisualizationIfAvailable``:
    Whether to enable visualization on the message if available or send message
    as is.)doc";

static const char *__doc_dai_RemoteConnection_addTopic_2 =
R"doc(Adds a topic with a message queue.

Parameter ``topicName``:
    The name of the topic.

Parameter ``group``:
    An optional group name for the topic.

Parameter ``maxSize``:
    The maximum queue size.

Parameter ``blocking``:
    Whether the queue is blocking or non-blocking.

Parameter ``useVisualizationIfAvailable``:
    Whether to enable visualization on the message if available or send message
    as is.

Returns:
    A shared pointer to the created message queue.)doc";

static const char *__doc_dai_RemoteConnection_impl = R"doc(PIMPL idiom for implementation hiding.)doc";

static const char *__doc_dai_RemoteConnection_registerPipeline =
R"doc(Registers a pipeline with the remote connection.

Parameter ``pipeline``:
    The pipeline to register.)doc";

static const char *__doc_dai_RemoteConnection_registerService =
R"doc(Registers a service with a callback function.

Parameter ``serviceName``:
    The name of the service.

Parameter ``callback``:
    The callback function to handle requests.)doc";

static const char *__doc_dai_RemoteConnection_removeTopic =
R"doc(Removes a topic from the remote connection.

Parameter ``topicName``:
    The name of the topic to remove. @note After removing a topic any messages
    sent to it will cause an exception to be called on the sender, since this
    closes the queue.

Returns:
    True if the topic was successfully removed, false otherwise.)doc";

static const char *__doc_dai_RemoteConnection_waitKey =
R"doc(Waits for a key event.

Parameter ``delayMs``:
    The delay in milliseconds to wait for a key press.

Returns:
    The key code of the pressed key.)doc";

static const char *__doc_dai_Resize = R"doc()doc";

static const char *__doc_dai_Resize_Mode = R"doc()doc";

static const char *__doc_dai_Resize_Mode_FILL = R"doc()doc";

static const char *__doc_dai_Resize_Mode_FIT = R"doc()doc";

static const char *__doc_dai_Resize_Mode_VALUE = R"doc()doc";

static const char *__doc_dai_Resize_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Resize_Resize = R"doc()doc";

static const char *__doc_dai_Resize_Resize_2 = R"doc()doc";

static const char *__doc_dai_Resize_fill = R"doc()doc";

static const char *__doc_dai_Resize_fit = R"doc()doc";

static const char *__doc_dai_Resize_height = R"doc()doc";

static const char *__doc_dai_Resize_mode = R"doc()doc";

static const char *__doc_dai_Resize_normalized = R"doc()doc";

static const char *__doc_dai_Resize_str = R"doc()doc";

static const char *__doc_dai_Resize_toStr = R"doc()doc";

static const char *__doc_dai_Resize_width = R"doc()doc";

static const char *__doc_dai_Rotate = R"doc()doc";

static const char *__doc_dai_Rotate_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Rotate_Rotate = R"doc()doc";

static const char *__doc_dai_Rotate_Rotate_2 = R"doc()doc";

static const char *__doc_dai_Rotate_angle = R"doc()doc";

static const char *__doc_dai_Rotate_center = R"doc()doc";

static const char *__doc_dai_Rotate_normalized = R"doc()doc";

static const char *__doc_dai_Rotate_offsetX = R"doc()doc";

static const char *__doc_dai_Rotate_offsetY = R"doc()doc";

static const char *__doc_dai_Rotate_str = R"doc()doc";

static const char *__doc_dai_Rotate_toStr = R"doc()doc";

static const char *__doc_dai_RotatedRect = R"doc(RotatedRect structure)doc";

static const char *__doc_dai_RotatedRect_RotatedRect = R"doc()doc";

static const char *__doc_dai_RotatedRect_RotatedRect_2 = R"doc()doc";

static const char *__doc_dai_RotatedRect_RotatedRect_3 = R"doc()doc";

static const char *__doc_dai_RotatedRect_angle = R"doc(degrees, increasing clockwise)doc";

static const char *__doc_dai_RotatedRect_center = R"doc()doc";

static const char *__doc_dai_RotatedRect_denormalize =
R"doc(Denormalize the rotated rectangle. The denormalized rectangle will have center
and size coordinates in range [0, width] and [0, height]

Returns:
    Denormalized rotated rectangle)doc";

static const char *__doc_dai_RotatedRect_getOuterRect =
R"doc(Returns the outer non-rotated rectangle

Returns:
    [minx, miny, maxx, maxy])doc";

static const char *__doc_dai_RotatedRect_getPoints =
R"doc(Get the 4 corner points of the rotated rectangle

Returns:
    4 corner points)doc";

static const char *__doc_dai_RotatedRect_isNormalized = R"doc()doc";

static const char *__doc_dai_RotatedRect_normalize =
R"doc(Normalize the rotated rectangle. The normalized rectangle will have center and
size coordinates in range [0,1]

Returns:
    Normalized rotated rectangle)doc";

static const char *__doc_dai_RotatedRect_size = R"doc()doc";

static const char *__doc_dai_SPIInProperties = R"doc(Properties for SPIIn node)doc";

static const char *__doc_dai_SPIInProperties_busId = R"doc(SPI bus to use)doc";

static const char *__doc_dai_SPIInProperties_maxDataSize = R"doc(Maximum input data size)doc";

static const char *__doc_dai_SPIInProperties_numFrames = R"doc(Number of frames in pool)doc";

static const char *__doc_dai_SPIInProperties_streamName = R"doc(Name of stream)doc";

static const char *__doc_dai_SPIOutProperties = R"doc(Specify properties for SPIOut node)doc";

static const char *__doc_dai_SPIOutProperties_busId = R"doc(SPI bus to use)doc";

static const char *__doc_dai_SPIOutProperties_streamName = R"doc(Name of stream)doc";

static const char *__doc_dai_ScriptProperties = R"doc(Specify ScriptProperties options such as script uri, script name, ...)doc";

static const char *__doc_dai_ScriptProperties_processor = R"doc(Which processor should execute the script)doc";

static const char *__doc_dai_ScriptProperties_scriptName = R"doc(Name of script)doc";

static const char *__doc_dai_ScriptProperties_scriptUri = R"doc(Uri which points to actual script)doc";

static const char *__doc_dai_SerializationType = R"doc()doc";

static const char *__doc_dai_SerializationType_JSON = R"doc()doc";

static const char *__doc_dai_SerializationType_JSON_MSGPACK = R"doc()doc";

static const char *__doc_dai_SerializationType_LIBNOP = R"doc()doc";

static const char *__doc_dai_SharedMemory = R"doc()doc";

static const char *__doc_dai_SharedMemory_SharedMemory = R"doc()doc";

static const char *__doc_dai_SharedMemory_SharedMemory_2 = R"doc()doc";

static const char *__doc_dai_SharedMemory_SharedMemory_3 = R"doc()doc";

static const char *__doc_dai_SharedMemory_SharedMemory_4 = R"doc()doc";

static const char *__doc_dai_SharedMemory_SharedMemory_5 = R"doc()doc";

static const char *__doc_dai_SharedMemory_fd = R"doc()doc";

static const char *__doc_dai_SharedMemory_getData = R"doc()doc";

static const char *__doc_dai_SharedMemory_getData_2 = R"doc()doc";

static const char *__doc_dai_SharedMemory_getFd = R"doc()doc";

static const char *__doc_dai_SharedMemory_getMaxSize = R"doc()doc";

static const char *__doc_dai_SharedMemory_getOffset = R"doc()doc";

static const char *__doc_dai_SharedMemory_getSize = R"doc()doc";

static const char *__doc_dai_SharedMemory_mapFd = R"doc()doc";

static const char *__doc_dai_SharedMemory_mapping = R"doc()doc";

static const char *__doc_dai_SharedMemory_operator_assign = R"doc()doc";

static const char *__doc_dai_SharedMemory_setSize = R"doc()doc";

static const char *__doc_dai_SharedMemory_unmapFd = R"doc()doc";

static const char *__doc_dai_Size2f =
R"doc(Size2f structure

width, height values define the size of the shape/frame)doc";

static const char *__doc_dai_Size2f_Size2f = R"doc()doc";

static const char *__doc_dai_Size2f_Size2f_2 = R"doc()doc";

static const char *__doc_dai_Size2f_Size2f_3 = R"doc()doc";

static const char *__doc_dai_Size2f_hasNormalized = R"doc()doc";

static const char *__doc_dai_Size2f_height = R"doc()doc";

static const char *__doc_dai_Size2f_isNormalized = R"doc()doc";

static const char *__doc_dai_Size2f_normalized = R"doc()doc";

static const char *__doc_dai_Size2f_width = R"doc()doc";

static const char *__doc_dai_SlugComponents = R"doc()doc";

static const char *__doc_dai_SlugComponents_merge = R"doc()doc";

static const char *__doc_dai_SlugComponents_modelRef = R"doc()doc";

static const char *__doc_dai_SlugComponents_modelSlug = R"doc()doc";

static const char *__doc_dai_SlugComponents_modelVariantSlug = R"doc()doc";

static const char *__doc_dai_SlugComponents_split = R"doc()doc";

static const char *__doc_dai_SlugComponents_teamName = R"doc()doc";

static const char *__doc_dai_SourceNode = R"doc()doc";

static const char *__doc_dai_SourceNode_getNodeRecordParams = R"doc()doc";

static const char *__doc_dai_SourceNode_getRecordOutput = R"doc()doc";

static const char *__doc_dai_SourceNode_getReplayInput = R"doc()doc";

static const char *__doc_dai_SpatialDetectionNetworkProperties = R"doc(Specify properties for SpatialDetectionNetwork)doc";

static const char *__doc_dai_SpatialDetectionNetworkProperties_calculationAlgorithm = R"doc()doc";

static const char *__doc_dai_SpatialDetectionNetworkProperties_depthThresholds = R"doc()doc";

static const char *__doc_dai_SpatialDetectionNetworkProperties_detectedBBScaleFactor = R"doc()doc";

static const char *__doc_dai_SpatialDetectionNetworkProperties_stepSize = R"doc()doc";

static const char *__doc_dai_SpatialImgDetection =
R"doc(SpatialImgDetection structure

Contains image detection results together with spatial location data.)doc";

static const char *__doc_dai_SpatialImgDetection_boundingBoxMapping = R"doc()doc";

static const char *__doc_dai_SpatialImgDetection_spatialCoordinates = R"doc()doc";

static const char *__doc_dai_SpatialImgDetections =
R"doc(SpatialImgDetections message. Carries detection results together with spatial
location data)doc";

static const char *__doc_dai_SpatialImgDetections_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_SpatialImgDetections_SpatialImgDetections = R"doc(Construct SpatialImgDetections message.)doc";

static const char *__doc_dai_SpatialImgDetections_detections = R"doc(Detection results.)doc";

static const char *__doc_dai_SpatialImgDetections_serialize = R"doc()doc";

static const char *__doc_dai_SpatialImgDetections_serializeProto =
R"doc(Serialize message to proto buffer

Returns:
    serialized message)doc";

static const char *__doc_dai_SpatialImgDetections_serializeSchema =
R"doc(Serialize schema to proto buffer

Returns:
    serialized schema)doc";

static const char *__doc_dai_SpatialImgDetections_str = R"doc()doc";

static const char *__doc_dai_SpatialImgDetections_transformation = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm =
R"doc(SpatialLocationCalculatorAlgorithm configuration modes

Contains calculation method used to obtain spatial locations.)doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_AVERAGE = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_MAX = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_MEAN = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_MEDIAN = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_MIN = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorAlgorithm_MODE = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig =
R"doc(SpatialLocationCalculatorConfig message. Carries ROI (region of interest) and
threshold for depth calculation)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigData = R"doc(SpatialLocation configuration data structure)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigData_calculationAlgorithm =
R"doc(Calculation method used to obtain spatial locations Average/mean: the average of
ROI is used for calculation. Min: the minimum value inside ROI is used for
calculation. Max: the maximum value inside ROI is used for calculation. Mode:
the most frequent value inside ROI is used for calculation. Median: the median
value inside ROI is used for calculation. Default: median.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigData_depthThresholds = R"doc(Upper and lower thresholds for depth values to take into consideration.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigData_roi = R"doc(Region of interest for spatial location calculation.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigData_stepSize =
R"doc(Step size for calculation. Step size 1 means that every pixel is taken into
calculation, size 2 means every second etc. Default value AUTO: for AVERAGE,
MIN, MAX step size is 1; for MODE/MEDIAN it's 2.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigThresholds =
R"doc(SpatialLocation configuration thresholds structure

Contains configuration data for lower and upper threshold in depth units
(millimeter by default) for ROI. Values outside of threshold range will be
ignored when calculating spatial coordinates from depth map.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigThresholds_lowerThreshold = R"doc(Values less or equal than this threshold are not taken into calculation.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfigThresholds_upperThreshold = R"doc(Values greater or equal than this threshold are not taken into calculation.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_SpatialLocationCalculatorConfig = R"doc(Construct SpatialLocationCalculatorConfig message.)doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_addROI =
R"doc(Add a new ROI to configuration data.

Parameter ``roi``:
    Configuration parameters for ROI (region of interest))doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_config = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_getConfigData =
R"doc(Retrieve configuration data for SpatialLocationCalculator

Returns:
    Vector of configuration parameters for ROIs (region of interests))doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_serialize = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_setROIs =
R"doc(Set a vector of ROIs as configuration data.

Parameter ``ROIs``:
    Vector of configuration parameters for ROIs (region of interests))doc";

static const char *__doc_dai_SpatialLocationCalculatorConfig_str = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorData =
R"doc(SpatialLocationCalculatorData message. Carries spatial information (X,Y,Z) and
their configuration parameters)doc";

static const char *__doc_dai_SpatialLocationCalculatorData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorData_SpatialLocationCalculatorData = R"doc(Construct SpatialLocationCalculatorData message.)doc";

static const char *__doc_dai_SpatialLocationCalculatorData_getSpatialLocations =
R"doc(Retrieve configuration data for SpatialLocationCalculatorData.

Returns:
    Vector of spatial location data, carrying spatial information (X,Y,Z))doc";

static const char *__doc_dai_SpatialLocationCalculatorData_serialize = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorData_spatialLocations = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorData_str = R"doc()doc";

static const char *__doc_dai_SpatialLocationCalculatorProperties = R"doc(Specify properties for SpatialLocationCalculator)doc";

static const char *__doc_dai_SpatialLocationCalculatorProperties_roiConfig = R"doc()doc";

static const char *__doc_dai_SpatialLocations =
R"doc(SpatialLocations structure

Contains configuration data, average depth for the calculated ROI on depth map.
Together with spatial coordinates: x,y,z relative to the center of depth map.
Units are in depth units (millimeter by default).)doc";

static const char *__doc_dai_SpatialLocations_config = R"doc(Configuration for selected ROI)doc";

static const char *__doc_dai_SpatialLocations_depthAverage =
R"doc(Average of depth values inside the ROI between the specified thresholds in
config. Calculated only if calculation method is set to AVERAGE or MIN oR MAX.)doc";

static const char *__doc_dai_SpatialLocations_depthAveragePixelCount = R"doc(Number of depth values used in calculations.)doc";

static const char *__doc_dai_SpatialLocations_depthMax =
R"doc(Maximum of depth values inside the ROI between the specified thresholds in
config. Calculated only if calculation method is set to AVERAGE or MIN oR MAX.)doc";

static const char *__doc_dai_SpatialLocations_depthMedian =
R"doc(Median of depth values inside the ROI between the specified thresholds in
config. Calculated only if calculation method is set to MEDIAN.)doc";

static const char *__doc_dai_SpatialLocations_depthMin =
R"doc(Minimum of depth values inside the ROI between the specified thresholds in
config. Calculated only if calculation method is set to AVERAGE or MIN oR MAX.)doc";

static const char *__doc_dai_SpatialLocations_depthMode =
R"doc(Most frequent of depth values inside the ROI between the specified thresholds in
config. Calculated only if calculation method is set to MODE.)doc";

static const char *__doc_dai_SpatialLocations_spatialCoordinates =
R"doc(Spatial coordinates - x,y,z; x,y are the relative positions of the center of ROI
to the center of depth map)doc";

static const char *__doc_dai_StereoDepthConfig = R"doc(StereoDepthConfig message.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthAlign = R"doc(Align the disparity/depth to the perspective of a rectified output, or center it)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthAlign_CENTER = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthAlign_RECTIFIED_LEFT = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthAlign_RECTIFIED_RIGHT = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit = R"doc(Measurement unit for depth data)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_CENTIMETER = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_CUSTOM = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_FOOT = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_INCH = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_METER = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_DepthUnit_MILLIMETER = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_centerAlignmentShiftFactor = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_customDepthUnitMultiplier =
R"doc(Custom depth unit multiplier, if custom depth unit is enabled, relative to 1
meter. A multiplier of 1000 effectively means depth unit in millimeter.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_depthAlign =
R"doc(Set the disparity/depth alignment to the perspective of a rectified output, or
center it)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_depthUnit =
R"doc(Measurement unit for depth data. Depth data is integer value, multiple of depth
unit.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_disparityShift =
R"doc(Shift input frame by a number of pixels to increase minimum depth. For example
shifting by 48 will change effective disparity search range from (0,95] to
[48,143]. An alternative approach to reducing the minZ. We normally only
recommend doing this when it is known that there will be no objects farther away
than MaxZ, such as having a depth camera mounted above a table pointing down at
the table surface.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_enableExtended =
R"doc(Disparity range increased from 95 to 190, combined from full resolution and
downscaled images. Suitable for short range objects)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_enableLeftRightCheck =
R"doc(Computes and combines disparities in both L-R and R-L directions, and combine
them. For better occlusion handling)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_enableSubpixel =
R"doc(Computes disparity with sub-pixel interpolation (5 fractional bits), suitable
for long range)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_enableSwLeftRightCheck = R"doc(Enables software left right check. Applicable to RVC4 only.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_leftRightCheckThreshold =
R"doc(Left-right check threshold for left-right, right-left disparity map combine,
0..128 Used only when left-right check mode is enabled. Defines the maximum
difference between the confidence of pixels from left-right and right-left
confidence maps)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_numInvalidateEdgePixels =
R"doc(Invalidate X amount of pixels at the edge of disparity frame. For right and
center alignment X pixels will be invalidated from the right edge, for left
alignment from the left edge.)doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_AlgorithmControl_subpixelFractionalBits =
R"doc(Number of fractional bits for subpixel mode

Valid values: 3,4,5

Defines the number of fractional disparities: 2^x

Median filter postprocessing is supported only for 3 fractional bits)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform =
R"doc(The basic cost function used by the Stereo Accelerator for matching the left and
right images is the Census Transform. It works on a block of pixels and computes
a bit vector which represents the structure of the image in that block. There
are two types of Census Transform based on how the middle pixel is used: Classic
Approach and Modified Census. The comparisons that are made between pixels can
be or not thresholded. In some cases a mask can be applied to filter out only
specific bits from the entire bit stream. All these approaches are: Classic
Approach: Uses middle pixel to compare against all its neighbors over a defined
window. Each comparison results in a new bit, that is 0 if central pixel is
smaller, or 1 if is it bigger than its neighbor. Modified Census Transform: same
as classic Census Transform, but instead of comparing central pixel with its
neighbors, the window mean will be compared with each pixel over the window.
Thresholding Census Transform: same as classic Census Transform, but it is not
enough that a neighbor pixel to be bigger than the central pixel, it must be
significant bigger (based on a threshold). Census Transform with Mask: same as
classic Census Transform, but in this case not all of the pixel from the support
window are part of the binary descriptor. We use a ma sk “M” to define which
pixels are part of the binary descriptor (1), and which pixels should be skipped
(0).)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_KernelSize = R"doc(Census transform kernel size possible values.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_KernelSize_AUTO = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_KernelSize_KERNEL_5x5 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_KernelSize_KERNEL_7x7 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_KernelSize_KERNEL_7x9 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_enableMeanMode =
R"doc(If enabled, each pixel in the window is compared with the mean window value
instead of the central pixel.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_kernelMask =
R"doc(Census transform mask, default - auto, mask is set based on resolution and
kernel size. Disabled for 400p input resolution. Enabled for 720p. 0XA82415 for
5x5 census transform kernel. 0XAA02A8154055 for 7x7 census transform kernel.
0X2AA00AA805540155 for 7x9 census transform kernel. Empirical values.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_kernelSize = R"doc(Census transform kernel size.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_noiseThresholdOffset =
R"doc(Used to reduce small fixed levels of noise across all luminance values in the
current image. Valid range is [0,127]. Default value is 0.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_noiseThresholdScale =
R"doc(Used to reduce noise values that increase with luminance in the current image.
Valid range is [-128,127]. Default value is 0.)doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CensusTransform_threshold = R"doc(Census transform comparison threshold value.)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_flatnessConfidenceThreshold = R"doc(Threshold for flatness check in SGM block. Valid range is [1,7].)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_flatnessConfidenceWeight =
R"doc(Weight used with flatness estimation to generate final confidence map. Valid
range is [0,32].)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_flatnessOverride =
R"doc(Flag to indicate whether final confidence value will be overidden by flatness
value. Valid range is {true,false}.)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_motionVectorConfidenceThreshold =
R"doc(Threshold offset for MV variance in confidence generation. A value of 0 allows
most variance. Valid range is [0,3].)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_motionVectorConfidenceWeight =
R"doc(Weight used with local neighborhood motion vector variance estimation to
generate final confidence map. Valid range is [0,32].)doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_occlusionConfidenceWeight =
R"doc(Weight used with occlusion estimation to generate final confidence map. Valid
range is [0,32])doc";

static const char *__doc_dai_StereoDepthConfig_ConfidenceMetrics_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation =
R"doc(Cost Aggregation is based on Semi Global Block Matching (SGBM). This algorithm
uses a semi global technique to aggregate the cost map. Ultimately the idea is
to build inertia into the stereo algorithm. If a pixel has very little texture
information, then odds are the correct disparity for this pixel is close to that
of the previous pixel considered. This means that we get improved results in
areas with low texture.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config = R"doc(Structure for adaptive P1 penalty configuration.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_defaultValue =
R"doc(Used as the default penalty value when nAdapEnable is disabled. A bigger value
enforces higher smoothness and reduced noise at the cost of lower edge accuracy.
This value must be smaller than P2 default penalty. Valid range is [10,50].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_edgeThreshold =
R"doc(Threshold value on edges when nAdapEnable is enabled. A bigger value permits
higher neighboring feature dissimilarity tolerance. This value is shared with P2
penalty configuration. Valid range is [8,16].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_edgeValue =
R"doc(Penalty value on edges when nAdapEnable is enabled. A smaller penalty value
permits higher change in disparity. This value must be smaller than or equal to
P2 edge penalty. Valid range is [10,50].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_enableAdaptive = R"doc(Used to disable/enable adaptive penalty.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_smoothThreshold =
R"doc(Threshold value on low texture regions when nAdapEnable is enabled. A bigger
value permits higher neighboring feature dissimilarity tolerance. This value is
shared with P2 penalty configuration. Valid range is [2,12].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_smoothValue =
R"doc(Penalty value on low texture regions when nAdapEnable is enabled. A smaller
penalty value permits higher change in disparity. This value must be smaller
than or equal to P2 smoothness penalty. Valid range is [10,50].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P1Config_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config = R"doc(Structure for adaptive P2 penalty configuration.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_defaultValue =
R"doc(Used as the default penalty value when nAdapEnable is disabled. A bigger value
enforces higher smoothness and reduced noise at the cost of lower edge accuracy.
This value must be larger than P1 default penalty. Valid range is [20,100].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_edgeValue =
R"doc(Penalty value on edges when nAdapEnable is enabled. A smaller penalty value
permits higher change in disparity. This value must be larger than or equal to
P1 edge penalty. Valid range is [20,100].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_enableAdaptive = R"doc(Used to disable/enable adaptive penalty.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_smoothValue =
R"doc(Penalty value on low texture regions when nAdapEnable is enabled. A smaller
penalty value permits higher change in disparity. This value must be larger than
or equal to P1 smoothness penalty. Valid range is [20,100].)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_P2Config_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_divisionFactor = R"doc(Cost calculation linear equation parameters.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_horizontalPenaltyCostP1 = R"doc(Horizontal P1 penalty cost parameter.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_horizontalPenaltyCostP2 = R"doc(Horizontal P2 penalty cost parameter.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_p1Config = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_p2Config = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_verticalPenaltyCostP1 = R"doc(Vertical P1 penalty cost parameter.)doc";

static const char *__doc_dai_StereoDepthConfig_CostAggregation_verticalPenaltyCostP2 = R"doc(Vertical P2 penalty cost parameter.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching =
R"doc(The matching cost is way of measuring the similarity of image locations in
stereo correspondence algorithm. Based on the configuration parameters and based
on the descriptor type, a linear equation is applied to computing the cost for
each candidate disparity at each pixel.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_DisparityWidth = R"doc(Disparity search range: 64 or 96 pixels are supported by the HW.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_DisparityWidth_DISPARITY_64 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_DisparityWidth_DISPARITY_96 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters =
R"doc(The linear equation applied for computing the cost is: COMB_COST = α*AD +
β*(CTC<<3). CLAMP(COMB_COST >> 5, threshold). Where AD is the Absolute
Difference between 2 pixels values. CTC is the Census Transform Cost between 2
pixels, based on Hamming distance (xor). The α and β parameters are subject to
fine tuning by the user.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters_alpha = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters_beta = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_LinearEquationParameters_threshold = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_confidenceThreshold = R"doc(Disparities with confidence value over this threshold are accepted.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_disparityWidth = R"doc(Disparity search range, default 96 pixels.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_enableCompanding =
R"doc(Disparity companding using sparse matching. Matching pixel by pixel for N
disparities. Matching every 2nd pixel for M disparitites. Matching every 4th
pixel for T disparities. In case of 96 disparities: N=48, M=32, T=16. This way
the search range is extended to 176 disparities, by sparse matching. Note: when
enabling this flag only depth map will be affected, disparity map is not.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_enableSwConfidenceThresholding = R"doc(Enable software confidence thresholding. Applicable to RVC4 only.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_invalidDisparityValue =
R"doc(Used only for debug purposes, SW postprocessing handled only invalid value of 0
properly.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_linearEquationParameters = R"doc(Cost calculation linear equation parameters.)doc";

static const char *__doc_dai_StereoDepthConfig_CostMatching_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_MedianFilter = R"doc(Median filter config for disparity post-processing)doc";

static const char *__doc_dai_StereoDepthConfig_MedianFilter_KERNEL_3x3 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_MedianFilter_KERNEL_5x5 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_MedianFilter_KERNEL_7x7 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_MedianFilter_MEDIAN_OFF = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing = R"doc(Post-processing filters, all the filters are applied in disparity domain.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_AdaptiveMedianFilter = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_AdaptiveMedianFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_AdaptiveMedianFilter_confidenceThreshold =
R"doc(Confidence threshold for adaptive median filtering. Should be less than
nFillConfThresh value used in evaDfsHoleFillConfig. Valid range is [0,255].)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_AdaptiveMedianFilter_enable =
R"doc(Flag to enable adaptive median filtering for a final pass of filtering on low
confidence pixels.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_AdaptiveMedianFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_BrightnessFilter =
R"doc(Brightness filtering. If input frame pixel is too dark or too bright, disparity
will be invalidated. The idea is that for too dark/too bright pixels we have low
confidence, since that area was under/over exposed and details were lost.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_BrightnessFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_BrightnessFilter_maxBrightness =
R"doc(Maximum range in depth units. If input pixel is less or equal than this value
the depth value is invalidated.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_BrightnessFilter_minBrightness =
R"doc(Minimum pixel brightness. If input pixel is less or equal than this value the
depth value is invalidated.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_BrightnessFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter =
R"doc(Decimation filter. Reduces the depth scene complexity. The filter runs on kernel
sizes [2x2] to [8x8] pixels.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_DecimationMode = R"doc(Decimation algorithm type.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_DecimationMode_NON_ZERO_MEAN = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_DecimationMode_NON_ZERO_MEDIAN = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_DecimationMode_PIXEL_SKIPPING = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_decimationFactor =
R"doc(Decimation factor. Valid values are 1,2,3,4. Disparity/depth map x/y resolution
will be decimated with this value.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_decimationMode = R"doc(Decimation algorithm type.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_DecimationFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_DECIMATION = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_FILTER_COUNT = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_MEDIAN = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_NONE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_SPATIAL = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_SPECKLE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_Filter_TEMPORAL = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_enable = R"doc(Flag to enable post-processing hole-filling.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_fillConfidenceThreshold =
R"doc(Pixels with confidence below this value will be filled with the average
disparity of their corresponding superpixel. Valid range is [1,255].)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_highConfidenceThreshold =
R"doc(Pixels with confidence higher than this value are used to calculate an average
disparity per superpixel. Valid range is [1,255])doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_invalidateDisparities =
R"doc(If enabled, sets to 0 the disparity of pixels with confidence below
nFillConfThresh, which did not pass nMinValidPixels criteria. Valid range is
{true, false}.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_minValidDisparity =
R"doc(Represents the required percentange of pixels with confidence value above
nHighConfThresh that are used to calculate average disparity per superpixel,
where 1 means 50% or half, 2 means 25% or a quarter and 3 means 12.5% or an
eighth. If the required number of pixels are not found, the holes will not be
filled.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_HoleFilling_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter = R"doc(1D edge-preserving spatial filter using high-order domain transform.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_alpha =
R"doc(The Alpha factor in an exponential moving average with Alpha=1 - no filter.
Alpha = 0 - infinite filter. Determines the amount of smoothing.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_delta =
R"doc(Step-size boundary. Establishes the threshold used to preserve "edges". If the
disparity value between neighboring pixels exceed the disparity threshold set by
this delta parameter, then filtering will be temporarily disabled. Default value
0 means auto: 3 disparity integer levels. In case of subpixel mode it's 3*number
of subpixel levels.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_enable = R"doc(Whether to enable or disable the filter.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_holeFillingRadius =
R"doc(An in-place heuristic symmetric hole-filling mode applied horizontally during
the filter passes. Intended to rectify minor artefacts with minimal performance
impact. Search radius for hole filling.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_numIterations = R"doc(Number of iterations over the image in both horizontal and vertical direction.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpatialFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter = R"doc(Speckle filtering. Removes speckle noise.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter_differenceThreshold =
R"doc(Maximum difference between neighbor disparity pixels to put them into the same
blob. Units in disparity integer levels.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter_enable = R"doc(Whether to enable or disable the filter.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter_speckleRange = R"doc(Speckle search range.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_SpeckleFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter = R"doc(Temporal filtering with optional persistence.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode = R"doc(Persistency algorithm type.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_PERSISTENCY_INDEFINITELY = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_PERSISTENCY_OFF = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_1_IN_LAST_2 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_1_IN_LAST_5 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_1_IN_LAST_8 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_2_IN_LAST_3 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_2_IN_LAST_4 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_2_OUT_OF_8 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_PersistencyMode_VALID_8_OUT_OF_8 = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_alpha =
R"doc(The Alpha factor in an exponential moving average with Alpha=1 - no filter.
Alpha = 0 - infinite filter. Determines the extent of the temporal history that
should be averaged.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_delta =
R"doc(Step-size boundary. Establishes the threshold used to preserve surfaces (edges).
If the disparity value between neighboring pixels exceed the disparity threshold
set by this delta parameter, then filtering will be temporarily disabled.
Default value 0 means auto: 3 disparity integer levels. In case of subpixel mode
it's 3*number of subpixel levels.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_enable = R"doc(Whether to enable or disable the filter.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_persistencyMode =
R"doc(Persistency mode. If the current disparity/depth value is invalid, it will be
replaced by an older value, based on persistency mode.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_TemporalFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_ThresholdFilter = R"doc(Threshold filtering. Filters out distances outside of a given interval.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_ThresholdFilter_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_ThresholdFilter_maxRange = R"doc(Maximum range in depth units. Depth values over this value are invalidated.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_ThresholdFilter_minRange = R"doc(Minimum range in depth units. Depth values under this value are invalidated.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_ThresholdFilter_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_adaptiveMedianFilter = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_bilateralSigmaValue =
R"doc(Sigma value for bilateral filter. 0 means disabled. A larger value of the
parameter means that farther colors within the pixel neighborhood will be mixed
together.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_brightnessFilter =
R"doc(Brightness filtering. If input frame pixel is too dark or too bright, disparity
will be invalidated. The idea is that for too dark/too bright pixels we have low
confidence, since that area was under/over exposed and details were lost.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_decimationFilter =
R"doc(Decimation filter. Reduces disparity/depth map x/y complexity, reducing runtime
complexity for other filters.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_filteringOrder = R"doc(Order of filters to be applied if filtering is enabled.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_holeFilling = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_median = R"doc(Set kernel size for disparity/depth median filtering, or disable)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_spatialFilter =
R"doc(Edge-preserving filtering: This type of filter will smooth the depth noise while
attempting to preserve edges.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_speckleFilter = R"doc(Speckle filtering. Removes speckle noise.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_str = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_temporalFilter = R"doc(Temporal filtering with optional persistence.)doc";

static const char *__doc_dai_StereoDepthConfig_PostProcessing_thresholdFilter = R"doc(Threshold filtering. Filters out distances outside of a given interval.)doc";

static const char *__doc_dai_StereoDepthConfig_StereoDepthConfig = R"doc(Construct StereoDepthConfig message.)doc";

static const char *__doc_dai_StereoDepthConfig_algorithmControl = R"doc(Controls the flow of stereo algorithm - left-right check, subpixel etc.)doc";

static const char *__doc_dai_StereoDepthConfig_censusTransform = R"doc(Census transform settings.)doc";

static const char *__doc_dai_StereoDepthConfig_confidenceMetrics = R"doc(Confidence metrics settings.)doc";

static const char *__doc_dai_StereoDepthConfig_costAggregation = R"doc(Cost aggregation settings.)doc";

static const char *__doc_dai_StereoDepthConfig_costMatching = R"doc(Cost matching settings.)doc";

static const char *__doc_dai_StereoDepthConfig_filtersBackend = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_getBilateralFilterSigma = R"doc(Get sigma value for 5x5 bilateral filter)doc";

static const char *__doc_dai_StereoDepthConfig_getConfidenceThreshold = R"doc(Get confidence threshold for disparity calculation)doc";

static const char *__doc_dai_StereoDepthConfig_getDepthUnit = R"doc(Get depth unit of depth map.)doc";

static const char *__doc_dai_StereoDepthConfig_getExtendedDisparity = R"doc(Get extended disparity setting)doc";

static const char *__doc_dai_StereoDepthConfig_getFiltersComputeBackend = R"doc(Get filters compute backend)doc";

static const char *__doc_dai_StereoDepthConfig_getLeftRightCheck = R"doc(Get left-right check setting)doc";

static const char *__doc_dai_StereoDepthConfig_getLeftRightCheckThreshold = R"doc(Get threshold for left-right check combine)doc";

static const char *__doc_dai_StereoDepthConfig_getMaxDisparity =
R"doc(Useful for normalization of the disparity map.

Returns:
    Maximum disparity value that the node can return)doc";

static const char *__doc_dai_StereoDepthConfig_getMedianFilter = R"doc(Get median filter setting)doc";

static const char *__doc_dai_StereoDepthConfig_getSubpixel = R"doc(Get subpixel setting)doc";

static const char *__doc_dai_StereoDepthConfig_getSubpixelFractionalBits = R"doc(Get number of fractional bits for subpixel mode)doc";

static const char *__doc_dai_StereoDepthConfig_postProcessing = R"doc(Controls the postprocessing of disparity and/or depth map.)doc";

static const char *__doc_dai_StereoDepthConfig_serialize = R"doc()doc";

static const char *__doc_dai_StereoDepthConfig_setBilateralFilterSigma =
R"doc(A larger value of the parameter means that farther colors within the pixel
neighborhood will be mixed together, resulting in larger areas of semi-equal
color.

Parameter ``sigma``:
    Set sigma value for 5x5 bilateral filter. 0..65535)doc";

static const char *__doc_dai_StereoDepthConfig_setConfidenceThreshold =
R"doc(Confidence threshold for disparity calculation

Parameter ``confThr``:
    Confidence threshold value 0..255)doc";

static const char *__doc_dai_StereoDepthConfig_setDepthAlign =
R"doc(Parameter ``align``:
    Set the disparity/depth alignment: centered (between the 'left' and 'right'
    inputs), or from the perspective of a rectified output stream)doc";

static const char *__doc_dai_StereoDepthConfig_setDepthUnit =
R"doc(Set depth unit of depth map.

Meter, centimeter, millimeter, inch, foot or custom unit is available.)doc";

static const char *__doc_dai_StereoDepthConfig_setDisparityShift =
R"doc(Shift input frame by a number of pixels to increase minimum depth. For example
shifting by 48 will change effective disparity search range from (0,95] to
[48,143]. An alternative approach to reducing the minZ. We normally only
recommend doing this when it is known that there will be no objects farther away
than MaxZ, such as having a depth camera mounted above a table pointing down at
the table surface.)doc";

static const char *__doc_dai_StereoDepthConfig_setExtendedDisparity =
R"doc(Disparity range increased from 95 to 190, combined from full resolution and
downscaled images. Suitable for short range objects)doc";

static const char *__doc_dai_StereoDepthConfig_setFiltersComputeBackend = R"doc(Set filters compute backend)doc";

static const char *__doc_dai_StereoDepthConfig_setLeftRightCheck =
R"doc(Computes and combines disparities in both L-R and R-L directions, and combine
them.

For better occlusion handling, discarding invalid disparity values)doc";

static const char *__doc_dai_StereoDepthConfig_setLeftRightCheckThreshold =
R"doc(Parameter ``threshold``:
    Set threshold for left-right, right-left disparity map combine, 0..255)doc";

static const char *__doc_dai_StereoDepthConfig_setMedianFilter =
R"doc(Parameter ``median``:
    Set kernel size for disparity/depth median filtering, or disable)doc";

static const char *__doc_dai_StereoDepthConfig_setNumInvalidateEdgePixels =
R"doc(Invalidate X amount of pixels at the edge of disparity frame. For right and
center alignment X pixels will be invalidated from the right edge, for left
alignment from the left edge.)doc";

static const char *__doc_dai_StereoDepthConfig_setSubpixel =
R"doc(Computes disparity with sub-pixel interpolation (3 fractional bits by default).

Suitable for long range. Currently incompatible with extended disparity)doc";

static const char *__doc_dai_StereoDepthConfig_setSubpixelFractionalBits =
R"doc(Number of fractional bits for subpixel mode. Default value: 3. Valid values:
3,4,5. Defines the number of fractional disparities: 2^x. Median filter
postprocessing is supported only for 3 fractional bits.)doc";

static const char *__doc_dai_StereoDepthConfig_str = R"doc()doc";

static const char *__doc_dai_StereoDepthProperties = R"doc(Specify properties for StereoDepth)doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh = R"doc()doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_meshLeftUri = R"doc(Uri which points to the mesh array for 'left' input rectification)doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_meshRightUri = R"doc(Uri which points to the mesh array for 'right' input rectification)doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_meshSize = R"doc(Mesh array size in bytes, for each of 'left' and 'right' (need to match))doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_stepHeight = R"doc(Distance between mesh points, in the vertical direction)doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_stepWidth = R"doc(Distance between mesh points, in the horizontal direction)doc";

static const char *__doc_dai_StereoDepthProperties_RectificationMesh_str = R"doc()doc";

static const char *__doc_dai_StereoDepthProperties_alphaScaling =
R"doc(Free scaling parameter between 0 (when all the pixels in the undistorted image
are valid) and 1 (when all the source image pixels are retained in the
undistorted image). On some high distortion lenses, and/or due to rectification
(image rotated) invalid areas may appear even with alpha=0, in these cases alpha
< 0.0 helps removing invalid areas. See getOptimalNewCameraMatrix from opencv
for more details.)doc";

static const char *__doc_dai_StereoDepthProperties_baseline = R"doc()doc";

static const char *__doc_dai_StereoDepthProperties_depthAlignCamera =
R"doc(Which camera to align disparity/depth to. When configured (not AUTO), takes
precedence over 'depthAlign')doc";

static const char *__doc_dai_StereoDepthProperties_depthAlignmentUseSpecTranslation =
R"doc(Use baseline information for depth alignment from specs (design data) or from
calibration. Suitable for debugging. Utilizes calibrated value as default)doc";

static const char *__doc_dai_StereoDepthProperties_disparityToDepthUseSpecTranslation =
R"doc(Use baseline information for disparity to depth conversion from specs (design
data) or from calibration. Suitable for debugging. Utilizes calibrated value as
default)doc";

static const char *__doc_dai_StereoDepthProperties_enableFrameSync =
R"doc(Whether to enable frame syncing inside stereo node or not. Suitable if inputs
are known to be synced.)doc";

static const char *__doc_dai_StereoDepthProperties_enableRectification =
R"doc(Enable stereo rectification/dewarp or not. Useful to disable when replaying pre-
recorded rectified frames.)doc";

static const char *__doc_dai_StereoDepthProperties_enableRuntimeStereoModeSwitch =
R"doc(Whether to enable switching stereo modes at runtime or not. E.g. standard to
subpixel, standard+LR-check to subpixel + LR-check. Note: It will allocate
resources for worst cases scenario, should be enabled only if dynamic mode
switch is required. Default value: false.)doc";

static const char *__doc_dai_StereoDepthProperties_focalLength =
R"doc(Override focal length from calibration. Used only in disparity to depth
conversion. Units are pixels.)doc";

static const char *__doc_dai_StereoDepthProperties_focalLengthFromCalibration =
R"doc(Whether to use horizontal focal length from calibration intrinsics (fx) or
calculate based on calibration FOV. Default value is true. If set to false it's
calculated from FOV and image resolution: focalLength = calib.width / (2.f *
tan(calib.fov / 2 / 180.f * pi));)doc";

static const char *__doc_dai_StereoDepthProperties_height = R"doc(Input frame height. Optional (taken from MonoCamera nodes if they exist))doc";

static const char *__doc_dai_StereoDepthProperties_initialConfig = R"doc(Initial stereo config)doc";

static const char *__doc_dai_StereoDepthProperties_mesh =
R"doc(Specify a direct warp mesh to be used for rectification, instead of intrinsics +
extrinsic matrices)doc";

static const char *__doc_dai_StereoDepthProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_StereoDepthProperties_numPostProcessingMemorySlices =
R"doc(Number of memory slices reserved for stereo depth post processing. -1 means
auto, memory will be allocated based on initial stereo settings and number of
shaves. 0 means that it will reuse the memory slices assigned for main stereo
algorithm. For optimal performance it's recommended to allocate more than 0, so
post processing will run in parallel with main stereo algorithm. Minimum 1,
maximum 6.)doc";

static const char *__doc_dai_StereoDepthProperties_numPostProcessingShaves =
R"doc(Number of shaves reserved for stereo depth post processing. Post processing can
use multiple shaves to increase performance. -1 means auto, resources will be
allocated based on enabled filters. 0 means that it will reuse the shave
assigned for main stereo algorithm. For optimal performance it's recommended to
allocate more than 0, so post processing will run in parallel with main stereo
algorithm. Minimum 1, maximum 10.)doc";

static const char *__doc_dai_StereoDepthProperties_outHeight = R"doc(Output disparity/depth height. Currently only used when aligning to RGB)doc";

static const char *__doc_dai_StereoDepthProperties_outKeepAspectRatio = R"doc(Whether to keep aspect ratio of the input (rectified) or not)doc";

static const char *__doc_dai_StereoDepthProperties_outWidth = R"doc(Output disparity/depth width. Currently only used when aligning to RGB)doc";

static const char *__doc_dai_StereoDepthProperties_rectificationUseSpecTranslation =
R"doc(Obtain rectification matrices using spec translation (design data) or from
calibration in calculations. Suitable for debugging. Default: false)doc";

static const char *__doc_dai_StereoDepthProperties_rectifyEdgeFillColor =
R"doc(Fill color for missing data at frame edges - grayscale 0..255, or -1 to
replicate pixels)doc";

static const char *__doc_dai_StereoDepthProperties_useHomographyRectification =
R"doc(Use 3x3 homography matrix for stereo rectification instead of sparse mesh
generated on device. Default behaviour is AUTO, for lenses with FOV over 85
degrees sparse mesh is used, otherwise 3x3 homography. If custom mesh data is
provided through loadMeshData or loadMeshFiles this option is ignored. true: 3x3
homography matrix generated from calibration data is used for stereo
rectification, can't correct lens distortion. false: sparse mesh is generated
on-device from calibration data with mesh step specified with setMeshStep
(Default: (16, 16)), can correct lens distortion. Implementation for generating
the mesh is same as opencv's initUndistortRectifyMap function. Only the first 8
distortion coefficients are used from calibration data.)doc";

static const char *__doc_dai_StereoDepthProperties_width = R"doc(Input frame width. Optional (taken from MonoCamera nodes if they exist))doc";

static const char *__doc_dai_StereoPair = R"doc(Describes which camera sockets can be used for stereo and their baseline.)doc";

static const char *__doc_dai_StereoPair_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_StereoPair_baseline = R"doc(Baseline in centimeters.)doc";

static const char *__doc_dai_StereoPair_isVertical = R"doc()doc";

static const char *__doc_dai_StereoPair_left = R"doc()doc";

static const char *__doc_dai_StereoPair_right = R"doc()doc";

static const char *__doc_dai_StereoPair_str = R"doc()doc";

static const char *__doc_dai_StereoRectification = R"doc(StereoRectification structure)doc";

static const char *__doc_dai_StereoRectification_leftCameraSocket = R"doc()doc";

static const char *__doc_dai_StereoRectification_rectifiedRotationLeft = R"doc()doc";

static const char *__doc_dai_StereoRectification_rectifiedRotationRight = R"doc()doc";

static const char *__doc_dai_StereoRectification_rightCameraSocket = R"doc()doc";

static const char *__doc_dai_StreamMessageParser = R"doc()doc";

static const char *__doc_dai_StreamMessageParser_parseMessage = R"doc()doc";

static const char *__doc_dai_StreamMessageParser_parseMessage_2 = R"doc()doc";

static const char *__doc_dai_StreamMessageParser_serializeMetadata = R"doc()doc";

static const char *__doc_dai_StreamMessageParser_serializeMetadata_2 = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc_StreamPacketDesc = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc_StreamPacketDesc_2 = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc_StreamPacketDesc_3 = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc_operator_assign = R"doc()doc";

static const char *__doc_dai_StreamPacketDesc_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_StreamPacketMemory = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_StreamPacketMemory_2 = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_getData = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_getData_2 = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_getMaxSize = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_getOffset = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_operator_assign = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_setSize = R"doc()doc";

static const char *__doc_dai_StreamPacketMemory_size = R"doc()doc";

static const char *__doc_dai_Subnode = R"doc()doc";

static const char *__doc_dai_Subnode_Subnode = R"doc()doc";

static const char *__doc_dai_Subnode_node = R"doc()doc";

static const char *__doc_dai_Subnode_operator_mul = R"doc()doc";

static const char *__doc_dai_Subnode_operator_sub = R"doc()doc";

static const char *__doc_dai_SyncProperties = R"doc(Specify properties for Sync.)doc";

static const char *__doc_dai_SyncProperties_syncAttempts = R"doc(The number of syncing attempts before fail (num of replaced messages).)doc";

static const char *__doc_dai_SyncProperties_syncThresholdNs = R"doc(The maximal interval the messages can be apart in nanoseconds.)doc";

static const char *__doc_dai_SystemInformation =
R"doc(SystemInformation message. Carries memory usage, cpu usage and chip
temperatures.)doc";

static const char *__doc_dai_SystemInformationS3 =
R"doc(SystemInformation message for series 3 devices. Carries memory usage, cpu usage
and chip temperatures.)doc";

static const char *__doc_dai_SystemInformationS3_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_SystemInformationS3 = R"doc(Construct SystemInformation message.)doc";

static const char *__doc_dai_SystemInformationS3_chipTemperature = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_cpuAvgUsage = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_cpuUsages = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_ddrMemoryUsage = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_serialize = R"doc()doc";

static const char *__doc_dai_SystemInformationS3_str = R"doc()doc";

static const char *__doc_dai_SystemInformation_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_SystemInformation_SystemInformation = R"doc(Construct SystemInformation message.)doc";

static const char *__doc_dai_SystemInformation_chipTemperature = R"doc()doc";

static const char *__doc_dai_SystemInformation_cmxMemoryUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_ddrMemoryUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_leonCssCpuUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_leonCssMemoryUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_leonMssCpuUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_leonMssMemoryUsage = R"doc()doc";

static const char *__doc_dai_SystemInformation_serialize = R"doc()doc";

static const char *__doc_dai_SystemInformation_str = R"doc()doc";

static const char *__doc_dai_SystemLoggerProperties = R"doc(SystemLoggerProperties structure)doc";

static const char *__doc_dai_SystemLoggerProperties_rateHz = R"doc(Rate at which the messages are going to be sent in hertz)doc";

static const char *__doc_dai_TensorInfo = R"doc(TensorInfo structure)doc";

static const char *__doc_dai_TensorInfo_DataType = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_FP16 = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_FP32 = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_FP64 = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_I8 = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_INT = R"doc()doc";

static const char *__doc_dai_TensorInfo_DataType_U8F = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_C = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_CHW = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_CN = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_CWH = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_H = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_HCW = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_HWC = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_NC = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_NCHW = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_NHCW = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_NHWC = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_W = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_WCH = R"doc()doc";

static const char *__doc_dai_TensorInfo_StorageOrder_WHC = R"doc()doc";

static const char *__doc_dai_TensorInfo_dataType = R"doc()doc";

static const char *__doc_dai_TensorInfo_dims = R"doc()doc";

static const char *__doc_dai_TensorInfo_getChannels = R"doc()doc";

static const char *__doc_dai_TensorInfo_getDataTypeSize = R"doc()doc";

static const char *__doc_dai_TensorInfo_getHeight = R"doc()doc";

static const char *__doc_dai_TensorInfo_getTensorSize = R"doc()doc";

static const char *__doc_dai_TensorInfo_getWidth = R"doc()doc";

static const char *__doc_dai_TensorInfo_name = R"doc()doc";

static const char *__doc_dai_TensorInfo_numDimensions = R"doc()doc";

static const char *__doc_dai_TensorInfo_offset = R"doc()doc";

static const char *__doc_dai_TensorInfo_order = R"doc()doc";

static const char *__doc_dai_TensorInfo_qpScale = R"doc()doc";

static const char *__doc_dai_TensorInfo_qpZp = R"doc()doc";

static const char *__doc_dai_TensorInfo_quantization = R"doc()doc";

static const char *__doc_dai_TensorInfo_strides = R"doc()doc";

static const char *__doc_dai_TensorInfo_validateStorageOrder = R"doc()doc";

static const char *__doc_dai_TextAnnotation = R"doc()doc";

static const char *__doc_dai_TextAnnotation_backgroundColor = R"doc()doc";

static const char *__doc_dai_TextAnnotation_fontSize = R"doc()doc";

static const char *__doc_dai_TextAnnotation_position = R"doc()doc";

static const char *__doc_dai_TextAnnotation_text = R"doc()doc";

static const char *__doc_dai_TextAnnotation_textColor = R"doc()doc";

static const char *__doc_dai_ThermalConfig = R"doc(ThermalConfig message. Currently unused.)doc";

static const char *__doc_dai_ThermalConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams = R"doc(Ambient factors that affect the temperature measurement of a Thermal sensor.)doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_atmosphericTemperature = R"doc(Atmospheric temperature. unit:K, range:230-500(high gain), 230-900(low gain))doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_atmosphericTransmittance = R"doc(Atmospheric transmittance. unit:1/128, range:1-128(0.01-1))doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_distance = R"doc(Distance to the measured object. unit:cnt(128cnt=1m), range:0-25600(0-200m))doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_gainMode = R"doc(Gain mode, low or high.)doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_reflectionTemperature = R"doc(Reflection temperature. unit:K, range:230-500(high gain), 230-900(low gain))doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_str = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalAmbientParams_targetEmissivity = R"doc(Emissivity. unit:1/128, range:1-128(0.01-1))doc";

static const char *__doc_dai_ThermalConfig_ThermalConfig = R"doc(Construct ThermalConfig message.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_antiFallProtectionThresholdHighGainMode = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_antiFallProtectionThresholdLowGainMode = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_autoFFC =
R"doc(Auto Flat-Field-Correction. Controls wheather the shutter is controlled by the
sensor module automatically or not.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_autoFFCTempThreshold =
R"doc(Auto FFC trigger threshold. The condition for triggering the auto FFC is that
the change of Vtemp value exceeds a certain threshold, which is called the Auto
FFC trigger threshold.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_closeManualShutter = R"doc(Set this to True/False to close/open the shutter when autoFFC is disabled.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_fallProtection =
R"doc(The shutter blade may open/close abnormally during strong mechanical shock (such
as fall), and a monitoring process is designed in the firmware to correct the
abnormal shutter switch in time. Turn on or off the fall protect mechanism.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_maxFFCInterval =
R"doc(Maximum FFC interval when auto FFC is enabled. The time interval between two FFC
should not be more than this value.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_minFFCInterval =
R"doc(Minimum FFC interval when auto FFC is enabled. The time interval between two FFC
should not be less than this value.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_minShutterInterval =
R"doc(Frequent FFC will cause shutter heating, resulting in abnormal FFC effect and
abnormal temperature measurement. Regardless of which mechanism triggers FFC,
the minimum trigger interval must be limited.)doc";

static const char *__doc_dai_ThermalConfig_ThermalFFCParams_str = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalGainMode = R"doc(Thermal sensor gain mode. Use low gain in high energy environments.)doc";

static const char *__doc_dai_ThermalConfig_ThermalGainMode_HIGH = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalGainMode_LOW = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageOrientation = R"doc(Orientation of the image.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageOrientation_Flip = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageOrientation_Mirror = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageOrientation_MirrorFlip = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageOrientation_Normal = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_brightnessLevel = R"doc(Image brightness level, 0-255.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_contrastLevel = R"doc(Image contrast level, 0-255.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_digitalDetailEnhanceLevel = R"doc(0-4 Digital etail enhance level.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_orientation = R"doc(Orientation of the image. Computed on the sensor.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_spatialNoiseFilterLevel = R"doc(0-3. Spatial noise filter level.)doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_str = R"doc()doc";

static const char *__doc_dai_ThermalConfig_ThermalImageParams_timeNoiseFilterLevel = R"doc(0-3. Time noise filter level. Filters out the noise that appears over time.)doc";

static const char *__doc_dai_ThermalConfig_ambientParams = R"doc(Ambient factors that affect the temperature measurement of a Thermal sensor.)doc";

static const char *__doc_dai_ThermalConfig_ffcParams = R"doc(Parameters for Flat-Field-Correction.)doc";

static const char *__doc_dai_ThermalConfig_imageParams = R"doc(Image signal processing parameters on the sensor.)doc";

static const char *__doc_dai_ThermalConfig_serialize = R"doc()doc";

static const char *__doc_dai_ThermalConfig_str = R"doc()doc";

static const char *__doc_dai_ThermalProperties = R"doc(Specify properties for Thermal)doc";

static const char *__doc_dai_ThermalProperties_boardSocket = R"doc(Which socket will color camera use)doc";

static const char *__doc_dai_ThermalProperties_fps = R"doc(Camera sensor FPS)doc";

static const char *__doc_dai_ThermalProperties_initialConfig = R"doc(Initial Thermal config)doc";

static const char *__doc_dai_ThermalProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_ThreadedNode = R"doc()doc";

static const char *__doc_dai_ThreadedNode_Impl = R"doc()doc";

static const char *__doc_dai_ThreadedNode_ThreadedNode = R"doc()doc";

static const char *__doc_dai_ThreadedNode_getLogLevel =
R"doc(Gets the logging severity level for this node.

Returns:
    Logging severity level)doc";

static const char *__doc_dai_ThreadedNode_isRunning = R"doc()doc";

static const char *__doc_dai_ThreadedNode_onStart =
R"doc(Function called at the beginning of the `start` function.

This function may be overridden by the user to perform any needed tasks prior to
starting this node's main thread.)doc";

static const char *__doc_dai_ThreadedNode_onStop =
R"doc(Function called at the end of the `stop` function.

This function may be overridden by the user to perform any needed tasks directly
after stopping this node's main thread.)doc";

static const char *__doc_dai_ThreadedNode_pimpl = R"doc()doc";

static const char *__doc_dai_ThreadedNode_run = R"doc()doc";

static const char *__doc_dai_ThreadedNode_running = R"doc()doc";

static const char *__doc_dai_ThreadedNode_setLogLevel =
R"doc(Sets the logging severity level for this node.

Parameter ``level``:
    Logging severity level)doc";

static const char *__doc_dai_ThreadedNode_start = R"doc()doc";

static const char *__doc_dai_ThreadedNode_stop = R"doc()doc";

static const char *__doc_dai_ThreadedNode_thread = R"doc()doc";

static const char *__doc_dai_ThreadedNode_wait = R"doc()doc";

static const char *__doc_dai_Timestamp = R"doc(Timestamp structure)doc";

static const char *__doc_dai_Timestamp_get = R"doc()doc";

static const char *__doc_dai_Timestamp_nsec = R"doc()doc";

static const char *__doc_dai_Timestamp_sec = R"doc()doc";

static const char *__doc_dai_ToFConfig = R"doc(ToFConfig message. Carries config for feature tracking algorithm)doc";

static const char *__doc_dai_ToFConfig_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_ToFConfig_ToFConfig = R"doc(Construct ToFConfig message.)doc";

static const char *__doc_dai_ToFConfig_enableBurstMode = R"doc()doc";

static const char *__doc_dai_ToFConfig_enableDistortionCorrection = R"doc()doc";

static const char *__doc_dai_ToFConfig_enableFPPNCorrection = R"doc()doc";

static const char *__doc_dai_ToFConfig_enableOpticalCorrection = R"doc()doc";

static const char *__doc_dai_ToFConfig_enablePhaseShuffleTemporalFilter = R"doc()doc";

static const char *__doc_dai_ToFConfig_enablePhaseUnwrapping = R"doc()doc";

static const char *__doc_dai_ToFConfig_enableTemperatureCorrection = R"doc()doc";

static const char *__doc_dai_ToFConfig_enableWiggleCorrection = R"doc()doc";

static const char *__doc_dai_ToFConfig_median = R"doc(Set kernel size for depth median filtering, or disable)doc";

static const char *__doc_dai_ToFConfig_phaseUnwrapErrorThreshold = R"doc()doc";

static const char *__doc_dai_ToFConfig_phaseUnwrappingLevel = R"doc()doc";

static const char *__doc_dai_ToFConfig_serialize = R"doc()doc";

static const char *__doc_dai_ToFConfig_setMedianFilter =
R"doc(Parameter ``median``:
    Set kernel size for median filtering, or disable)doc";

static const char *__doc_dai_ToFConfig_str = R"doc()doc";

static const char *__doc_dai_ToFProperties = R"doc(Specify properties for ToF)doc";

static const char *__doc_dai_ToFProperties_boardSocket = R"doc(Which socket will color camera use)doc";

static const char *__doc_dai_ToFProperties_cameraName = R"doc(Which camera name will color camera use)doc";

static const char *__doc_dai_ToFProperties_fps = R"doc(Camera sensor FPS)doc";

static const char *__doc_dai_ToFProperties_imageOrientation = R"doc(Camera sensor image orientation / pixel readout)doc";

static const char *__doc_dai_ToFProperties_initialConfig = R"doc(Initial ToF config)doc";

static const char *__doc_dai_ToFProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_ToFProperties_numFramesPoolRaw = R"doc(Pool sizes)doc";

static const char *__doc_dai_ToFProperties_numShaves = R"doc(Number of shaves reserved for ToF decoding.)doc";

static const char *__doc_dai_ToFProperties_warpHwIds = R"doc(Warp HW IDs to use for undistortion, if empty, use auto/default)doc";

static const char *__doc_dai_TraceEvent = R"doc()doc";

static const char *__doc_dai_TraceEvent_Event = R"doc()doc";

static const char *__doc_dai_TraceEvent_Event_RECEIVE = R"doc()doc";

static const char *__doc_dai_TraceEvent_Event_SEND = R"doc()doc";

static const char *__doc_dai_TraceEvent_Status = R"doc()doc";

static const char *__doc_dai_TraceEvent_Status_END = R"doc()doc";

static const char *__doc_dai_TraceEvent_Status_START = R"doc()doc";

static const char *__doc_dai_TraceEvent_Status_TIMEOUT = R"doc()doc";

static const char *__doc_dai_TraceEvent_dstId = R"doc()doc";

static const char *__doc_dai_TraceEvent_event = R"doc()doc";

static const char *__doc_dai_TraceEvent_srcId = R"doc()doc";

static const char *__doc_dai_TraceEvent_status = R"doc()doc";

static const char *__doc_dai_TraceEvent_timestamp = R"doc()doc";

static const char *__doc_dai_TrackedFeature = R"doc(TrackedFeature structure)doc";

static const char *__doc_dai_TrackedFeature_age = R"doc(Feature age in frames)doc";

static const char *__doc_dai_TrackedFeature_descriptor = R"doc(Feature descriptor)doc";

static const char *__doc_dai_TrackedFeature_harrisScore = R"doc(Feature harris score)doc";

static const char *__doc_dai_TrackedFeature_id = R"doc(Feature ID. Persistent between frames if motion estimation is enabled.)doc";

static const char *__doc_dai_TrackedFeature_position = R"doc(x, y position of the detected feature)doc";

static const char *__doc_dai_TrackedFeature_trackingError = R"doc(Feature tracking error)doc";

static const char *__doc_dai_TrackedFeatures =
R"doc(TrackedFeatures message. Carries position (X, Y) of tracked features and their
ID.)doc";

static const char *__doc_dai_TrackedFeatures_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_TrackedFeatures_TrackedFeatures = R"doc(Construct TrackedFeatures message.)doc";

static const char *__doc_dai_TrackedFeatures_serialize = R"doc()doc";

static const char *__doc_dai_TrackedFeatures_str = R"doc()doc";

static const char *__doc_dai_TrackedFeatures_trackedFeatures = R"doc()doc";

static const char *__doc_dai_TrackerIdAssignmentPolicy = R"doc()doc";

static const char *__doc_dai_TrackerIdAssignmentPolicy_SMALLEST_ID = R"doc(Take the smallest available ID)doc";

static const char *__doc_dai_TrackerIdAssignmentPolicy_UNIQUE_ID = R"doc(Always take a new, unique ID)doc";

static const char *__doc_dai_TrackerType = R"doc()doc";

static const char *__doc_dai_TrackerType_SHORT_TERM_IMAGELESS = R"doc(Short term tracking without using image data)doc";

static const char *__doc_dai_TrackerType_SHORT_TERM_KCF = R"doc(Kernelized Correlation Filter tracking)doc";

static const char *__doc_dai_TrackerType_ZERO_TERM_COLOR_HISTOGRAM = R"doc(Tracking using image data too.)doc";

static const char *__doc_dai_TrackerType_ZERO_TERM_IMAGELESS = R"doc(Ability to track the objects without accessing image data.)doc";

static const char *__doc_dai_Tracklet =
R"doc(Tracklet structure

Contains tracklets from object tracker output.)doc";

static const char *__doc_dai_Tracklet_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Tracklet_TrackingStatus = R"doc()doc";

static const char *__doc_dai_Tracklet_TrackingStatus_LOST =
R"doc(The object gets lost now. The object can be tracked again automatically(long
term tracking) or by specifying detected object manually(short term and zero
term tracking).)doc";

static const char *__doc_dai_Tracklet_TrackingStatus_NEW = R"doc(The object is newly added.)doc";

static const char *__doc_dai_Tracklet_TrackingStatus_REMOVED = R"doc(The object is removed.)doc";

static const char *__doc_dai_Tracklet_TrackingStatus_TRACKED = R"doc(The object is being tracked.)doc";

static const char *__doc_dai_Tracklet_age = R"doc(Number of frames it is being tracked for.)doc";

static const char *__doc_dai_Tracklet_id = R"doc(Tracklet's ID.)doc";

static const char *__doc_dai_Tracklet_label = R"doc(Tracklet's label ID.)doc";

static const char *__doc_dai_Tracklet_roi = R"doc(Tracked region of interest.)doc";

static const char *__doc_dai_Tracklet_spatialCoordinates = R"doc(Spatial coordinates of tracklet.)doc";

static const char *__doc_dai_Tracklet_srcImgDetection = R"doc(Image detection that is tracked.)doc";

static const char *__doc_dai_Tracklet_status = R"doc(Status of tracklet.)doc";

static const char *__doc_dai_Tracklet_str = R"doc()doc";

static const char *__doc_dai_Tracklets = R"doc(Tracklets message. Carries object tracking information.)doc";

static const char *__doc_dai_Tracklets_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Tracklets_Tracklets = R"doc(Construct Tracklets message.)doc";

static const char *__doc_dai_Tracklets_serialize = R"doc()doc";

static const char *__doc_dai_Tracklets_str = R"doc()doc";

static const char *__doc_dai_Tracklets_tracklets =
R"doc(Retrieve data for Tracklets.

Returns:
    Vector of object tracker data, carrying tracking information.)doc";

static const char *__doc_dai_Transform = R"doc()doc";

static const char *__doc_dai_TransformData = R"doc(TransformData message. Carries transform in x,y,z,qx,qy,qz,qw format.)doc";

static const char *__doc_dai_TransformData_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_TransformData_TransformData = R"doc(Construct TransformData message.)doc";

static const char *__doc_dai_TransformData_TransformData_2 = R"doc()doc";

static const char *__doc_dai_TransformData_TransformData_3 = R"doc()doc";

static const char *__doc_dai_TransformData_TransformData_4 = R"doc()doc";

static const char *__doc_dai_TransformData_TransformData_5 = R"doc()doc";

static const char *__doc_dai_TransformData_getQuaternion = R"doc()doc";

static const char *__doc_dai_TransformData_getRotationEuler = R"doc()doc";

static const char *__doc_dai_TransformData_getTranslation = R"doc()doc";

static const char *__doc_dai_TransformData_serialize = R"doc()doc";

static const char *__doc_dai_TransformData_str = R"doc()doc";

static const char *__doc_dai_TransformData_transform = R"doc(Transform)doc";

static const char *__doc_dai_Transform_matrix = R"doc()doc";

static const char *__doc_dai_Translate = R"doc()doc";

static const char *__doc_dai_Translate_NOP_STRUCTURE = R"doc()doc";

static const char *__doc_dai_Translate_Translate = R"doc()doc";

static const char *__doc_dai_Translate_Translate_2 = R"doc()doc";

static const char *__doc_dai_Translate_normalized = R"doc()doc";

static const char *__doc_dai_Translate_offsetX = R"doc()doc";

static const char *__doc_dai_Translate_offsetY = R"doc()doc";

static const char *__doc_dai_Translate_str = R"doc()doc";

static const char *__doc_dai_Translate_toStr = R"doc()doc";

static const char *__doc_dai_UVCProperties = R"doc(Properties for UVC node)doc";

static const char *__doc_dai_UVCProperties_gpioInit = R"doc(<gpio_number, value> list for GPIOs to set at init)doc";

static const char *__doc_dai_UVCProperties_gpioStreamOff = R"doc(<gpio_number, value> list for GPIOs to set when streaming is disabled)doc";

static const char *__doc_dai_UVCProperties_gpioStreamOn = R"doc(<gpio_number, value> list for GPIOs to set when streaming is enabled)doc";

static const char *__doc_dai_UsbSpeed = R"doc(Get USB Speed)doc";

static const char *__doc_dai_UsbSpeed_FULL = R"doc()doc";

static const char *__doc_dai_UsbSpeed_HIGH = R"doc()doc";

static const char *__doc_dai_UsbSpeed_LOW = R"doc()doc";

static const char *__doc_dai_UsbSpeed_SUPER = R"doc()doc";

static const char *__doc_dai_UsbSpeed_SUPER_PLUS = R"doc()doc";

static const char *__doc_dai_UsbSpeed_UNKNOWN = R"doc()doc";

static const char *__doc_dai_VectorMemory = R"doc()doc";

static const char *__doc_dai_VectorMemory_VectorMemory = R"doc()doc";

static const char *__doc_dai_VectorMemory_VectorMemory_2 = R"doc()doc";

static const char *__doc_dai_VectorMemory_VectorMemory_3 = R"doc()doc";

static const char *__doc_dai_VectorMemory_getData = R"doc()doc";

static const char *__doc_dai_VectorMemory_getData_2 = R"doc()doc";

static const char *__doc_dai_VectorMemory_getMaxSize = R"doc()doc";

static const char *__doc_dai_VectorMemory_getOffset = R"doc()doc";

static const char *__doc_dai_VectorMemory_operator_assign = R"doc()doc";

static const char *__doc_dai_VectorMemory_setSize = R"doc()doc";

static const char *__doc_dai_Version = R"doc(Version structure)doc";

static const char *__doc_dai_Version_Impl = R"doc()doc";

static const char *__doc_dai_Version_PreReleaseType = R"doc()doc";

static const char *__doc_dai_Version_PreReleaseType_ALPHA = R"doc()doc";

static const char *__doc_dai_Version_PreReleaseType_BETA = R"doc()doc";

static const char *__doc_dai_Version_PreReleaseType_NONE = R"doc()doc";

static const char *__doc_dai_Version_PreReleaseType_RC = R"doc()doc";

static const char *__doc_dai_Version_Version = R"doc(Construct Version from string)doc";

static const char *__doc_dai_Version_Version_2 = R"doc(Construct Version major, minor, patch, and pre-release information)doc";

static const char *__doc_dai_Version_Version_3 = R"doc()doc";

static const char *__doc_dai_Version_getBuildInfo = R"doc(Get build info)doc";

static const char *__doc_dai_Version_operator_eq = R"doc()doc";

static const char *__doc_dai_Version_operator_ge = R"doc()doc";

static const char *__doc_dai_Version_operator_gt = R"doc()doc";

static const char *__doc_dai_Version_operator_le = R"doc()doc";

static const char *__doc_dai_Version_operator_lt = R"doc()doc";

static const char *__doc_dai_Version_operator_ne = R"doc()doc";

static const char *__doc_dai_Version_pimpl = R"doc()doc";

static const char *__doc_dai_Version_toString = R"doc(Convert Version to string)doc";

static const char *__doc_dai_Version_toStringSemver = R"doc(Convert Version to semver (no build information) string)doc";

static const char *__doc_dai_VideoEncoderProperties = R"doc(Specify properties for VideoEncoder such as profile, bitrate, ...)doc";

static const char *__doc_dai_VideoEncoderProperties_Profile = R"doc(Encoding profile, H264 (AVC), H265 (HEVC) or MJPEG)doc";

static const char *__doc_dai_VideoEncoderProperties_Profile_H264_BASELINE = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_Profile_H264_HIGH = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_Profile_H264_MAIN = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_Profile_H265_MAIN = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_Profile_MJPEG = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_RateControlMode =
R"doc(Rate control mode specifies if constant or variable bitrate should be used (H264
/ H265))doc";

static const char *__doc_dai_VideoEncoderProperties_RateControlMode_CBR = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_RateControlMode_VBR = R"doc()doc";

static const char *__doc_dai_VideoEncoderProperties_bitrate =
R"doc(Specifies preferred bitrate (in bit/s) of compressed output bitstream in CBR
mode

"0" for automatic computation, based on input resolution and FPS: 720p30: 4Mbps,
1080p30: 8.5Mbps, 1440p30: 14Mbps, 2160p30: 20Mbps)doc";

static const char *__doc_dai_VideoEncoderProperties_frameRate = R"doc(Frame rate)doc";

static const char *__doc_dai_VideoEncoderProperties_keyframeFrequency = R"doc(Every x number of frames a keyframe will be inserted)doc";

static const char *__doc_dai_VideoEncoderProperties_lossless = R"doc(Lossless mode ([M]JPEG only))doc";

static const char *__doc_dai_VideoEncoderProperties_maxBitrate =
R"doc(Specifies maximum bitrate (in bit/s) of compressed output bitstream in CBR mode

"0" to follow `bitrate` setting)doc";

static const char *__doc_dai_VideoEncoderProperties_numBFrames = R"doc(Specifies number of B frames to be inserted)doc";

static const char *__doc_dai_VideoEncoderProperties_numFramesPool =
R"doc(This options specifies how many frames are available in this node's pool. Helps
when receiver is slow at consuming.

Value "0" indicates automatic number of frames assignment)doc";

static const char *__doc_dai_VideoEncoderProperties_outputFrameSize = R"doc(Specifies max output frame size in pool. Value "0" indicates auto)doc";

static const char *__doc_dai_VideoEncoderProperties_profile = R"doc(Encoding profile, H264, H265 or MJPEG)doc";

static const char *__doc_dai_VideoEncoderProperties_quality = R"doc(Value between 0-100% (approximates quality))doc";

static const char *__doc_dai_VideoEncoderProperties_rateCtrlMode =
R"doc(Rate control mode specifies if constant or variable bitrate should be used (H264
/ H265))doc";

static const char *__doc_dai_WarpProperties = R"doc(Specify properties for Warp)doc";

static const char *__doc_dai_WarpProperties_interpolation = R"doc()doc";

static const char *__doc_dai_WarpProperties_meshHeight = R"doc(Custom warp mesh height. Set to zero to disable.)doc";

static const char *__doc_dai_WarpProperties_meshUri = R"doc(Custom warp mesh uri. Set to empty string to disable.)doc";

static const char *__doc_dai_WarpProperties_meshWidth = R"doc(Custom warp mesh width. Set to zero to disable)doc";

static const char *__doc_dai_WarpProperties_numFramesPool = R"doc(Num frames in output pool)doc";

static const char *__doc_dai_WarpProperties_outputFrameSize = R"doc(Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes))doc";

static const char *__doc_dai_WarpProperties_outputHeight = R"doc(Output height)doc";

static const char *__doc_dai_WarpProperties_outputWidth = R"doc(Output width)doc";

static const char *__doc_dai_WarpProperties_warpHwIds = R"doc(Warp HW IDs to use, if empty, use auto/default)doc";

static const char *__doc_dai_XLinkConnection = R"doc(Represents connection between host and device over XLink protocol)doc";

static const char *__doc_dai_XLinkConnection_XLinkConnection = R"doc()doc";

static const char *__doc_dai_XLinkConnection_XLinkConnection_2 = R"doc()doc";

static const char *__doc_dai_XLinkConnection_XLinkConnection_3 = R"doc()doc";

static const char *__doc_dai_XLinkConnection_bootAvailableDevice = R"doc()doc";

static const char *__doc_dai_XLinkConnection_bootAvailableDevice_2 = R"doc()doc";

static const char *__doc_dai_XLinkConnection_bootBootloader =
R"doc(Tries booting the given device into bootloader state

Parameter ``devInfo``:
    Information of device which it should boot into bootloader state

Returns:
    New device information if successful)doc";

static const char *__doc_dai_XLinkConnection_bootDevice = R"doc()doc";

static const char *__doc_dai_XLinkConnection_bootWithPath = R"doc()doc";

static const char *__doc_dai_XLinkConnection_close =
R"doc(Explicitly closes xlink connection. @note This function does not need to be
explicitly called as destructor closes the connection automatically)doc";

static const char *__doc_dai_XLinkConnection_closed = R"doc()doc";

static const char *__doc_dai_XLinkConnection_closedMtx = R"doc()doc";

static const char *__doc_dai_XLinkConnection_convertErrorCodeToString = R"doc()doc";

static const char *__doc_dai_XLinkConnection_deviceInfo = R"doc()doc";

static const char *__doc_dai_XLinkConnection_deviceLinkId = R"doc()doc";

static const char *__doc_dai_XLinkConnection_getAllConnectedDevices =
R"doc(Returns information of all connected devices with given state

Parameter ``state``:
    State which the devices should be in

Parameter ``skipInvalidDevices``:
    whether or not to skip over devices that cannot be successfully communicated
    with

Returns:
    Vector of connected device information)doc";

static const char *__doc_dai_XLinkConnection_getDeviceById =
R"doc(Finds a device by Device ID. Example: 14442C10D13EABCE00

Parameter ``deviceId``:
    Device ID which uniquely specifies a device

Parameter ``state``:
    Which state should the device be in

Parameter ``skipInvalidDevices``:
    Whether or not to skip devices that cannot be fully detected

Returns:
    Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo
    specifies the found device)doc";

static const char *__doc_dai_XLinkConnection_getFirstDevice =
R"doc(Returns information of first device with given state

Parameter ``state``:
    State which the device should be in

Returns:
    Device information)doc";

static const char *__doc_dai_XLinkConnection_getGlobalProfilingData =
R"doc(Get current accumulated profiling data

Returns:
    ProfilingData from the specific connection)doc";

static const char *__doc_dai_XLinkConnection_getLinkId = R"doc()doc";

static const char *__doc_dai_XLinkConnection_getProfilingData =
R"doc(Get current accumulated profiling data

Returns:
    ProfilingData from the specific connection)doc";

static const char *__doc_dai_XLinkConnection_getRebootOnDestruction = R"doc()doc";

static const char *__doc_dai_XLinkConnection_initDevice = R"doc()doc";

static const char *__doc_dai_XLinkConnection_isClosed =
R"doc(Is the connection already closed (or disconnected)

.. warning::
    This function is thread-unsafe and may return outdated incorrect values. It
    is only meant for use in simple single-threaded code. Well written code
    should handle exceptions when calling any DepthAI apis to handle hardware
    events and multithreaded use.)doc";

static const char *__doc_dai_XLinkConnection_mvcmd = R"doc()doc";

static const char *__doc_dai_XLinkConnection_pathToMvcmd = R"doc()doc";

static const char *__doc_dai_XLinkConnection_rebootOnDestruction = R"doc()doc";

static const char *__doc_dai_XLinkConnection_setRebootOnDestruction = R"doc()doc";

static const char *__doc_dai_XLinkError = R"doc()doc";

static const char *__doc_dai_XLinkError_XLinkError = R"doc()doc";

static const char *__doc_dai_XLinkError_status = R"doc()doc";

static const char *__doc_dai_XLinkError_streamName = R"doc()doc";

static const char *__doc_dai_XLinkReadError = R"doc()doc";

static const char *__doc_dai_XLinkReadError_XLinkReadError = R"doc()doc";

static const char *__doc_dai_XLinkStream = R"doc()doc";

static const char *__doc_dai_XLinkStream_XLinkStream = R"doc()doc";

static const char *__doc_dai_XLinkStream_XLinkStream_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_XLinkStream_3 = R"doc()doc";

static const char *__doc_dai_XLinkStream_connection = R"doc()doc";

static const char *__doc_dai_XLinkStream_getStreamId = R"doc()doc";

static const char *__doc_dai_XLinkStream_getStreamName = R"doc()doc";

static const char *__doc_dai_XLinkStream_operator_assign = R"doc()doc";

static const char *__doc_dai_XLinkStream_operator_assign_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_3 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_4 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_5 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_6 = R"doc()doc";

static const char *__doc_dai_XLinkStream_read_7 = R"doc()doc";

static const char *__doc_dai_XLinkStream_readMove = R"doc()doc";

static const char *__doc_dai_XLinkStream_readMove_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_readRaw = R"doc()doc";

static const char *__doc_dai_XLinkStream_readRaw_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_readRawRelease = R"doc()doc";

static const char *__doc_dai_XLinkStream_streamId = R"doc()doc";

static const char *__doc_dai_XLinkStream_streamName = R"doc()doc";

static const char *__doc_dai_XLinkStream_write = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_2 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_3 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_4 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_5 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_6 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_7 = R"doc()doc";

static const char *__doc_dai_XLinkStream_write_8 = R"doc()doc";

static const char *__doc_dai_XLinkStream_writeSplit = R"doc()doc";

static const char *__doc_dai_XLinkStream_writeSplit_2 = R"doc()doc";

static const char *__doc_dai_XLinkWriteError = R"doc()doc";

static const char *__doc_dai_XLinkWriteError_XLinkWriteError = R"doc()doc";

static const char *__doc_dai_as_bytes = R"doc()doc";

static const char *__doc_dai_as_writable_bytes = R"doc()doc";

static const char *__doc_dai_bootloader_Config = R"doc()doc";

static const char *__doc_dai_bootloader_Config_appMem = R"doc()doc";

static const char *__doc_dai_bootloader_Config_network = R"doc()doc";

static const char *__doc_dai_bootloader_Config_usb = R"doc()doc";

static const char *__doc_dai_bootloader_Config_userBlChecksum = R"doc()doc";

static const char *__doc_dai_bootloader_Config_userBlSize = R"doc()doc";

static const char *__doc_dai_bootloader_Memory = R"doc()doc";

static const char *__doc_dai_bootloader_Memory_AUTO = R"doc()doc";

static const char *__doc_dai_bootloader_Memory_EMMC = R"doc()doc";

static const char *__doc_dai_bootloader_Memory_FLASH = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkBootloaderStructure = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkBootloaderStructure_NetworkBootloaderStructure = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv4 = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv4Dns = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv4DnsAlt = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv4Gateway = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv4Mask = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv6 = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv6Dns = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv6DnsAlt = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv6Gateway = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_ipv6Prefix = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_mac = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_staticIpv4 = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_staticIpv6 = R"doc()doc";

static const char *__doc_dai_bootloader_NetworkConfig_timeoutMs =
R"doc(If timeout < 0 - waits forever if timeout == 0 - no timeout if timeout > 0 -
waits timeout milliseconds)doc";

static const char *__doc_dai_bootloader_Section = R"doc()doc";

static const char *__doc_dai_bootloader_Section_APPLICATION = R"doc()doc";

static const char *__doc_dai_bootloader_Section_AUTO = R"doc()doc";

static const char *__doc_dai_bootloader_Section_BOOTLOADER = R"doc()doc";

static const char *__doc_dai_bootloader_Section_BOOTLOADER_CONFIG = R"doc()doc";

static const char *__doc_dai_bootloader_Section_HEADER = R"doc()doc";

static const char *__doc_dai_bootloader_Section_USER_BOOTLOADER = R"doc()doc";

static const char *__doc_dai_bootloader_Structure = R"doc()doc";

static const char *__doc_dai_bootloader_Structure_Structure = R"doc()doc";

static const char *__doc_dai_bootloader_Structure_Structure_2 = R"doc()doc";

static const char *__doc_dai_bootloader_Structure_offset = R"doc()doc";

static const char *__doc_dai_bootloader_Structure_size = R"doc()doc";

static const char *__doc_dai_bootloader_Type = R"doc()doc";

static const char *__doc_dai_bootloader_Type_AUTO = R"doc()doc";

static const char *__doc_dai_bootloader_Type_NETWORK = R"doc()doc";

static const char *__doc_dai_bootloader_Type_USB = R"doc()doc";

static const char *__doc_dai_bootloader_UsbBootloaderStructure = R"doc()doc";

static const char *__doc_dai_bootloader_UsbBootloaderStructure_UsbBootloaderStructure = R"doc()doc";

static const char *__doc_dai_bootloader_UsbConfig = R"doc()doc";

static const char *__doc_dai_bootloader_UsbConfig_maxUsbSpeed = R"doc(UNKNOWN = 0, LOW, FULL, HIGH, SUPER, SUPER_PLUS)doc";

static const char *__doc_dai_bootloader_UsbConfig_pid = R"doc(VID/PID pair used by bootloader)doc";

static const char *__doc_dai_bootloader_UsbConfig_timeoutMs =
R"doc(If timeout < 0 - waits forever if timeout == 0 - no timeout if timeout > 0 -
waits timeout milliseconds)doc";

static const char *__doc_dai_bootloader_UsbConfig_vid = R"doc(VID/PID pair used by bootloader)doc";

static const char *__doc_dai_bootloader_from_json = R"doc()doc";

static const char *__doc_dai_bootloader_from_json_2 = R"doc()doc";

static const char *__doc_dai_bootloader_from_json_3 = R"doc()doc";

static const char *__doc_dai_bootloader_getStructure = R"doc()doc";

static const char *__doc_dai_bootloader_request_BaseRequest = R"doc()doc";

static const char *__doc_dai_bootloader_request_BaseRequest_BaseRequest = R"doc()doc";

static const char *__doc_dai_bootloader_request_BaseRequest_cmd = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootApplication = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootApplication_BootApplication = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootMemory = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootMemory_BootMemory = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootMemory_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootMemory_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootloaderMemory = R"doc()doc";

static const char *__doc_dai_bootloader_request_BootloaderMemory_BootloaderMemory = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_BOOTLOADER_MEMORY = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_BOOT_APPLICATION = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_BOOT_MEMORY = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_APPLICATION_DETAILS = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_BOOTLOADER_COMMIT = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_BOOTLOADER_CONFIG = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_BOOTLOADER_TYPE = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_BOOTLOADER_VERSION = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_GET_MEMORY_DETAILS = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_IS_USER_BOOTLOADER = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_NO_OP = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_READ_FLASH = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_SET_BOOTLOADER_CONFIG = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_UPDATE_FLASH = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_UPDATE_FLASH_BOOT_HEADER = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_UPDATE_FLASH_EX = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_UPDATE_FLASH_EX_2 = R"doc()doc";

static const char *__doc_dai_bootloader_request_Command_USB_ROM_BOOT = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetApplicationDetails = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetApplicationDetails_GetApplicationDetails = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetApplicationDetails_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderCommit = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderCommit_GetBootloaderCommit = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderConfig_GetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderConfig_maxSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderConfig_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderConfig_offset = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderType = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderType_GetBootloaderType = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderVersion = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetBootloaderVersion_GetBootloaderVersion = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetMemoryDetails = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetMemoryDetails_GetMemoryDetails = R"doc()doc";

static const char *__doc_dai_bootloader_request_GetMemoryDetails_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_IsUserBootloader = R"doc()doc";

static const char *__doc_dai_bootloader_request_IsUserBootloader_IsUserBootloader = R"doc()doc";

static const char *__doc_dai_bootloader_request_ReadFlash = R"doc()doc";

static const char *__doc_dai_bootloader_request_ReadFlash_ReadFlash = R"doc()doc";

static const char *__doc_dai_bootloader_request_ReadFlash_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_ReadFlash_offset = R"doc()doc";

static const char *__doc_dai_bootloader_request_ReadFlash_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_SetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_clearConfig = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_offset = R"doc()doc";

static const char *__doc_dai_bootloader_request_SetBootloaderConfig_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_Type = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_Type_FAST = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_Type_GPIO_MODE = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_Type_NORMAL = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_Type_USB_RECOVERY = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_UpdateFlashBootHeader = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_dummyCycles = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_frequency = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_gpioMode = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_location = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_offset = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashBootHeader_type = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2 = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2_UpdateFlashEx2 = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2_offset = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx2_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx_UpdateFlashEx = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx_memory = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx_section = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlashEx_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_Storage = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_Storage_BOOTLOADER = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_Storage_SBR = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_UpdateFlash = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_storage = R"doc()doc";

static const char *__doc_dai_bootloader_request_UpdateFlash_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_request_UsbRomBoot = R"doc()doc";

static const char *__doc_dai_bootloader_request_UsbRomBoot_UsbRomBoot = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_ApplicationDetails = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_applicationNameStr = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_errorMsg = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_firmwareVersionStr = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_hasApplication = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_hasApplicationName = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_hasFirmwareVersion = R"doc()doc";

static const char *__doc_dai_bootloader_response_ApplicationDetails_success = R"doc()doc";

static const char *__doc_dai_bootloader_response_BaseResponse = R"doc()doc";

static const char *__doc_dai_bootloader_response_BaseResponse_BaseResponse = R"doc()doc";

static const char *__doc_dai_bootloader_response_BaseResponse_cmd = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootApplication = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootApplication_BootApplication = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootApplication_errorMsg = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootApplication_success = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderCommit = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderCommit_BootloaderCommit = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderCommit_commitStr = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderMemory = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderMemory_BootloaderMemory = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderMemory_memory = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderType = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderType_BootloaderType = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderType_type = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderVersion = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderVersion_BootloaderVersion = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderVersion_major = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderVersion_minor = R"doc()doc";

static const char *__doc_dai_bootloader_response_BootloaderVersion_patch = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_APPLICATION_DETAILS = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_BOOTLOADER_COMMIT = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_BOOTLOADER_MEMORY = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_BOOTLOADER_TYPE = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_BOOTLOADER_VERSION = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_BOOT_APPLICATION = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_FLASH_COMPLETE = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_FLASH_STATUS_UPDATE = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_GET_BOOTLOADER_CONFIG = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_IS_USER_BOOTLOADER = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_MEMORY_DETAILS = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_NO_OP = R"doc()doc";

static const char *__doc_dai_bootloader_response_Command_READ_FLASH = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashComplete = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashComplete_FlashComplete = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashComplete_errorMsg = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashComplete_success = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashStatusUpdate = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashStatusUpdate_FlashStatusUpdate = R"doc()doc";

static const char *__doc_dai_bootloader_response_FlashStatusUpdate_progress = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig_GetBootloaderConfig = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig_errorMsg = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig_success = R"doc()doc";

static const char *__doc_dai_bootloader_response_GetBootloaderConfig_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_response_IsUserBootloader = R"doc()doc";

static const char *__doc_dai_bootloader_response_IsUserBootloader_IsUserBootloader = R"doc()doc";

static const char *__doc_dai_bootloader_response_IsUserBootloader_isUserBootloader = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails_MemoryDetails = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails_hasMemory = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails_memory = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails_memoryInfo = R"doc()doc";

static const char *__doc_dai_bootloader_response_MemoryDetails_memorySize = R"doc()doc";

static const char *__doc_dai_bootloader_response_NoOp = R"doc()doc";

static const char *__doc_dai_bootloader_response_NoOp_NoOp = R"doc()doc";

static const char *__doc_dai_bootloader_response_NoOp_invalidOp = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash_ReadFlash = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash_errorMsg = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash_numPackets = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash_success = R"doc()doc";

static const char *__doc_dai_bootloader_response_ReadFlash_totalSize = R"doc()doc";

static const char *__doc_dai_bootloader_to_json = R"doc()doc";

static const char *__doc_dai_bootloader_to_json_2 = R"doc()doc";

static const char *__doc_dai_bootloader_to_json_3 = R"doc()doc";

static const char *__doc_dai_contract_violation = R"doc()doc";

static const char *__doc_dai_copyable_unique_ptr =
R"doc(A smart pointer with deep copy semantics.

This is _similar_ to `std::unique_ptr` in that it does not permit shared
ownership of the contained object. However, unlike `std::unique_ptr`,
%copyable_unique_ptr supports copy and assignment operations, by insisting that
the contained object be "copyable". To be copyable, the class must have either
an accessible copy constructor, or it must have an accessible clone method with
signature

```
std::unique_ptr<Foo> Clone() const;
```

where Foo is the type of the managed object. By "accessible" we mean either that
the copy constructor or clone method is public, or `friend
copyable_unique_ptr<Foo>;` appears in Foo's class declaration.

<!-- Developer note: if you change or extend the definition of an acceptable
clone method here, be sure to consider whether drake::is_cloneable should be
changed as well. -->

Generally, the API is modeled as closely as possible on the C++ standard
`std::unique_ptr` API and %copyable_unique_ptr<T> is interoperable with
`unique_ptr<T>` wherever that makes sense. However, there are some differences:

1. It always uses a default deleter. 2. There is no array version. 3. To allow
for future copy-on-write optimizations, there is a distinction between writable
and const access, the get() method is modified to return only a const pointer,
with get_mutable() added to return a writable pointer. Furthermore, derefencing
(operator*()) a mutable pointer will give a mutable reference (in so far as T is
not declared const), and dereferencing a const pointer will give a const
reference.

This class is entirely inline and has no computational or space overhead except
when copying is required; it contains just a single pointer and does no
reference counting.

__Usage__

In the simplest use case, the instantiation type will match the type of object
it references, e.g.:

```
copyable_unique_ptr<Foo> ptr = make_unique<Foo>(...);
```

In this case, as long `Foo` is deemed compatible, the behavior will be as
expected, i.e., when `ptr` copies, it will contain a reference to a new instance
of `Foo`.

%copyable_unique_ptr can also be used with polymorphic classes -- a
%copyable_unique_ptr, instantiated on a _base_ class, references an instance of
a _derived_ class. When copying the object, we would want the copy to likewise
contain an instance of the derived class. For example:

```
copyable_unique_ptr<Base> cu_ptr = make_unique<Derived>();
copyable_unique_ptr<Base> other_cu_ptr = cu_ptr;           // Triggers a copy.
is_dynamic_castable<Derived>(other_cu_ptr.get());          // Should be true.
```

This works for well-designed polymorphic classes.

.. warning::
    Ill-formed polymorphic classes can lead to fatal type slicing of the
    referenced object, such that the new copy contains an instance of `Base`
    instead of `Derived`. Some mistakes that would lead to this degenerate
    behavior:

- The `Base` class's Clone() implementation does not invoke the `Derived`
class's implementation of a suitable virtual method.

<!-- For future developers: - the copyability of a base class does *not* imply
anything about the copyability of a derived class. In other words,
`copyable_unique_ptr<Base>` can be compilable while
`copyable_unique_ptr<Derived>` is not. - Given the pointer
`copyable_unique_ptr<Base> ptr(new Derived())`, even if this copies "correctly"
(such that the copy contains an instance of `Derived`), this does _not_ imply
that `copyable_unique_ptr<Derived>` is compilable. -->

Template parameter ``T``:
    The type of the contained object, which *must* be copyable as defined above.
    May be an abstract or concrete type.)doc";

static const char *__doc_dai_copyable_unique_ptr_CopyOrClone = R"doc(@})doc";

static const char *__doc_dai_copyable_unique_ptr_CopyOrClone_2 = R"doc()doc";

static const char *__doc_dai_copyable_unique_ptr_CopyOrNull = R"doc()doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr =
R"doc(Default constructor stores a `nullptr`. No heap allocation is performed. The
empty() method will return true when called on a default-constructed
%copyable_unique_ptr.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_2 =
R"doc(Given a raw pointer to a writable heap-allocated object, take over ownership of
that object. No copying occurs.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_3 = R"doc(Constructs a unique instance of T as a copy of the provided model value.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_4 =
R"doc(Copy constructor is deep; the new %copyable_unique_ptr object contains a new
copy of the object in the source, created via the source object's copy
constructor or `Clone()` method. If the source container is empty this one will
be empty also.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_5 =
R"doc(Copy constructor from a standard `unique_ptr` of _compatible_ type. The copy is
deep; the new %copyable_unique_ptr object contains a new copy of the object in
the source, created via the source object's copy constructor or `Clone()`
method. If the source container is empty this one will be empty also.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_6 =
R"doc(Move constructor is very fast and leaves the source empty. Ownership is
transferred from the source to the new %copyable_unique_ptr. If the source was
empty this one will be empty also. No heap activity occurs.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_7 =
R"doc(Move constructor from a standard `unique_ptr`. The move is very fast and leaves
the source empty. Ownership is transferred from the source to the new
%copyable_unique_ptr. If the source was empty this one will be empty also. No
heap activity occurs.)doc";

static const char *__doc_dai_copyable_unique_ptr_copyable_unique_ptr_8 =
R"doc(Move construction from a compatible standard `unique_ptr`. Type `U*` must be
implicitly convertible to type `T*`. Ownership is transferred from the source to
the new %copyable_unique_ptr. If the source was empty this one will be empty
also. No heap activity occurs.)doc";

static const char *__doc_dai_copyable_unique_ptr_empty =
R"doc(Return true if this container is empty, which is the state the container is in
immediately after default construction and various other operations.)doc";

static const char *__doc_dai_copyable_unique_ptr_get =
R"doc(Return a const pointer to the contained object if any, or `nullptr`. Note that
this is different than `%get()` for the standard smart pointers like
`std::unique_ptr` which return a writable pointer. Use get_mutable() here for
that purpose.)doc";

static const char *__doc_dai_copyable_unique_ptr_get_mutable =
R"doc(Return a writable pointer to the contained object if any, or `nullptr`. Note
that you need write access to this container in order to get write access to the
object it contains.

@warning If %copyable_unique_ptr is instantiated on a const template parameter
(e.g., `copyable_unique_ptr<const Foo>`), then get_mutable() returns a const
pointer.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign =
R"doc(This form of assignment replaces the currently-held object by the given source
object and takes over ownership of the source object. The currently-held object
(if any) is deleted.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_2 =
R"doc(This form of assignment replaces the currently-held object by a heap-allocated
copy of the source object, created using its copy constructor or `Clone()`
method. The currently-held object (if any) is deleted.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_3 =
R"doc(Copy assignment from %copyable_unique_ptr replaces the currently-held object by
a copy of the object held in the source container, created using the source
object's copy constructor or `Clone()` method. The currently-held object (if
any) is deleted. If the source container is empty this one will be empty also
after the assignment. Nothing happens if the source and destination are the same
container.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_4 =
R"doc(Copy assignment from a compatible %copyable_unique_ptr replaces the currently-
held object by a copy of the object held in the source container, created using
the source object's copy constructor or `Clone()` method. The currently-held
object (if any) is deleted. If the source container is empty this one will be
empty also after the assignment. Nothing happens if the source and destination
are the same container.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_5 =
R"doc(Copy assignment from a standard `unique_ptr` replaces the currently-held object
by a copy of the object held in the source container, created using the source
object's copy constructor or `Clone()` method. The currently-held object (if
any) is deleted. If the source container is empty this one will be empty also
after the assignment. Nothing happens if the source and destination are the same
container.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_6 =
R"doc(Copy assignment from a compatible standard `unique_ptr` replaces the currently-
held object by a copy of the object held in the source container, created using
the source object's copy constructor or `Clone()` method. The currently-held
object (if any) is deleted. If the source container is empty this one will be
empty also after the assignment. Nothing happens if the source and destination
are the same container.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_7 =
R"doc(Move assignment replaces the currently-held object by the source object, leaving
the source empty. The currently-held object (if any) is deleted. The instance is
_not_ copied. Nothing happens if the source and destination are the same
containers.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_8 =
R"doc(Move assignment replaces the currently-held object by the compatible source
object, leaving the source empty. The currently-held object (if any) is deleted.
The instance is _not_ copied. Nothing happens if the source and destination are
the same containers.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_9 =
R"doc(Move assignment replaces the currently-held object by the source object, leaving
the source empty. The currently-held object (if any) is deleted. The instance is
_not_ copied. Nothing happens if the source and destination are the same
containers.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_assign_10 =
R"doc(Move assignment replaces the currently-held object by the compatible source
object, leaving the source empty. The currently-held object (if any) is deleted.
The instance is _not_ copied. Nothing happens if the source and destination are
the same containers.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_mul =
R"doc(Return a const reference to the contained object. Note that this is different
from `std::unique_ptr::operator*()` which would return a non-const reference (if
`T` is non-const), even if the container itself is const. For a const
%copyable_unique_ptr will always return a const reference to its contained
value.

.. warning::
    Currently %copyable_unique_ptr is a std::unique_ptr. As such, a const
    copyable_unique_ptr<Foo> can be upcast to a const unique_ptr<Foo> and the
    parent's behavior will provide a mutable reference. This is strongly
    discouraged and will break as the implementation of this class changes to
    shore up this gap in the const correctness protection.

@pre `this != nullptr` reports `true`.)doc";

static const char *__doc_dai_copyable_unique_ptr_operator_mul_2 =
R"doc(Return a writable reference to the contained object (if T is itself not const).
Note that you need write access to this container in order to get write access
to the object it contains.

We *strongly* recommend, that, if dereferencing a %copyable_unique_ptr without
the intention of mutating the underlying value, prefer to dereference a *const*
%copyable_unique_ptr (or use *my_ptr.get()) and not a mutable
%copyable_unique_ptr. As "copy-on-write" behavior is introduced in the future,
this recommended practice will prevent unwanted copies of the underlying value.

If %copyable_unique_ptr is instantiated on a const template parameter (e.g.,
`copyable_unique_ptr<const Foo>`), then operator*() must return a const
reference.

@pre `this != nullptr` reports `true`.)doc";

static const char *__doc_dai_detail_has_size_and_data = R"doc()doc";

static const char *__doc_dai_detail_is_complete = R"doc()doc";

static const char *__doc_dai_detail_is_container = R"doc()doc";

static const char *__doc_dai_detail_is_container_element_type_compatible = R"doc()doc";

static const char *__doc_dai_detail_is_span = R"doc()doc";

static const char *__doc_dai_detail_is_std_array = R"doc()doc";

static const char *__doc_dai_detail_span_storage = R"doc()doc";

static const char *__doc_dai_detail_span_storage_ptr = R"doc()doc";

static const char *__doc_dai_detail_span_storage_span_storage = R"doc()doc";

static const char *__doc_dai_detail_span_storage_span_storage_2 = R"doc()doc";

static const char *__doc_dai_downloadModelsFromZoo =
R"doc(Helper function allowing one to download all models specified in yaml files in
the given path and store them in the cache directory

Parameter ``path:``:
    Path to the directory containing yaml files

Parameter ``cacheDirectory:``:
    Cache directory where the cached models are stored, default is "". If
    cacheDirectory is set to "", this function checks the DEPTHAI_ZOO_CACHE_PATH
    environment variable and uses that if set, otherwise the default is used
    (see getDefaultCachePath).

Parameter ``apiKey:``:
    API key for the model zoo, default is "". If apiKey is set to "", this
    function checks the DEPTHAI_ZOO_API_KEY environment variable and uses that
    if set. Otherwise, no API key is used.

Parameter ``progressFormat:``:
    Format to use for progress output (possible values: pretty, json, none),
    default is "pretty"

Returns:
    bool: True if all models were downloaded successfully, false otherwise)doc";

static const char *__doc_dai_from_json = R"doc()doc";

static const char *__doc_dai_from_json_2 = R"doc()doc";

static const char *__doc_dai_from_json_3 = R"doc()doc";

static const char *__doc_dai_from_json_4 = R"doc()doc";

static const char *__doc_dai_from_json_5 = R"doc()doc";

static const char *__doc_dai_from_json_6 = R"doc()doc";

static const char *__doc_dai_from_json_7 = R"doc()doc";

static const char *__doc_dai_from_json_8 = R"doc()doc";

static const char *__doc_dai_from_json_9 = R"doc()doc";

static const char *__doc_dai_from_json_10 = R"doc()doc";

static const char *__doc_dai_from_json_11 = R"doc()doc";

static const char *__doc_dai_from_json_12 = R"doc()doc";

static const char *__doc_dai_from_json_13 = R"doc()doc";

static const char *__doc_dai_from_json_14 = R"doc()doc";

static const char *__doc_dai_from_json_15 = R"doc()doc";

static const char *__doc_dai_from_json_16 = R"doc()doc";

static const char *__doc_dai_from_json_17 = R"doc()doc";

static const char *__doc_dai_from_json_18 = R"doc()doc";

static const char *__doc_dai_from_json_19 = R"doc()doc";

static const char *__doc_dai_from_json_20 = R"doc()doc";

static const char *__doc_dai_from_json_21 = R"doc()doc";

static const char *__doc_dai_from_json_22 = R"doc()doc";

static const char *__doc_dai_from_json_23 = R"doc()doc";

static const char *__doc_dai_from_json_24 = R"doc()doc";

static const char *__doc_dai_from_json_25 = R"doc()doc";

static const char *__doc_dai_from_json_26 = R"doc()doc";

static const char *__doc_dai_from_json_27 = R"doc()doc";

static const char *__doc_dai_from_json_28 = R"doc()doc";

static const char *__doc_dai_from_json_29 = R"doc()doc";

static const char *__doc_dai_from_json_30 = R"doc()doc";

static const char *__doc_dai_from_json_31 = R"doc()doc";

static const char *__doc_dai_from_json_32 = R"doc()doc";

static const char *__doc_dai_from_json_33 = R"doc()doc";

static const char *__doc_dai_from_json_34 = R"doc()doc";

static const char *__doc_dai_from_json_35 = R"doc()doc";

static const char *__doc_dai_from_json_36 = R"doc()doc";

static const char *__doc_dai_from_json_37 = R"doc()doc";

static const char *__doc_dai_from_json_38 = R"doc()doc";

static const char *__doc_dai_from_json_39 = R"doc()doc";

static const char *__doc_dai_from_json_40 = R"doc()doc";

static const char *__doc_dai_from_json_41 = R"doc()doc";

static const char *__doc_dai_from_json_42 = R"doc()doc";

static const char *__doc_dai_from_json_43 = R"doc()doc";

static const char *__doc_dai_from_json_44 = R"doc()doc";

static const char *__doc_dai_from_json_45 = R"doc()doc";

static const char *__doc_dai_from_json_46 = R"doc()doc";

static const char *__doc_dai_from_json_47 = R"doc()doc";

static const char *__doc_dai_from_json_48 = R"doc()doc";

static const char *__doc_dai_from_json_49 = R"doc()doc";

static const char *__doc_dai_from_json_50 = R"doc()doc";

static const char *__doc_dai_from_json_51 = R"doc()doc";

static const char *__doc_dai_from_json_52 = R"doc()doc";

static const char *__doc_dai_from_json_53 = R"doc()doc";

static const char *__doc_dai_from_json_54 = R"doc()doc";

static const char *__doc_dai_from_json_55 = R"doc()doc";

static const char *__doc_dai_from_json_56 = R"doc()doc";

static const char *__doc_dai_from_json_57 = R"doc()doc";

static const char *__doc_dai_from_json_58 = R"doc()doc";

static const char *__doc_dai_from_json_59 = R"doc()doc";

static const char *__doc_dai_from_json_60 = R"doc()doc";

static const char *__doc_dai_from_json_61 = R"doc()doc";

static const char *__doc_dai_from_json_62 = R"doc()doc";

static const char *__doc_dai_from_json_63 = R"doc()doc";

static const char *__doc_dai_from_json_64 = R"doc()doc";

static const char *__doc_dai_from_json_65 = R"doc()doc";

static const char *__doc_dai_from_json_66 = R"doc()doc";

static const char *__doc_dai_from_json_67 = R"doc()doc";

static const char *__doc_dai_from_json_68 = R"doc()doc";

static const char *__doc_dai_from_json_69 = R"doc()doc";

static const char *__doc_dai_from_json_70 = R"doc()doc";

static const char *__doc_dai_from_json_71 = R"doc()doc";

static const char *__doc_dai_from_json_72 = R"doc()doc";

static const char *__doc_dai_from_json_73 = R"doc()doc";

static const char *__doc_dai_from_json_74 = R"doc()doc";

static const char *__doc_dai_from_json_75 = R"doc()doc";

static const char *__doc_dai_from_json_76 = R"doc()doc";

static const char *__doc_dai_from_json_77 = R"doc()doc";

static const char *__doc_dai_from_json_78 = R"doc()doc";

static const char *__doc_dai_from_json_79 = R"doc()doc";

static const char *__doc_dai_from_json_80 = R"doc()doc";

static const char *__doc_dai_from_json_81 = R"doc()doc";

static const char *__doc_dai_from_json_82 = R"doc()doc";

static const char *__doc_dai_from_json_83 = R"doc()doc";

static const char *__doc_dai_from_json_84 = R"doc()doc";

static const char *__doc_dai_from_json_85 = R"doc()doc";

static const char *__doc_dai_from_json_86 = R"doc()doc";

static const char *__doc_dai_get = R"doc()doc";

static const char *__doc_dai_getConfig = R"doc()doc";

static const char *__doc_dai_getMatrixInverse = R"doc()doc";

static const char *__doc_dai_getModelFromZoo =
R"doc(Get model from model zoo

Parameter ``modelDescription:``:
    Model description

Parameter ``useCached:``:
    Use cached model if present, default is true

Parameter ``cacheDirectory:``:
    Cache directory where the cached models are stored, default is "". If
    cacheDirectory is set to "", this function checks the DEPTHAI_ZOO_CACHE_PATH
    environment variable and uses that if set, otherwise the default value is
    used (see getDefaultCachePath).

Parameter ``apiKey:``:
    API key for the model zoo, default is "". If apiKey is set to "", this
    function checks the DEPTHAI_ZOO_API_KEY environment variable and uses that
    if set. Otherwise, no API key is used.

Parameter ``progressFormat:``:
    Format to use for progress output (possible values: pretty, json, none),
    default is "pretty"

Returns:
    std::filesystem::path: Path to the model in cache)doc";

static const char *__doc_dai_getNNArchiveConfigVersion = R"doc()doc";

static const char *__doc_dai_impl_ALIGN_UP = R"doc()doc";

static const char *__doc_dai_impl_ColorChange = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_ColorChange = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_ColorChange_2 = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_apply = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_build = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_ccAuxFrame = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvert = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToBGR888i = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToBGR888p = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToGRAY8 = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToNV12 = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToRGB888i = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToRGB888p = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_colorConvertToYUV420p = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_dstSpecs = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_from = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_logger = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_setLogger = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_srcSpecs = R"doc()doc";

static const char *__doc_dai_impl_ColorChange_to = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_height = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p1Offset = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p1Stride = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p2Offset = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p2Stride = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p3Offset = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_p3Stride = R"doc()doc";

static const char *__doc_dai_impl_FrameSpecs_width = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_ImageManipBuffer = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_ImageManipBuffer_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_data = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_data_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_data_3 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_getData = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_getData_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipBuffer_size = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_ImageManipMemory = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_ImageManipMemory_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_ImageManipMemory_3 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_data = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_data_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_data_3 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_getData = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_getData_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_getMaxSize = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_getOffset = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_offset = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_offset_2 = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_setOffset = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_setSize = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_shallowCopyFrom = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_size = R"doc()doc";

static const char *__doc_dai_impl_ImageManipMemory_span = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_ImageManipOperations = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_apply = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_base = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_build = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_clrChange = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_colormapFrame = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_convertInput = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_convertOnly = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_convertedFrame = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getMatrix = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputFrameSpecs = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputFrameType = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputHeight = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputPlaneSize = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputSize = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputStride = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getOutputWidth = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_getSrcCrops = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_inType = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_logger = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_matrix = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_matrixInv = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_mode = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_outputFrameType = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_outputOps = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_preprocCc = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_prevConfig = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_properties = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_srcCorners = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_srcSpecs = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_toString = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_type = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_warpEngine = R"doc()doc";

static const char *__doc_dai_impl_ImageManipOperations_warpedFrame = R"doc()doc";

static const char *__doc_dai_impl_RGBfromYUV = R"doc()doc";

static const char *__doc_dai_impl_Warp = R"doc()doc";

static const char *__doc_dai_impl_WarpH = R"doc()doc";

static const char *__doc_dai_impl_WarpH_apply = R"doc()doc";

static const char *__doc_dai_impl_WarpH_build = R"doc()doc";

static const char *__doc_dai_impl_WarpH_fastCvBorder = R"doc()doc";

static const char *__doc_dai_impl_WarpH_transform = R"doc()doc";

static const char *__doc_dai_impl_Warp_Warp = R"doc()doc";

static const char *__doc_dai_impl_Warp_Warp_2 = R"doc()doc";

static const char *__doc_dai_impl_Warp_apply = R"doc()doc";

static const char *__doc_dai_impl_Warp_background = R"doc()doc";

static const char *__doc_dai_impl_Warp_backgroundColor = R"doc()doc";

static const char *__doc_dai_impl_Warp_build = R"doc()doc";

static const char *__doc_dai_impl_Warp_dstSpecs = R"doc()doc";

static const char *__doc_dai_impl_Warp_init = R"doc()doc";

static const char *__doc_dai_impl_Warp_logger = R"doc()doc";

static const char *__doc_dai_impl_Warp_matrix = R"doc()doc";

static const char *__doc_dai_impl_Warp_setBackgroundColor = R"doc()doc";

static const char *__doc_dai_impl_Warp_setLogger = R"doc()doc";

static const char *__doc_dai_impl_Warp_sourceMaxX = R"doc()doc";

static const char *__doc_dai_impl_Warp_sourceMaxY = R"doc()doc";

static const char *__doc_dai_impl_Warp_sourceMinX = R"doc()doc";

static const char *__doc_dai_impl_Warp_sourceMinY = R"doc()doc";

static const char *__doc_dai_impl_Warp_srcSpecs = R"doc()doc";

static const char *__doc_dai_impl_Warp_type = R"doc()doc";

static const char *__doc_dai_impl_YUVfromRGB = R"doc()doc";

static const char *__doc_dai_impl_clampf = R"doc()doc";

static const char *__doc_dai_impl_clampi = R"doc()doc";

static const char *__doc_dai_impl_floatEq = R"doc()doc";

static const char *__doc_dai_impl_getAlignedOutputFrameSize = R"doc()doc";

static const char *__doc_dai_impl_getCcDstFrameSpecs = R"doc()doc";

static const char *__doc_dai_impl_getConfigString = R"doc()doc";

static const char *__doc_dai_impl_getDstFrameSpecs = R"doc()doc";

static const char *__doc_dai_impl_getFrameTypeInfo = R"doc()doc";

static const char *__doc_dai_impl_getFullTransform = R"doc()doc";

static const char *__doc_dai_impl_getHull = R"doc()doc";

static const char *__doc_dai_impl_getInverse = R"doc()doc";

static const char *__doc_dai_impl_getInverse_2 = R"doc()doc";

static const char *__doc_dai_impl_getOpStr = R"doc()doc";

static const char *__doc_dai_impl_getOuterRect = R"doc()doc";

static const char *__doc_dai_impl_getOuterRotatedRect = R"doc()doc";

static const char *__doc_dai_impl_getOutputSizeFromCorners = R"doc()doc";

static const char *__doc_dai_impl_getResizeMat = R"doc()doc";

static const char *__doc_dai_impl_getRotatedRectFromPoints = R"doc()doc";

static const char *__doc_dai_impl_getSrcFrameSpecs = R"doc()doc";

static const char *__doc_dai_impl_getSrcFrameSpecs_2 = R"doc()doc";

static const char *__doc_dai_impl_getTransform = R"doc()doc";

static const char *__doc_dai_impl_getTransformImpl = R"doc()doc";

static const char *__doc_dai_impl_getValidType = R"doc()doc";

static const char *__doc_dai_impl_isSingleChannel = R"doc()doc";

static const char *__doc_dai_impl_isSingleChannelu8 = R"doc()doc";

static const char *__doc_dai_impl_isSingleChannelu8_2 = R"doc()doc";

static const char *__doc_dai_impl_isTypeSupported = R"doc()doc";

static const char *__doc_dai_impl_loop = R"doc()doc";

static const char *__doc_dai_impl_matmul = R"doc()doc";

static const char *__doc_dai_impl_matvecmul = R"doc()doc";

static const char *__doc_dai_impl_matvecmul_2 = R"doc()doc";

static const char *__doc_dai_impl_overloaded = R"doc()doc";

static const char *__doc_dai_impl_transformFastCV = R"doc()doc";

static const char *__doc_dai_impl_transformOpenCV = R"doc()doc";

static const char *__doc_dai_initialize = R"doc()doc";

static const char *__doc_dai_initialize_2 = R"doc()doc";

static const char *__doc_dai_initialize_3 = R"doc()doc";

static const char *__doc_dai_initialize_4 = R"doc()doc";

static const char *__doc_dai_internal_NOP_EXTERNAL_STRUCTURE = R"doc()doc";

static const char *__doc_dai_internal_NOP_EXTERNAL_STRUCTURE_2 = R"doc()doc";

static const char *__doc_dai_internal_VariantReadNop = R"doc()doc";

static const char *__doc_dai_internal_VariantReadNop_2 = R"doc()doc";

static const char *__doc_dai_internal_VariantReadNop_operator_call = R"doc()doc";

static const char *__doc_dai_internal_VariantReadNop_operator_call_2 = R"doc()doc";

static const char *__doc_dai_internal_VariantSwitch = R"doc()doc";

static const char *__doc_dai_internal_VariantSwitch_2 = R"doc()doc";

static const char *__doc_dai_internal_VariantSwitch_operator_call = R"doc()doc";

static const char *__doc_dai_internal_VariantSwitch_operator_call_2 = R"doc()doc";

static const char *__doc_dai_internal_XLinkInProperties = R"doc(Specify properties for XLinkIn such as stream name, ...)doc";

static const char *__doc_dai_internal_XLinkInProperties_maxDataSize =
R"doc(Maximum input data size: NV12 --> 1.5 = (3/2) bytes per pixel, roughly 17.6MB
for a 4032x3056 frame)doc";

static const char *__doc_dai_internal_XLinkInProperties_numFrames = R"doc(Number of frames in pool)doc";

static const char *__doc_dai_internal_XLinkInProperties_streamName = R"doc(Name of stream)doc";

static const char *__doc_dai_internal_XLinkOutProperties = R"doc(Specify properties for XLinkOut such as stream name, ...)doc";

static const char *__doc_dai_internal_XLinkOutProperties_maxFpsLimit = R"doc(Set a limit to how many packets will be sent further to host)doc";

static const char *__doc_dai_internal_XLinkOutProperties_metadataOnly = R"doc(Whether to transfer data or only object attributes)doc";

static const char *__doc_dai_internal_XLinkOutProperties_streamName = R"doc(Name of stream)doc";

static const char *__doc_dai_internal_from_json = R"doc()doc";

static const char *__doc_dai_internal_from_json_2 = R"doc()doc";

static const char *__doc_dai_internal_to_json = R"doc()doc";

static const char *__doc_dai_internal_to_json_2 = R"doc()doc";

static const char *__doc_dai_isDatatypeSubclassOf = R"doc()doc";

static const char *__doc_dai_make_span = R"doc()doc";

static const char *__doc_dai_make_span_2 = R"doc()doc";

static const char *__doc_dai_make_span_3 = R"doc()doc";

static const char *__doc_dai_make_span_4 = R"doc()doc";

static const char *__doc_dai_make_span_5 = R"doc()doc";

static const char *__doc_dai_make_span_6 = R"doc()doc";

static const char *__doc_dai_matrix_createRotationMatrix = R"doc()doc";

static const char *__doc_dai_matrix_createScalingMatrix = R"doc()doc";

static const char *__doc_dai_matrix_createTranslationMatrix = R"doc()doc";

static const char *__doc_dai_matrix_matInv = R"doc()doc";

static const char *__doc_dai_matrix_matMul = R"doc()doc";

static const char *__doc_dai_matrix_printMatrix = R"doc()doc";

static const char *__doc_dai_model_ModelType = R"doc(Neural network model type)doc";

static const char *__doc_dai_model_ModelType_BLOB = R"doc()doc";

static const char *__doc_dai_model_ModelType_DLC = R"doc()doc";

static const char *__doc_dai_model_ModelType_NNARCHIVE = R"doc()doc";

static const char *__doc_dai_model_ModelType_OTHER = R"doc()doc";

static const char *__doc_dai_model_ModelType_SUPERBLOB = R"doc()doc";

static const char *__doc_dai_model_readModelType =
R"doc(Read model type from model path

Parameter ``modelPath``:
    Path to model

Returns:
    ModelType)doc";

static const char *__doc_dai_modelzoo_getDefaultCachePath = R"doc(Get the default cache path (where models are cached))doc";

static const char *__doc_dai_modelzoo_getDefaultModelsPath = R"doc(Get the default models path (where yaml files are stored))doc";

static const char *__doc_dai_modelzoo_getDownloadEndpoint = R"doc(Get the download endpoint (for model querying))doc";

static const char *__doc_dai_modelzoo_getHealthEndpoint = R"doc(Get the health endpoint (for internet check))doc";

static const char *__doc_dai_modelzoo_setDefaultCachePath =
R"doc(Set the default cache path (where models are cached)

Parameter ``path``:)doc";

static const char *__doc_dai_modelzoo_setDefaultModelsPath =
R"doc(Set the default models path (where yaml files are stored)

Parameter ``path``:)doc";

static const char *__doc_dai_modelzoo_setDownloadEndpoint =
R"doc(Set the download endpoint (for model querying)

Parameter ``endpoint``:)doc";

static const char *__doc_dai_modelzoo_setHealthEndpoint =
R"doc(Set the health endpoint (for internet check)

Parameter ``endpoint``:)doc";

static const char *__doc_dai_nn_archive_v1_Config =
R"doc(The main class of the multi/single-stage model config scheme (multi- stage
models consists of interconnected single-stage models).

@type config_version: str @ivar config_version: String representing config
schema version in format 'x.y' where x is major version and y is minor version
@type model: Model @ivar model: A Model object representing the neural network
used in the archive.)doc";

static const char *__doc_dai_nn_archive_v1_Config_configVersion =
R"doc(String representing config schema version in format 'x.y' where x is major
version and y is minor version.)doc";

static const char *__doc_dai_nn_archive_v1_Config_model = R"doc(A Model object representing the neural network used in the archive.)doc";

static const char *__doc_dai_nn_archive_v1_DataType =
R"doc(Data type of the input data (e.g., 'float32').

Represents all existing data types used in i/o streams of the model.

Precision of the model weights.

Data type of the output data (e.g., 'float32').)doc";

static const char *__doc_dai_nn_archive_v1_DataType_2 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_3 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_4 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_BOOLEAN = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_FLOAT16 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_FLOAT32 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_FLOAT64 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_INT16 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_INT32 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_INT4 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_INT64 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_INT8 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_STRING = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_UINT16 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_UINT32 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_UINT4 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_UINT64 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_DataType_UINT8 = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_Head =
R"doc(Represents head of a model.

@type name: str | None @ivar name: Optional name of the head. @type parser: str
@ivar parser: Name of the parser responsible for processing the models output.
@type outputs: List[str] | None @ivar outputs: Specify which outputs are fed
into the parser. If None, all outputs are fed. @type metadata: C{HeadMetadata} |
C{HeadObjectDetectionMetadata} | C{HeadClassificationMetadata} |
C{HeadObjectDetectionSSDMetadata} | C{HeadSegmentationMetadata} |
C{HeadYOLOMetadata} @ivar metadata: Metadata of the parser.)doc";

static const char *__doc_dai_nn_archive_v1_Head_metadata = R"doc(Metadata of the parser.)doc";

static const char *__doc_dai_nn_archive_v1_Head_name = R"doc(Optional name of the head.)doc";

static const char *__doc_dai_nn_archive_v1_Head_outputs = R"doc(Specify which outputs are fed into the parser. If None, all outputs are fed.)doc";

static const char *__doc_dai_nn_archive_v1_Head_parser = R"doc(Name of the parser responsible for processing the models output.)doc";

static const char *__doc_dai_nn_archive_v1_Input =
R"doc(Represents input stream of a model.

@type name: str @ivar name: Name of the input layer.

@type dtype: DataType @ivar dtype: Data type of the input data (e.g.,
'float32').

@type input_type: InputType @ivar input_type: Type of input data (e.g.,
'image').

@type shape: list @ivar shape: Shape of the input data as a list of integers
(e.g. [H,W], [H,W,C], [N,H,W,C], ...).

@type layout: str @ivar layout: Lettercode interpretation of the input data
dimensions (e.g., 'NCHW').

@type preprocessing: PreprocessingBlock @ivar preprocessing: Preprocessing steps
applied to the input data.)doc";

static const char *__doc_dai_nn_archive_v1_InputType = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_InputType_2 =
R"doc(Type of input data (e.g., 'image').

Represents a type of input the model is expecting.)doc";

static const char *__doc_dai_nn_archive_v1_InputType_IMAGE = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_InputType_RAW = R"doc()doc";

static const char *__doc_dai_nn_archive_v1_Input_dtype = R"doc(Data type of the input data (e.g., 'float32').)doc";

static const char *__doc_dai_nn_archive_v1_Input_inputType = R"doc(Type of input data (e.g., 'image').)doc";

static const char *__doc_dai_nn_archive_v1_Input_layout = R"doc(Lettercode interpretation of the input data dimensions (e.g., 'NCHW'))doc";

static const char *__doc_dai_nn_archive_v1_Input_name = R"doc(Name of the input layer.)doc";

static const char *__doc_dai_nn_archive_v1_Input_preprocessing = R"doc(Preprocessing steps applied to the input data.)doc";

static const char *__doc_dai_nn_archive_v1_Input_shape =
R"doc(Shape of the input data as a list of integers (e.g. [H,W], [H,W,C], [N,H,W,C],
...).)doc";

static const char *__doc_dai_nn_archive_v1_Metadata =
R"doc(Metadata of the parser.

Metadata for the object detection head.

@type classes: list @ivar classes: Names of object classes detected by the
model. @type n_classes: int @ivar n_classes: Number of object classes detected
by the model. @type iou_threshold: float @ivar iou_threshold: Non-max supression
threshold limiting boxes intersection. @type conf_threshold: float @ivar
conf_threshold: Confidence score threshold above which a detected object is
considered valid. @type max_det: int @ivar max_det: Maximum detections per
image. @type anchors: list @ivar anchors: Predefined bounding boxes of different
sizes and aspect ratios. The innermost lists are length 2 tuples of box sizes.
The middle lists are anchors for each output. The outmost lists go from smallest
to largest output.

Metadata for the classification head.

@type classes: list @ivar classes: Names of object classes classified by the
model. @type n_classes: int @ivar n_classes: Number of object classes classified
by the model. @type is_softmax: bool @ivar is_softmax: True, if output is
already softmaxed

Metadata for the SSD object detection head.

@type boxes_outputs: str @ivar boxes_outputs: Output name corresponding to
predicted bounding box coordinates. @type scores_outputs: str @ivar
scores_outputs: Output name corresponding to predicted bounding box confidence
scores.

Metadata for the segmentation head.

@type classes: list @ivar classes: Names of object classes segmented by the
model. @type n_classes: int @ivar n_classes: Number of object classes segmented
by the model. @type is_softmax: bool @ivar is_softmax: True, if output is
already softmaxed

Metadata for the YOLO head.

@type yolo_outputs: list @ivar yolo_outputs: A list of output names for each of
the different YOLO grid sizes. @type mask_outputs: list | None @ivar
mask_outputs: A list of output names for each mask output. @type protos_outputs:
str | None @ivar protos_outputs: Output name for the protos. @type
keypoints_outputs: list | None @ivar keypoints_outputs: A list of output names
for the keypoints. @type angles_outputs: list | None @ivar angles_outputs: A
list of output names for the angles. @type subtype: str @ivar subtype: YOLO
family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.) @type n_prototypes:
int | None @ivar n_prototypes: Number of prototypes per bbox in YOLO instance
segmnetation. @type n_keypoints: int | None @ivar n_keypoints: Number of
keypoints per bbox in YOLO keypoint detection. @type is_softmax: bool | None
@ivar is_softmax: True, if output is already softmaxed in YOLO instance
segmentation

Metadata for the basic head. It allows you to specify additional fields.

@type postprocessor_path: str | None @ivar postprocessor_path: Path to the
postprocessor.)doc";

static const char *__doc_dai_nn_archive_v1_MetadataClass =
R"doc(Metadata object defining the model metadata.

Represents metadata of a model.

@type name: str @ivar name: Name of the model. @type path: str @ivar path:
Relative path to the model executable.)doc";

static const char *__doc_dai_nn_archive_v1_MetadataClass_name = R"doc(Name of the model.)doc";

static const char *__doc_dai_nn_archive_v1_MetadataClass_path = R"doc(Relative path to the model executable.)doc";

static const char *__doc_dai_nn_archive_v1_MetadataClass_precision = R"doc(Precision of the model weights.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_anchors =
R"doc(Predefined bounding boxes of different sizes and aspect ratios. The innermost
lists are length 2 tuples of box sizes. The middle lists are anchors for each
output. The outmost lists go from smallest to largest output.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_anglesOutputs = R"doc(A list of output names for the angles.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_boxesOutputs = R"doc(Output name corresponding to predicted bounding box coordinates.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_classes = R"doc(Names of object classes recognized by the model.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_confThreshold = R"doc(Confidence score threshold above which a detected object is considered valid.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_extraParams = R"doc(Additional parameters)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_iouThreshold = R"doc(Non-max supression threshold limiting boxes intersection.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_isSoftmax =
R"doc(True, if output is already softmaxed.

True, if output is already softmaxed in YOLO instance segmentation.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_keypointsOutputs = R"doc(A list of output names for the keypoints.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_maskOutputs = R"doc(A list of output names for each mask output.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_maxDet = R"doc(Maximum detections per image.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_nClasses = R"doc(Number of object classes recognized by the model.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_nKeypoints = R"doc(Number of keypoints per bbox in YOLO keypoint detection.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_nPrototypes = R"doc(Number of prototypes per bbox in YOLO instance segmnetation.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_postprocessorPath = R"doc(Path to the postprocessor.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_protosOutputs = R"doc(Output name for the protos.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_scoresOutputs = R"doc(Output name corresponding to predicted bounding box confidence scores.)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_subtype = R"doc(YOLO family decoding subtype (e.g. yolov5, yolov6, yolov7 etc.).)doc";

static const char *__doc_dai_nn_archive_v1_Metadata_yoloOutputs = R"doc(A list of output names for each of the different YOLO grid sizes.)doc";

static const char *__doc_dai_nn_archive_v1_Model =
R"doc(A Model object representing the neural network used in the archive.

Class defining a single-stage model config scheme.

@type metadata: Metadata @ivar metadata: Metadata object defining the model
metadata. @type inputs: list @ivar inputs: List of Input objects defining the
model inputs. @type outputs: list @ivar outputs: List of Output objects defining
the model outputs. @type heads: list @ivar heads: List of Head objects defining
the model heads. If not defined, we assume a raw output.)doc";

static const char *__doc_dai_nn_archive_v1_Model_heads =
R"doc(List of Head objects defining the model heads. If not defined, we assume a raw
output.)doc";

static const char *__doc_dai_nn_archive_v1_Model_inputs = R"doc(List of Input objects defining the model inputs.)doc";

static const char *__doc_dai_nn_archive_v1_Model_metadata = R"doc(Metadata object defining the model metadata.)doc";

static const char *__doc_dai_nn_archive_v1_Model_outputs = R"doc(List of Output objects defining the model outputs.)doc";

static const char *__doc_dai_nn_archive_v1_Output =
R"doc(Represents output stream of a model.

@type name: str @ivar name: Name of the output layer. @type dtype: DataType
@ivar dtype: Data type of the output data (e.g., 'float32').)doc";

static const char *__doc_dai_nn_archive_v1_Output_dtype = R"doc(Data type of the output data (e.g., 'float32').)doc";

static const char *__doc_dai_nn_archive_v1_Output_layout = R"doc(List of letters describing the output layout (e.g. 'NC').)doc";

static const char *__doc_dai_nn_archive_v1_Output_name = R"doc(Name of the output layer.)doc";

static const char *__doc_dai_nn_archive_v1_Output_shape = R"doc(Shape of the output as a list of integers (e.g. [1, 1000]).)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock =
R"doc(Preprocessing steps applied to the input data.

Represents preprocessing operations applied to the input data.

@type mean: list | None @ivar mean: Mean values in channel order. Order depends
on the order in which the model was trained on. @type scale: list | None @ivar
scale: Standardization values in channel order. Order depends on the order in
which the model was trained on. @type reverse_channels: bool | None @ivar
reverse_channels: If True input to the model is RGB else BGR. @type
interleaved_to_planar: bool | None @ivar interleaved_to_planar: If True input to
the model is interleaved (NHWC) else planar (NCHW). @type dai_type: str | None
@ivar dai_type: DepthAI input type which is read by DepthAI to automatically
setup the pipeline.)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock_daiType = R"doc(DepthAI input type which is read by DepthAI to automatically setup the pipeline.)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock_interleavedToPlanar = R"doc(If True input to the model is interleaved (NHWC) else planar (NCHW).)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock_mean =
R"doc(Mean values in channel order. Order depends on the order in which the model was
trained on.)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock_reverseChannels = R"doc(If True input to the model is RGB else BGR.)doc";

static const char *__doc_dai_nn_archive_v1_PreprocessingBlock_scale =
R"doc(Standardization values in channel order. Order depends on the order in which the
model was trained on.)doc";

static const char *__doc_dai_node_AprilTag = R"doc(AprilTag node.)doc";

static const char *__doc_dai_node_AprilTag_AprilTag = R"doc()doc";

static const char *__doc_dai_node_AprilTag_AprilTag_2 = R"doc()doc";

static const char *__doc_dai_node_AprilTag_buildInternal = R"doc()doc";

static const char *__doc_dai_node_AprilTag_getNumThreads =
R"doc(Get number of threads to use for AprilTag detection.

Returns:
    Number of threads to use.)doc";

static const char *__doc_dai_node_AprilTag_getProperties = R"doc()doc";

static const char *__doc_dai_node_AprilTag_getWaitForConfigInput =
R"doc(Get whether or not wait until configuration message arrives to inputConfig
Input.)doc";

static const char *__doc_dai_node_AprilTag_initialConfig = R"doc(Initial config to use when calculating spatial location data.)doc";

static const char *__doc_dai_node_AprilTag_inputConfig =
R"doc(Input AprilTagConfig message with ability to modify parameters in runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_AprilTag_inputImage =
R"doc(Input message with depth data used to retrieve spatial information about
detected object. Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_AprilTag_out = R"doc(Outputs AprilTags message that carries spatial location results.)doc";

static const char *__doc_dai_node_AprilTag_outConfig = R"doc(Outputs AprilTagConfig message that contains current configuration.)doc";

static const char *__doc_dai_node_AprilTag_passthroughInputImage =
R"doc(Passthrough message on which the calculation was performed. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_AprilTag_run = R"doc()doc";

static const char *__doc_dai_node_AprilTag_runOnHost = R"doc(Check if the node is set to run on host)doc";

static const char *__doc_dai_node_AprilTag_runOnHostVar = R"doc()doc";

static const char *__doc_dai_node_AprilTag_setNumThreads =
R"doc(Set number of threads to use for AprilTag detection.

Parameter ``numThreads``:
    Number of threads to use.)doc";

static const char *__doc_dai_node_AprilTag_setRunOnHost =
R"doc(Specify whether to run on host or device By default, the node will run on
device.)doc";

static const char *__doc_dai_node_AprilTag_setWaitForConfigInput =
R"doc(Specify whether or not wait until configuration message arrives to inputConfig
Input.

Parameter ``wait``:
    True to wait for configuration message, false otherwise.)doc";

static const char *__doc_dai_node_BasaltVIO =
R"doc(Basalt Visual Inertial Odometry node. Performs VIO on stereo images and IMU
data.)doc";

static const char *__doc_dai_node_BasaltVIO_BasaltVIO = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_Impl = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_buildInternal = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_configPath = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_imu = R"doc(Input IMU data.)doc";

static const char *__doc_dai_node_BasaltVIO_imuCB = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_imuUpdateRate = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_inSync = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_initialize = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_initialized = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_inputs = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_lastImgData = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_left = R"doc(Input left image on which VIO is performed.)doc";

static const char *__doc_dai_node_BasaltVIO_leftImg = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_leftInputName = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_optFlowPtr = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_passthrough = R"doc(Output passthrough of left image.)doc";

static const char *__doc_dai_node_BasaltVIO_pimpl = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_right = R"doc(Input right image on which VIO is performed.)doc";

static const char *__doc_dai_node_BasaltVIO_rightInputName = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_run = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_setConfigPath = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_setDefaultVIOConfig = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_setImuUpdateRate = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_setLocalTransform = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_setUseSpecTranslation = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_stereoCB = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_stop = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_sync = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_threadNum = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_transform = R"doc(Output transform data.)doc";

static const char *__doc_dai_node_BasaltVIO_useSpecTranslation = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_vio = R"doc()doc";

static const char *__doc_dai_node_BasaltVIO_vioConfig = R"doc(VIO configuration file.)doc";

static const char *__doc_dai_node_BasaltVIO_vioTNSec = R"doc()doc";

static const char *__doc_dai_node_BenchmarkIn = R"doc()doc";

static const char *__doc_dai_node_BenchmarkIn_input = R"doc(Receive messages as fast as possible)doc";

static const char *__doc_dai_node_BenchmarkIn_logReportsAsWarnings = R"doc(Log the reports as warnings)doc";

static const char *__doc_dai_node_BenchmarkIn_measureIndividualLatencies = R"doc(Attach latencies to the report)doc";

static const char *__doc_dai_node_BenchmarkIn_passthrough = R"doc(Passthrough for input messages (so the node can be placed between other nodes))doc";

static const char *__doc_dai_node_BenchmarkIn_report = R"doc(Send a benchmark report when the set number of messages are received)doc";

static const char *__doc_dai_node_BenchmarkIn_run = R"doc()doc";

static const char *__doc_dai_node_BenchmarkIn_runOnHost = R"doc(Check if the node is set to run on host)doc";

static const char *__doc_dai_node_BenchmarkIn_runOnHostVar = R"doc()doc";

static const char *__doc_dai_node_BenchmarkIn_sendReportEveryNMessages = R"doc(Specify how many messages to measure for each report)doc";

static const char *__doc_dai_node_BenchmarkIn_setRunOnHost =
R"doc(Specify whether to run on host or device By default, the node will run on
device.)doc";

static const char *__doc_dai_node_BenchmarkOut = R"doc()doc";

static const char *__doc_dai_node_BenchmarkOut_input = R"doc(Message that will be sent repeatedly)doc";

static const char *__doc_dai_node_BenchmarkOut_out = R"doc(Send messages out as fast as possible)doc";

static const char *__doc_dai_node_BenchmarkOut_run = R"doc()doc";

static const char *__doc_dai_node_BenchmarkOut_runOnHost = R"doc(Check if the node is set to run on host)doc";

static const char *__doc_dai_node_BenchmarkOut_runOnHostVar = R"doc()doc";

static const char *__doc_dai_node_BenchmarkOut_setFps = R"doc(Set FPS at which the node is sending out messages. 0 means as fast as possible)doc";

static const char *__doc_dai_node_BenchmarkOut_setNumMessagesToSend =
R"doc(Sets number of messages to send, by default send messages indefinitely

Parameter ``num``:
    number of messages to send)doc";

static const char *__doc_dai_node_BenchmarkOut_setRunOnHost =
R"doc(Specify whether to run on host or device By default, the node will run on
device.)doc";

static const char *__doc_dai_node_Camera = R"doc()doc";

static const char *__doc_dai_node_Camera_Camera = R"doc()doc";

static const char *__doc_dai_node_Camera_Camera_2 = R"doc()doc";

static const char *__doc_dai_node_Camera_Camera_3 = R"doc()doc";

static const char *__doc_dai_node_Camera_Impl = R"doc()doc";

static const char *__doc_dai_node_Camera_build =
R"doc(Build with a specific board socket

Parameter ``boardSocket``:
    Board socket to use

Parameter ``sensorResolution``:
    Sensor resolution to use - by default it's auto-detected from the requested
    outputs

Parameter ``sensorFps``:
    Sensor FPS to use - by default it's auto-detected from the requested outputs
    (maximum is used))doc";

static const char *__doc_dai_node_Camera_build_2 = R"doc(Build with a specific board socket and mock input)doc";

static const char *__doc_dai_node_Camera_build_3 = R"doc(Build with mock input)doc";

static const char *__doc_dai_node_Camera_buildStage1 = R"doc()doc";

static const char *__doc_dai_node_Camera_cameraFeatures = R"doc()doc";

static const char *__doc_dai_node_Camera_create = R"doc()doc";

static const char *__doc_dai_node_Camera_create_2 = R"doc()doc";

static const char *__doc_dai_node_Camera_dynamicOutputs = R"doc()doc";

static const char *__doc_dai_node_Camera_getBoardSocket =
R"doc(Retrieves which board socket to use

Returns:
    Board socket to use)doc";

static const char *__doc_dai_node_Camera_getMaxHeight = R"doc(Get max height of the camera (can only be called after build))doc";

static const char *__doc_dai_node_Camera_getMaxRequestedFps = R"doc()doc";

static const char *__doc_dai_node_Camera_getMaxRequestedHeight = R"doc()doc";

static const char *__doc_dai_node_Camera_getMaxRequestedWidth = R"doc()doc";

static const char *__doc_dai_node_Camera_getMaxWidth = R"doc(Get max width of the camera (can only be called after build))doc";

static const char *__doc_dai_node_Camera_getNodeRecordParams = R"doc()doc";

static const char *__doc_dai_node_Camera_getProperties = R"doc()doc";

static const char *__doc_dai_node_Camera_getReplayInput = R"doc()doc";

static const char *__doc_dai_node_Camera_initialControl = R"doc(Initial control options to apply to sensor)doc";

static const char *__doc_dai_node_Camera_inputControl = R"doc(Input for CameraControl message, which can modify camera parameters in runtime)doc";

static const char *__doc_dai_node_Camera_isBuilt = R"doc()doc";

static const char *__doc_dai_node_Camera_isSourceNode = R"doc()doc";

static const char *__doc_dai_node_Camera_mockIsp =
R"doc(Input for mocking 'isp' functionality on RVC2. Default queue is blocking with
size 8)doc";

static const char *__doc_dai_node_Camera_pimpl = R"doc()doc";

static const char *__doc_dai_node_Camera_raw =
R"doc(Outputs ImgFrame message that carries RAW10-packed (MIPI CSI-2 format) frame
data.

Captured directly from the camera sensor, and the source for the 'isp' output.)doc";

static const char *__doc_dai_node_Camera_requestFullResolutionOutput =
R"doc(Get a high resolution output with full FOV on the sensor. By default the
function will not use the resolutions higher than 5000x4000, as those often need
a lot of resources, making them hard to use in combination with other nodes.

Parameter ``type``:
    Type of the output (NV12, BGR, ...) - by default it's auto-selected for best
    performance

Parameter ``fps``:
    FPS of the output - by default it's auto-selected to highest possible that a
    sensor config support or 30, whichever is lower

Parameter ``useHighestResolution``:
    If true, the function will use the highest resolution available on the
    sensor, even if it's higher than 5000x4000)doc";

static const char *__doc_dai_node_Camera_requestOutput = R"doc(Get video output with specified size.)doc";

static const char *__doc_dai_node_Camera_requestOutput_2 = R"doc(Request output with advanced controls. Mainly to be used by custom node writers.)doc";

static const char *__doc_dai_node_Camera_setMockIsp =
R"doc(Set mock ISP for Camera node. Automatically sets mockIsp size.

Parameter ``replay``:
    ReplayVideo node to use as mock ISP)doc";

static const char *__doc_dai_node_ColorCamera = R"doc(ColorCamera node. For use with color sensors.)doc";

static const char *__doc_dai_node_ColorCamera_ColorCamera = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_ColorCamera_2 = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_frameEvent =
R"doc(Outputs metadata-only ImgFrame message as an early indicator of an incoming
frame.

It's sent on the MIPI SoF (start-of-frame) event, just after the exposure of the
current frame has finished and before the exposure for next frame starts. Could
be used to synchronize various processes with camera capture. Fields populated:
camera id, sequence number, timestamp)doc";

static const char *__doc_dai_node_ColorCamera_getBoardSocket =
R"doc(Retrieves which board socket to use

Returns:
    Board socket to use)doc";

static const char *__doc_dai_node_ColorCamera_getCamId = R"doc(Get which color camera to use)doc";

static const char *__doc_dai_node_ColorCamera_getCamera =
R"doc(Retrieves which camera to use by name

Returns:
    Name of the camera to use)doc";

static const char *__doc_dai_node_ColorCamera_getColorOrder = R"doc(Get color order of preview output frames. RGB or BGR)doc";

static const char *__doc_dai_node_ColorCamera_getFp16 = R"doc(Get fp16 (0..255) data of preview output frames)doc";

static const char *__doc_dai_node_ColorCamera_getFps =
R"doc(Get rate at which camera should produce frames

Returns:
    Rate in frames per second)doc";

static const char *__doc_dai_node_ColorCamera_getFrameEventFilter = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_getImageOrientation = R"doc(Get camera image orientation)doc";

static const char *__doc_dai_node_ColorCamera_getInterleaved = R"doc(Get planar or interleaved data of preview output frames)doc";

static const char *__doc_dai_node_ColorCamera_getIspHeight = R"doc(Get 'isp' output height)doc";

static const char *__doc_dai_node_ColorCamera_getIspNumFramesPool = R"doc(Get number of frames in isp pool)doc";

static const char *__doc_dai_node_ColorCamera_getIspSize = R"doc(Get 'isp' output resolution as size, after scaling)doc";

static const char *__doc_dai_node_ColorCamera_getIspWidth = R"doc(Get 'isp' output width)doc";

static const char *__doc_dai_node_ColorCamera_getNodeRecordParams = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_getPreviewHeight = R"doc(Get preview height)doc";

static const char *__doc_dai_node_ColorCamera_getPreviewKeepAspectRatio =
R"doc(See also:
    setPreviewKeepAspectRatio

Returns:
    Preview keep aspect ratio option)doc";

static const char *__doc_dai_node_ColorCamera_getPreviewNumFramesPool = R"doc(Get number of frames in preview pool)doc";

static const char *__doc_dai_node_ColorCamera_getPreviewSize = R"doc(Get preview size as tuple)doc";

static const char *__doc_dai_node_ColorCamera_getPreviewType = R"doc(Get the preview type)doc";

static const char *__doc_dai_node_ColorCamera_getPreviewWidth = R"doc(Get preview width)doc";

static const char *__doc_dai_node_ColorCamera_getProperties = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_getRawNumFramesPool = R"doc(Get number of frames in raw pool)doc";

static const char *__doc_dai_node_ColorCamera_getRecordOutput = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_getReplayInput = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_getResolution = R"doc(Get sensor resolution)doc";

static const char *__doc_dai_node_ColorCamera_getResolutionHeight = R"doc(Get sensor resolution height)doc";

static const char *__doc_dai_node_ColorCamera_getResolutionSize = R"doc(Get sensor resolution as size)doc";

static const char *__doc_dai_node_ColorCamera_getResolutionWidth = R"doc(Get sensor resolution width)doc";

static const char *__doc_dai_node_ColorCamera_getScaledSize = R"doc(Computes the scaled size given numerator and denominator)doc";

static const char *__doc_dai_node_ColorCamera_getSensorCrop =
R"doc(Returns:
    Sensor top left crop coordinates)doc";

static const char *__doc_dai_node_ColorCamera_getSensorCropX = R"doc(Get sensor top left x crop coordinate)doc";

static const char *__doc_dai_node_ColorCamera_getSensorCropY = R"doc(Get sensor top left y crop coordinate)doc";

static const char *__doc_dai_node_ColorCamera_getStillHeight = R"doc(Get still height)doc";

static const char *__doc_dai_node_ColorCamera_getStillNumFramesPool = R"doc(Get number of frames in still pool)doc";

static const char *__doc_dai_node_ColorCamera_getStillSize = R"doc(Get still size as tuple)doc";

static const char *__doc_dai_node_ColorCamera_getStillWidth = R"doc(Get still width)doc";

static const char *__doc_dai_node_ColorCamera_getVideoHeight = R"doc(Get video height)doc";

static const char *__doc_dai_node_ColorCamera_getVideoNumFramesPool = R"doc(Get number of frames in video pool)doc";

static const char *__doc_dai_node_ColorCamera_getVideoSize = R"doc(Get video size as tuple)doc";

static const char *__doc_dai_node_ColorCamera_getVideoWidth = R"doc(Get video width)doc";

static const char *__doc_dai_node_ColorCamera_initialControl = R"doc(Initial control options to apply to sensor)doc";

static const char *__doc_dai_node_ColorCamera_inputControl = R"doc(Input for CameraControl message, which can modify camera parameters in runtime)doc";

static const char *__doc_dai_node_ColorCamera_isSourceNode = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_isp =
R"doc(Outputs ImgFrame message that carries YUV420 planar (I420/IYUV) frame data.

Generated by the ISP engine, and the source for the 'video', 'preview' and
'still' outputs)doc";

static const char *__doc_dai_node_ColorCamera_mockIsp = R"doc(Input for mocking 'isp' functionality. Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_ColorCamera_preview =
R"doc(Outputs ImgFrame message that carries BGR/RGB planar/interleaved encoded frame
data.

Suitable for use with NeuralNetwork node)doc";

static const char *__doc_dai_node_ColorCamera_raw =
R"doc(Outputs ImgFrame message that carries RAW10-packed (MIPI CSI-2 format) frame
data.

Captured directly from the camera sensor, and the source for the 'isp' output.)doc";

static const char *__doc_dai_node_ColorCamera_sensorCenterCrop = R"doc(Specify sensor center crop. Resolution size / video size)doc";

static const char *__doc_dai_node_ColorCamera_setBoardSocket =
R"doc(Specify which board socket to use

Parameter ``boardSocket``:
    Board socket to use)doc";

static const char *__doc_dai_node_ColorCamera_setCamId = R"doc(Set which color camera to use)doc";

static const char *__doc_dai_node_ColorCamera_setCamera =
R"doc(Specify which camera to use by name

Parameter ``name``:
    Name of the camera to use)doc";

static const char *__doc_dai_node_ColorCamera_setColorOrder = R"doc(Set color order of preview output images. RGB or BGR)doc";

static const char *__doc_dai_node_ColorCamera_setFp16 = R"doc(Set fp16 (0..255) data type of preview output frames)doc";

static const char *__doc_dai_node_ColorCamera_setFps =
R"doc(Set rate at which camera should produce frames

Parameter ``fps``:
    Rate in frames per second)doc";

static const char *__doc_dai_node_ColorCamera_setFrameEventFilter = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_setImageOrientation = R"doc(Set camera image orientation)doc";

static const char *__doc_dai_node_ColorCamera_setInterleaved = R"doc(Set planar or interleaved data of preview output frames)doc";

static const char *__doc_dai_node_ColorCamera_setIsp3aFps =
R"doc(Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls
etc.). Default (0) matches the camera FPS, meaning that 3A is running on each
frame. Reducing the rate of 3A reduces the CPU usage on CSS, but also increases
the convergence rate of 3A. Note that camera controls will be processed at this
rate. E.g. if camera is running at 30 fps, and camera control is sent at every
frame, but 3A fps is set to 15, the camera control messages will be processed at
15 fps rate, which will lead to queueing.)doc";

static const char *__doc_dai_node_ColorCamera_setIspNumFramesPool = R"doc(Set number of frames in isp pool)doc";

static const char *__doc_dai_node_ColorCamera_setIspScale =
R"doc(Set 'isp' output scaling (numerator/denominator), preserving the aspect ratio.
The fraction numerator/denominator is simplified first to a irreducible form,
then a set of hardware scaler constraints applies: max numerator = 16, max
denominator = 63)doc";

static const char *__doc_dai_node_ColorCamera_setIspScale_2 = R"doc(Set 'isp' output scaling, as a tuple <numerator, denominator>)doc";

static const char *__doc_dai_node_ColorCamera_setIspScale_3 =
R"doc(Set 'isp' output scaling, per each direction. If the horizontal scaling factor
(horizNum/horizDen) is different than the vertical scaling factor
(vertNum/vertDen), a distorted (stretched or squished) image is generated)doc";

static const char *__doc_dai_node_ColorCamera_setIspScale_4 = R"doc(Set 'isp' output scaling, per each direction, as <numerator, denominator> tuples)doc";

static const char *__doc_dai_node_ColorCamera_setMockIspSize = R"doc()doc";

static const char *__doc_dai_node_ColorCamera_setNumFramesPool = R"doc(Set number of frames in all pools)doc";

static const char *__doc_dai_node_ColorCamera_setPreviewKeepAspectRatio =
R"doc(Specifies whether preview output should preserve aspect ratio, after downscaling
from video size or not.

Parameter ``keep``:
    If true, a larger crop region will be considered to still be able to create
    the final image in the specified aspect ratio. Otherwise video size is
    resized to fit preview size)doc";

static const char *__doc_dai_node_ColorCamera_setPreviewNumFramesPool = R"doc(Set number of frames in preview pool)doc";

static const char *__doc_dai_node_ColorCamera_setPreviewSize = R"doc(Set preview output size)doc";

static const char *__doc_dai_node_ColorCamera_setPreviewSize_2 = R"doc(Set preview output size, as a tuple <width, height>)doc";

static const char *__doc_dai_node_ColorCamera_setPreviewType = R"doc(Set type of preview output images.)doc";

static const char *__doc_dai_node_ColorCamera_setRawNumFramesPool = R"doc(Set number of frames in raw pool)doc";

static const char *__doc_dai_node_ColorCamera_setRawOutputPacked =
R"doc(Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
The packed format is more efficient, consuming less memory on device, and less
data to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on
3 bytes. When packing is disabled (`false`), data is saved lsb-aligned, e.g. a
RAW10 pixel will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
Default is auto: enabled for standard color/monochrome cameras where ISP can
work with both packed/unpacked, but disabled for other cameras like ToF.)doc";

static const char *__doc_dai_node_ColorCamera_setResolution = R"doc(Set sensor resolution)doc";

static const char *__doc_dai_node_ColorCamera_setSensorCrop =
R"doc(Specifies the cropping that happens when converting ISP to video output. By
default, video will be center cropped from the ISP output. Note that this
doesn't actually do on-sensor cropping (and MIPI-stream only that region), but
it does postprocessing on the ISP (on RVC).

Parameter ``x``:
    Top left X coordinate

Parameter ``y``:
    Top left Y coordinate)doc";

static const char *__doc_dai_node_ColorCamera_setStillNumFramesPool = R"doc(Set number of frames in preview pool)doc";

static const char *__doc_dai_node_ColorCamera_setStillSize = R"doc(Set still output size)doc";

static const char *__doc_dai_node_ColorCamera_setStillSize_2 = R"doc(Set still output size, as a tuple <width, height>)doc";

static const char *__doc_dai_node_ColorCamera_setVideoNumFramesPool = R"doc(Set number of frames in preview pool)doc";

static const char *__doc_dai_node_ColorCamera_setVideoSize = R"doc(Set video output size)doc";

static const char *__doc_dai_node_ColorCamera_setVideoSize_2 = R"doc(Set video output size, as a tuple <width, height>)doc";

static const char *__doc_dai_node_ColorCamera_still =
R"doc(Outputs ImgFrame message that carries NV12 encoded (YUV420, UV plane
interleaved) frame data.

The message is sent only when a CameraControl message arrives to inputControl
with captureStill command set.)doc";

static const char *__doc_dai_node_ColorCamera_video =
R"doc(Outputs ImgFrame message that carries NV12 encoded (YUV420, UV plane
interleaved) frame data.

Suitable for use with VideoEncoder node)doc";

static const char *__doc_dai_node_DetectionNetwork = R"doc(DetectionNetwork, base for different network specializations)doc";

static const char *__doc_dai_node_DetectionNetwork_DetectionNetwork = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_build = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_build_2 = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_build_3 = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_build_4 = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_build_5 = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_buildInternal = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_create = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_createNNArchive = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_detectionParser = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_getClasses = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_getConfidenceThreshold =
R"doc(Retrieves threshold at which to filter the rest of the detections.

Returns:
    Detection confidence)doc";

static const char *__doc_dai_node_DetectionNetwork_getNumInferenceThreads =
R"doc(How many inference threads will be used to run the network

Returns:
    Number of threads, 0, 1 or 2. Zero means AUTO)doc";

static const char *__doc_dai_node_DetectionNetwork_getRequiredInputs = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_input =
R"doc(Input message with data to be inferred upon Default queue is blocking with size
5)doc";

static const char *__doc_dai_node_DetectionNetwork_neuralNetwork = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_out =
R"doc(Outputs ImgDetections message that carries parsed detection results. Overrides
NeuralNetwork 'out' with ImgDetections output message type.)doc";

static const char *__doc_dai_node_DetectionNetwork_outNetwork = R"doc(Outputs unparsed inference results.)doc";

static const char *__doc_dai_node_DetectionNetwork_passthrough =
R"doc(Passthrough message on which the inference was performed.

Suitable for when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_DetectionNetwork_setBackend =
R"doc(Specifies backend to use

Parameter ``backend``:
    String specifying backend to use)doc";

static const char *__doc_dai_node_DetectionNetwork_setBackendProperties =
R"doc(Set backend properties

Parameter ``backendProperties``:
    backend properties map)doc";

static const char *__doc_dai_node_DetectionNetwork_setBlob =
R"doc(Load network blob into assets and use once pipeline is started.

Parameter ``blob``:
    Network blob)doc";

static const char *__doc_dai_node_DetectionNetwork_setBlob_2 =
R"doc(Same functionality as the setBlobPath(). Load network blob into assets and use
once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_DetectionNetwork_setBlobPath =
R"doc(Load network blob into assets and use once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_DetectionNetwork_setConfidenceThreshold =
R"doc(Specifies confidence threshold at which to filter the rest of the detections.

Parameter ``thresh``:
    Detection confidence must be greater than specified threshold to be added to
    the list)doc";

static const char *__doc_dai_node_DetectionNetwork_setFromModelZoo =
R"doc(Download model from zoo and set it for this Node

Parameter ``description:``:
    Model description to download

Parameter ``useCached:``:
    Use cached model if available)doc";

static const char *__doc_dai_node_DetectionNetwork_setFromModelZoo_2 =
R"doc(Download model from zoo and set it for this node.

Parameter ``description:``:
    Model description to download

Parameter ``numShaves:``:
    Number of shaves to use

Parameter ``useCached:``:
    Use cached model if available)doc";

static const char *__doc_dai_node_DetectionNetwork_setModelPath =
R"doc(Load network model into assets.

Parameter ``modelPath``:
    Path to the model file.)doc";

static const char *__doc_dai_node_DetectionNetwork_setNNArchive =
R"doc(Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default
number of shaves.

Parameter ``nnArchive:``:
    NNArchive to set)doc";

static const char *__doc_dai_node_DetectionNetwork_setNNArchive_2 =
R"doc(Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB

Parameter ``nnArchive:``:
    NNArchive to set

Parameter ``numShaves:``:
    Number of shaves to use)doc";

static const char *__doc_dai_node_DetectionNetwork_setNNArchiveBlob = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_setNNArchiveOther = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_setNNArchiveSuperblob = R"doc()doc";

static const char *__doc_dai_node_DetectionNetwork_setNumInferenceThreads =
R"doc(How many threads should the node use to run the network.

Parameter ``numThreads``:
    Number of threads to dedicate to this node)doc";

static const char *__doc_dai_node_DetectionNetwork_setNumNCEPerInferenceThread =
R"doc(How many Neural Compute Engines should a single thread use for inference

Parameter ``numNCEPerThread``:
    Number of NCE per thread)doc";

static const char *__doc_dai_node_DetectionNetwork_setNumPoolFrames =
R"doc(Specifies how many frames will be available in the pool

Parameter ``numFrames``:
    How many frames will pool have)doc";

static const char *__doc_dai_node_DetectionNetwork_setNumShavesPerInferenceThread =
R"doc(How many Shaves should a single thread use for inference

Parameter ``numShavesPerThread``:
    Number of shaves per thread)doc";

static const char *__doc_dai_node_DetectionParser =
R"doc(DetectionParser node. Parses detection results from different neural networks
and is being used internally by MobileNetDetectionNetwork and
YoloDetectionNetwork.)doc";

static const char *__doc_dai_node_DetectionParser_archiveConfig = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_build =
R"doc(Build DetectionParser node. Connect output to this node's input. Also call
setNNArchive() with provided NNArchive.

Parameter ``nnInput:``:
    Output to link

Parameter ``nnArchive:``:
    Neural network archive)doc";

static const char *__doc_dai_node_DetectionParser_getAnchorMasks = R"doc(Get anchor masks)doc";

static const char *__doc_dai_node_DetectionParser_getAnchors = R"doc(Get anchors)doc";

static const char *__doc_dai_node_DetectionParser_getClasses = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_getConfidenceThreshold =
R"doc(Retrieves threshold at which to filter the rest of the detections.

Returns:
    Detection confidence)doc";

static const char *__doc_dai_node_DetectionParser_getCoordinateSize = R"doc(Get coordianate size)doc";

static const char *__doc_dai_node_DetectionParser_getIouThreshold = R"doc(Get Iou threshold)doc";

static const char *__doc_dai_node_DetectionParser_getNNArchiveVersionedConfig = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_getNNFamily = R"doc(Gets NN Family to parse)doc";

static const char *__doc_dai_node_DetectionParser_getNumClasses = R"doc(Get num classes)doc";

static const char *__doc_dai_node_DetectionParser_getNumFramesPool = R"doc(Returns number of frames in pool)doc";

static const char *__doc_dai_node_DetectionParser_imageIn = R"doc(Input for image that produced the detection - image size can be taken from here)doc";

static const char *__doc_dai_node_DetectionParser_input =
R"doc(Input NN results with detection data to parse Default queue is blocking with
size 5)doc";

static const char *__doc_dai_node_DetectionParser_mArchive = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_out = R"doc(Outputs image frame with detected edges)doc";

static const char *__doc_dai_node_DetectionParser_setAnchorMasks = R"doc(Set anchor masks)doc";

static const char *__doc_dai_node_DetectionParser_setAnchors = R"doc(Set anchors)doc";

static const char *__doc_dai_node_DetectionParser_setAnchors_2 = R"doc(Set anchors with masks)doc";

static const char *__doc_dai_node_DetectionParser_setBlob =
R"doc(Retrieves some input tensor information from the blob

Parameter ``blob``:
    OpenVINO blob to retrieve the information from)doc";

static const char *__doc_dai_node_DetectionParser_setBlob_2 =
R"doc(Same functionality as the setBlobPath(). Load network blob into assets and use
once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_DetectionParser_setBlobPath =
R"doc(Load network blob into assets and use once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_DetectionParser_setClasses = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setConfidenceThreshold =
R"doc(Specifies confidence threshold at which to filter the rest of the detections.

Parameter ``thresh``:
    Detection confidence must be greater than specified threshold to be added to
    the list)doc";

static const char *__doc_dai_node_DetectionParser_setConfig = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setCoordinateSize = R"doc(Set coordianate size)doc";

static const char *__doc_dai_node_DetectionParser_setInputImageSize =
R"doc(Set input image size

This should only be used instead of setBlob, not besides it)doc";

static const char *__doc_dai_node_DetectionParser_setInputImageSize_2 = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setIouThreshold = R"doc(Set Iou threshold)doc";

static const char *__doc_dai_node_DetectionParser_setModelPath =
R"doc(Load network xml and bin files into assets.

Parameter ``xmlModelPath``:
    Path to the neural network model file.)doc";

static const char *__doc_dai_node_DetectionParser_setNNArchive =
R"doc(Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default
number of shaves.

Parameter ``nnArchive:``:
    NNArchive to set)doc";

static const char *__doc_dai_node_DetectionParser_setNNArchiveBlob = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setNNArchiveOther = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setNNArchiveSuperblob = R"doc()doc";

static const char *__doc_dai_node_DetectionParser_setNNFamily = R"doc(Sets NN Family to parse)doc";

static const char *__doc_dai_node_DetectionParser_setNumClasses = R"doc(Set num classes)doc";

static const char *__doc_dai_node_DetectionParser_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_Display = R"doc()doc";

static const char *__doc_dai_node_Display_Display = R"doc()doc";

static const char *__doc_dai_node_Display_input = R"doc()doc";

static const char *__doc_dai_node_Display_name = R"doc()doc";

static const char *__doc_dai_node_Display_run = R"doc()doc";

static const char *__doc_dai_node_EdgeDetector = R"doc(EdgeDetector node. Performs edge detection using 3x3 Sobel filter)doc";

static const char *__doc_dai_node_EdgeDetector_EdgeDetector = R"doc()doc";

static const char *__doc_dai_node_EdgeDetector_EdgeDetector_2 = R"doc()doc";

static const char *__doc_dai_node_EdgeDetector_getProperties = R"doc()doc";

static const char *__doc_dai_node_EdgeDetector_initialConfig = R"doc(Initial config to use for edge detection.)doc";

static const char *__doc_dai_node_EdgeDetector_inputConfig =
R"doc(Input EdgeDetectorConfig message with ability to modify parameters in runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_EdgeDetector_inputImage =
R"doc(Input image on which edge detection is performed. Default queue is non-blocking
with size 4.)doc";

static const char *__doc_dai_node_EdgeDetector_outputImage = R"doc(Outputs image frame with detected edges)doc";

static const char *__doc_dai_node_EdgeDetector_passthroughInputImage = R"doc(Passthrough message on which the calculation was performed.)doc";

static const char *__doc_dai_node_EdgeDetector_setMaxOutputFrameSize =
R"doc(Specify maximum size of output image.

Parameter ``maxFrameSize``:
    Maximum frame size in bytes)doc";

static const char *__doc_dai_node_EdgeDetector_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_FeatureTracker =
R"doc(FeatureTracker node. Performs feature tracking and reidentification using motion
estimation between 2 consecutive frames.)doc";

static const char *__doc_dai_node_FeatureTracker_FeatureTracker = R"doc()doc";

static const char *__doc_dai_node_FeatureTracker_FeatureTracker_2 = R"doc()doc";

static const char *__doc_dai_node_FeatureTracker_getProperties = R"doc()doc";

static const char *__doc_dai_node_FeatureTracker_initialConfig = R"doc(Initial config to use for feature tracking.)doc";

static const char *__doc_dai_node_FeatureTracker_inputConfig =
R"doc(Input FeatureTrackerConfig message with ability to modify parameters in runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_FeatureTracker_inputImage =
R"doc(Input message with frame data on which feature tracking is performed. Default
queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_FeatureTracker_outputFeatures = R"doc(Outputs TrackedFeatures message that carries tracked features results.)doc";

static const char *__doc_dai_node_FeatureTracker_passthroughInputImage =
R"doc(Passthrough message on which the calculation was performed. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_FeatureTracker_setHardwareResources =
R"doc(Specify allocated hardware resources for feature tracking. 2 shaves/memory
slices are required for optical flow, 1 for corner detection only.

Parameter ``numShaves``:
    Number of shaves. Maximum 2.

Parameter ``numMemorySlices``:
    Number of memory slices. Maximum 2.)doc";

static const char *__doc_dai_node_HostCamera = R"doc()doc";

static const char *__doc_dai_node_HostCamera_out = R"doc()doc";

static const char *__doc_dai_node_HostCamera_run = R"doc()doc";

static const char *__doc_dai_node_HostNode = R"doc()doc";

static const char *__doc_dai_node_HostNode_buildStage1 = R"doc()doc";

static const char *__doc_dai_node_HostNode_input = R"doc()doc";

static const char *__doc_dai_node_HostNode_inputs = R"doc()doc";

static const char *__doc_dai_node_HostNode_out = R"doc()doc";

static const char *__doc_dai_node_HostNode_processGroup = R"doc()doc";

static const char *__doc_dai_node_HostNode_run = R"doc()doc";

static const char *__doc_dai_node_HostNode_runSyncingOnDevice = R"doc()doc";

static const char *__doc_dai_node_HostNode_runSyncingOnHost = R"doc()doc";

static const char *__doc_dai_node_HostNode_sendProcessToPipeline = R"doc()doc";

static const char *__doc_dai_node_HostNode_sendProcessingToPipeline =
R"doc(Send processing to pipeline. If set to true, it's important to call
`pipeline.run()` in the main thread or `pipeline.processTasks()` in the main
thread. Otherwise, if set to false, such action is not needed.)doc";

static const char *__doc_dai_node_HostNode_sync = R"doc()doc";

static const char *__doc_dai_node_HostNode_syncOnHost = R"doc()doc";

static const char *__doc_dai_node_IMU = R"doc(IMU node for BNO08X.)doc";

static const char *__doc_dai_node_IMU_enableFirmwareUpdate = R"doc()doc";

static const char *__doc_dai_node_IMU_enableIMUSensor = R"doc(Enable a new IMU sensor with explicit configuration)doc";

static const char *__doc_dai_node_IMU_enableIMUSensor_2 = R"doc(Enable a list of IMU sensors with explicit configuration)doc";

static const char *__doc_dai_node_IMU_enableIMUSensor_3 = R"doc(Enable a new IMU sensor with default configuration)doc";

static const char *__doc_dai_node_IMU_enableIMUSensor_4 = R"doc(Enable a list of IMU sensors with default configuration)doc";

static const char *__doc_dai_node_IMU_getBatchReportThreshold = R"doc(Above this packet threshold data will be sent to host, if queue is not blocked)doc";

static const char *__doc_dai_node_IMU_getMaxBatchReports = R"doc(Maximum number of IMU packets in a batch report)doc";

static const char *__doc_dai_node_IMU_getNodeRecordParams = R"doc()doc";

static const char *__doc_dai_node_IMU_getRecordOutput = R"doc()doc";

static const char *__doc_dai_node_IMU_getReplayInput = R"doc()doc";

static const char *__doc_dai_node_IMU_isSourceNode = R"doc()doc";

static const char *__doc_dai_node_IMU_mockIn = R"doc(Mock IMU data for replaying recorded data)doc";

static const char *__doc_dai_node_IMU_out = R"doc(Outputs IMUData message that carries IMU packets.)doc";

static const char *__doc_dai_node_IMU_setBatchReportThreshold = R"doc(Above this packet threshold data will be sent to host, if queue is not blocked)doc";

static const char *__doc_dai_node_IMU_setMaxBatchReports = R"doc(Maximum number of IMU packets in a batch report)doc";

static const char *__doc_dai_node_ImageAlign = R"doc(ImageAlign node. Calculates spatial location data on a set of ROIs on depth map.)doc";

static const char *__doc_dai_node_ImageAlign_getProperties = R"doc()doc";

static const char *__doc_dai_node_ImageAlign_initialConfig = R"doc(Initial config to use when calculating spatial location data.)doc";

static const char *__doc_dai_node_ImageAlign_input = R"doc(Input message. Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_ImageAlign_inputAlignTo = R"doc(Input align to message. Default queue is non-blocking with size 1.)doc";

static const char *__doc_dai_node_ImageAlign_inputConfig =
R"doc(Input message with ability to modify parameters in runtime. Default queue is
non-blocking with size 4.)doc";

static const char *__doc_dai_node_ImageAlign_outputAligned = R"doc(Outputs ImgFrame message that is aligned to inputAlignTo.)doc";

static const char *__doc_dai_node_ImageAlign_passthroughInput =
R"doc(Passthrough message on which the calculation was performed. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_ImageAlign_setInterpolation = R"doc(Specify interpolation method to use when resizing)doc";

static const char *__doc_dai_node_ImageAlign_setNumFramesPool = R"doc(Specify number of frames in the pool)doc";

static const char *__doc_dai_node_ImageAlign_setNumShaves = R"doc(Specify number of shaves to use for this node)doc";

static const char *__doc_dai_node_ImageAlign_setOutKeepAspectRatio = R"doc(Specify whether to keep aspect ratio when resizing)doc";

static const char *__doc_dai_node_ImageAlign_setOutputSize = R"doc(Specify the output size of the aligned image)doc";

static const char *__doc_dai_node_ImageManip = R"doc(ImageManip node. Capability to crop, resize, warp, ... incoming image frames)doc";

static const char *__doc_dai_node_ImageManip_ImageManip = R"doc()doc";

static const char *__doc_dai_node_ImageManip_ImageManip_2 = R"doc()doc";

static const char *__doc_dai_node_ImageManip_build = R"doc()doc";

static const char *__doc_dai_node_ImageManip_getProperties = R"doc()doc";

static const char *__doc_dai_node_ImageManip_initialConfig = R"doc(Initial config to use when manipulating frames)doc";

static const char *__doc_dai_node_ImageManip_inputConfig = R"doc(Input ImageManipConfig message with ability to modify parameters in runtime)doc";

static const char *__doc_dai_node_ImageManip_inputImage = R"doc(Input image to be modified)doc";

static const char *__doc_dai_node_ImageManip_out = R"doc()doc";

static const char *__doc_dai_node_ImageManip_run = R"doc()doc";

static const char *__doc_dai_node_ImageManip_runOnHost = R"doc(Check if the node is set to run on host)doc";

static const char *__doc_dai_node_ImageManip_runOnHostVar = R"doc()doc";

static const char *__doc_dai_node_ImageManip_setBackend =
R"doc(Set CPU as backend preference

Parameter ``backend``:
    Backend preference)doc";

static const char *__doc_dai_node_ImageManip_setMaxOutputFrameSize =
R"doc(Specify maximum size of output image.

Parameter ``maxFrameSize``:
    Maximum frame size in bytes)doc";

static const char *__doc_dai_node_ImageManip_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_ImageManip_setPerformanceMode =
R"doc(Set performance mode

Parameter ``performanceMode``:
    Performance mode)doc";

static const char *__doc_dai_node_ImageManip_setRunOnHost =
R"doc(Specify whether to run on host or device

Parameter ``runOnHost``:
    Run node on host)doc";

static const char *__doc_dai_node_MessageDemux = R"doc()doc";

static const char *__doc_dai_node_MessageDemux_input = R"doc(Input message of type MessageGroup)doc";

static const char *__doc_dai_node_MessageDemux_outputs = R"doc(A map of outputs, where keys are same as in the input MessageGroup)doc";

static const char *__doc_dai_node_MonoCamera = R"doc(MonoCamera node. For use with grayscale sensors.)doc";

static const char *__doc_dai_node_MonoCamera_MonoCamera = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_MonoCamera_2 = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_frameEvent = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getBoardSocket =
R"doc(Retrieves which board socket to use

Returns:
    Board socket to use)doc";

static const char *__doc_dai_node_MonoCamera_getCamId = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getCamera =
R"doc(Retrieves which camera to use by name

Returns:
    Name of the camera to use)doc";

static const char *__doc_dai_node_MonoCamera_getFps =
R"doc(Get rate at which camera should produce frames

Returns:
    Rate in frames per second)doc";

static const char *__doc_dai_node_MonoCamera_getFrameEventFilter = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getImageOrientation = R"doc(Get camera image orientation)doc";

static const char *__doc_dai_node_MonoCamera_getNodeRecordParams = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getNumFramesPool = R"doc(Get number of frames in main (ISP output) pool)doc";

static const char *__doc_dai_node_MonoCamera_getProperties = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getRawNumFramesPool = R"doc(Get number of frames in raw pool)doc";

static const char *__doc_dai_node_MonoCamera_getRecordOutput = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getReplayInput = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_getResolution = R"doc(Get sensor resolution)doc";

static const char *__doc_dai_node_MonoCamera_getResolutionHeight = R"doc(Get sensor resolution height)doc";

static const char *__doc_dai_node_MonoCamera_getResolutionSize = R"doc(Get sensor resolution as size)doc";

static const char *__doc_dai_node_MonoCamera_getResolutionWidth = R"doc(Get sensor resolution width)doc";

static const char *__doc_dai_node_MonoCamera_initialControl = R"doc(Initial control options to apply to sensor)doc";

static const char *__doc_dai_node_MonoCamera_inputControl = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_isSourceNode = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_mockIsp = R"doc(Input for mocking 'isp' functionality. Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_MonoCamera_out = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_raw = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_setBoardSocket =
R"doc(Specify which board socket to use

Parameter ``boardSocket``:
    Board socket to use)doc";

static const char *__doc_dai_node_MonoCamera_setCamId = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_setCamera =
R"doc(Specify which camera to use by name

Parameter ``name``:
    Name of the camera to use)doc";

static const char *__doc_dai_node_MonoCamera_setFps =
R"doc(Set rate at which camera should produce frames

Parameter ``fps``:
    Rate in frames per second)doc";

static const char *__doc_dai_node_MonoCamera_setFrameEventFilter = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_setImageOrientation = R"doc(Set camera image orientation)doc";

static const char *__doc_dai_node_MonoCamera_setIsp3aFps =
R"doc(Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls
etc.). Default (0) matches the camera FPS, meaning that 3A is running on each
frame. Reducing the rate of 3A reduces the CPU usage on CSS, but also increases
the convergence rate of 3A. Note that camera controls will be processed at this
rate. E.g. if camera is running at 30 fps, and camera control is sent at every
frame, but 3A fps is set to 15, the camera control messages will be processed at
15 fps rate, which will lead to queueing.)doc";

static const char *__doc_dai_node_MonoCamera_setMockIspSize = R"doc()doc";

static const char *__doc_dai_node_MonoCamera_setNumFramesPool = R"doc(Set number of frames in main (ISP output) pool)doc";

static const char *__doc_dai_node_MonoCamera_setRawNumFramesPool = R"doc(Set number of frames in raw pool)doc";

static const char *__doc_dai_node_MonoCamera_setRawOutputPacked =
R"doc(Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
The packed format is more efficient, consuming less memory on device, and less
data to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on
3 bytes. When packing is disabled (`false`), data is saved lsb-aligned, e.g. a
RAW10 pixel will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
Default is auto: enabled for standard color/monochrome cameras where ISP can
work with both packed/unpacked, but disabled for other cameras like ToF.)doc";

static const char *__doc_dai_node_MonoCamera_setResolution = R"doc(Set sensor resolution)doc";

static const char *__doc_dai_node_NeuralNetwork = R"doc(NeuralNetwork node. Runs a neural inference on input data.)doc";

static const char *__doc_dai_node_NeuralNetwork_build =
R"doc(Build NeuralNetwork node. Connect output to this node's input. Also call
setNNArchive() with provided NNArchive.

Parameter ``output:``:
    Output to link

Parameter ``nnArchive:``:
    Neural network archive

Returns:
    Shared pointer to NeuralNetwork node)doc";

static const char *__doc_dai_node_NeuralNetwork_build_2 = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_build_3 = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_build_4 = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_build_5 = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_createNNArchive = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_getFrameCapability = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_getNumInferenceThreads =
R"doc(How many inference threads will be used to run the network

Returns:
    Number of threads, 0, 1 or 2. Zero means AUTO)doc";

static const char *__doc_dai_node_NeuralNetwork_input = R"doc(Input message with data to be inferred upon)doc";

static const char *__doc_dai_node_NeuralNetwork_inputs =
R"doc(Inputs mapped to network inputs. Useful for inferring from separate data sources
Default input is non-blocking with queue size 1 and waits for messages)doc";

static const char *__doc_dai_node_NeuralNetwork_nnArchive = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_out = R"doc(Outputs NNData message that carries inference results)doc";

static const char *__doc_dai_node_NeuralNetwork_passthrough =
R"doc(Passthrough message on which the inference was performed.

Suitable for when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_NeuralNetwork_passthroughs = R"doc(Passthroughs which correspond to specified input)doc";

static const char *__doc_dai_node_NeuralNetwork_setBackend =
R"doc(Specifies backend to use

Parameter ``backend``:
    String specifying backend to use)doc";

static const char *__doc_dai_node_NeuralNetwork_setBackendProperties =
R"doc(Set backend properties

Parameter ``backendProperties``:
    backend properties map)doc";

static const char *__doc_dai_node_NeuralNetwork_setBlob =
R"doc(Load network blob into assets and use once pipeline is started.

Parameter ``blob``:
    Network blob)doc";

static const char *__doc_dai_node_NeuralNetwork_setBlob_2 =
R"doc(Same functionality as the setBlobPath(). Load network blob into assets and use
once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_NeuralNetwork_setBlobPath =
R"doc(Load network blob into assets and use once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_NeuralNetwork_setFromModelZoo =
R"doc(Download model from zoo and set it for this Node

Parameter ``description:``:
    Model description to download

Parameter ``useCached:``:
    Use cached model if available)doc";

static const char *__doc_dai_node_NeuralNetwork_setModelPath =
R"doc(Load network xml and bin files into assets.

Parameter ``xmlModelPath``:
    Path to the neural network model file.)doc";

static const char *__doc_dai_node_NeuralNetwork_setNNArchive =
R"doc(Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default
number of shaves.

Parameter ``nnArchive:``:
    NNArchive to set)doc";

static const char *__doc_dai_node_NeuralNetwork_setNNArchive_2 =
R"doc(Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB

Parameter ``nnArchive:``:
    NNArchive to set

Parameter ``numShaves:``:
    Number of shaves to use)doc";

static const char *__doc_dai_node_NeuralNetwork_setNNArchiveBlob = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_setNNArchiveOther = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_setNNArchiveSuperblob = R"doc()doc";

static const char *__doc_dai_node_NeuralNetwork_setNumInferenceThreads =
R"doc(How many threads should the node use to run the network.

Parameter ``numThreads``:
    Number of threads to dedicate to this node)doc";

static const char *__doc_dai_node_NeuralNetwork_setNumNCEPerInferenceThread =
R"doc(How many Neural Compute Engines should a single thread use for inference

Parameter ``numNCEPerThread``:
    Number of NCE per thread)doc";

static const char *__doc_dai_node_NeuralNetwork_setNumPoolFrames =
R"doc(Specifies how many frames will be available in the pool

Parameter ``numFrames``:
    How many frames will pool have)doc";

static const char *__doc_dai_node_NeuralNetwork_setNumShavesPerInferenceThread =
R"doc(How many Shaves should a single thread use for inference

Parameter ``numShavesPerThread``:
    Number of shaves per thread)doc";

static const char *__doc_dai_node_ObjectTracker =
R"doc(ObjectTracker node. Performs object tracking using Kalman filter and hungarian
algorithm.)doc";

static const char *__doc_dai_node_ObjectTracker_inputConfig =
R"doc(Input ObjectTrackerConfig message with ability to modify parameters at runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_ObjectTracker_inputDetectionFrame =
R"doc(Input ImgFrame message on which object detection was performed. Default queue is
non-blocking with size 4.)doc";

static const char *__doc_dai_node_ObjectTracker_inputDetections =
R"doc(Input message with image detection from neural network. Default queue is non-
blocking with size 4.)doc";

static const char *__doc_dai_node_ObjectTracker_inputTrackerFrame =
R"doc(Input ImgFrame message on which tracking will be performed. RGBp, BGRp, NV12,
YUV420p types are supported. Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_ObjectTracker_out = R"doc(Outputs Tracklets message that carries object tracking results.)doc";

static const char *__doc_dai_node_ObjectTracker_passthroughDetectionFrame =
R"doc(Passthrough ImgFrame message on which object detection was performed. Suitable
for when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_ObjectTracker_passthroughDetections =
R"doc(Passthrough image detections message from neural network output. Suitable for
when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_ObjectTracker_passthroughTrackerFrame =
R"doc(Passthrough ImgFrame message on which tracking was performed. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_ObjectTracker_setDetectionLabelsToTrack =
R"doc(Specify detection labels to track.

Parameter ``labels``:
    Detection labels to track. Default every label is tracked from image
    detection network output.)doc";

static const char *__doc_dai_node_ObjectTracker_setMaxObjectsToTrack =
R"doc(Specify maximum number of object to track.

Parameter ``maxObjectsToTrack``:
    Maximum number of object to track. Maximum 60 in case of SHORT_TERM_KCF,
    otherwise 1000.)doc";

static const char *__doc_dai_node_ObjectTracker_setTrackerIdAssignmentPolicy =
R"doc(Specify tracker ID assignment policy.

Parameter ``type``:
    Tracker ID assignment policy.)doc";

static const char *__doc_dai_node_ObjectTracker_setTrackerThreshold =
R"doc(Specify tracker threshold.

Parameter ``threshold``:
    Above this threshold the detected objects will be tracked. Default 0, all
    image detections are tracked.)doc";

static const char *__doc_dai_node_ObjectTracker_setTrackerType =
R"doc(Specify tracker type algorithm.

Parameter ``type``:
    Tracker type.)doc";

static const char *__doc_dai_node_ObjectTracker_setTrackingPerClass = R"doc(Whether tracker should take into consideration class label for tracking.)doc";

static const char *__doc_dai_node_PointCloud = R"doc(PointCloud node. Computes point cloud from depth frames.)doc";

static const char *__doc_dai_node_PointCloud_getProperties = R"doc()doc";

static const char *__doc_dai_node_PointCloud_initialConfig = R"doc(Initial config to use when computing the point cloud.)doc";

static const char *__doc_dai_node_PointCloud_inputConfig =
R"doc(Input PointCloudConfig message with ability to modify parameters in runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_PointCloud_inputDepth =
R"doc(Input message with depth data used to create the point cloud. Default queue is
non-blocking with size 4.)doc";

static const char *__doc_dai_node_PointCloud_outputPointCloud = R"doc(Outputs PointCloudData message)doc";

static const char *__doc_dai_node_PointCloud_passthroughDepth =
R"doc(Passthrough depth from which the point cloud was calculated. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_PointCloud_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_RGBD = R"doc(RGBD node. Combines depth and color frames into a single point cloud.)doc";

static const char *__doc_dai_node_RGBD_Impl = R"doc()doc";

static const char *__doc_dai_node_RGBD_RGBD = R"doc()doc";

static const char *__doc_dai_node_RGBD_build = R"doc()doc";

static const char *__doc_dai_node_RGBD_build_2 =
R"doc(Build RGBD node with specified size. Note that this API is global and if used
autocreated cameras can't be reused.

Parameter ``autocreate``:
    If true, will create color and depth nodes if they don't exist.

Parameter ``size``:
    Size of the frames)doc";

static const char *__doc_dai_node_RGBD_buildInternal = R"doc()doc";

static const char *__doc_dai_node_RGBD_colorInputName = R"doc()doc";

static const char *__doc_dai_node_RGBD_depthInputName = R"doc()doc";

static const char *__doc_dai_node_RGBD_inColor = R"doc()doc";

static const char *__doc_dai_node_RGBD_inDepth = R"doc()doc";

static const char *__doc_dai_node_RGBD_inSync = R"doc()doc";

static const char *__doc_dai_node_RGBD_initialize = R"doc()doc";

static const char *__doc_dai_node_RGBD_initialized = R"doc()doc";

static const char *__doc_dai_node_RGBD_inputs = R"doc()doc";

static const char *__doc_dai_node_RGBD_pcl = R"doc(Output point cloud.)doc";

static const char *__doc_dai_node_RGBD_pimpl = R"doc()doc";

static const char *__doc_dai_node_RGBD_printDevices = R"doc(Print available GPU devices)doc";

static const char *__doc_dai_node_RGBD_rgbd = R"doc(Output RGBD frames.)doc";

static const char *__doc_dai_node_RGBD_run = R"doc()doc";

static const char *__doc_dai_node_RGBD_setDepthUnit = R"doc()doc";

static const char *__doc_dai_node_RGBD_sync = R"doc()doc";

static const char *__doc_dai_node_RGBD_useCPU = R"doc(Use single-threaded CPU for processing)doc";

static const char *__doc_dai_node_RGBD_useCPUMT =
R"doc(Use multi-threaded CPU for processing

Parameter ``numThreads``:
    Number of threads to use)doc";

static const char *__doc_dai_node_RGBD_useGPU =
R"doc(Use GPU for processing (needs to be compiled with Kompute support)

Parameter ``device``:
    GPU device index)doc";

static const char *__doc_dai_node_RTABMapSLAM =
R"doc(RTABMap SLAM node. Performs SLAM on given odometry pose, rectified frame and
depth frame.)doc";

static const char *__doc_dai_node_RTABMapSLAM_alphaScaling = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_buildInternal = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_cloudMap = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_databasePath = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_databaseSaveInterval = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_depth = R"doc(Input depth image on which SLAM is performed.)doc";

static const char *__doc_dai_node_RTABMapSLAM_depthInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_features = R"doc(Input tracked features on which SLAM is performed (optional).)doc";

static const char *__doc_dai_node_RTABMapSLAM_featuresInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_freq = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_getLocalTransform = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_groundPCL = R"doc(Output ground point cloud.)doc";

static const char *__doc_dai_node_RTABMapSLAM_imuCB = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_inSync = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_initialize = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_initialized = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_inputs = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_landmarks = R"doc(Input Landmark poses (optional).)doc";

static const char *__doc_dai_node_RTABMapSLAM_landmarksInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_lastProcessTime = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_loadDatabaseOnStart = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_localMaps = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_model = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_obstaclePCL = R"doc(Output obstacle point cloud.)doc";

static const char *__doc_dai_node_RTABMapSLAM_occupancyGrid = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_occupancyGridMap = R"doc(Output occupancy grid map.)doc";

static const char *__doc_dai_node_RTABMapSLAM_odom = R"doc(Input odometry pose.)doc";

static const char *__doc_dai_node_RTABMapSLAM_odomCorrection = R"doc(Output odometry correction (map to odom).)doc";

static const char *__doc_dai_node_RTABMapSLAM_odomPoseCB = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_passthroughDepth = R"doc(Output passthrough depth image.)doc";

static const char *__doc_dai_node_RTABMapSLAM_passthroughFeatures = R"doc(Output passthrough features.)doc";

static const char *__doc_dai_node_RTABMapSLAM_passthroughOdom = R"doc(Output passthrough odometry pose.)doc";

static const char *__doc_dai_node_RTABMapSLAM_passthroughRect = R"doc(Output passthrough rectified image.)doc";

static const char *__doc_dai_node_RTABMapSLAM_publishGrid = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_publishGridMap = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_publishGroundCloud = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_publishObstacleCloud = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_publishPointClouds = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_rect = R"doc(Input rectified image on which SLAM is performed.)doc";

static const char *__doc_dai_node_RTABMapSLAM_rectInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_rtabParams = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_rtabmap = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_run = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_saveDatabase = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_saveDatabaseOnClose = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_saveDatabasePeriodically = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_setAlphaScaling = R"doc(Set the alpha scaling factor for the camera model.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setDatabasePath = R"doc(Set RTABMap database path. "/tmp/rtabmap.tmp.db" by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setFreq = R"doc(Set the frequency at which the node processes data. 1Hz by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setLoadDatabaseOnStart = R"doc(Whether to load the database on start. False by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setLocalTransform = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_setParams = R"doc(Set RTABMap parameters.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setPublishGrid = R"doc(Whether to publish the ground point cloud. True by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setPublishGroundCloud = R"doc(Whether to publish the ground point cloud. True by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setPublishObstacleCloud = R"doc(Whether to publish the obstacle point cloud. True by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setSaveDatabaseOnClose = R"doc(Whether to save the database on close. False by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setSaveDatabasePeriod = R"doc(Set the interval at which the database is saved. 30.0s by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setSaveDatabasePeriodically = R"doc(Whether to save the database periodically. False by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setUseFeatures = R"doc(Whether to use input features for SLAM. False by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_setUseLandmarks = R"doc(WhWhether to use input landmarks for SLAM. False by default.)doc";

static const char *__doc_dai_node_RTABMapSLAM_startTime = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_sync = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_syncCB = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_transform = R"doc(Output transform.)doc";

static const char *__doc_dai_node_RTABMapSLAM_triggerNewMap = R"doc(Trigger a new map.)doc";

static const char *__doc_dai_node_RTABMapSLAM_useFeatures = R"doc()doc";

static const char *__doc_dai_node_RTABMapSLAM_useLandmarks = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO =
R"doc(RTABMap Visual Inertial Odometry node. Performs VIO on rectified frame, depth
frame and IMU data.)doc";

static const char *__doc_dai_node_RTABMapVIO_accBuffer = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_alphaScaling = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_buildInternal = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_depth = R"doc(Input depth image on which VIO is performed.)doc";

static const char *__doc_dai_node_RTABMapVIO_depthInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_features = R"doc(Input tracked features on which VIO is performed (optional).)doc";

static const char *__doc_dai_node_RTABMapVIO_featuresInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_gyroBuffer = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_imu = R"doc(Input IMU data.)doc";

static const char *__doc_dai_node_RTABMapVIO_imuCB = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_imuLocalTransform = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_imuMtx = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_inSync = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_initialize = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_initialized = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_inputs = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_localTransform = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_model = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_odom = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_passthroughDepth = R"doc(Passthrough depth frame.)doc";

static const char *__doc_dai_node_RTABMapVIO_passthroughFeatures = R"doc(Passthrough features.)doc";

static const char *__doc_dai_node_RTABMapVIO_passthroughRect = R"doc(Passthrough rectified frame.)doc";

static const char *__doc_dai_node_RTABMapVIO_rect = R"doc(Input rectified image on which VIO is performed.)doc";

static const char *__doc_dai_node_RTABMapVIO_rectInputName = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_reset = R"doc(Reset Odometry.)doc";

static const char *__doc_dai_node_RTABMapVIO_rtabParams = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_run = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_setLocalTransform = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_setParams = R"doc(Set RTABMap parameters.)doc";

static const char *__doc_dai_node_RTABMapVIO_setUseFeatures = R"doc(Whether to use input features or calculate them internally.)doc";

static const char *__doc_dai_node_RTABMapVIO_sync = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_syncCB = R"doc()doc";

static const char *__doc_dai_node_RTABMapVIO_transform = R"doc(Output transform.)doc";

static const char *__doc_dai_node_RTABMapVIO_useFeatures = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly = R"doc(RecordMetadataOnly node, used to record a source stream to a file)doc";

static const char *__doc_dai_node_RecordMetadataOnly_compressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_getCompressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_getRecordFile = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_input =
R"doc(Input IMU messages to be recorded (will support other types in the future)

Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_RecordMetadataOnly_recordFile = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_run = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_setCompressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordMetadataOnly_setRecordFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo = R"doc(RecordVideo node, used to record a video source stream to a file)doc";

static const char *__doc_dai_node_RecordVideo_compressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_fpsInitLength = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_getCompressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_getRecordMetadataFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_getRecordVideoFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_input =
R"doc(Input for ImgFrame or EncodedFrame messages to be recorded

Default queue is blocking with size 15)doc";

static const char *__doc_dai_node_RecordVideo_recordMetadataFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_recordVideoFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_run = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_setCompressionLevel = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_setRecordMetadataFile = R"doc()doc";

static const char *__doc_dai_node_RecordVideo_setRecordVideoFile = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly = R"doc(Replay node, used to replay a file to a source node)doc";

static const char *__doc_dai_node_ReplayMetadataOnly_fps = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_getFps = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_getLoop = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_getReplayFile = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_loop = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_out =
R"doc(Output for any type of messages to be transferred over XLink stream

Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_ReplayMetadataOnly_replayFile = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_run = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_setFps = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_setLoop = R"doc()doc";

static const char *__doc_dai_node_ReplayMetadataOnly_setReplayFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo = R"doc(Replay node, used to replay a file to a source node)doc";

static const char *__doc_dai_node_ReplayVideo_fps = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getFps = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getLoop = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getOutFrameType = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getReplayMetadataFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getReplayVideoFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_getSize = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_loop = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_out =
R"doc(Output for any type of messages to be transferred over XLink stream

Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_ReplayVideo_outFrameType = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_replayFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_replayVideo = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_run = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setFps = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setLoop = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setOutFrameType = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setReplayMetadataFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setReplayVideoFile = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setSize = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_setSize_2 = R"doc()doc";

static const char *__doc_dai_node_ReplayVideo_size = R"doc()doc";

static const char *__doc_dai_node_SPIIn = R"doc(SPIIn node. Receives messages over SPI.)doc";

static const char *__doc_dai_node_SPIIn_buildInternal = R"doc()doc";

static const char *__doc_dai_node_SPIIn_getBusId = R"doc(Get bus id)doc";

static const char *__doc_dai_node_SPIIn_getMaxDataSize = R"doc(Get maximum messages size in bytes)doc";

static const char *__doc_dai_node_SPIIn_getNumFrames = R"doc(Get number of frames in pool)doc";

static const char *__doc_dai_node_SPIIn_getStreamName = R"doc(Get stream name)doc";

static const char *__doc_dai_node_SPIIn_out = R"doc(Outputs message of same type as send from host.)doc";

static const char *__doc_dai_node_SPIIn_setBusId =
R"doc(Specifies SPI Bus number to use

Parameter ``id``:
    SPI Bus id)doc";

static const char *__doc_dai_node_SPIIn_setMaxDataSize =
R"doc(Set maximum message size it can receive

Parameter ``maxDataSize``:
    Maximum size in bytes)doc";

static const char *__doc_dai_node_SPIIn_setNumFrames =
R"doc(Set number of frames in pool for sending messages forward

Parameter ``numFrames``:
    Maximum number of frames in pool)doc";

static const char *__doc_dai_node_SPIIn_setStreamName =
R"doc(Specifies stream name over which the node will receive data

Parameter ``name``:
    Stream name)doc";

static const char *__doc_dai_node_SPIOut = R"doc(SPIOut node. Sends messages over SPI.)doc";

static const char *__doc_dai_node_SPIOut_buildInternal = R"doc()doc";

static const char *__doc_dai_node_SPIOut_input =
R"doc(Input for any type of messages to be transferred over SPI stream Default queue
is blocking with size 8)doc";

static const char *__doc_dai_node_SPIOut_setBusId =
R"doc(Specifies SPI Bus number to use

Parameter ``id``:
    SPI Bus id)doc";

static const char *__doc_dai_node_SPIOut_setStreamName =
R"doc(Specifies stream name over which the node will send data

Parameter ``name``:
    Stream name)doc";

static const char *__doc_dai_node_Script = R"doc()doc";

static const char *__doc_dai_node_Script_buildInternal = R"doc()doc";

static const char *__doc_dai_node_Script_getProcessor =
R"doc(Get on which processor the script should run

Returns:
    Processor type - Leon CSS or Leon MSS)doc";

static const char *__doc_dai_node_Script_getScriptName =
R"doc(Get the script name in utf-8.

When name set with setScript() or setScriptPath(), returns that name. When
script loaded with setScriptPath() with name not provided, returns the utf-8
string of that path. Otherwise, returns "<script>"

Returns:
    std::string of script name in utf-8)doc";

static const char *__doc_dai_node_Script_getScriptPath =
R"doc(Get filesystem path from where script was loaded.

Returns:
    std::filesystem::path from where script was loaded, otherwise returns empty
    path)doc";

static const char *__doc_dai_node_Script_inputs =
R"doc(Inputs to Script node. Can be accessed using subscript operator (Eg:
inputs['in1']) By default inputs are set to blocking with queue size 8)doc";

static const char *__doc_dai_node_Script_outputs =
R"doc(Outputs from Script node. Can be accessed subscript operator (Eg:
outputs['out1']))doc";

static const char *__doc_dai_node_Script_scriptPath = R"doc()doc";

static const char *__doc_dai_node_Script_setProcessor =
R"doc(Set on which processor the script should run

Parameter ``type``:
    Processor type - Leon CSS or Leon MSS)doc";

static const char *__doc_dai_node_Script_setScript =
R"doc(Sets script data to be interpreted

Parameter ``script``:
    Script string to be interpreted

Parameter ``name``:
    Optionally set a name of this script)doc";

static const char *__doc_dai_node_Script_setScript_2 =
R"doc(Sets script data to be interpreted

Parameter ``data``:
    Binary data that represents the script to be interpreted

Parameter ``name``:
    Optionally set a name of this script)doc";

static const char *__doc_dai_node_Script_setScriptPath =
R"doc(Specify local filesystem path to load the script

Parameter ``path``:
    Filesystem path to load the script

Parameter ``name``:
    Optionally set a name of this script, otherwise the name defaults to the
    path)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork =
R"doc(SpatialDetectionNetwork node. Runs a neural inference on input image and
calculates spatial location data.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_SpatialDetectionNetwork = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_SpatialDetectionNetwork_2 = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_SpatialDetectionNetwork_3 = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_SpatialDetectionNetwork_4 = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_alignDepth = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_boundingBoxMapping =
R"doc(Outputs mapping of detected bounding boxes relative to depth map Suitable for
when displaying remapped bounding boxes on depth frame)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_build = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_build_2 = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_buildInternal = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_createNNArchive = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_depthAlign = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_detectionParser = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_getClasses = R"doc(Get classes labels)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_getConfidenceThreshold =
R"doc(Retrieves threshold at which to filter the rest of the detections.

Returns:
    Detection confidence)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_getNumInferenceThreads =
R"doc(How many inference threads will be used to run the network

Returns:
    Number of threads, 0, 1 or 2. Zero means AUTO)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_input =
R"doc(Input message with data to be inferred upon Default queue is blocking with size
5)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_inputDepth =
R"doc(Input message with depth data used to retrieve spatial information about
detected object Default queue is non-blocking with size 4)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_inputDetections = R"doc(Input message with input detections object Default queue is blocking with size 1)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_inputImg =
R"doc(Input message with image data used to retrieve image transformation from
detected object Default queue is blocking with size 1)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_neuralNetwork = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_out = R"doc(Outputs ImgDetections message that carries parsed detection results.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_outNetwork = R"doc(Outputs unparsed inference results.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_passthrough =
R"doc(Passthrough message on which the inference was performed.

Suitable for when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_passthroughDepth =
R"doc(Passthrough message for depth frame on which the spatial location calculation
was performed. Suitable for when input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBackend =
R"doc(Specifies backend to use

Parameter ``backend``:
    String specifying backend to use)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBackendProperties =
R"doc(Set backend properties

Parameter ``backendProperties``:
    backend properties map)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBlob =
R"doc(Load network blob into assets and use once pipeline is started.

Parameter ``blob``:
    Network blob)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBlob_2 =
R"doc(Same functionality as the setBlobPath(). Load network blob into assets and use
once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBlobPath =
R"doc(Load network blob into assets and use once pipeline is started.

Throws:
    Error if file doesn't exist or isn't a valid network blob.

Parameter ``path``:
    Path to network blob)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setBoundingBoxScaleFactor =
R"doc(Custom interface

Specifies scale factor for detected bounding boxes.

Parameter ``scaleFactor``:
    Scale factor must be in the interval (0,1].)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setConfidenceThreshold =
R"doc(Specifies confidence threshold at which to filter the rest of the detections.

Parameter ``thresh``:
    Detection confidence must be greater than specified threshold to be added to
    the list)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setDepthLowerThreshold =
R"doc(Specifies lower threshold in depth units (millimeter by default) for depth
values which will used to calculate spatial data

Parameter ``lowerThreshold``:
    LowerThreshold must be in the interval [0,upperThreshold] and less than
    upperThreshold.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setDepthUpperThreshold =
R"doc(Specifies upper threshold in depth units (millimeter by default) for depth
values which will used to calculate spatial data

Parameter ``upperThreshold``:
    UpperThreshold must be in the interval (lowerThreshold,65535].)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setFromModelZoo =
R"doc(Download model from zoo and set it for this Node

Parameter ``description:``:
    Model description to download

Parameter ``useCached:``:
    Use cached model if available)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setModelPath =
R"doc(Load network file into assets.

Parameter ``modelPath``:
    Path to the model file.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNNArchive =
R"doc(Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default
number of shaves.

Parameter ``nnArchive:``:
    NNArchive to set)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNNArchive_2 =
R"doc(Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB

Parameter ``nnArchive:``:
    NNArchive to set

Parameter ``numShaves:``:
    Number of shaves to use)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNNArchiveBlob = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNNArchiveOther = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNNArchiveSuperblob = R"doc()doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNumInferenceThreads =
R"doc(How many threads should the node use to run the network.

Parameter ``numThreads``:
    Number of threads to dedicate to this node)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNumNCEPerInferenceThread =
R"doc(How many Neural Compute Engines should a single thread use for inference

Parameter ``numNCEPerThread``:
    Number of NCE per thread)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNumPoolFrames =
R"doc(Specifies how many frames will be available in the pool

Parameter ``numFrames``:
    How many frames will pool have)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setNumShavesPerInferenceThread =
R"doc(How many Shaves should a single thread use for inference

Parameter ``numShavesPerThread``:
    Number of shaves per thread)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setSpatialCalculationAlgorithm =
R"doc(Specifies spatial location calculator algorithm: Average/Min/Max

Parameter ``calculationAlgorithm``:
    Calculation algorithm.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_setSpatialCalculationStepSize =
R"doc(Specifies spatial location calculator step size for depth calculation. Step size
1 means that every pixel is taken into calculation, size 2 means every second
etc.

Parameter ``stepSize``:
    Step size.)doc";

static const char *__doc_dai_node_SpatialDetectionNetwork_spatialLocationCalculatorOutput =
R"doc(Output of SpatialLocationCalculator node, which is used internally by
SpatialDetectionNetwork. Suitable when extra information is required from
SpatialLocationCalculator node, e.g. minimum, maximum distance.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator =
R"doc(SpatialLocationCalculator node. Calculates spatial location data on a set of
ROIs on depth map.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator_SpatialLocationCalculator = R"doc()doc";

static const char *__doc_dai_node_SpatialLocationCalculator_SpatialLocationCalculator_2 = R"doc()doc";

static const char *__doc_dai_node_SpatialLocationCalculator_getProperties = R"doc()doc";

static const char *__doc_dai_node_SpatialLocationCalculator_initialConfig = R"doc(Initial config to use when calculating spatial location data.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator_inputConfig =
R"doc(Input SpatialLocationCalculatorConfig message with ability to modify parameters
in runtime. Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator_inputDepth =
R"doc(Input message with depth data used to retrieve spatial information about
detected object. Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator_out =
R"doc(Outputs SpatialLocationCalculatorData message that carries spatial location
results.)doc";

static const char *__doc_dai_node_SpatialLocationCalculator_passthroughDepth =
R"doc(Passthrough message on which the calculation was performed. Suitable for when
input queue is set to non-blocking behavior.)doc";

static const char *__doc_dai_node_StereoDepth = R"doc(StereoDepth node. Compute stereo disparity and depth from left-right image pair.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode = R"doc(Preset modes for stereo depth.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_DEFAULT =
R"doc(Prefers density over accuracy. Less invalid depth values, but more outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_FACE =
R"doc(Prefers density over accuracy. Less invalid depth values, but more outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_FAST_ACCURACY =
R"doc(Prefers accuracy over density. More invalid depth values, but less outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_FAST_DENSITY =
R"doc(Prefers density over accuracy. Less invalid depth values, but more outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_HIGH_DETAIL =
R"doc(Prefers density over accuracy. Less invalid depth values, but more outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_PresetMode_ROBOTICS =
R"doc(Prefers density over accuracy. Less invalid depth values, but more outliers.
This mode does not turn on any post-processing and is light on resources.)doc";

static const char *__doc_dai_node_StereoDepth_StereoDepth = R"doc()doc";

static const char *__doc_dai_node_StereoDepth_StereoDepth_2 = R"doc()doc";

static const char *__doc_dai_node_StereoDepth_build = R"doc()doc";

static const char *__doc_dai_node_StereoDepth_build_2 =
R"doc(Create StereoDepth node. Note that this API is global and if used autocreated
cameras can't be reused.

Parameter ``autoCreateCameras``:
    If true, will create left and right nodes if they don't exist

Parameter ``presetMode``:
    Preset mode for stereo depth)doc";

static const char *__doc_dai_node_StereoDepth_confidenceMap =
R"doc(Outputs ImgFrame message that carries RAW8 confidence map. Lower values mean
lower confidence of the calculated disparity value. RGB alignment, left-right
check or any postprocessing (e.g., median filter) is not performed on confidence
map.)doc";

static const char *__doc_dai_node_StereoDepth_debugDispCostDump =
R"doc(Outputs ImgFrame message that carries cost dump of disparity map. Useful for
debugging/fine tuning.)doc";

static const char *__doc_dai_node_StereoDepth_debugDispLrCheckIt1 =
R"doc(Outputs ImgFrame message that carries left-right check first iteration (before
combining with second iteration) disparity map. Useful for debugging/fine
tuning.)doc";

static const char *__doc_dai_node_StereoDepth_debugDispLrCheckIt2 =
R"doc(Outputs ImgFrame message that carries left-right check second iteration (before
combining with first iteration) disparity map. Useful for debugging/fine tuning.)doc";

static const char *__doc_dai_node_StereoDepth_debugExtDispLrCheckIt1 =
R"doc(Outputs ImgFrame message that carries extended left-right check first iteration
(downscaled frame, before combining with second iteration) disparity map. Useful
for debugging/fine tuning.)doc";

static const char *__doc_dai_node_StereoDepth_debugExtDispLrCheckIt2 =
R"doc(Outputs ImgFrame message that carries extended left-right check second iteration
(downscaled frame, before combining with first iteration) disparity map. Useful
for debugging/fine tuning.)doc";

static const char *__doc_dai_node_StereoDepth_depth =
R"doc(Outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in
depth units (millimeter by default).

Non-determined / invalid depth values are set to 0)doc";

static const char *__doc_dai_node_StereoDepth_disparity =
R"doc(Outputs ImgFrame message that carries RAW8 / RAW16 encoded disparity data: RAW8
encoded (0..95) for standard mode; RAW8 encoded (0..190) for extended disparity
mode; RAW16 encoded for subpixel disparity mode: - 0..760 for 3 fractional bits
(by default) - 0..1520 for 4 fractional bits - 0..3040 for 5 fractional bits)doc";

static const char *__doc_dai_node_StereoDepth_enableDistortionCorrection = R"doc(Equivalent to useHomographyRectification(!enableDistortionCorrection))doc";

static const char *__doc_dai_node_StereoDepth_getProperties = R"doc()doc";

static const char *__doc_dai_node_StereoDepth_initialConfig = R"doc(Initial config to use for StereoDepth.)doc";

static const char *__doc_dai_node_StereoDepth_inputAlignTo = R"doc(Input align to message. Default queue is non-blocking with size 1.)doc";

static const char *__doc_dai_node_StereoDepth_inputConfig = R"doc(Input StereoDepthConfig message with ability to modify parameters in runtime.)doc";

static const char *__doc_dai_node_StereoDepth_left = R"doc(Input for left ImgFrame of left-right pair)doc";

static const char *__doc_dai_node_StereoDepth_loadMeshData =
R"doc(Specify mesh calibration data for 'left' and 'right' inputs, as vectors of
bytes. Overrides useHomographyRectification behavior. See `loadMeshFiles` for
the expected data format)doc";

static const char *__doc_dai_node_StereoDepth_loadMeshFiles =
R"doc(Specify local filesystem paths to the mesh calibration files for 'left' and
'right' inputs.

When a mesh calibration is set, it overrides the camera intrinsics/extrinsics
matrices. Overrides useHomographyRectification behavior. Mesh format: a sequence
of (y,x) points as 'float' with coordinates from the input image to be mapped in
the output. The mesh can be subsampled, configured by `setMeshStep`.

With a 1280x800 resolution and the default (16,16) step, the required mesh size
is:

width: 1280 / 16 + 1 = 81

height: 800 / 16 + 1 = 51)doc";

static const char *__doc_dai_node_StereoDepth_outConfig = R"doc(Outputs StereoDepthConfig message that contains current stereo configuration.)doc";

static const char *__doc_dai_node_StereoDepth_presetMode = R"doc()doc";

static const char *__doc_dai_node_StereoDepth_rectifiedLeft =
R"doc(Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame
data.)doc";

static const char *__doc_dai_node_StereoDepth_rectifiedRight =
R"doc(Outputs ImgFrame message that carries RAW8 encoded (grayscale) rectified frame
data.)doc";

static const char *__doc_dai_node_StereoDepth_right = R"doc(Input for right ImgFrame of left-right pair)doc";

static const char *__doc_dai_node_StereoDepth_setAlphaScaling =
R"doc(Free scaling parameter between 0 (when all the pixels in the undistorted image
are valid) and 1 (when all the source image pixels are retained in the
undistorted image). On some high distortion lenses, and/or due to rectification
(image rotated) invalid areas may appear even with alpha=0, in these cases alpha
< 0.0 helps removing invalid areas. See getOptimalNewCameraMatrix from opencv
for more details.)doc";

static const char *__doc_dai_node_StereoDepth_setBaseline =
R"doc(Override baseline from calibration. Used only in disparity to depth conversion.
Units are centimeters.)doc";

static const char *__doc_dai_node_StereoDepth_setDefaultProfilePreset =
R"doc(Sets a default preset based on specified option.

Parameter ``mode``:
    Stereo depth preset mode)doc";

static const char *__doc_dai_node_StereoDepth_setDepthAlign =
R"doc(Parameter ``align``:
    Set the disparity/depth alignment: centered (between the 'left' and 'right'
    inputs), or from the perspective of a rectified output stream)doc";

static const char *__doc_dai_node_StereoDepth_setDepthAlign_2 =
R"doc(Parameter ``camera``:
    Set the camera from whose perspective the disparity/depth will be aligned)doc";

static const char *__doc_dai_node_StereoDepth_setDepthAlignmentUseSpecTranslation =
R"doc(Use baseline information for depth alignment from specs (design data) or from
calibration. Default: true)doc";

static const char *__doc_dai_node_StereoDepth_setDisparityToDepthUseSpecTranslation =
R"doc(Use baseline information for disparity to depth conversion from specs (design
data) or from calibration. Default: true)doc";

static const char *__doc_dai_node_StereoDepth_setExtendedDisparity =
R"doc(Disparity range increased from 0-95 to 0-190, combined from full resolution and
downscaled images.

Suitable for short range objects. Currently incompatible with sub-pixel
disparity)doc";

static const char *__doc_dai_node_StereoDepth_setFocalLength =
R"doc(Override focal length from calibration. Used only in disparity to depth
conversion. Units are pixels.)doc";

static const char *__doc_dai_node_StereoDepth_setFrameSync =
R"doc(Whether to enable frame syncing inside stereo node or not. Suitable if inputs
are known to be synced.)doc";

static const char *__doc_dai_node_StereoDepth_setInputResolution =
R"doc(Specify input resolution size

Optional if MonoCamera exists, otherwise necessary)doc";

static const char *__doc_dai_node_StereoDepth_setInputResolution_2 =
R"doc(Specify input resolution size

Optional if MonoCamera exists, otherwise necessary)doc";

static const char *__doc_dai_node_StereoDepth_setLeftRightCheck =
R"doc(Computes and combines disparities in both L-R and R-L directions, and combine
them.

For better occlusion handling, discarding invalid disparity values)doc";

static const char *__doc_dai_node_StereoDepth_setMeshStep = R"doc(Set the distance between mesh points. Default: (16, 16))doc";

static const char *__doc_dai_node_StereoDepth_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_StereoDepth_setOutputKeepAspectRatio =
R"doc(Specifies whether the frames resized by `setOutputSize` should preserve aspect
ratio, with potential cropping when enabled. Default `true`)doc";

static const char *__doc_dai_node_StereoDepth_setOutputSize =
R"doc(Specify disparity/depth output resolution size, implemented by scaling.

Currently only applicable when aligning to RGB camera)doc";

static const char *__doc_dai_node_StereoDepth_setPostProcessingHardwareResources =
R"doc(Specify allocated hardware resources for stereo depth. Suitable only to increase
post processing runtime.

Parameter ``numShaves``:
    Number of shaves.

Parameter ``numMemorySlices``:
    Number of memory slices.)doc";

static const char *__doc_dai_node_StereoDepth_setRectification = R"doc(Rectify input images or not.)doc";

static const char *__doc_dai_node_StereoDepth_setRectificationUseSpecTranslation =
R"doc(Obtain rectification matrices using spec translation (design data) or from
calibration in calculations. Should be used only for debugging. Default: false)doc";

static const char *__doc_dai_node_StereoDepth_setRectifyEdgeFillColor =
R"doc(Fill color for missing data at frame edges

Parameter ``color``:
    Grayscale 0..255, or -1 to replicate pixels)doc";

static const char *__doc_dai_node_StereoDepth_setRuntimeModeSwitch =
R"doc(Enable runtime stereo mode switch, e.g. from standard to LR-check. Note: when
enabled resources allocated for worst case to enable switching to any mode.)doc";

static const char *__doc_dai_node_StereoDepth_setSubpixel =
R"doc(Computes disparity with sub-pixel interpolation (3 fractional bits by default).

Suitable for long range. Currently incompatible with extended disparity)doc";

static const char *__doc_dai_node_StereoDepth_setSubpixelFractionalBits =
R"doc(Number of fractional bits for subpixel mode. Default value: 3. Valid values:
3,4,5. Defines the number of fractional disparities: 2^x. Median filter
postprocessing is supported only for 3 fractional bits.)doc";

static const char *__doc_dai_node_StereoDepth_syncedLeft = R"doc(Passthrough ImgFrame message from 'left' Input.)doc";

static const char *__doc_dai_node_StereoDepth_syncedRight = R"doc(Passthrough ImgFrame message from 'right' Input.)doc";

static const char *__doc_dai_node_StereoDepth_useHomographyRectification =
R"doc(Use 3x3 homography matrix for stereo rectification instead of sparse mesh
generated on device. Default behaviour is AUTO, for lenses with FOV over 85
degrees sparse mesh is used, otherwise 3x3 homography. If custom mesh data is
provided through loadMeshData or loadMeshFiles this option is ignored.

Parameter ``useHomographyRectification``:
    true: 3x3 homography matrix generated from calibration data is used for
    stereo rectification, can't correct lens distortion. false: sparse mesh is
    generated on-device from calibration data with mesh step specified with
    setMeshStep (Default: (16, 16)), can correct lens distortion. Implementation
    for generating the mesh is same as opencv's initUndistortRectifyMap
    function. Only the first 8 distortion coefficients are used from calibration
    data.)doc";

static const char *__doc_dai_node_Sync = R"doc(Sync node. Performs syncing between image frames)doc";

static const char *__doc_dai_node_Sync_getSyncAttempts = R"doc(Gets the number of sync attempts)doc";

static const char *__doc_dai_node_Sync_getSyncThreshold = R"doc(Gets the maximal interval between messages in the group in milliseconds)doc";

static const char *__doc_dai_node_Sync_inputs = R"doc(A map of inputs)doc";

static const char *__doc_dai_node_Sync_out = R"doc()doc";

static const char *__doc_dai_node_Sync_run = R"doc()doc";

static const char *__doc_dai_node_Sync_runOnHost = R"doc(Check if the node is set to run on host)doc";

static const char *__doc_dai_node_Sync_runOnHostVar = R"doc()doc";

static const char *__doc_dai_node_Sync_setRunOnHost =
R"doc(Specify whether to run on host or device By default, the node will run on
device.)doc";

static const char *__doc_dai_node_Sync_setSyncAttempts =
R"doc(Set the number of attempts to get the specified max interval between messages in
the group

Parameter ``syncAttempts``:
    Number of attempts to get the specified max interval between messages in the
    group: - if syncAttempts = 0 then the node sends a message as soon at the
    group is filled - if syncAttempts > 0 then the node will make syncAttemts
    attempts to synchronize before sending out a message - if syncAttempts = -1
    (default) then the node will only send a message if successfully
    synchronized)doc";

static const char *__doc_dai_node_Sync_setSyncThreshold =
R"doc(Set the maximal interval between messages in the group

Parameter ``syncThreshold``:
    Maximal interval between messages in the group)doc";

static const char *__doc_dai_node_SystemLogger = R"doc(SystemLogger node. Send system information periodically.)doc";

static const char *__doc_dai_node_SystemLogger_buildInternal = R"doc()doc";

static const char *__doc_dai_node_SystemLogger_getRate = R"doc(Gets logging rate, at which messages will be sent out)doc";

static const char *__doc_dai_node_SystemLogger_out =
R"doc(Outputs SystemInformation[S3] message that carries various system information
like memory and CPU usage, temperatures, ... For series 2 devices outputs
SystemInformation message, for series 3 devices outputs SystemInformationS3
message)doc";

static const char *__doc_dai_node_SystemLogger_setRate =
R"doc(Specify logging rate, at which messages will be sent out

Parameter ``hz``:
    Sending rate in hertz (messages per second))doc";

static const char *__doc_dai_node_Thermal = R"doc(Thermal node.)doc";

static const char *__doc_dai_node_Thermal_Thermal = R"doc()doc";

static const char *__doc_dai_node_Thermal_Thermal_2 = R"doc()doc";

static const char *__doc_dai_node_Thermal_build = R"doc(Build with a specific board socket and fps.)doc";

static const char *__doc_dai_node_Thermal_color = R"doc(Outputs YUV422i grayscale thermal image.)doc";

static const char *__doc_dai_node_Thermal_getBoardSocket =
R"doc(Retrieves which board socket to use

Returns:
    Board socket to use)doc";

static const char *__doc_dai_node_Thermal_getProperties = R"doc()doc";

static const char *__doc_dai_node_Thermal_initialConfig = R"doc(Initial config to use for thermal sensor.)doc";

static const char *__doc_dai_node_Thermal_inputConfig =
R"doc(Input ThermalConfig message with ability to modify parameters in runtime.
Default queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_Thermal_isBuilt = R"doc()doc";

static const char *__doc_dai_node_Thermal_setFps = R"doc()doc";

static const char *__doc_dai_node_Thermal_temperature = R"doc(Outputs FP16 (degC) thermal image.)doc";

static const char *__doc_dai_node_ThreadedHostNode = R"doc()doc";

static const char *__doc_dai_node_ThreadedHostNode_runOnHost = R"doc()doc";

static const char *__doc_dai_node_ToF =
R"doc(ToF node. Performs feature tracking and reidentification using motion estimation
between 2 consecutive frames.)doc";

static const char *__doc_dai_node_ToF_ToF = R"doc()doc";

static const char *__doc_dai_node_ToF_ToF_2 = R"doc()doc";

static const char *__doc_dai_node_ToF_amplitude = R"doc()doc";

static const char *__doc_dai_node_ToF_build = R"doc(Build with a specific board socket)doc";

static const char *__doc_dai_node_ToF_depth = R"doc()doc";

static const char *__doc_dai_node_ToF_getBoardSocket =
R"doc(Retrieves which board socket to use

Returns:
    Board socket to use)doc";

static const char *__doc_dai_node_ToF_getProperties = R"doc()doc";

static const char *__doc_dai_node_ToF_initialConfig = R"doc(Initial config to use for feature tracking.)doc";

static const char *__doc_dai_node_ToF_inputConfig =
R"doc(Input ToFConfig message with ability to modify parameters in runtime. Default
queue is non-blocking with size 4.)doc";

static const char *__doc_dai_node_ToF_intensity = R"doc()doc";

static const char *__doc_dai_node_ToF_isBuilt = R"doc()doc";

static const char *__doc_dai_node_ToF_maxHeight = R"doc()doc";

static const char *__doc_dai_node_ToF_maxWidth = R"doc()doc";

static const char *__doc_dai_node_ToF_phase = R"doc()doc";

static const char *__doc_dai_node_UVC = R"doc(UVC (USB Video Class) node)doc";

static const char *__doc_dai_node_UVC_UVC = R"doc()doc";

static const char *__doc_dai_node_UVC_UVC_2 = R"doc()doc";

static const char *__doc_dai_node_UVC_input =
R"doc(Input for image frames to be streamed over UVC Default queue is blocking with
size 8)doc";

static const char *__doc_dai_node_UVC_setGpiosOnInit = R"doc(Set GPIO list <gpio_number, value> for GPIOs to set (on/off) at init)doc";

static const char *__doc_dai_node_UVC_setGpiosOnStreamOff = R"doc(Set GPIO list <gpio_number, value> for GPIOs to set when streaming is disabled)doc";

static const char *__doc_dai_node_UVC_setGpiosOnStreamOn = R"doc(Set GPIO list <gpio_number, value> for GPIOs to set when streaming is enabled)doc";

static const char *__doc_dai_node_VideoEncoder = R"doc(VideoEncoder node. Encodes frames into MJPEG, H264 or H265.)doc";

static const char *__doc_dai_node_VideoEncoder_bitstream =
R"doc(Outputs ImgFrame message that carries BITSTREAM encoded (MJPEG, H264 or H265)
frame data. Mutually exclusive with out.)doc";

static const char *__doc_dai_node_VideoEncoder_build = R"doc()doc";

static const char *__doc_dai_node_VideoEncoder_getBitrate = R"doc(Get bitrate in bps)doc";

static const char *__doc_dai_node_VideoEncoder_getBitrateKbps = R"doc(Get bitrate in kbps)doc";

static const char *__doc_dai_node_VideoEncoder_getFrameRate = R"doc(Get frame rate)doc";

static const char *__doc_dai_node_VideoEncoder_getKeyframeFrequency = R"doc(Get keyframe frequency)doc";

static const char *__doc_dai_node_VideoEncoder_getLossless = R"doc(Get lossless mode. Applies only when using [M]JPEG profile.)doc";

static const char *__doc_dai_node_VideoEncoder_getMaxOutputFrameSize = R"doc()doc";

static const char *__doc_dai_node_VideoEncoder_getNumBFrames = R"doc(Get number of B frames)doc";

static const char *__doc_dai_node_VideoEncoder_getNumFramesPool =
R"doc(Get number of frames in pool

Returns:
    Number of pool frames)doc";

static const char *__doc_dai_node_VideoEncoder_getProfile = R"doc(Get profile)doc";

static const char *__doc_dai_node_VideoEncoder_getQuality = R"doc(Get quality)doc";

static const char *__doc_dai_node_VideoEncoder_getRateControlMode = R"doc(Get rate control mode)doc";

static const char *__doc_dai_node_VideoEncoder_input = R"doc(Input for NV12 ImgFrame to be encoded)doc";

static const char *__doc_dai_node_VideoEncoder_out =
R"doc(Outputs EncodedFrame message that carries encoded (MJPEG, H264 or H265) frame
data. Mutually exclusive with bitstream.)doc";

static const char *__doc_dai_node_VideoEncoder_setBitrate =
R"doc(Set output bitrate in bps, for CBR rate control mode. 0 for auto (based on frame
size and FPS))doc";

static const char *__doc_dai_node_VideoEncoder_setBitrateKbps =
R"doc(Set output bitrate in kbps, for CBR rate control mode. 0 for auto (based on
frame size and FPS))doc";

static const char *__doc_dai_node_VideoEncoder_setDefaultProfilePreset =
R"doc(Sets a default preset based on specified frame rate and profile

Parameter ``fps``:
    Frame rate in frames per second

Parameter ``profile``:
    Encoding profile)doc";

static const char *__doc_dai_node_VideoEncoder_setFrameRate =
R"doc(Sets expected frame rate

Parameter ``frameRate``:
    Frame rate in frames per second)doc";

static const char *__doc_dai_node_VideoEncoder_setKeyframeFrequency =
R"doc(Set keyframe frequency. Every Nth frame a keyframe is inserted.

Applicable only to H264 and H265 profiles

Examples:

- 30 FPS video, keyframe frequency: 30. Every 1s a keyframe will be inserted

- 60 FPS video, keyframe frequency: 180. Every 3s a keyframe will be inserted)doc";

static const char *__doc_dai_node_VideoEncoder_setLossless =
R"doc(Set lossless mode. Applies only to [M]JPEG profile

Parameter ``lossless``:
    True to enable lossless jpeg encoding, false otherwise)doc";

static const char *__doc_dai_node_VideoEncoder_setMaxOutputFrameSize = R"doc(Specifies maximum output encoded frame size)doc";

static const char *__doc_dai_node_VideoEncoder_setNumBFrames = R"doc(Set number of B frames to be inserted)doc";

static const char *__doc_dai_node_VideoEncoder_setNumFramesPool =
R"doc(Set number of frames in pool

Parameter ``frames``:
    Number of pool frames)doc";

static const char *__doc_dai_node_VideoEncoder_setProfile = R"doc(Set encoding profile)doc";

static const char *__doc_dai_node_VideoEncoder_setQuality =
R"doc(Set quality

Parameter ``quality``:
    Value between 0-100%. Approximates quality)doc";

static const char *__doc_dai_node_VideoEncoder_setRateControlMode = R"doc(Set rate control mode)doc";

static const char *__doc_dai_node_Warp = R"doc(Warp node. Capability to crop, resize, warp, ... incoming image frames)doc";

static const char *__doc_dai_node_Warp_getHwIds = R"doc(Retrieve which hardware warp engines to use)doc";

static const char *__doc_dai_node_Warp_getInterpolation = R"doc(Retrieve which interpolation method to use)doc";

static const char *__doc_dai_node_Warp_inputImage = R"doc(Input image to be modified Default queue is blocking with size 8)doc";

static const char *__doc_dai_node_Warp_out = R"doc(Outputs ImgFrame message that carries warped image.)doc";

static const char *__doc_dai_node_Warp_setHwIds =
R"doc(Specify which hardware warp engines to use

Parameter ``ids``:
    Which warp engines to use (0, 1, 2))doc";

static const char *__doc_dai_node_Warp_setInterpolation =
R"doc(Specify which interpolation method to use

Parameter ``interpolation``:
    type of interpolation)doc";

static const char *__doc_dai_node_Warp_setMaxOutputFrameSize =
R"doc(Specify maximum size of output image.

Parameter ``maxFrameSize``:
    Maximum frame size in bytes)doc";

static const char *__doc_dai_node_Warp_setNumFramesPool =
R"doc(Specify number of frames in pool.

Parameter ``numFramesPool``:
    How many frames should the pool have)doc";

static const char *__doc_dai_node_Warp_setOutputSize =
R"doc(Sets output frame size in pixels

Parameter ``size``:
    width and height in pixels)doc";

static const char *__doc_dai_node_Warp_setOutputSize_2 = R"doc()doc";

static const char *__doc_dai_node_Warp_setWarpMesh = R"doc()doc";

static const char *__doc_dai_node_Warp_setWarpMesh_2 =
R"doc(Set a custom warp mesh

Parameter ``meshData``:
    2D plane of mesh points, starting from top left to bottom right

Parameter ``width``:
    Width of mesh

Parameter ``height``:
    Height of mesh)doc";

static const char *__doc_dai_node_Warp_setWarpMesh_3 = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkIn = R"doc(XLinkIn node. Receives messages over XLink.)doc";

static const char *__doc_dai_node_internal_XLinkInHost = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_conn = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_disconnect = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_isDisconnected = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_isWaitingForReconnect = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_mtx = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_out = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_run = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_setConnection = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_setStreamName = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkInHost_streamName = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkIn_getMaxDataSize = R"doc(Get maximum messages size in bytes)doc";

static const char *__doc_dai_node_internal_XLinkIn_getNumFrames = R"doc(Get number of frames in pool)doc";

static const char *__doc_dai_node_internal_XLinkIn_getStreamName = R"doc(Get stream name)doc";

static const char *__doc_dai_node_internal_XLinkIn_out = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkIn_setMaxDataSize =
R"doc(Set maximum message size it can receive

Parameter ``maxDataSize``:
    Maximum size in bytes)doc";

static const char *__doc_dai_node_internal_XLinkIn_setNumFrames =
R"doc(Set number of frames in pool for sending messages forward

Parameter ``numFrames``:
    Maximum number of frames in pool)doc";

static const char *__doc_dai_node_internal_XLinkIn_setStreamName =
R"doc(Specifies XLink stream name to use.

The name should not start with double underscores '__', as those are reserved
for internal use.

Parameter ``name``:
    Stream name)doc";

static const char *__doc_dai_node_internal_XLinkOut = R"doc(XLinkOut node. Sends messages over XLink.)doc";

static const char *__doc_dai_node_internal_XLinkOutHost = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_allowResize = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_allowStreamResize = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_conn = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_disconnect = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_in = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_isDisconnected = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_isWaitingForReconnect = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_mtx = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_run = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_setConnection = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_setStreamName = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOutHost_streamName = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOut_buildInternal = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOut_getFpsLimit = R"doc(Get rate limit in messages per second)doc";

static const char *__doc_dai_node_internal_XLinkOut_getMetadataOnly = R"doc(Get whether to transfer only messages attributes and not buffer data)doc";

static const char *__doc_dai_node_internal_XLinkOut_getStreamName = R"doc(Get stream name)doc";

static const char *__doc_dai_node_internal_XLinkOut_input = R"doc()doc";

static const char *__doc_dai_node_internal_XLinkOut_setFpsLimit =
R"doc(Specifies a message sending limit. It's approximated from specified rate.

Parameter ``fps``:
    Approximate rate limit in messages per second)doc";

static const char *__doc_dai_node_internal_XLinkOut_setMetadataOnly = R"doc(Specify whether to transfer only messages attributes and not buffer data)doc";

static const char *__doc_dai_node_internal_XLinkOut_setStreamName =
R"doc(Specifies XLink stream name to use.

The name should not start with double underscores '__', as those are reserved
for internal use.

Parameter ``name``:
    Stream name)doc";

static const char *__doc_dai_node_test_MyConsumer = R"doc(XLinkOut node. Sends messages over XLink.)doc";

static const char *__doc_dai_node_test_MyConsumer_input =
R"doc(Input for any type of messages to be transferred over XLink stream Default queue
is blocking with size 8)doc";

static const char *__doc_dai_node_test_MyConsumer_run = R"doc()doc";

static const char *__doc_dai_node_test_MyProducer = R"doc(XLinkOut node. Sends messages over XLink.)doc";

static const char *__doc_dai_node_test_MyProducer_out = R"doc(Outputs message of same type as sent from host.)doc";

static const char *__doc_dai_node_test_MyProducer_run = R"doc()doc";

static const char *__doc_dai_operator_lshift = R"doc()doc";

static const char *__doc_dai_operator_lshift_2 = R"doc()doc";

static const char *__doc_dai_operator_lshift_3 = R"doc()doc";

static const char *__doc_dai_operator_lshift_4 =
R"doc(Output the system-dependent representation of the pointer contained in a
copyable_unique_ptr object. This is equivalent to `os << p.get();`. @relates
copyable_unique_ptr)doc";

static const char *__doc_dai_platform2string =
R"doc(Convert Platform enum to string

Parameter ``platform``:
    Platform enum

Returns:
    std::string String representation of Platform)doc";

static const char *__doc_dai_proto_event_Event = R"doc()doc";

static const char *__doc_dai_span = R"doc()doc";

static const char *__doc_dai_span_2 = R"doc()doc";

static const char *__doc_dai_span_back = R"doc()doc";

static const char *__doc_dai_span_begin = R"doc()doc";

static const char *__doc_dai_span_data = R"doc()doc";

static const char *__doc_dai_span_empty = R"doc()doc";

static const char *__doc_dai_span_end = R"doc()doc";

static const char *__doc_dai_span_first = R"doc()doc";

static const char *__doc_dai_span_first_2 = R"doc()doc";

static const char *__doc_dai_span_front = R"doc()doc";

static const char *__doc_dai_span_last = R"doc()doc";

static const char *__doc_dai_span_last_2 = R"doc()doc";

static const char *__doc_dai_span_operator_array = R"doc()doc";

static const char *__doc_dai_span_operator_assign = R"doc()doc";

static const char *__doc_dai_span_rbegin = R"doc()doc";

static const char *__doc_dai_span_rend = R"doc()doc";

static const char *__doc_dai_span_size = R"doc()doc";

static const char *__doc_dai_span_size_bytes = R"doc()doc";

static const char *__doc_dai_span_span = R"doc()doc";

static const char *__doc_dai_span_span_2 = R"doc()doc";

static const char *__doc_dai_span_span_3 = R"doc()doc";

static const char *__doc_dai_span_span_4 = R"doc()doc";

static const char *__doc_dai_span_span_5 = R"doc()doc";

static const char *__doc_dai_span_span_6 = R"doc()doc";

static const char *__doc_dai_span_span_7 = R"doc()doc";

static const char *__doc_dai_span_span_8 = R"doc()doc";

static const char *__doc_dai_span_span_9 = R"doc()doc";

static const char *__doc_dai_span_span_10 = R"doc()doc";

static const char *__doc_dai_span_storage = R"doc()doc";

static const char *__doc_dai_span_subspan = R"doc()doc";

static const char *__doc_dai_span_subspan_2 = R"doc()doc";

static const char *__doc_dai_string2platform =
R"doc(Convert string to Platform enum

Parameter ``platform``:
    String representation of Platform

Returns:
    Platform Platform enum)doc";

static const char *__doc_dai_swap = R"doc()doc";

static const char *__doc_dai_toString = R"doc()doc";

static const char *__doc_dai_to_json = R"doc()doc";

static const char *__doc_dai_to_json_2 = R"doc()doc";

static const char *__doc_dai_to_json_3 = R"doc()doc";

static const char *__doc_dai_to_json_4 = R"doc()doc";

static const char *__doc_dai_to_json_5 = R"doc()doc";

static const char *__doc_dai_to_json_6 = R"doc()doc";

static const char *__doc_dai_to_json_7 = R"doc()doc";

static const char *__doc_dai_to_json_8 = R"doc()doc";

static const char *__doc_dai_to_json_9 = R"doc()doc";

static const char *__doc_dai_to_json_10 = R"doc()doc";

static const char *__doc_dai_to_json_11 = R"doc()doc";

static const char *__doc_dai_to_json_12 = R"doc()doc";

static const char *__doc_dai_to_json_13 = R"doc()doc";

static const char *__doc_dai_to_json_14 = R"doc()doc";

static const char *__doc_dai_to_json_15 = R"doc()doc";

static const char *__doc_dai_to_json_16 = R"doc()doc";

static const char *__doc_dai_to_json_17 = R"doc()doc";

static const char *__doc_dai_to_json_18 = R"doc()doc";

static const char *__doc_dai_to_json_19 = R"doc()doc";

static const char *__doc_dai_to_json_20 = R"doc()doc";

static const char *__doc_dai_to_json_21 = R"doc()doc";

static const char *__doc_dai_to_json_22 = R"doc()doc";

static const char *__doc_dai_to_json_23 = R"doc()doc";

static const char *__doc_dai_to_json_24 = R"doc()doc";

static const char *__doc_dai_to_json_25 = R"doc()doc";

static const char *__doc_dai_to_json_26 = R"doc()doc";

static const char *__doc_dai_to_json_27 = R"doc()doc";

static const char *__doc_dai_to_json_28 = R"doc()doc";

static const char *__doc_dai_to_json_29 = R"doc()doc";

static const char *__doc_dai_to_json_30 = R"doc()doc";

static const char *__doc_dai_to_json_31 = R"doc()doc";

static const char *__doc_dai_to_json_32 = R"doc()doc";

static const char *__doc_dai_to_json_33 = R"doc()doc";

static const char *__doc_dai_to_json_34 = R"doc()doc";

static const char *__doc_dai_to_json_35 = R"doc()doc";

static const char *__doc_dai_to_json_36 = R"doc()doc";

static const char *__doc_dai_to_json_37 = R"doc()doc";

static const char *__doc_dai_to_json_38 = R"doc()doc";

static const char *__doc_dai_to_json_39 = R"doc()doc";

static const char *__doc_dai_to_json_40 = R"doc()doc";

static const char *__doc_dai_to_json_41 = R"doc()doc";

static const char *__doc_dai_to_json_42 = R"doc()doc";

static const char *__doc_dai_to_json_43 = R"doc()doc";

static const char *__doc_dai_to_json_44 = R"doc()doc";

static const char *__doc_dai_to_json_45 = R"doc()doc";

static const char *__doc_dai_to_json_46 = R"doc()doc";

static const char *__doc_dai_to_json_47 = R"doc()doc";

static const char *__doc_dai_to_json_48 = R"doc()doc";

static const char *__doc_dai_to_json_49 = R"doc()doc";

static const char *__doc_dai_to_json_50 = R"doc()doc";

static const char *__doc_dai_to_json_51 = R"doc()doc";

static const char *__doc_dai_to_json_52 = R"doc()doc";

static const char *__doc_dai_to_json_53 = R"doc()doc";

static const char *__doc_dai_to_json_54 = R"doc()doc";

static const char *__doc_dai_to_json_55 = R"doc()doc";

static const char *__doc_dai_to_json_56 = R"doc()doc";

static const char *__doc_dai_to_json_57 = R"doc()doc";

static const char *__doc_dai_to_json_58 = R"doc()doc";

static const char *__doc_dai_to_json_59 = R"doc()doc";

static const char *__doc_dai_to_json_60 = R"doc()doc";

static const char *__doc_dai_to_json_61 = R"doc()doc";

static const char *__doc_dai_to_json_62 = R"doc()doc";

static const char *__doc_dai_to_json_63 = R"doc()doc";

static const char *__doc_dai_to_json_64 = R"doc()doc";

static const char *__doc_dai_to_json_65 = R"doc()doc";

static const char *__doc_dai_to_json_66 = R"doc()doc";

static const char *__doc_dai_to_json_67 = R"doc()doc";

static const char *__doc_dai_to_json_68 = R"doc()doc";

static const char *__doc_dai_to_json_69 = R"doc()doc";

static const char *__doc_dai_to_json_70 = R"doc()doc";

static const char *__doc_dai_to_json_71 = R"doc()doc";

static const char *__doc_dai_to_json_72 = R"doc()doc";

static const char *__doc_dai_to_json_73 = R"doc()doc";

static const char *__doc_dai_to_json_74 = R"doc()doc";

static const char *__doc_dai_to_json_75 = R"doc()doc";

static const char *__doc_dai_to_json_76 = R"doc()doc";

static const char *__doc_dai_to_json_77 = R"doc()doc";

static const char *__doc_dai_to_json_78 = R"doc()doc";

static const char *__doc_dai_to_json_79 = R"doc()doc";

static const char *__doc_dai_to_json_80 = R"doc()doc";

static const char *__doc_dai_to_json_81 = R"doc()doc";

static const char *__doc_dai_to_json_82 = R"doc()doc";

static const char *__doc_dai_to_json_83 = R"doc()doc";

static const char *__doc_dai_to_json_84 = R"doc()doc";

static const char *__doc_dai_to_json_85 = R"doc()doc";

static const char *__doc_dai_to_json_86 = R"doc()doc";

static const char *__doc_dai_utility_EventData = R"doc()doc";

static const char *__doc_dai_utility_EventDataType = R"doc()doc";

static const char *__doc_dai_utility_EventDataType_DATA = R"doc()doc";

static const char *__doc_dai_utility_EventDataType_ENCODED_FRAME = R"doc()doc";

static const char *__doc_dai_utility_EventDataType_FILE_URL = R"doc()doc";

static const char *__doc_dai_utility_EventDataType_IMG_FRAME = R"doc()doc";

static const char *__doc_dai_utility_EventDataType_NN_DATA = R"doc()doc";

static const char *__doc_dai_utility_EventData_EventData = R"doc()doc";

static const char *__doc_dai_utility_EventData_EventData_2 = R"doc()doc";

static const char *__doc_dai_utility_EventData_EventData_3 = R"doc()doc";

static const char *__doc_dai_utility_EventData_EventData_4 = R"doc()doc";

static const char *__doc_dai_utility_EventData_EventData_5 = R"doc()doc";

static const char *__doc_dai_utility_EventData_data = R"doc()doc";

static const char *__doc_dai_utility_EventData_fileName = R"doc()doc";

static const char *__doc_dai_utility_EventData_mimeType = R"doc()doc";

static const char *__doc_dai_utility_EventData_toFile = R"doc()doc";

static const char *__doc_dai_utility_EventData_type = R"doc()doc";

static const char *__doc_dai_utility_EventsManager = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_EventMessage = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_EventMessage_cachePath = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_EventMessage_data = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_EventMessage_event = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_EventsManager = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_cacheDir = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_cacheEvents = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_cacheIfCannotSend = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_checkConnection =
R"doc(Check if the device is connected to Hub. Performs a simple GET request to the
URL/health endpoint

Returns:
    bool)doc";

static const char *__doc_dai_utility_EventsManager_checkForCachedData = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_createUUID = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_deviceSerialNumber = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_eventBuffer = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_eventBufferCondition = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_eventBufferConditionMutex = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_eventBufferMutex = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_eventBufferThread = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_logResponse = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_publishInterval = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_queueSize = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_sendEvent =
R"doc(Send an event to the events service

Parameter ``name``:
    Name of the event

Parameter ``imgFrame``:
    Image frame to send

Parameter ``data``:
    List of EventData objects to send

Parameter ``tags``:
    List of tags to send

Parameter ``extraData``:
    Extra data to send

Parameter ``deviceSerialNo``:
    Device serial number

Returns:
    bool)doc";

static const char *__doc_dai_utility_EventsManager_sendEventBuffer = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_sendFile = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_sendSnap =
R"doc(Send a snap to the events service. Snaps should be used for sending images and
other large files.

Parameter ``name``:
    Name of the snap

Parameter ``imgFrame``:
    Image frame to send

Parameter ``data``:
    List of EventData objects to send

Parameter ``tags``:
    List of tags to send

Parameter ``extraData``:
    Extra data to send

Parameter ``deviceSerialNo``:
    Device serial number

Returns:
    bool)doc";

static const char *__doc_dai_utility_EventsManager_setCacheDir =
R"doc(Set the cache directory for storing cached data. By default, the cache directory
is set to /internal/private

Parameter ``cacheDir``:
    Cache directory

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setCacheIfCannotSend =
R"doc(Set whether to cache data if it cannot be sent. By default, cacheIfCannotSend is
set to false

Parameter ``cacheIfCannotSend``:
    bool

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setDeviceSerialNumber = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_setLogResponse =
R"doc(Set whether to log the responses from the server. By default, logResponse is set
to false

Parameter ``logResponse``:
    bool

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setQueueSize =
R"doc(Set the queue size for the amount of events that can be added and sent. By
default, the queue size is set to 10

Parameter ``queueSize``:
    Queue size

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setSourceAppId =
R"doc(Set the source app ID. By default, the source app ID is taken from the
environment variable AGENT_APP_ID

Parameter ``sourceAppId``:
    Source app ID

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setSourceAppIdentifier =
R"doc(Set the source app identifier. By default, the source app identifier is taken
from the environment variable AGENT_APP_IDENTIFIER

Parameter ``sourceAppIdentifier``:
    Source app identifier

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setToken =
R"doc(Set the token for the events service. By default, the token is taken from the
environment variable DEPTHAI_HUB_API_KEY

Parameter ``token``:
    Token for the events service

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setUrl =
R"doc(Set the URL of the events service. By default, the URL is set to https://events-
ingest.cloud.luxonis.com

Parameter ``url``:
    URL of the events service

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_setVerifySsl =
R"doc(Set whether to verify the SSL certificate. By default, verifySsl is set to false

Parameter ``verifySsl``:
    bool

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_sourceAppId = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_sourceAppIdentifier = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_stopEventBuffer = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_token = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_uploadCachedData =
R"doc(Upload cached data to the events service

Returns:
    void)doc";

static const char *__doc_dai_utility_EventsManager_url = R"doc()doc";

static const char *__doc_dai_utility_EventsManager_verifySsl = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_Prepare = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_ReturnStatus = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_Skip = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_VectorWriter = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_VectorWriter_2 = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_Write = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_Write_2 = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_operator_assign = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_ref = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_ref_2 = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_take = R"doc()doc";

static const char *__doc_dai_utility_VectorWriter_vector = R"doc()doc";

static const char *__doc_dai_utility_checksum =
R"doc(Simple hash function - djb2

Parameter ``buffer``:
    Pointer to buffer of data to hash

Parameter ``size``:
    Size of buffer in bytes

Parameter ``prevChecksum``:
    Previous checksum - useful for doing hash on blocks of data)doc";

static const char *__doc_dai_utility_checksum_2 =
R"doc(Simple hash function - djb2

Parameter ``buffer``:
    Pointer to buffer of data to hash

Parameter ``size``:
    Size of buffer in bytes)doc";

static const char *__doc_dai_utility_deflate = R"doc()doc";

static const char *__doc_dai_utility_deserialize = R"doc()doc";

static const char *__doc_dai_utility_deserialize_2 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_3 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_4 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_5 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_6 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_7 = R"doc()doc";

static const char *__doc_dai_utility_deserialize_8 = R"doc()doc";

static const char *__doc_dai_utility_filenamesInTar =
R"doc(Gets a list of filenames contained within a tar archive.

Parameter ``tarPath``:
    Path to the tar file to read

Returns:
    Vector of paths for the files within the tar archive)doc";

static const char *__doc_dai_utility_inflate = R"doc()doc";

static const char *__doc_dai_utility_jsonDisplay = R"doc()doc";

static const char *__doc_dai_utility_serialize = R"doc()doc";

static const char *__doc_dai_utility_serialize_2 = R"doc()doc";

static const char *__doc_dai_utility_serialize_3 = R"doc()doc";

static const char *__doc_dai_utility_serialize_4 = R"doc()doc";

static const char *__doc_dai_utility_serialize_5 = R"doc()doc";

static const char *__doc_dai_utility_serialize_6 = R"doc()doc";

static const char *__doc_dai_utility_serialize_7 = R"doc()doc";

static const char *__doc_dai_utility_serialize_8 = R"doc()doc";

static const char *__doc_dai_utility_tarFiles =
R"doc(Creates a tar archive containing the specified files.

Parameter ``tarPath``:
    Path where the tar file will be created

Parameter ``filesOnDisk``:
    Vector of paths to file on the host filesystem to include in the archive

Parameter ``filesInTar``:
    Vector of paths for the files within the tar archive)doc";

static const char *__doc_dai_utility_untarFiles =
R"doc(Extracts files from a tar archive.

Parameter ``tarPath``:
    Path to the tar file to extract from

Parameter ``filesInTar``:
    Vector of paths for the files within the tar to extract

Parameter ``filesOnDisk``:
    Vector of paths where the extracted files should be written)doc";

static const char *__doc_mcap_Attachment =
R"doc(An Attachment is an arbitrary file embedded in an MCAP file, including a name,
media type, timestamps, and optional CRC. Attachment records are written in the
Data section, outside of Chunks.)doc";

static const char *__doc_mcap_AttachmentIndex =
R"doc(Attachment Index records are found in the Summary section, providing summary
information for a single Attachment.)doc";

static const char *__doc_mcap_AttachmentIndex_AttachmentIndex = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_AttachmentIndex_2 = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_createTime = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_dataSize = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_length = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_logTime = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_mediaType = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_name = R"doc()doc";

static const char *__doc_mcap_AttachmentIndex_offset = R"doc()doc";

static const char *__doc_mcap_Attachment_crc = R"doc()doc";

static const char *__doc_mcap_Attachment_createTime = R"doc()doc";

static const char *__doc_mcap_Attachment_data = R"doc()doc";

static const char *__doc_mcap_Attachment_dataSize = R"doc()doc";

static const char *__doc_mcap_Attachment_logTime = R"doc()doc";

static const char *__doc_mcap_Attachment_mediaType = R"doc()doc";

static const char *__doc_mcap_Attachment_name = R"doc()doc";

static const char *__doc_mcap_BufferReader =
R"doc(A "null" compressed reader that directly passes through uncompressed data. No
internal buffers are allocated.)doc";

static const char *__doc_mcap_BufferReader_BufferReader = R"doc()doc";

static const char *__doc_mcap_BufferReader_BufferReader_2 = R"doc()doc";

static const char *__doc_mcap_BufferReader_BufferReader_3 = R"doc()doc";

static const char *__doc_mcap_BufferReader_data = R"doc()doc";

static const char *__doc_mcap_BufferReader_operator_assign = R"doc()doc";

static const char *__doc_mcap_BufferReader_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_BufferReader_read = R"doc()doc";

static const char *__doc_mcap_BufferReader_reset = R"doc()doc";

static const char *__doc_mcap_BufferReader_size = R"doc()doc";

static const char *__doc_mcap_BufferReader_size_2 = R"doc()doc";

static const char *__doc_mcap_BufferReader_status = R"doc()doc";

static const char *__doc_mcap_BufferWriter = R"doc(An in-memory IChunkWriter implementation backed by a growable buffer.)doc";

static const char *__doc_mcap_BufferWriter_buffer = R"doc()doc";

static const char *__doc_mcap_BufferWriter_compressedData = R"doc()doc";

static const char *__doc_mcap_BufferWriter_compressedSize = R"doc()doc";

static const char *__doc_mcap_BufferWriter_data = R"doc()doc";

static const char *__doc_mcap_BufferWriter_empty = R"doc()doc";

static const char *__doc_mcap_BufferWriter_end = R"doc()doc";

static const char *__doc_mcap_BufferWriter_handleClear = R"doc()doc";

static const char *__doc_mcap_BufferWriter_handleWrite = R"doc()doc";

static const char *__doc_mcap_BufferWriter_size = R"doc()doc";

static const char *__doc_mcap_Channel =
R"doc(Describes a Channel that messages are written to. A Channel represents a single
connection from a publisher to a topic, so each topic will have one Channel per
publisher. Channels optionally reference a Schema, for message encodings that
are not self-describing (e.g. JSON) or when schema information is available
(e.g. JSONSchema).)doc";

static const char *__doc_mcap_Channel_Channel = R"doc()doc";

static const char *__doc_mcap_Channel_Channel_2 = R"doc()doc";

static const char *__doc_mcap_Channel_id = R"doc()doc";

static const char *__doc_mcap_Channel_messageEncoding = R"doc()doc";

static const char *__doc_mcap_Channel_metadata = R"doc()doc";

static const char *__doc_mcap_Channel_schemaId = R"doc()doc";

static const char *__doc_mcap_Channel_topic = R"doc()doc";

static const char *__doc_mcap_Chunk =
R"doc(An collection of Schemas, Channels, and Messages that supports compression and
indexing.)doc";

static const char *__doc_mcap_ChunkIndex =
R"doc(Chunk Index records are found in the Summary section, providing summary
information for a single Chunk and pointing to each Message Index record
associated with that Chunk.)doc";

static const char *__doc_mcap_ChunkIndex_chunkLength = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_chunkStartOffset = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_compressedSize = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_compression = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_messageEndTime = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_messageIndexLength = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_messageIndexOffsets = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_messageStartTime = R"doc()doc";

static const char *__doc_mcap_ChunkIndex_uncompressedSize = R"doc()doc";

static const char *__doc_mcap_Chunk_compressedSize = R"doc()doc";

static const char *__doc_mcap_Chunk_compression = R"doc()doc";

static const char *__doc_mcap_Chunk_messageEndTime = R"doc()doc";

static const char *__doc_mcap_Chunk_messageStartTime = R"doc()doc";

static const char *__doc_mcap_Chunk_records = R"doc()doc";

static const char *__doc_mcap_Chunk_uncompressedCrc = R"doc()doc";

static const char *__doc_mcap_Chunk_uncompressedSize = R"doc()doc";

static const char *__doc_mcap_Compression = R"doc(Supported MCAP compression algorithms.)doc";

static const char *__doc_mcap_CompressionLevel =
R"doc(Compression level to use when compression is enabled. Slower generally produces
smaller files, at the expense of more CPU time. These levels map to different
internal settings for each compression algorithm.)doc";

static const char *__doc_mcap_CompressionLevel_Default = R"doc()doc";

static const char *__doc_mcap_CompressionLevel_Fast = R"doc()doc";

static const char *__doc_mcap_CompressionLevel_Fastest = R"doc()doc";

static const char *__doc_mcap_CompressionLevel_Slow = R"doc()doc";

static const char *__doc_mcap_CompressionLevel_Slowest = R"doc()doc";

static const char *__doc_mcap_Compression_Lz4 = R"doc()doc";

static const char *__doc_mcap_Compression_None = R"doc()doc";

static const char *__doc_mcap_Compression_Zstd = R"doc()doc";

static const char *__doc_mcap_DataEnd =
R"doc(The final record in the Data section, signaling the end of Data and beginning of
Summary. Optionally contains a CRC of the entire Data section.)doc";

static const char *__doc_mcap_DataEnd_dataSectionCrc = R"doc()doc";

static const char *__doc_mcap_FileReader =
R"doc(IReadable implementation wrapping a FILE* pointer created by fopen() and a read
buffer.)doc";

static const char *__doc_mcap_FileReader_FileReader = R"doc()doc";

static const char *__doc_mcap_FileReader_buffer = R"doc()doc";

static const char *__doc_mcap_FileReader_file = R"doc()doc";

static const char *__doc_mcap_FileReader_position = R"doc()doc";

static const char *__doc_mcap_FileReader_read = R"doc()doc";

static const char *__doc_mcap_FileReader_size = R"doc()doc";

static const char *__doc_mcap_FileReader_size_2 = R"doc()doc";

static const char *__doc_mcap_FileStreamReader = R"doc(IReadable implementation wrapping a std::ifstream input file stream.)doc";

static const char *__doc_mcap_FileStreamReader_FileStreamReader = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_buffer = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_position = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_read = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_size = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_size_2 = R"doc()doc";

static const char *__doc_mcap_FileStreamReader_stream = R"doc()doc";

static const char *__doc_mcap_FileWriter =
R"doc(Implements the IWritable interface used by McapWriter by wrapping a FILE*
pointer created by fopen().)doc";

static const char *__doc_mcap_FileWriter_end = R"doc()doc";

static const char *__doc_mcap_FileWriter_file = R"doc()doc";

static const char *__doc_mcap_FileWriter_handleWrite = R"doc()doc";

static const char *__doc_mcap_FileWriter_open = R"doc()doc";

static const char *__doc_mcap_FileWriter_size = R"doc()doc";

static const char *__doc_mcap_FileWriter_size_2 = R"doc()doc";

static const char *__doc_mcap_Footer =
R"doc(The final record in an MCAP file (before the trailing magic byte sequence).
Contains byte offsets from the start of the file to the Summary and Summary
Offset sections, along with an optional CRC of the combined Summary and Summary
Offset sections. A `summaryStart` and `summaryOffsetStart` of zero indicates no
Summary section is available.)doc";

static const char *__doc_mcap_Footer_Footer = R"doc()doc";

static const char *__doc_mcap_Footer_Footer_2 = R"doc()doc";

static const char *__doc_mcap_Footer_summaryCrc = R"doc()doc";

static const char *__doc_mcap_Footer_summaryOffsetStart = R"doc()doc";

static const char *__doc_mcap_Footer_summaryStart = R"doc()doc";

static const char *__doc_mcap_Header =
R"doc(Appears at the beginning of every MCAP file (after the magic byte sequence) and
contains the recording profile (see
<https://github.com/foxglove/mcap/tree/main/docs/specification/profiles>) and a
string signature of the recording library.)doc";

static const char *__doc_mcap_Header_library = R"doc()doc";

static const char *__doc_mcap_Header_profile = R"doc()doc";

static const char *__doc_mcap_IChunkWriter =
R"doc(An abstract interface for writing Chunk data. Chunk data is buffered in memory
and written to disk as a single record, to support optimal compression and
calculating the final Chunk data size.)doc";

static const char *__doc_mcap_IChunkWriter_clear = R"doc(Clear the internal state of the writer, discarding any input or output buffers.)doc";

static const char *__doc_mcap_IChunkWriter_compressedData =
R"doc(Returns a pointer to the compressed data. This will only be called after
`end()`.)doc";

static const char *__doc_mcap_IChunkWriter_compressedSize =
R"doc(Returns the size in bytes of the compressed data. This will only be called after
`end()`.)doc";

static const char *__doc_mcap_IChunkWriter_data = R"doc(Returns a pointer to the uncompressed data.)doc";

static const char *__doc_mcap_IChunkWriter_empty =
R"doc(Returns true if `write()` has never been called since initialization or the last
call to `clear()`.)doc";

static const char *__doc_mcap_IChunkWriter_end =
R"doc(Called when the writer wants to close the current output Chunk. After this call,
`data()` and `size()` should return the data and size of the compressed data.)doc";

static const char *__doc_mcap_IChunkWriter_handleClear = R"doc()doc";

static const char *__doc_mcap_IChunkWriter_size = R"doc(Returns the size in bytes of the uncompressed data.)doc";

static const char *__doc_mcap_ICompressedReader = R"doc(An abstract interface for compressed readers.)doc";

static const char *__doc_mcap_ICompressedReader_reset =
R"doc(Reset the reader state, clearing any internal buffers and state, and initialize
with new compressed data.

Parameter ``data``:
    Compressed data to read from.

Parameter ``size``:
    Size of the compressed data in bytes.

Parameter ``uncompressedSize``:
    Size of the data in bytes after decompression. A buffer of this size will be
    allocated for the uncompressed data.)doc";

static const char *__doc_mcap_ICompressedReader_status =
R"doc(Report the current status of decompression. A StatusCode other than
`StatusCode::Success` after `reset()` is called indicates the decompression was
not successful and the reader is in an invalid state.)doc";

static const char *__doc_mcap_IReadable = R"doc(An abstract interface for reading MCAP data.)doc";

static const char *__doc_mcap_IReadable_read =
R"doc(This method is called by MCAP reader classes when they need to read a portion of
the file.

Parameter ``output``:
    A pointer to a pointer to the buffer to write to. This method is expected to
    either maintain an internal buffer, read data into it, and update this
    pointer to point at the internal buffer, or update this pointer to point
    directly at the source data if possible. The pointer and data must remain
    valid and unmodified until the next call to read().

Parameter ``offset``:
    The offset in bytes from the beginning of the file to read.

Parameter ``size``:
    The number of bytes to read.

Returns:
    uint64_t Number of bytes actually read. This may be less than the requested
    size if the end of the file is reached. The output pointer must be readable
    from `output` to `output + size`. If the read fails, this method should
    return 0.)doc";

static const char *__doc_mcap_IReadable_size =
R"doc(Returns the size of the file in bytes.

Returns:
    uint64_t The total number of bytes in the MCAP file.)doc";

static const char *__doc_mcap_IWritable = R"doc(An abstract interface for writing MCAP data.)doc";

static const char *__doc_mcap_IWritable_IWritable = R"doc()doc";

static const char *__doc_mcap_IWritable_crc = R"doc(Returns the CRC32 of the uncompressed data.)doc";

static const char *__doc_mcap_IWritable_crc_2 = R"doc()doc";

static const char *__doc_mcap_IWritable_crcEnabled = R"doc()doc";

static const char *__doc_mcap_IWritable_end = R"doc(Called when the writer is finished writing data to the output MCAP file.)doc";

static const char *__doc_mcap_IWritable_handleWrite = R"doc()doc";

static const char *__doc_mcap_IWritable_resetCrc = R"doc(Resets the CRC32 calculation.)doc";

static const char *__doc_mcap_IWritable_size =
R"doc(Returns the current size of the file in bytes. This must be equal to the sum of
all `size` parameters passed to `write()`.)doc";

static const char *__doc_mcap_IWritable_write =
R"doc(Called whenever the writer needs to write data to the output MCAP file.

Parameter ``data``:
    A pointer to the data to write.

Parameter ``size``:
    Size of the data in bytes.)doc";

static const char *__doc_mcap_IndexedMessageReader =
R"doc(Uses message indices to read messages out of an MCAP in log time order. The
underlying MCAP must be chunked, with a summary section and message indexes. The
required McapWriterOptions are: - noChunking: false - noMessageIndex: false -
noSummary: false)doc";

static const char *__doc_mcap_IndexedMessageReader_ChunkSlot = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_ChunkSlot_chunkStartOffset = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_ChunkSlot_decompressedChunk = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_ChunkSlot_unreadMessages = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_IndexedMessageReader = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_chunkSlots = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_decompressChunk = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_findFreeChunkSlot = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_lz4Reader = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_mcapReader = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_next =
R"doc(reads the next message out of the MCAP.

Returns:
    true if a message was found.

Returns:
    false if no more messages are to be read. If there was some error reading
    the MCAP, `status()` will return a non-Success status.)doc";

static const char *__doc_mcap_IndexedMessageReader_onMessage = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_options = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_queue = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_recordReader = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_selectedChannels = R"doc()doc";

static const char *__doc_mcap_IndexedMessageReader_status =
R"doc(gets the status of the reader.

Returns:
    Status)doc";

static const char *__doc_mcap_IndexedMessageReader_status_2 = R"doc()doc";

static const char *__doc_mcap_LZ4Reader =
R"doc(ICompressedReader implementation that decompresses LZ4
(https://lz4.github.io/lz4/) data.)doc";

static const char *__doc_mcap_LZ4Reader_LZ4Reader = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_LZ4Reader_2 = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_LZ4Reader_3 = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_compressedData = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_compressedSize = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_decompressAll =
R"doc(Decompresses an entire LZ4-encoded chunk into `output`.

Parameter ``data``:
    The LZ4-compressed input chunk.

Parameter ``size``:
    The size of the LZ4-compressed input.

Parameter ``uncompressedSize``:
    The size of the data once uncompressed.

Parameter ``output``:
    The output vector. This will be resized to `uncompressedSize` to fit the
    data, or 0 if the decompression encountered an error.

Returns:
    Status)doc";

static const char *__doc_mcap_LZ4Reader_decompressionContext = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_operator_assign = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_read = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_reset = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_size = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_status = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_status_2 = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_uncompressedData = R"doc()doc";

static const char *__doc_mcap_LZ4Reader_uncompressedSize = R"doc()doc";

static const char *__doc_mcap_LZ4Writer =
R"doc(An in-memory IChunkWriter implementation that holds data in a temporary buffer
before flushing to an LZ4-compressed buffer.)doc";

static const char *__doc_mcap_LZ4Writer_LZ4Writer = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_compressedBuffer = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_compressedData = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_compressedSize = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_compressionLevel = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_data = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_empty = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_end = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_handleClear = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_handleWrite = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_size = R"doc()doc";

static const char *__doc_mcap_LZ4Writer_uncompressedBuffer = R"doc()doc";

static const char *__doc_mcap_LinearMessageView = R"doc(An iterable view of Messages in an MCAP file.)doc";

static const char *__doc_mcap_LinearMessageView_2 = R"doc(An iterable view of Messages in an MCAP file.)doc";

static const char *__doc_mcap_LinearMessageView_Iterator = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_Impl = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_Impl_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_Impl_3 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_curMessage = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_curMessageView = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_dereference = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_has_value = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_increment = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_indexedMessageReader = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_onMessage = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_operator_assign = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_recordReader = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Impl_view = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Iterator = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_Iterator_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_begun = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_impl = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_operator_inc = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_operator_inc_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_operator_mul = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_Iterator_operator_sub = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_LinearMessageView = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_LinearMessageView_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_LinearMessageView_3 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_LinearMessageView_4 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_LinearMessageView_5 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_begin = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_dataEnd = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_dataStart = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_end = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_mcapReader = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_onProblem = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_operator_assign = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_LinearMessageView_readMessageOptions = R"doc()doc";

static const char *__doc_mcap_McapReader = R"doc(Provides a read interface to an MCAP file.)doc";

static const char *__doc_mcap_McapReader_ParseAttachment = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseAttachmentIndex = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseChannel = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseChunk = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseChunkIndex = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseCompression = R"doc(Converts a compression string ("", "zstd", "lz4") to the Compression enum.)doc";

static const char *__doc_mcap_McapReader_ParseDataEnd = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseFooter = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseHeader = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseMessage = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseMessageIndex = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseMetadata = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseMetadataIndex = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseSchema = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseStatistics = R"doc()doc";

static const char *__doc_mcap_McapReader_ParseSummaryOffset = R"doc()doc";

static const char *__doc_mcap_McapReader_ReadFooter = R"doc()doc";

static const char *__doc_mcap_McapReader_ReadRecord = R"doc()doc";

static const char *__doc_mcap_McapReader_attachmentIndexes = R"doc()doc";

static const char *__doc_mcap_McapReader_byteRange =
R"doc(Returns starting and ending byte offsets that must be read to iterate all
messages in the given time range. If `readSummary()` has been successfully
called and the recording contains Chunk records, this range will be narrowed to
Chunk records that contain messages in the given time range. Otherwise, this
range will be the entire Data section if the Data End record has been found or
the entire file otherwise.

This method is automatically used by `readMessages()`, and only needs to be
called directly if the caller is manually constructing an iterator.

Parameter ``startTime``:
    Start time in nanoseconds.

Parameter ``endTime``:
    Optional end time in nanoseconds.

Returns:
    Start and end byte offsets.)doc";

static const char *__doc_mcap_McapReader_channel =
R"doc(Look up a Channel record by channel ID. If the Channel has not been encountered
yet or does not exist in the file, this will return nullptr.

Parameter ``channelId``:
    Channel ID to search for

Returns:
    ChannelPtr A shared pointer to a Channel record, or nullptr)doc";

static const char *__doc_mcap_McapReader_channels =
R"doc(Returns all of the parsed Channel records. Call `readSummary()` first to fully
populate this data structure.)doc";

static const char *__doc_mcap_McapReader_channels_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_chunkIndexes =
R"doc(Returns all of the parsed ChunkIndex records. Call `readSummary()` first to
fully populate this data structure.)doc";

static const char *__doc_mcap_McapReader_chunkIndexes_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_chunkRanges = R"doc()doc";

static const char *__doc_mcap_McapReader_close =
R"doc(Closes the MCAP file, clearing any internal data structures and state and
dropping the data source reference.)doc";

static const char *__doc_mcap_McapReader_dataEnd = R"doc()doc";

static const char *__doc_mcap_McapReader_dataSource =
R"doc(Returns a pointer to the IReadable data source backing this reader. Will return
nullptr if the reader is not open.)doc";

static const char *__doc_mcap_McapReader_dataStart = R"doc()doc";

static const char *__doc_mcap_McapReader_endTime = R"doc()doc";

static const char *__doc_mcap_McapReader_file = R"doc()doc";

static const char *__doc_mcap_McapReader_fileInput = R"doc()doc";

static const char *__doc_mcap_McapReader_fileStreamInput = R"doc()doc";

static const char *__doc_mcap_McapReader_footer = R"doc(Returns the parsed Footer record, if it has been encountered.)doc";

static const char *__doc_mcap_McapReader_footer_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_header = R"doc(Returns the parsed Header record, if it has been encountered.)doc";

static const char *__doc_mcap_McapReader_header_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_input = R"doc()doc";

static const char *__doc_mcap_McapReader_metadataIndexes =
R"doc(Returns all of the parsed MetadataIndex records. Call `readSummary()` first to
fully populate this data structure. The multimap's keys are the `name` field
from each indexed Metadata.)doc";

static const char *__doc_mcap_McapReader_metadataIndexes_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_open =
R"doc(Opens an MCAP file for reading from an already constructed IReadable
implementation.

Parameter ``reader``:
    An implementation of the IReader interface that provides raw MCAP data.

Returns:
    Status StatusCode::Success on success. If a non-success Status is returned,
    the data source is not considered open and McapReader is not usable until
    `open()` is called and a success response is returned.)doc";

static const char *__doc_mcap_McapReader_open_2 =
R"doc(Opens an MCAP file for reading from a given filename.

Parameter ``filename``:
    Filename to open.

Returns:
    Status StatusCode::Success on success. If a non-success Status is returned,
    the data source is not considered open and McapReader is not usable until
    `open()` is called and a success response is returned.)doc";

static const char *__doc_mcap_McapReader_open_3 =
R"doc(Opens an MCAP file for reading from a std::ifstream input file stream.

Parameter ``stream``:
    Input file stream to read MCAP data from.

Returns:
    Status StatusCode::Success on success. If a non-success Status is returned,
    the file is not considered open and McapReader is not usable until `open()`
    is called and a success response is returned.)doc";

static const char *__doc_mcap_McapReader_parsedSummary = R"doc()doc";

static const char *__doc_mcap_McapReader_readMessages =
R"doc(Returns an iterable view with `begin()` and `end()` methods for iterating
Messages in the MCAP file. If a non-zero `startTime` is provided, this will
first parse the Summary section (by calling `readSummary()`) if allowed by the
configuration options and it has not been parsed yet.

Parameter ``startTime``:
    Optional start time in nanoseconds. Messages before this time will not be
    returned.

Parameter ``endTime``:
    Optional end time in nanoseconds. Messages equal to or after this time will
    not be returned.)doc";

static const char *__doc_mcap_McapReader_readMessages_2 =
R"doc(Returns an iterable view with `begin()` and `end()` methods for iterating
Messages in the MCAP file. If a non-zero `startTime` is provided, this will
first parse the Summary section (by calling `readSummary()`) if allowed by the
configuration options and it has not been parsed yet.

Parameter ``onProblem``:
    A callback that will be called when a parsing error occurs. Problems can
    either be recoverable, indicating some data could not be read, or non-
    recoverable, stopping the iteration.

Parameter ``startTime``:
    Optional start time in nanoseconds. Messages before this time will not be
    returned.

Parameter ``endTime``:
    Optional end time in nanoseconds. Messages equal to or after this time will
    not be returned.)doc";

static const char *__doc_mcap_McapReader_readMessages_3 =
R"doc(Returns an iterable view with `begin()` and `end()` methods for iterating
Messages in the MCAP file. Uses the options from `options` to select the
messages that are yielded.)doc";

static const char *__doc_mcap_McapReader_readSummary =
R"doc(Read and parse the Summary section at the end of the MCAP file, if available.
This will populate internal indexes to allow for efficient summarization and
random access. This method will automatically be called upon requesting summary
data or first seek if Summary section parsing is allowed by the configuration
options.)doc";

static const char *__doc_mcap_McapReader_readSummaryFromScan = R"doc()doc";

static const char *__doc_mcap_McapReader_readSummarySection = R"doc()doc";

static const char *__doc_mcap_McapReader_reset = R"doc()doc";

static const char *__doc_mcap_McapReader_schema =
R"doc(Look up a Schema record by schema ID. If the Schema has not been encountered yet
or does not exist in the file, this will return nullptr.

Parameter ``schemaId``:
    Schema ID to search for

Returns:
    SchemaPtr A shared pointer to a Schema record, or nullptr)doc";

static const char *__doc_mcap_McapReader_schemas =
R"doc(Returns all of the parsed Schema records. Call `readSummary()` first to fully
populate this data structure.)doc";

static const char *__doc_mcap_McapReader_schemas_2 = R"doc()doc";

static const char *__doc_mcap_McapReader_startTime = R"doc()doc";

static const char *__doc_mcap_McapReader_statistics = R"doc(Returns the parsed Statistics record, if it has been encountered.)doc";

static const char *__doc_mcap_McapReader_statistics_2 = R"doc()doc";

static const char *__doc_mcap_McapWriter = R"doc(Provides a write interface to an MCAP file.)doc";

static const char *__doc_mcap_McapWriterOptions = R"doc(Configuration options for McapWriter.)doc";

static const char *__doc_mcap_McapWriterOptions_McapWriterOptions = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_chunkSize =
R"doc(Target uncompressed Chunk payload size in bytes. Once a Chunk's uncompressed
data meets or exceeds this size, the Chunk will be compressed (if compression is
enabled) and written to disk. Note that smaller Chunks may be written, such as
the last Chunk in the Data section. This option is ignored if `noChunking=true`.)doc";

static const char *__doc_mcap_McapWriterOptions_compression =
R"doc(Compression algorithm to use when writing Chunks. This option is ignored if
`noChunking=true`.)doc";

static const char *__doc_mcap_McapWriterOptions_compressionLevel =
R"doc(Compression level to use when writing Chunks. Slower generally produces smaller
files, at the expense of more CPU time. These levels map to different internal
settings for each compression algorithm.)doc";

static const char *__doc_mcap_McapWriterOptions_enableDataCRC = R"doc(Enable CRC calculations for all records in the data section.)doc";

static const char *__doc_mcap_McapWriterOptions_forceCompression =
R"doc(By default, Chunks that do not benefit from compression will be written
uncompressed. This option can be used to force compression on all Chunks. This
option is ignored if `noChunking=true`.)doc";

static const char *__doc_mcap_McapWriterOptions_library =
R"doc(A freeform string written by recording libraries. For this library, the default
is "libmcap {Major}.{Minor}.{Patch}".)doc";

static const char *__doc_mcap_McapWriterOptions_noAttachmentCRC = R"doc(Disable CRC calculations for Attachments.)doc";

static const char *__doc_mcap_McapWriterOptions_noAttachmentIndex = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noChunkCRC = R"doc(Disable CRC calculations for Chunks.)doc";

static const char *__doc_mcap_McapWriterOptions_noChunkIndex = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noChunking =
R"doc(Do not write Chunks to the file, instead writing Schema, Channel, and Message
records directly into the Data section.)doc";

static const char *__doc_mcap_McapWriterOptions_noMessageIndex =
R"doc(Do not write Message Index records to the file. If `noSummary=true` and
`noChunkIndex=false`, Chunk Index records will still be written to the Summary
section, providing a coarse message index.)doc";

static const char *__doc_mcap_McapWriterOptions_noMetadataIndex = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noRepeatedChannels = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noRepeatedSchemas = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noStatistics = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_noSummary =
R"doc(Do not write Summary or Summary Offset sections to the file, placing the Footer
record immediately after DataEnd. This can provide some speed boost to file
writing and produce smaller files, at the expense of requiring a conversion
process later if fast summarization or indexed access is desired.)doc";

static const char *__doc_mcap_McapWriterOptions_noSummaryCRC = R"doc(Disable CRC calculations for the summary section.)doc";

static const char *__doc_mcap_McapWriterOptions_noSummaryOffsets = R"doc()doc";

static const char *__doc_mcap_McapWriterOptions_profile =
R"doc(The recording profile. See https://mcap.dev/spec/registry#well-known-profiles
for more information on well-known profiles.)doc";

static const char *__doc_mcap_McapWriter_addChannel =
R"doc(Add a new channel to the MCAP file and set `channel.id` to a generated channel
id. The channel id is used when adding messages to the file.

Parameter ``channel``:
    Description of the channel to register. The `id` value is ignored and will
    be set to a generated channel id.)doc";

static const char *__doc_mcap_McapWriter_addSchema =
R"doc(Add a new schema to the MCAP file and set `schema.id` to a generated schema id.
The schema id is used when adding channels to the file.

Parameter ``schema``:
    Description of the schema to register. The `id` field is ignored and will be
    set to a generated schema id.)doc";

static const char *__doc_mcap_McapWriter_attachmentIndex = R"doc()doc";

static const char *__doc_mcap_McapWriter_channels = R"doc()doc";

static const char *__doc_mcap_McapWriter_chunkIndex = R"doc()doc";

static const char *__doc_mcap_McapWriter_chunkSize = R"doc()doc";

static const char *__doc_mcap_McapWriter_close =
R"doc(Write the MCAP footer, flush pending writes to the output stream, and reset
internal state.)doc";

static const char *__doc_mcap_McapWriter_closeLastChunk =
R"doc(finishes the current chunk in progress and writes it to the file, if a chunk is
in progress.)doc";

static const char *__doc_mcap_McapWriter_compression = R"doc()doc";

static const char *__doc_mcap_McapWriter_currentChunkEnd = R"doc()doc";

static const char *__doc_mcap_McapWriter_currentChunkStart = R"doc()doc";

static const char *__doc_mcap_McapWriter_currentMessageIndex = R"doc()doc";

static const char *__doc_mcap_McapWriter_dataSink =
R"doc(Returns a pointer to the IWritable data destination backing this writer. Will
return nullptr if the writer is not open.)doc";

static const char *__doc_mcap_McapWriter_fileOutput = R"doc()doc";

static const char *__doc_mcap_McapWriter_getChunkWriter = R"doc()doc";

static const char *__doc_mcap_McapWriter_getOutput = R"doc()doc";

static const char *__doc_mcap_McapWriter_lz4Chunk = R"doc()doc";

static const char *__doc_mcap_McapWriter_metadataIndex = R"doc()doc";

static const char *__doc_mcap_McapWriter_open =
R"doc(Open a new MCAP file for writing and write the header.

Parameter ``filename``:
    Filename of the MCAP file to write.

Parameter ``options``:
    Options for MCAP writing. `profile` is required.

Returns:
    A non-success status if the file could not be opened for writing.)doc";

static const char *__doc_mcap_McapWriter_open_2 =
R"doc(Open a new MCAP file for writing and write the header.

Parameter ``writer``:
    An implementation of the IWritable interface. Output bytes will be written
    to this object.

Parameter ``options``:
    Options for MCAP writing. `profile` is required.)doc";

static const char *__doc_mcap_McapWriter_open_3 =
R"doc(Open a new MCAP file for writing and write the header.

Parameter ``stream``:
    Output stream to write to.

Parameter ``options``:
    Options for MCAP writing. `profile` is required.)doc";

static const char *__doc_mcap_McapWriter_opened = R"doc()doc";

static const char *__doc_mcap_McapWriter_options = R"doc()doc";

static const char *__doc_mcap_McapWriter_output = R"doc()doc";

static const char *__doc_mcap_McapWriter_schemas = R"doc()doc";

static const char *__doc_mcap_McapWriter_statistics =
R"doc(Current MCAP file-level statistics. This is written as a Statistics record in
the Summary section of the MCAP file.)doc";

static const char *__doc_mcap_McapWriter_statistics_2 = R"doc()doc";

static const char *__doc_mcap_McapWriter_streamOutput = R"doc()doc";

static const char *__doc_mcap_McapWriter_terminate =
R"doc(Reset internal state without writing the MCAP footer or flushing pending writes.
This should only be used in error cases as the output MCAP file will be
truncated.)doc";

static const char *__doc_mcap_McapWriter_uncompressedChunk = R"doc()doc";

static const char *__doc_mcap_McapWriter_uncompressedSize = R"doc()doc";

static const char *__doc_mcap_McapWriter_write =
R"doc(Write a message to the output stream.

Parameter ``msg``:
    Message to add.

Returns:
    A non-zero error code on failure.)doc";

static const char *__doc_mcap_McapWriter_write_2 =
R"doc(Write an attachment to the output stream.

Parameter ``attachment``:
    Attachment to add. The `attachment.crc` will be calculated and set if
    configuration options allow CRC calculation.

Returns:
    A non-zero error code on failure.)doc";

static const char *__doc_mcap_McapWriter_write_3 =
R"doc(Write a metadata record to the output stream.

Parameter ``metadata``:
    Named group of key/value string pairs to add.

Returns:
    A non-zero error code on failure.)doc";

static const char *__doc_mcap_McapWriter_write_4 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_5 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_6 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_7 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_8 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_9 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_10 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_11 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_12 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_13 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_14 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_15 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_16 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_17 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_18 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_19 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_20 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_21 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_22 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_23 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_24 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_25 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_26 = R"doc()doc";

static const char *__doc_mcap_McapWriter_write_27 = R"doc()doc";

static const char *__doc_mcap_McapWriter_writeChunk = R"doc()doc";

static const char *__doc_mcap_McapWriter_writeMagic = R"doc()doc";

static const char *__doc_mcap_McapWriter_writtenSchemas = R"doc()doc";

static const char *__doc_mcap_McapWriter_zstdChunk = R"doc()doc";

static const char *__doc_mcap_Message = R"doc(A single Message published to a Channel.)doc";

static const char *__doc_mcap_MessageIndex =
R"doc(A list of timestamps to byte offsets for a single Channel. This record appears
after each Chunk, one per Channel that appeared in that Chunk.)doc";

static const char *__doc_mcap_MessageIndex_channelId = R"doc()doc";

static const char *__doc_mcap_MessageIndex_records = R"doc()doc";

static const char *__doc_mcap_MessageView =
R"doc(Returned when iterating over Messages in a file, MessageView contains a
reference to one Message, a pointer to its Channel, and an optional pointer to
that Channel's Schema. The Channel pointer is guaranteed to be valid, while the
Schema pointer may be null if the Channel references schema_id 0.)doc";

static const char *__doc_mcap_MessageView_MessageView = R"doc()doc";

static const char *__doc_mcap_MessageView_channel = R"doc()doc";

static const char *__doc_mcap_MessageView_message = R"doc()doc";

static const char *__doc_mcap_MessageView_messageOffset = R"doc()doc";

static const char *__doc_mcap_MessageView_schema = R"doc()doc";

static const char *__doc_mcap_Message_channelId = R"doc()doc";

static const char *__doc_mcap_Message_data =
R"doc(A pointer to the message payload. For readers, this pointer is only valid for
the lifetime of an onMessage callback or before the message iterator is
advanced.)doc";

static const char *__doc_mcap_Message_dataSize = R"doc(Size of the message payload in bytes, pointed to via `data`.)doc";

static const char *__doc_mcap_Message_logTime = R"doc(Nanosecond timestamp when this message was recorded or received for recording.)doc";

static const char *__doc_mcap_Message_publishTime =
R"doc(Nanosecond timestamp when this message was initially published. If not
available, this should be set to `logTime`.)doc";

static const char *__doc_mcap_Message_sequence =
R"doc(An optional sequence number. If non-zero, sequence numbers should be unique per
channel and increasing over time.)doc";

static const char *__doc_mcap_Metadata =
R"doc(Holds a named map of key/value strings containing arbitrary user data. Metadata
records are found in the Data section, outside of Chunks.)doc";

static const char *__doc_mcap_MetadataIndex =
R"doc(Metadata Index records are found in the Summary section, providing summary
information for a single Metadata record.)doc";

static const char *__doc_mcap_MetadataIndex_MetadataIndex = R"doc()doc";

static const char *__doc_mcap_MetadataIndex_MetadataIndex_2 = R"doc()doc";

static const char *__doc_mcap_MetadataIndex_length = R"doc()doc";

static const char *__doc_mcap_MetadataIndex_name = R"doc()doc";

static const char *__doc_mcap_MetadataIndex_offset = R"doc()doc";

static const char *__doc_mcap_Metadata_metadata = R"doc()doc";

static const char *__doc_mcap_Metadata_name = R"doc()doc";

static const char *__doc_mcap_OpCode = R"doc(MCAP record types.)doc";

static const char *__doc_mcap_OpCodeString = R"doc(Get the string representation of an OpCode.)doc";

static const char *__doc_mcap_OpCode_Attachment = R"doc()doc";

static const char *__doc_mcap_OpCode_AttachmentIndex = R"doc()doc";

static const char *__doc_mcap_OpCode_Channel = R"doc()doc";

static const char *__doc_mcap_OpCode_Chunk = R"doc()doc";

static const char *__doc_mcap_OpCode_ChunkIndex = R"doc()doc";

static const char *__doc_mcap_OpCode_DataEnd = R"doc()doc";

static const char *__doc_mcap_OpCode_Footer = R"doc()doc";

static const char *__doc_mcap_OpCode_Header = R"doc()doc";

static const char *__doc_mcap_OpCode_Message = R"doc()doc";

static const char *__doc_mcap_OpCode_MessageIndex = R"doc()doc";

static const char *__doc_mcap_OpCode_Metadata = R"doc()doc";

static const char *__doc_mcap_OpCode_MetadataIndex = R"doc()doc";

static const char *__doc_mcap_OpCode_Schema = R"doc()doc";

static const char *__doc_mcap_OpCode_Statistics = R"doc()doc";

static const char *__doc_mcap_OpCode_SummaryOffset = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions = R"doc(Options for reading messages out of an MCAP file.)doc";

static const char *__doc_mcap_ReadMessageOptions_ReadMessageOptions = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_ReadMessageOptions_2 = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_ReadOrder = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_ReadOrder_FileOrder = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_ReadOrder_LogTimeOrder = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_ReadOrder_ReverseLogTimeOrder = R"doc()doc";

static const char *__doc_mcap_ReadMessageOptions_endTime = R"doc(Only messages with log timestamps less than endTime will be included.)doc";

static const char *__doc_mcap_ReadMessageOptions_readOrder =
R"doc(Set the expected order that messages should be returned in. if readOrder ==
FileOrder, messages will be returned in the order they appear in the MCAP file.
if readOrder == LogTimeOrder, messages will be returned in ascending log time
order. if readOrder == ReverseLogTimeOrder, messages will be returned in
descending log time order.)doc";

static const char *__doc_mcap_ReadMessageOptions_startTime =
R"doc(Only messages with log timestamps greater or equal to startTime will be
included.)doc";

static const char *__doc_mcap_ReadMessageOptions_topicFilter =
R"doc(If provided, `topicFilter` is called on all topics found in the MCAP file. If
`topicFilter` returns true for a given channel, messages from that channel will
be included. if not provided, messages from all channels are provided.)doc";

static const char *__doc_mcap_ReadMessageOptions_validate = R"doc(validate the configuration.)doc";

static const char *__doc_mcap_ReadSummaryMethod = R"doc()doc";

static const char *__doc_mcap_ReadSummaryMethod_AllowFallbackScan =
R"doc(If the Summary section is missing or incomplete, allow falling back to reading
the file sequentially to produce seeking indexes and summary statistics.)doc";

static const char *__doc_mcap_ReadSummaryMethod_ForceScan =
R"doc(Read the file sequentially from Header to DataEnd to produce seeking indexes and
summary statistics.)doc";

static const char *__doc_mcap_ReadSummaryMethod_NoFallbackScan =
R"doc(Parse the Summary section to produce seeking indexes and summary statistics. If
the Summary section is not present or corrupt, a failure Status is returned and
the seeking indexes and summary statistics are not populated.)doc";

static const char *__doc_mcap_Record =
R"doc(A generic Type-Length-Value record using a uint8 type and uint64 length. This is
the generic form of all MCAP records.)doc";

static const char *__doc_mcap_RecordOffset = R"doc()doc";

static const char *__doc_mcap_RecordOffset_RecordOffset = R"doc()doc";

static const char *__doc_mcap_RecordOffset_RecordOffset_2 = R"doc()doc";

static const char *__doc_mcap_RecordOffset_RecordOffset_3 = R"doc()doc";

static const char *__doc_mcap_RecordOffset_chunkOffset = R"doc()doc";

static const char *__doc_mcap_RecordOffset_offset = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_eq = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_ge = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_gt = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_le = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_lt = R"doc()doc";

static const char *__doc_mcap_RecordOffset_operator_ne = R"doc()doc";

static const char *__doc_mcap_RecordReader = R"doc(A low-level interface for parsing MCAP-style TLV records from a data source.)doc";

static const char *__doc_mcap_RecordReader_RecordReader = R"doc()doc";

static const char *__doc_mcap_RecordReader_curRecord = R"doc()doc";

static const char *__doc_mcap_RecordReader_curRecordOffset = R"doc()doc";

static const char *__doc_mcap_RecordReader_dataSource = R"doc()doc";

static const char *__doc_mcap_RecordReader_endOffset = R"doc()doc";

static const char *__doc_mcap_RecordReader_next = R"doc()doc";

static const char *__doc_mcap_RecordReader_offset = R"doc()doc";

static const char *__doc_mcap_RecordReader_reset = R"doc()doc";

static const char *__doc_mcap_RecordReader_status = R"doc()doc";

static const char *__doc_mcap_RecordReader_status_2 = R"doc()doc";

static const char *__doc_mcap_Record_data = R"doc()doc";

static const char *__doc_mcap_Record_dataSize = R"doc()doc";

static const char *__doc_mcap_Record_opcode = R"doc()doc";

static const char *__doc_mcap_Record_recordSize = R"doc()doc";

static const char *__doc_mcap_Schema =
R"doc(Describes a schema used for message encoding and decoding and/or describing the
shape of messages. One or more Channel records map to a single Schema.)doc";

static const char *__doc_mcap_Schema_Schema = R"doc()doc";

static const char *__doc_mcap_Schema_Schema_2 = R"doc()doc";

static const char *__doc_mcap_Schema_Schema_3 = R"doc()doc";

static const char *__doc_mcap_Schema_data = R"doc()doc";

static const char *__doc_mcap_Schema_encoding = R"doc()doc";

static const char *__doc_mcap_Schema_id = R"doc()doc";

static const char *__doc_mcap_Schema_name = R"doc()doc";

static const char *__doc_mcap_Statistics =
R"doc(The Statistics record is found in the Summary section, providing counts and
timestamp ranges for the entire file.)doc";

static const char *__doc_mcap_Statistics_attachmentCount = R"doc()doc";

static const char *__doc_mcap_Statistics_channelCount = R"doc()doc";

static const char *__doc_mcap_Statistics_channelMessageCounts = R"doc()doc";

static const char *__doc_mcap_Statistics_chunkCount = R"doc()doc";

static const char *__doc_mcap_Statistics_messageCount = R"doc()doc";

static const char *__doc_mcap_Statistics_messageEndTime = R"doc()doc";

static const char *__doc_mcap_Statistics_messageStartTime = R"doc()doc";

static const char *__doc_mcap_Statistics_metadataCount = R"doc()doc";

static const char *__doc_mcap_Statistics_schemaCount = R"doc()doc";

static const char *__doc_mcap_Status = R"doc(Wraps a status code and string message carrying additional context.)doc";

static const char *__doc_mcap_StatusCode = R"doc(Status codes for MCAP readers and writers.)doc";

static const char *__doc_mcap_StatusCode_DecompressionFailed = R"doc()doc";

static const char *__doc_mcap_StatusCode_DecompressionSizeMismatch = R"doc()doc";

static const char *__doc_mcap_StatusCode_FileTooSmall = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidChannelId = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidChunkOffset = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidFile = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidFooter = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidMessageReadOptions = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidOpCode = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidRecord = R"doc()doc";

static const char *__doc_mcap_StatusCode_InvalidSchemaId = R"doc()doc";

static const char *__doc_mcap_StatusCode_MagicMismatch = R"doc()doc";

static const char *__doc_mcap_StatusCode_MissingStatistics = R"doc()doc";

static const char *__doc_mcap_StatusCode_NoMessageIndexesAvailable = R"doc()doc";

static const char *__doc_mcap_StatusCode_NotOpen = R"doc()doc";

static const char *__doc_mcap_StatusCode_OpenFailed = R"doc()doc";

static const char *__doc_mcap_StatusCode_ReadFailed = R"doc()doc";

static const char *__doc_mcap_StatusCode_Success = R"doc()doc";

static const char *__doc_mcap_StatusCode_UnrecognizedCompression = R"doc()doc";

static const char *__doc_mcap_StatusCode_UnsupportedCompression = R"doc()doc";

static const char *__doc_mcap_Status_Status = R"doc()doc";

static const char *__doc_mcap_Status_Status_2 = R"doc()doc";

static const char *__doc_mcap_Status_Status_3 = R"doc()doc";

static const char *__doc_mcap_Status_code = R"doc()doc";

static const char *__doc_mcap_Status_message = R"doc()doc";

static const char *__doc_mcap_Status_ok = R"doc()doc";

static const char *__doc_mcap_StreamWriter =
R"doc(Implements the IWritable interface used by McapWriter by wrapping a std::ostream
stream.)doc";

static const char *__doc_mcap_StreamWriter_StreamWriter = R"doc()doc";

static const char *__doc_mcap_StreamWriter_end = R"doc()doc";

static const char *__doc_mcap_StreamWriter_handleWrite = R"doc()doc";

static const char *__doc_mcap_StreamWriter_size = R"doc()doc";

static const char *__doc_mcap_StreamWriter_size_2 = R"doc()doc";

static const char *__doc_mcap_StreamWriter_stream = R"doc()doc";

static const char *__doc_mcap_SummaryOffset =
R"doc(Summary Offset records are found in the Summary Offset section. Records in the
Summary section are grouped together, and for each record type found in the
Summary section, a Summary Offset references the file offset and length where
that type of Summary record can be found.)doc";

static const char *__doc_mcap_SummaryOffset_groupLength = R"doc()doc";

static const char *__doc_mcap_SummaryOffset_groupOpCode = R"doc()doc";

static const char *__doc_mcap_SummaryOffset_groupStart = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_TypedChunkReader = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_TypedChunkReader_2 = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_TypedChunkReader_3 = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_lz4Reader = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_next = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_offset = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_onChannel = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_onMessage = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_onSchema = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_onUnknownRecord = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_operator_assign = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_reader = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_reset = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_status = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_status_2 = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_uncompressedReader = R"doc()doc";

static const char *__doc_mcap_TypedChunkReader_zstdReader = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader =
R"doc(A mid-level interface for parsing and validating MCAP records from a data
source.)doc";

static const char *__doc_mcap_TypedRecordReader_TypedRecordReader = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_TypedRecordReader_2 = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_TypedRecordReader_3 = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_chunkReader = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_next = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_offset = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onAttachment = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onAttachmentIndex = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onChannel = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onChunk = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onChunkEnd = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onChunkIndex = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onDataEnd = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onFooter = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onHeader = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onMessage = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onMessageIndex = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onMetadata = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onMetadataIndex = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onSchema = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onStatistics = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onSummaryOffset = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_onUnknownRecord = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_operator_assign = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_parsingChunk = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_reader = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_status = R"doc()doc";

static const char *__doc_mcap_TypedRecordReader_status_2 = R"doc()doc";

static const char *__doc_mcap_ZStdReader =
R"doc(ICompressedReader implementation that decompresses Zstandard
(https://facebook.github.io/zstd/) data.)doc";

static const char *__doc_mcap_ZStdReader_DecompressAll =
R"doc(Decompresses an entire Zstd-compressed chunk into `output`.

Parameter ``data``:
    The Zstd-compressed input chunk.

Parameter ``compressedSize``:
    The size of the Zstd-compressed input.

Parameter ``uncompressedSize``:
    The size of the data once uncompressed.

Parameter ``output``:
    The output vector. This will be resized to `uncompressedSize` to fit the
    data, or 0 if the decompression encountered an error.

Returns:
    Status)doc";

static const char *__doc_mcap_ZStdReader_ZStdReader = R"doc()doc";

static const char *__doc_mcap_ZStdReader_ZStdReader_2 = R"doc()doc";

static const char *__doc_mcap_ZStdReader_ZStdReader_3 = R"doc()doc";

static const char *__doc_mcap_ZStdReader_operator_assign = R"doc()doc";

static const char *__doc_mcap_ZStdReader_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_ZStdReader_read = R"doc()doc";

static const char *__doc_mcap_ZStdReader_reset = R"doc()doc";

static const char *__doc_mcap_ZStdReader_size = R"doc()doc";

static const char *__doc_mcap_ZStdReader_status = R"doc()doc";

static const char *__doc_mcap_ZStdReader_status_2 = R"doc()doc";

static const char *__doc_mcap_ZStdReader_uncompressedData = R"doc()doc";

static const char *__doc_mcap_ZStdWriter =
R"doc(An in-memory IChunkWriter implementation that holds data in a temporary buffer
before flushing to an ZStandard-compressed buffer.)doc";

static const char *__doc_mcap_ZStdWriter_ZStdWriter = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_compressedBuffer = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_compressedData = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_compressedSize = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_data = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_empty = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_end = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_handleClear = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_handleWrite = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_size = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_uncompressedBuffer = R"doc()doc";

static const char *__doc_mcap_ZStdWriter_zstdContext = R"doc()doc";

static const char *__doc_mcap_internal_CRC32Table =
R"doc(Compute CRC32 lookup tables as described at:
https://github.com/komrad36/CRC#option-6-1-byte-tabular

An iteration of CRC computation can be performed on 8 bits of input at once. By
pre-computing a table of the values of CRC(?) for all 2^8 = 256 possible byte
values, during the final computation we can replace a loop over 8 bits with a
single lookup in the table.

For further speedup, we can also pre-compute the values of CRC(?0) for all
possible bytes when a zero byte is appended. Then we can process two bytes of
input at once by computing CRC(AB) = CRC(A0) ^ CRC(B), using one lookup in the
CRC(?0) table and one lookup in the CRC(?) table.

The same technique applies for any number of bytes to be processed at once,
although the speed improvements diminish.

Parameter ``Polynomial``:
    The binary representation of the polynomial to use (reversed, i.e. most
    significant bit represents x^0).

Parameter ``NumTables``:
    The number of bytes of input that will be processed at once.)doc";

static const char *__doc_mcap_internal_CRC32Table_CRC32Table = R"doc()doc";

static const char *__doc_mcap_internal_CRC32Table_operator_array = R"doc()doc";

static const char *__doc_mcap_internal_CRC32Table_table = R"doc()doc";

static const char *__doc_mcap_internal_CompressionString = R"doc()doc";

static const char *__doc_mcap_internal_DecompressChunkJob =
R"doc(A job to decompress the chunk starting at `chunkStartOffset`. The message
indices starting directly after the chunk record and ending at
`messageIndexEndOffset` will be used to find specific messages within the chunk.)doc";

static const char *__doc_mcap_internal_DecompressChunkJob_chunkStartOffset = R"doc()doc";

static const char *__doc_mcap_internal_DecompressChunkJob_messageEndTime = R"doc()doc";

static const char *__doc_mcap_internal_DecompressChunkJob_messageIndexEndOffset = R"doc()doc";

static const char *__doc_mcap_internal_DecompressChunkJob_messageStartTime = R"doc()doc";

static const char *__doc_mcap_internal_Interval = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalStartCmp = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalStartCmp_operator_call = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalStopCmp = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalStopCmp_operator_call = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalTree = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalTree_2 = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalTree_3 = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_IntervalTree_4 = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_center = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_clone = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_empty = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_extent = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_find_contained = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_find_overlapping = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_intervals = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_is_valid = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_left = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_operator_assign = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_operator_assign_2 = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_right = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_visit_all = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_visit_contained = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_visit_near = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_visit_overlapping = R"doc()doc";

static const char *__doc_mcap_internal_IntervalTree_visit_overlapping_2 = R"doc()doc";

static const char *__doc_mcap_internal_Interval_Interval = R"doc()doc";

static const char *__doc_mcap_internal_Interval_start = R"doc()doc";

static const char *__doc_mcap_internal_Interval_stop = R"doc()doc";

static const char *__doc_mcap_internal_Interval_value = R"doc()doc";

static const char *__doc_mcap_internal_KeyValueMapSize = R"doc()doc";

static const char *__doc_mcap_internal_MagicToHex = R"doc()doc";

static const char *__doc_mcap_internal_ParseByteArray = R"doc()doc";

static const char *__doc_mcap_internal_ParseKeyValueMap = R"doc()doc";

static const char *__doc_mcap_internal_ParseString = R"doc()doc";

static const char *__doc_mcap_internal_ParseStringView = R"doc()doc";

static const char *__doc_mcap_internal_ParseUint16 = R"doc()doc";

static const char *__doc_mcap_internal_ParseUint32 = R"doc()doc";

static const char *__doc_mcap_internal_ParseUint32_2 = R"doc()doc";

static const char *__doc_mcap_internal_ParseUint64 = R"doc()doc";

static const char *__doc_mcap_internal_ParseUint64_2 = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue = R"doc(A priority queue of jobs for an indexed MCAP reader to execute.)doc";

static const char *__doc_mcap_internal_ReadJobQueue_CompareForward = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_CompareReverse = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_PositionComparisonKey = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_ReadJobQueue = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_TimeComparisonKey = R"doc(return the timestamp key that should be used to compare jobs.)doc";

static const char *__doc_mcap_internal_ReadJobQueue_heap = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_len = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_pop = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_push = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_push_2 = R"doc()doc";

static const char *__doc_mcap_internal_ReadJobQueue_reverse = R"doc()doc";

static const char *__doc_mcap_internal_ReadMessageJob =
R"doc(A job to read a specific message at offset `offset` from the decompressed chunk
stored in `chunkReaderIndex`. A timestamp is provided to order this job relative
to other jobs.)doc";

static const char *__doc_mcap_internal_ReadMessageJob_chunkReaderIndex = R"doc()doc";

static const char *__doc_mcap_internal_ReadMessageJob_offset = R"doc()doc";

static const char *__doc_mcap_internal_ReadMessageJob_timestamp = R"doc()doc";

static const char *__doc_mcap_internal_StrCat = R"doc()doc";

static const char *__doc_mcap_internal_ToHex = R"doc()doc";

static const char *__doc_mcap_internal_ToHex_2 = R"doc()doc";

static const char *__doc_mcap_internal_crc32Final = R"doc(Finalize a CRC32 by inverting the output value.)doc";

static const char *__doc_mcap_internal_crc32Update =
R"doc(Update a streaming CRC32 calculation.

For performance, this implementation processes the data 8 bytes at a time, using
the algorithm presented at: https://github.com/komrad36/CRC#option-9-8-byte-
tabular)doc";

static const char *__doc_mcap_internal_getUint32LE = R"doc()doc";

static const char *__doc_mcap_internal_intervalStart = R"doc()doc";

static const char *__doc_mcap_internal_intervalStop = R"doc()doc";

static const char *__doc_mcap_internal_operator_lshift = R"doc()doc";

static const char *__doc_mcap_internal_to_string = R"doc()doc";

static const char *__doc_mcap_internal_to_string_2 = R"doc()doc";

static const char *__doc_mcap_internal_to_string_3 = R"doc()doc";

static const char *__doc_memfd_create = R"doc()doc";

static const char *__doc_nop_Encoding = R"doc()doc";

static const char *__doc_nop_Encoding_2 = R"doc()doc";

static const char *__doc_nop_Encoding_3 = R"doc()doc";

static const char *__doc_nop_Encoding_Match = R"doc()doc";

static const char *__doc_nop_Encoding_Match_2 = R"doc()doc";

static const char *__doc_nop_Encoding_Match_3 = R"doc()doc";

static const char *__doc_nop_Encoding_Prefix = R"doc()doc";

static const char *__doc_nop_Encoding_Prefix_2 = R"doc()doc";

static const char *__doc_nop_Encoding_Prefix_3 = R"doc()doc";

static const char *__doc_nop_Encoding_ReadPayload = R"doc()doc";

static const char *__doc_nop_Encoding_ReadPayload_2 = R"doc()doc";

static const char *__doc_nop_Encoding_ReadPayload_3 = R"doc()doc";

static const char *__doc_nop_Encoding_Size = R"doc()doc";

static const char *__doc_nop_Encoding_Size_2 = R"doc()doc";

static const char *__doc_nop_Encoding_Size_3 = R"doc()doc";

static const char *__doc_nop_Encoding_WritePayload = R"doc()doc";

static const char *__doc_nop_Encoding_WritePayload_2 = R"doc()doc";

static const char *__doc_nop_Encoding_WritePayload_3 = R"doc()doc";

static const char *__doc_operator_lshift = R"doc()doc";

static const char *__doc_operator_lshift_2 = R"doc()doc";

static const char *__doc_operator_lshift_3 = R"doc()doc";

static const char *__doc_operator_lshift_4 = R"doc()doc";

static const char *__doc_operator_lshift_5 = R"doc()doc";

static const char *__doc_operator_lshift_6 = R"doc()doc";

static const char *__doc_operator_lshift_7 = R"doc()doc";

static const char *__doc_operator_lshift_8 = R"doc()doc";

static const char *__doc_operator_lshift_9 = R"doc(Get USB Speed)doc";

static const char *__doc_operator_lshift_10 = R"doc()doc";

static const char *__doc_operator_lshift_11 = R"doc()doc";

static const char *__doc_operator_lshift_12 = R"doc()doc";

static const char *__doc_operator_lshift_13 = R"doc()doc";

static const char *__doc_operator_lshift_14 = R"doc()doc";

static const char *__doc_std_hash = R"doc()doc";

static const char *__doc_std_hash_operator_call = R"doc()doc";

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

