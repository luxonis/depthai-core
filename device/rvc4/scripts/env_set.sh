# Environment of the RVC4 firmware process (depthai-device-rvc4).
# depthai_gate sources this file through custom_scripts.py (on_start) before it spawns the firmware;
# utility_scripts/device_based/*.sh source it for manual runs on the device.

echo "RVC4"
export RVC4=1

# Set ADSP_LIBRARY_PATH
QAIRT_SNPE_VERSION='2.32.6.250402'
if [ -d "/opt/qcom/aistack/qairt/2.41.0.251128" ]; then
    QAIRT_SNPE_VERSION='2.41.0.251128'
fi
export ADSP_LIBRARY_PATH="/opt/qcom/aistack/qairt/${QAIRT_SNPE_VERSION}/lib/hexagon-v73/unsigned/;/etc/models/luxonis;/usr/lib;/dsp;/opt/qcom/aistack/qairt/2.23.0.24.06.24/lib/hexagon-v73/unsigned/"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/qcom/aistack/qairt/${QAIRT_SNPE_VERSION}/lib/aarch64-oe-linux-gcc11.2/"
echo "interceptor_via_fun:evaInitWarp" > /tmp/asan.supp
export ASAN_OPTIONS=alloc_dealloc_mismatch=0:abort_on_error=1:suppressions=/tmp/asan.supp
# the camera HAL module pulls in libclang_rt.ubsan_standalone whose deadly-signal handler
# would swallow SIGSEGV and _exit(1) without a core dump - keep it disarmed
export UBSAN_OPTIONS=handle_segv=0:handle_sigbus=0:handle_sigill=0:handle_sigfpe=0:handle_abort=0:handle_sigtrap=0

# Dynamic calibration library (DynamicCalibration / AutoCalibration nodes) thread policy.
# Every image load is a short burst on all cores; keeping the prime core (cpu5) free costs ~14 % calibration
# throughput and leaves one fast core to the rest of the pipeline. A DynamicCalibration::setThreadPolicy()
# call from the host overrides these defaults.
export DCL_OPENCV_THREADS=5
export DCL_THREAD_AFFINITY=0-4
