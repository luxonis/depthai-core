# for core dump parsing:
source /usr/local/oe-sdk-hardcoded-buildpath/environment-setup-aarch64-oe-linux
$GDB \
  $1 \
-ex "set sysroot /usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux" \
-ex "add-auto-load-safe-path /usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux/usr/lib/libstdc++.so.6.0.29-gdb.py" \
-ex "file /work/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4" \
-ex "core-file $2" \
-ex "bt" \


