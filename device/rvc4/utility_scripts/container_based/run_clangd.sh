sudo apt-get update -y
sudo apt-get install -y socat clangd-18
socat TCP-LISTEN:6010,reuseaddr,fork EXEC:"/usr/bin/clangd-18 --clang-tidy --query-driver=/usr/local/oe-sdk-hardcoded-buildpath/sysroots/x86_64-qtisdk-linux/usr/bin/aarch64-oe-linux/* --compile-commands-dir=/work/build_docker_arm64_rvc4"
