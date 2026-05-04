#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"
#include "subprocess.hpp"

int main(int argc, char* argv[]) {
    if(argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <timeout> <script> [args...]" << std::endl;
        return 1;
    }

    // Parse timeout from first argument
    int timeout = 30;  // Default fallback
    try {
        timeout = std::stoi(argv[1]);
        if(timeout <= 0) {
            std::cout << "Timeout must be a positive integer, using default (30s)" << std::endl;
            timeout = 30;
        }
    } catch(const std::exception& e) {
        std::cerr << "Failed to parse timeout, using default (30s): " << e.what() << std::endl;
    }

    // Create args vector starting from the script (second argument)
    std::vector<std::string> args;
    for(int i = 2; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    try {
        auto devicesBefore = 0;
        std::string targetDeviceId;

        while(devicesBefore < 1) {
            devicesBefore = dai::Device::getAllAvailableDevices().size();
            std::cout << "Devices now: " << devicesBefore << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        // Mirror child auto-selection logic, then pin child to that same device id.
        bool foundTargetDevice = false;
        dai::DeviceInfo targetDeviceInfo;
        std::tie(foundTargetDevice, targetDeviceInfo) = dai::Device::getAnyAvailableDevice();
        if(foundTargetDevice) {
            targetDeviceId = targetDeviceInfo.getDeviceId();
        }
        if(!targetDeviceId.empty()) {
#ifdef _WIN32
            _putenv_s("DEPTHAI_DEVICE_ID_LIST", targetDeviceId.c_str());
#else
            setenv("DEPTHAI_DEVICE_ID_LIST", targetDeviceId.c_str(), 1);
#endif
            std::cout << "Pinned child process to device id: " << targetDeviceId << std::endl;
        } else {
            std::cout << "No target device id resolved; child will select device automatically." << std::endl;
        }
        // Re-baseline after potential filtering via DEPTHAI_DEVICE_ID_LIST.
        devicesBefore = static_cast<int>(dai::Device::getAllAvailableDevices().size());

        // Run the process with captured output and timeout
        auto start = std::chrono::steady_clock::now();
        bool timedOut = false;

        subprocess::Popen proc(args);

        // Wait for the specified timeout
        std::cout << "Running with timeout of " << timeout << " seconds" << std::endl;
        while(proc.poll() == -1) {
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > timeout) {
                timedOut = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if(timedOut) {
            std::cerr << "=== Test exceeded timeout of " << timeout << " seconds, terminating ===" << std::endl;

#if defined(_WIN32) || defined(__MINGW32__)

            std::cerr << "Attempting termination via proc.kill() (TerminateProcess)..." << std::endl;
            proc.kill();  // Call kill (uses TerminateProcess). Let library use its default exit code (9).
                          // Or you could use proc.kill(1); for exit code 1.

            // Wait a short time to allow the OS to terminate the process
            // TerminateProcess is generally forceful, but cleanup might take a moment.
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // 1.5 seconds

            if(proc.poll() == -1) {
                std::cerr << "Process still reported as running after TerminateProcess attempt." << std::endl;
                // No further escalation possible with this library's API on Windows.
            }

#else
            std::cerr << "Sending SIGINT..." << std::endl;
            proc.kill(SIGINT);                                     // Try graceful termination first
            std::this_thread::sleep_for(std::chrono::seconds(7));  // Wait for it to exit

            if(proc.poll() == -1) {  // Check if it's still running
                std::cerr << "Process still running after SIGINT, sending SIGKILL..." << std::endl;
                proc.kill(SIGKILL);                                    // Force kill
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Short wait after SIGKILL
            }
#endif
            std::cerr << "Process terminated." << std::endl;
        }

        // Device recovery wait
        std::cout << "Devices before: " << devicesBefore << std::endl;
        auto recoveryStart = std::chrono::steady_clock::now();
        while(devicesBefore > dai::Device::getAllAvailableDevices().size() && std::chrono::steady_clock::now() - recoveryStart < std::chrono::seconds(30)) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "Devices now: " << dai::Device::getAllAvailableDevices().size() << std::endl;
        }

        // Now we can safely check the return code
        int retcode = proc.retcode();

        bool crashDumpFound = false;
        if(retcode != 0) {
            // Post-failure sweep: reconnect to the same device and check if a pending
            // crash dump exists for this failed run.
            try {
                bool found = false;
                dai::DeviceInfo deviceInfo;
                auto startSweep = std::chrono::steady_clock::now();
                while(!found && std::chrono::steady_clock::now() - startSweep < std::chrono::seconds(15)) {
                    if(!targetDeviceId.empty()) {
                        std::tie(found, deviceInfo) = dai::Device::getDeviceById(targetDeviceId);
                        if(!found) {
                            std::tie(found, deviceInfo) = dai::XLinkConnection::getDeviceById(targetDeviceId, X_LINK_ANY_STATE, false);
                        }
                    } else {
                        break;
                    }
                    if(!found) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                }
                if(found) {
                    dai::Device::Config config;
                    config.logLevel = dai::LogLevel::CRITICAL;
                    config.outputLogLevel = dai::LogLevel::CRITICAL;
                    dai::Device crashDumpSweep(config, deviceInfo);
                    crashDumpFound = crashDumpSweep.hasCrashDump();
                    if(crashDumpFound) {
                        std::cout << "Post-failure crash dump sweep found pending crash dump." << std::endl;
                    } else {
                        std::cout << "Post-failure crash dump sweep found no pending crash dump." << std::endl;
                    }
                } else if(targetDeviceId.empty()) {
                    std::cout << "Post-failure crash dump sweep skipped (no target device id)." << std::endl;
                } else {
                    std::cout << "Post-failure crash dump sweep skipped (device not found)." << std::endl;
                }
            } catch(const std::exception& ex) {
                std::cout << "[wrapper] Post-failure crash dump sweep failed: " << ex.what() << std::endl;
            }
        }

        // 2 signifies that proc was killed by a timeout
        if(retcode == 2 && !crashDumpFound) {
            retcode = 0;
        }

        return retcode;

    } catch(const std::exception& e) {
        std::cerr << "[wrapper] Exception: " << e.what() << std::endl;
        return 1;
    }
}
