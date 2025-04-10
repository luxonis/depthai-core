#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
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
        auto devicesBefore = dai::Device::getAllAvailableDevices().size();

        // Run the process with captured output and timeout
        auto start = std::chrono::steady_clock::now();
        bool timed_out = false;

        subprocess::Popen proc(args, subprocess::output{subprocess::PIPE}, subprocess::error{subprocess::PIPE});

        // Wait for the specified timeout
        std::cout << "Running with timeout of " << timeout << " seconds" << std::endl;
        while(proc.poll() == -1) {
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > timeout) {
                timed_out = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if(timed_out) {
            std::cerr << "=== Test exceeded timeout of " << timeout << " seconds, terminating ===" << std::endl;
            proc.kill(SIGINT);  // Use the built-in kill method
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if(proc.poll() == -1) {
                std::cerr << "Still running, killing..." << std::endl;
                proc.kill(SIGKILL);  // Use SIGKILL if SIGTERM didn't work
            }

            // Device recovery wait
            std::cout << "Devices before: " << devicesBefore << std::endl;
            while(devicesBefore > dai::Device::getAllAvailableDevices().size()) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                std::cout << "Devices now: " << dai::Device::getAllAvailableDevices().size() << std::endl;
            }

            std::cout << "=== Device rebooted ===" << std::endl;
            return 0;
        }

        // Only call communicate() once and save the results
        auto results = proc.communicate();

        // Now we can safely check the return code
        int retcode = proc.retcode();

        // Always print the output regardless of return code
        std::string stdoutStr(results.first.buf.data(), results.first.length);
        std::string stderrStr(results.second.buf.data(), results.second.length);

        std::cout << "=== Subprocess STDOUT ===\n" << stdoutStr << std::endl;
        std::cerr << "=== Subprocess STDERR ===\n" << stderrStr << std::endl;

        return retcode;

    } catch(const std::exception& e) {
        std::cerr << "[wrapper] Exception: " << e.what() << std::endl;
        return 1;
    }
}
