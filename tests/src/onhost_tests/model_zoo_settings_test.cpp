#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <depthai/modelzoo/Zoo.hpp>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

TEST_CASE("Model zoo settings support concurrent readers and writers", "[modelzoo][concurrency]") {
    // This native test exercises actual parallel access without Python's GIL
    // and can also be run under ThreadSanitizer to detect unsynchronized copies.
    struct RestoreSettings {
        const std::string health = dai::modelzoo::getHealthEndpoint();
        const std::string download = dai::modelzoo::getDownloadEndpoint();
        const std::filesystem::path cache = dai::modelzoo::getDefaultCachePath();
        const std::filesystem::path models = dai::modelzoo::getDefaultModelsPath();
        ~RestoreSettings() {
            dai::modelzoo::setHealthEndpoint(health);
            dai::modelzoo::setDownloadEndpoint(download);
            dai::modelzoo::setDefaultCachePath(cache);
            dai::modelzoo::setDefaultModelsPath(models);
        }
    } restore;

    const std::string first(256, 'a');
    const std::string second(1024, 'b');
    const auto writeSettings = [](const std::string& value) {
        dai::modelzoo::setHealthEndpoint(value);
        dai::modelzoo::setDownloadEndpoint(value);
        dai::modelzoo::setDefaultCachePath(value);
        dai::modelzoo::setDefaultModelsPath(value);
    };
    const auto isValid = [&](const std::string& value) { return value == first || value == second; };
    writeSettings(first);

    std::atomic<bool> start{false};
    std::atomic<bool> valid{true};
    std::vector<std::thread> threads;
    for(int worker = 0; worker < 4; ++worker) {
        threads.emplace_back([&, worker] {
            while(!start.load()) std::this_thread::yield();
            for(int iteration = 0; iteration < 2000; ++iteration) {
                if(worker < 2) {
                    writeSettings(iteration % 2 == 0 ? first : second);
                } else if(!isValid(dai::modelzoo::getHealthEndpoint()) || !isValid(dai::modelzoo::getDownloadEndpoint())
                          || !isValid(dai::modelzoo::getDefaultCachePath().string()) || !isValid(dai::modelzoo::getDefaultModelsPath().string())) {
                    valid.store(false);
                }
            }
        });
    }
    start.store(true);
    for(auto& thread : threads) thread.join();
    REQUIRE(valid.load());
}
