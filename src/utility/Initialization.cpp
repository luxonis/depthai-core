#include "utility/Initialization.hpp"

// project
#include "build/version.hpp"
#include "utility/Resources.hpp"

// libraries
#include "spdlog/cfg/env.h"
#include "spdlog/cfg/helpers.h"
#include "spdlog/details/os.h"
#include "spdlog/spdlog.h"

// For easier access to dai namespaced symbols
namespace dai {

// Anonymous namespace to hide 'Preloader' symbol and variable as its not needed to be visible to other compilation units
namespace {

// Doing early static initialization hits this stage faster than some libraries initialize their global static members

// Preloader uses static global object constructor (works only for shared libraries)
// to execute some code upon final executable launch  or library import
// Preloader
// struct Preloader {
//     Preloader(){
//         initialize();
//     }
// } preloader;

}  // namespace

bool initialize(std::string additionalInfo) {
    // atomic bool for checking whether depthai was already initialized
    static std::atomic<bool> initialized{false};

    if(initialized.exchange(true)) return true;

    // Set global logging level from ENV variable 'DEPTHAI_LEVEL'
    // Taken from spdlog, to replace with DEPTHAI_LEVEL instead of SPDLOG_LEVEL
    // spdlog::cfg::load_env_levels();
    auto env_val = spdlog::details::os::getenv("DEPTHAI_LEVEL");
    if(!env_val.empty()) {
        spdlog::cfg::helpers::load_levels(env_val);
    } else {
        // Otherwise set default level to WARN
        spdlog::set_level(spdlog::level::warn);
    }

    // auto debugger_val = spdlog::details::os::getenv("DEPTHAI_DEBUGGER");
    // if(!debugger_val.empty()){
    //    // TODO(themarpe) - instruct Device class that first available device is also a booted device
    // }

    // Print core commit and build datetime
    if(!additionalInfo.empty()) {
        spdlog::debug("{}", additionalInfo);
    }
    spdlog::debug(
        "Library information - version: {}, commit: {} from {}, build: {}", build::VERSION, build::COMMIT, build::COMMIT_DATETIME, build::BUILD_DATETIME);

    // Executed at library load time

    // Preload Resources (getting instance causes some internal lazy loading to start)
    Resources::getInstance();

    spdlog::debug("Initialize - finished");

    return true;
}

}  // namespace dai
