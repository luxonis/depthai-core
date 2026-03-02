#include <catch2/catch_all.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>

namespace {

std::string quotePath(const std::filesystem::path& p) {
    return "\"" + p.string() + "\"";
}

struct RunResult {
    int exitCode = -1;
    float avgFps = 0.0f;
    std::string output;
};

RunResult runChild(const std::filesystem::path& childExe, const std::string& mode, const std::string& suffix) {
    const auto logPath = std::filesystem::temp_directory_path() / ("autocal_bench_" + suffix + ".log");

    std::string cmd;
#ifdef _WIN32
    cmd = "set DEPTHAI_AUTOCALIBRATION=" + mode + " && DEPTHAI_LEVEL=info && " + quotePath(childExe) + " > " + quotePath(logPath) + " 2>&1";
#else
    cmd = "DEPTHAI_AUTOCALIBRATION=" + mode + " DEPTHAI_LEVEL=info " + quotePath(childExe) + " > " + quotePath(logPath) + " 2>&1";
#endif

    RunResult result;
    result.exitCode = std::system(cmd.c_str());

    std::ifstream ifs(logPath);
    result.output.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    std::error_code ec;
    std::filesystem::remove(logPath, ec);

    std::regex avgPattern(R"(avg_fps=([0-9]+(?:\.[0-9]+)?))");
    for(std::sregex_iterator it(result.output.begin(), result.output.end(), avgPattern), end; it != end; ++it) {
        result.avgFps = std::stof((*it)[1].str());
    }

    return result;
}

}  // namespace

TEST_CASE("camera_concurrency_autocalibration_parent_compare") {
    std::filesystem::path selfExe;
#ifdef _WIN32
    selfExe = std::filesystem::current_path() / "regression_camera_concurrency_autocal_compare_test";
#else
    selfExe = std::filesystem::canonical("/proc/self/exe");
#endif
    const auto testsDir = selfExe.parent_path();
    const auto childExe = testsDir / "regression_camera_concurrency_autocal_off_test";

    REQUIRE(std::filesystem::exists(childExe));

    const auto off = runChild(childExe, "OFF", "off");
    INFO("OFF child output:\n" << off.output);
    REQUIRE(off.exitCode == 0);
    REQUIRE(off.avgFps > 0.0f);

    const auto on = runChild(childExe, "ON_START", "on");
    INFO("ON_START child output:\n" << on.output);
    REQUIRE(on.exitCode == 0);
    REQUIRE(on.avgFps > 0.0f);

    const float fpsDrop = off.avgFps - on.avgFps;
    const float fpsDropPct = off.avgFps > 0.0f ? (fpsDrop / off.avgFps) * 100.0f : 0.0f;

    std::cout << "[AUTOCAL_COMPARE] off_avg_fps=" << off.avgFps << " on_avg_fps=" << on.avgFps << " drop_fps=" << fpsDrop
              << " drop_pct=" << fpsDropPct << std::endl;
}
