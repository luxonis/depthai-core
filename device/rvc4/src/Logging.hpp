#pragma once

// std
#include <vector>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

// shared
#include "depthai/log/LogMessage.hpp"



struct Logging {
    ~Logging();
    std::vector<dai::LogMessage> logBuffer;
    std::condition_variable cv;
    std::mutex cvMutex;
    std::thread thread;
    std::atomic<bool> isRunning{false};

    void addLog(dai::LogMessage log);
    void start();

};