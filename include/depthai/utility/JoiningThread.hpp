#pragma once

#include <thread>

namespace dai {

class JoiningThread : private std::thread {
   public:
    using std::thread::thread;
    JoiningThread() = default;
    // Create an empty copy constructor
    JoiningThread(const JoiningThread&) : JoiningThread() {}
    JoiningThread(JoiningThread&&) = default;
    JoiningThread& operator=(JoiningThread&& thr) {
        if(joinable()) {
            join();
        }
        swap(thr);
        return *this;
    };
    ~JoiningThread() {
        if(joinable()) {
            join();
        }
    }
    JoiningThread(std::thread t) : std::thread(std::move(t)) {}

    using std::thread::detach;
    using std::thread::get_id;
    using std::thread::hardware_concurrency;
    using std::thread::join;
    using std::thread::joinable;
    using std::thread::native_handle;

    void swap(JoiningThread& x) {
        std::thread::swap(x);
    }
};

inline void swap(JoiningThread& x, JoiningThread& y) {
    x.swap(y);
}

}  // namespace dai
