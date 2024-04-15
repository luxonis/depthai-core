#pragma once
#include <atomic>
#include <cstdint>

namespace dai {

class AtomicBool : public std::atomic<bool> {
   public:
    using std::atomic<bool>::atomic;
    AtomicBool() = default;
    ~AtomicBool() = default;
    AtomicBool(const AtomicBool& othr) : AtomicBool() {
        store(othr.load());
    }
    AtomicBool& operator=(const AtomicBool& othr) {
        store(othr);
        return *this;
    };
    AtomicBool(AtomicBool&&) : AtomicBool() {}
    AtomicBool& operator=(AtomicBool&& othr) {
        store(othr);
        return *this;
    };
};

}  // namespace dai
