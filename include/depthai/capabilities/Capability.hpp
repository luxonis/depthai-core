#pragma once

#include <cstring>

namespace dai {

class Capability {
   public:
    virtual const char* getName() const = 0;
    // virtual Capability getIntersection(const Capability& other) = 0;
    virtual ~Capability() = default;
};

// Capability CRTP class
template <typename Base, typename Derived>
class CapabilityCRTP : public Base {
   public:
    virtual ~CapabilityCRTP() = default;

    const char* getName() const override {
        // This has to be a global unique name, please use your prefix and / to separate. Example: dai/img-frame
        return Derived::NAME;
    };

    static const Derived* get(const Capability& cap) {
        if(strcmp(cap.getName(), Derived::NAME) == 0) {
            return static_cast<const Derived*>(&cap);
        }
        return nullptr;
    }

    friend Derived;
    friend Base;
};

}  // namespace dai
