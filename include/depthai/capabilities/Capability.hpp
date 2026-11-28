#pragma once

#include <cstring>
namespace dai {

class Capability {
   public:
    virtual const char* getName() const = 0;
    virtual ~Capability();
};

template <typename Base, typename Derived>
class CapabilityCRTP : public Base {
   public:
    virtual ~CapabilityCRTP() = default;

    const char* getName() const override {
        return Derived::NAME;  // NAME must be globally unique: prefix/name, e.g. dai/img-frame
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
