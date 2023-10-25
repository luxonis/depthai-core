// https://herbsutter.com/gotw/_101/
#pragma once

#include <utility>

namespace dai {

template<typename T>
Pimpl<T>::Pimpl() : m{ new T{} } { }

template<typename T>
template<typename ...Args>
Pimpl<T>::Pimpl( Args&& ...args )
    : m{ new T{ std::forward<Args>(args)... } } { }

template<typename T>
Pimpl<T>::~Pimpl() { }

template<typename T>
Pimpl<T>::Pimpl(T* raw_ptr) : m{ raw_ptr } {}

template<typename T>
Pimpl<T>& Pimpl<T>::operator=(const Pimpl<T>& other) {
    if (&other == this) {
        return *this;
    }
    m = std::make_unique<T>(*other.m);
    return *this;
}

template<typename T>
Pimpl<T>::Pimpl(Pimpl<T>&& other) noexcept : m{std::move(other.m)} {}

template<typename T>
Pimpl<T>& Pimpl<T>::operator=(Pimpl<T>&& other) noexcept {
    if (this != &other) {
        m = std::move(other.m);
    }
    return *this;
}

template<typename T>
T* Pimpl<T>::operator->() { return m.get(); }

template<typename T>
T& Pimpl<T>::operator*() { return *m.get(); }

} // namespace dai
