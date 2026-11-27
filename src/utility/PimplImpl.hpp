#pragma once  // Pimpl pattern from https://herbsutter.com/gotw/_101/

#include <utility>

namespace dai {

template <typename T>
Pimpl<T>::Pimpl() : m{new T{}} {}

template <typename T>
template <typename... Args>
Pimpl<T>::Pimpl(Args&&... args) : m{new T{std::forward<Args>(args)...}} {}

template <typename T>
Pimpl<T>::~Pimpl() {}

template <typename T>
T* Pimpl<T>::operator->() {
    return m.get();
}

template <typename T>
const T* Pimpl<T>::operator->() const {
    return m.get();
}

template <typename T>
T& Pimpl<T>::operator*() {
    return *m.get();
}

template <typename T>
const T& Pimpl<T>::operator*() const {
    return *m.get();
}

}  // namespace dai
