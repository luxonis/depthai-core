#pragma once

// Taken from PyTorch's arg.h: https://github.com/pytorch/pytorch/blob/main/torch/csrc/api/include/torch/arg.h
// Thank you Pytorch :)

#define DEPTAHI_ARG(T, name)                                              \
   public:                                                                \
    inline auto name(const T& new_##name)->decltype(*this) { /* NOLINT */ \
        this->name##_ = new_##name;                                       \
        return *this;                                                     \
    }                                                                     \
    inline auto name(T&& new_##name)->decltype(*this) { /* NOLINT */      \
        this->name##_ = std::move(new_##name);                            \
        return *this;                                                     \
    }                                                                     \
    inline const T& name() const noexcept { /* NOLINT */                  \
        return this->name##_;                                             \
    }                                                                     \
    inline T& name() noexcept { /* NOLINT */                              \
        return this->name##_;                                             \
    }                                                                     \
                                                                          \
   private:                                                               \
    T name##_ /* NOLINT */

#define DEPTAHI_ARG_DEFAULT(T, name, default_value)                       \
   public:                                                                \
    inline auto name(const T& new_##name)->decltype(*this) { /* NOLINT */ \
        this->name##_ = new_##name;                                       \
        return *this;                                                     \
    }                                                                     \
    inline auto name(T&& new_##name)->decltype(*this) { /* NOLINT */      \
        this->name##_ = std::move(new_##name);                            \
        return *this;                                                     \
    }                                                                     \
    inline const T& name() const noexcept { /* NOLINT */                  \
        return this->name##_;                                             \
    }                                                                     \
    inline T& name() noexcept { /* NOLINT */                              \
        return this->name##_;                                             \
    }                                                                     \
                                                                          \
   private:                                                               \
    T name##_ = default_value /* NOLINT */
