#pragma once
#include <stdexcept>
#include <vector>

namespace dai {
namespace utility {

/**
 * @brief A simple circular buffer implementation with forward and reverse iterators
 * @tparam T The type of elements stored in the buffer
 *
 * This implementation is not thread-safe.
 */
template <typename T>
class CircularBuffer {
    std::vector<T> buffer;
    size_t maxSize;
    size_t index = 0;

    size_t start() const {
        return (buffer.size() < maxSize) ? 0 : index;
    }

    virtual void onSwap(T&, T&) {}
    virtual void onAdd(T&) {}

   public:
    CircularBuffer(size_t size) : maxSize(size) {
        buffer.reserve(size);
    }
    virtual ~CircularBuffer() = default;
    // Add a new element to the buffer after the last one or overwrite the oldest one
    T& add(T value) {
        if(buffer.size() < maxSize) {
            onAdd(value);
            buffer.push_back(value);
            return buffer.back();
        } else {
            onSwap(buffer[index], value);
            buffer[index] = value;
            index = (index + 1) % maxSize;
            return buffer[(index + maxSize - 1) % maxSize];
        }
    }
    // Get the current buffer contents from oldest to newest
    std::vector<T> getBuffer() const {
        std::vector<T> result;
        if(buffer.size() < maxSize) {
            result = buffer;
        } else {
            result.insert(result.end(), buffer.begin() + index, buffer.end());
            result.insert(result.end(), buffer.begin(), buffer.begin() + index);
        }
        return result;
    }
    // Get the oldest element
    T first() const {
        if(buffer.empty()) {
            throw std::runtime_error("CircularBuffer is empty");
        }
        if(buffer.size() < maxSize) {
            return buffer.front();
        } else {
            return buffer[index];
        }
    }
    // Get the newest element
    T last() const {
        if(buffer.empty()) {
            throw std::runtime_error("CircularBuffer is empty");
        }
        if(buffer.size() < maxSize) {
            return buffer.back();
        } else {
            return buffer[(index + maxSize - 1) % maxSize];
        }
    }
    // Get current size of the buffer. When full, size() == maxSize
    size_t size() const {
        return buffer.size();
    }
    void clear() {
        buffer.clear();
        index = 0;
    }
    T& at(size_t pos) {
        if(pos >= buffer.size()) {
            throw std::out_of_range("CircularBuffer index out of range");
        }
        return buffer[(start() + pos) % buffer.size()];
    }
    const T& at(size_t pos) const {
        if(pos >= buffer.size()) {
            throw std::out_of_range("CircularBuffer index out of range");
        }
        return buffer[(start() + pos) % buffer.size()];
    }

    // ===========================================================
    // Forward iterator
    // ===========================================================
    class iterator {
       public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        iterator(CircularBuffer* parent, size_t pos) : parent(parent), pos(pos) {}

        reference operator*() {
            return parent->buffer[(parent->start() + pos) % parent->buffer.size()];
        }
        pointer operator->() {
            return &(**this);
        }

        iterator& operator++() {
            ++pos;
            return *this;
        }
        iterator operator++(int) {
            iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const iterator& other) const {
            return parent == other.parent && pos == other.pos;
        }
        bool operator!=(const iterator& other) const {
            return !(*this == other);
        }

       private:
        CircularBuffer* parent;
        size_t pos;
    };

    iterator begin() {
        return iterator(this, 0);
    }
    iterator end() {
        return iterator(this, buffer.size());
    }

    // ===========================================================
    // Reverse iterator
    // ===========================================================
    class reverse_iterator {
       public:
        using iterator_category = std::bidirectional_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T*;
        using reference = T&;

        reverse_iterator(CircularBuffer* parent, size_t pos) : parent(parent), pos(pos) {}

        reference operator*() {
            // Map reverse position to logical order
            size_t logicalIndex = (parent->start() + parent->buffer.size() - 1 - pos) % parent->buffer.size();
            return parent->buffer[logicalIndex];
        }
        pointer operator->() {
            return &(**this);
        }

        reverse_iterator& operator++() {
            ++pos;
            return *this;
        }
        reverse_iterator operator++(int) {
            reverse_iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const reverse_iterator& other) const {
            return parent == other.parent && pos == other.pos;
        }
        bool operator!=(const reverse_iterator& other) const {
            return !(*this == other);
        }

       private:
        CircularBuffer* parent;
        size_t pos;
    };

    reverse_iterator rbegin() {
        return reverse_iterator(this, 0);
    }
    reverse_iterator rend() {
        return reverse_iterator(this, buffer.size());
    }
};

/**
 * @brief A circular buffer that maintains a running average of its elements
 * @tparam T The type of elements stored in the buffer (must support addition and division)
 */
template <typename T>
class WindowedAverageBuffer : public CircularBuffer<T> {
    T sum{};

   public:
    // Initialize with buffer size and optional initial sum value
    WindowedAverageBuffer(size_t size, T initialValue = T()) : CircularBuffer<T>(size), sum(initialValue) {}

    void onSwap(T& oldValue, T& newValue) override {
        sum -= oldValue;
        sum += newValue;
    }
    void onAdd(T& newValue) override {
        sum += newValue;
    }
    T getAverage() const {
        if(this->size() == 0) {
            throw std::runtime_error("WindowedAverageBuffer is empty");
        }
        return sum / this->size();
    }
};

}  // namespace utility
}  // namespace dai
