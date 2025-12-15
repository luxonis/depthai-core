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

   public:
    CircularBuffer(size_t size) : maxSize(size) {
        buffer.reserve(size);
    }
    // Add a new element to the buffer after the last one or overwrite the oldest one
    T& add(T value) {
        if(buffer.size() < maxSize) {
            buffer.push_back(value);
            return buffer.back();
        } else {
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

}  // namespace utility
}  // namespace dai
