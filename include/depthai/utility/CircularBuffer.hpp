#pragma once
#include <stdexcept>
#include <vector>

namespace dai {
namespace utility {

template <typename T>
class CircularBuffer {
   public:
    CircularBuffer(size_t size) : maxSize(size) {
        buffer.reserve(size);
    }
    void add(int value) {
        if(buffer.size() < maxSize) {
            buffer.push_back(value);
        } else {
            buffer[index] = value;
            index = (index + 1) % maxSize;
        }
    }
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
    size_t size() const {
        return buffer.size();
    }

   private:
    std::vector<T> buffer;
    size_t maxSize;
    size_t index = 0;
};

}  // namespace utility
}  // namespace dai
