#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Independent SHA-256. The replay node has its own hash; tests must not call it.
namespace test_hash {

inline std::uint32_t rotr(std::uint32_t value, std::uint32_t bits) {
    return (value >> bits) | (value << (32U - bits));
}

inline std::string sha256Hex(const std::uint8_t* data, std::size_t length) {
    static const std::uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

    std::uint32_t h0 = 0x6a09e667;
    std::uint32_t h1 = 0xbb67ae85;
    std::uint32_t h2 = 0x3c6ef372;
    std::uint32_t h3 = 0xa54ff53a;
    std::uint32_t h4 = 0x510e527f;
    std::uint32_t h5 = 0x9b05688c;
    std::uint32_t h6 = 0x1f83d9ab;
    std::uint32_t h7 = 0x5be0cd19;

    std::vector<std::uint8_t> message(data, data + length);
    const std::uint64_t bitLength = static_cast<std::uint64_t>(length) * 8ULL;
    message.push_back(0x80);
    while((message.size() % 64ULL) != 56ULL) {
        message.push_back(0);
    }
    for(int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<std::uint8_t>((bitLength >> shift) & 0xffU));
    }

    for(std::size_t offset = 0; offset < message.size(); offset += 64) {
        std::uint32_t w[64];
        for(int i = 0; i < 16; ++i) {
            const std::size_t base = offset + static_cast<std::size_t>(i) * 4ULL;
            w[i] = (static_cast<std::uint32_t>(message[base]) << 24) | (static_cast<std::uint32_t>(message[base + 1]) << 16)
                   | (static_cast<std::uint32_t>(message[base + 2]) << 8) | static_cast<std::uint32_t>(message[base + 3]);
        }
        for(int i = 16; i < 64; ++i) {
            const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        std::uint32_t a = h0;
        std::uint32_t b = h1;
        std::uint32_t c = h2;
        std::uint32_t d = h3;
        std::uint32_t e = h4;
        std::uint32_t f = h5;
        std::uint32_t g = h6;
        std::uint32_t h = h7;
        for(int i = 0; i < 64; ++i) {
            const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const std::uint32_t ch = (e & f) ^ ((~e) & g);
            const std::uint32_t temp1 = h + s1 + ch + k[i] + w[i];
            const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t temp2 = s0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
        h5 += f;
        h6 += g;
        h7 += h;
    }

    const std::array<std::uint32_t, 8> words{{h0, h1, h2, h3, h4, h5, h6, h7}};
    std::ostringstream out;
    out << std::hex;
    for(std::uint32_t word : words) {
        for(int shift = 28; shift >= 0; shift -= 4) {
            out << "0123456789abcdef"[(word >> shift) & 0xfU];
        }
    }
    return out.str();
}

inline std::string sha256Hex(const std::vector<std::uint8_t>& bytes) {
    return sha256Hex(bytes.data(), bytes.size());
}

inline std::vector<std::uint8_t> readBytes(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) {
        throw std::runtime_error("cannot read " + path.string());
    }
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

inline std::string sha256File(const std::filesystem::path& path) {
    const auto bytes = readBytes(path);
    return sha256Hex(bytes.data(), bytes.size());
}

inline bool nameBefore(const std::string& left, const std::string& right) {
    return std::lexicographical_compare(reinterpret_cast<const unsigned char*>(left.data()),
                                         reinterpret_cast<const unsigned char*>(left.data() + left.size()),
                                         reinterpret_cast<const unsigned char*>(right.data()),
                                         reinterpret_cast<const unsigned char*>(right.data() + right.size()));
}

// Directory hash: each non-dot regular file, ordered by unsigned-char UTF-8 name: name, 0x00, bytes.
inline std::string sha256Dir(const std::filesystem::path& dir) {
    std::vector<std::string> names;
    for(const auto& entry : std::filesystem::directory_iterator(dir)) {
        if(!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if(name.empty() || name[0] == '.') continue;
        names.push_back(name);
    }
    std::sort(names.begin(), names.end(), nameBefore);
    std::vector<std::uint8_t> blob;
    for(const auto& name : names) {
        blob.insert(blob.end(), name.begin(), name.end());
        blob.push_back(0);
        const auto bytes = readBytes(dir / name);
        blob.insert(blob.end(), bytes.begin(), bytes.end());
    }
    return sha256Hex(blob.data(), blob.size());
}

// File plus metadata: fileName, 0x00, file bytes, metaName, 0x00, meta bytes. Names are base names.
inline std::string sha256FileAndMetadata(const std::filesystem::path& file, const std::filesystem::path& metadata) {
    std::vector<std::uint8_t> blob;
    const auto fileName = file.filename().string();
    const auto metaName = metadata.filename().string();
    blob.insert(blob.end(), fileName.begin(), fileName.end());
    blob.push_back(0);
    const auto fileBytes = readBytes(file);
    blob.insert(blob.end(), fileBytes.begin(), fileBytes.end());
    blob.insert(blob.end(), metaName.begin(), metaName.end());
    blob.push_back(0);
    const auto metaBytes = readBytes(metadata);
    blob.insert(blob.end(), metaBytes.begin(), metaBytes.end());
    return sha256Hex(blob.data(), blob.size());
}

}  // namespace test_hash
