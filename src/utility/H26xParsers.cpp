#include "H26xParsers.hpp"

#include <cmath>
#include <tuple>

namespace dai {
namespace utility {

template <typename T>
struct H26xParser {
   protected:
    virtual void parseNal(const std::vector<std::uint8_t>& bs, unsigned int start, std::vector<SliceType>& out) = 0;
    std::vector<SliceType> parseBytestream(const std::vector<std::uint8_t>& bs, bool breakOnFirst);

   public:
    static std::vector<SliceType> getTypes(const std::vector<std::uint8_t>& bs, bool breakOnFirst);
    virtual ~H26xParser() = default;
};

struct H264Parser : H26xParser<H264Parser> {
    void parseNal(const std::vector<std::uint8_t>& bs, unsigned int start, std::vector<SliceType>& out);
};

struct H265Parser : H26xParser<H265Parser> {
    unsigned int nalUnitType = 0;                        // In NAL header
    unsigned int dependentSliceSegmentsEnabledFlag = 0;  // In picture parameter set
    unsigned int numExtraSliceHeaderBits = 0;            // In picture parameter set
    // Getting the length of slice_segment address
    unsigned int picWidthInLumaSamples = 0;              // In sequence parameter set
    unsigned int picHeightInLumaSamples = 0;             // In sequence parameter set
    unsigned int chromaFormatIdc = 0;                    // In sequence parameter set
    unsigned int log2DiffMaxMinLumaCodingBlockSize = 0;  // In sequence parameter set
    unsigned int log2MinLumaCodingBlockSizeMinus3 = 0;   // In sequence parameter set

    void parseNal(const std::vector<std::uint8_t>& bs, unsigned int start, std::vector<SliceType>& out);
};

typedef unsigned int uint;
typedef unsigned long ulong;
typedef const std::vector<std::uint8_t> buf;

SliceType getSliceType(uint num, Profile p) {
    switch(p) {
        case Profile::H264:
            switch(num) {
                case 0:
                case 5:
                    return SliceType::P;
                case 1:
                case 6:
                    return SliceType::B;
                case 2:
                case 7:
                    return SliceType::I;
                case 3:
                case 8:
                    return SliceType::SP;
                case 4:
                case 9:
                    return SliceType::SI;
                default:
                    return SliceType::Unknown;
            }
        case Profile::H265:
            switch(num) {
                case 0:
                    return SliceType::B;
                case 1:
                    return SliceType::P;
                case 2:
                    return SliceType::I;
                default:
                    return SliceType::Unknown;
            }
        default:
            return SliceType::Unknown;
    }
}

bool scodeEq(buf& bs, uint pos, buf code) {
    if(bs.size() - pos > code.size()) {
        for(uint i = 0; i < code.size(); ++i) {
            if(bs[pos + i] != code[i]) return false;
        }
        return true;
    } else
        return false;
}

uint findStart(buf& bs, uint pos) {
    buf codeLong = {0, 0, 0, 1};
    buf codeShort = {0, 0, 1};
    uint size = bs.size();
    for(uint i = pos; i < size; ++i) {
        if(bs[i] == 0) {
            if(scodeEq(bs, i, codeLong)) {
                return i + 4;
            } else if(scodeEq(bs, i, codeShort)) {
                return i + 3;
            }
        }
    }
    return size;
}

uint findEnd(buf& bs, uint pos) {
    buf end1 = {0, 0, 0};
    buf end2 = {0, 0, 1};
    uint size = bs.size();
    for(uint i = pos; i < size; ++i) {
        if(bs[i] == 0) {
            if(scodeEq(bs, i, end1) || scodeEq(bs, i, end2)) {
                return i;
            }
        }
    }
    return size;
}

uint readUint(buf& bs, ulong start, ulong end) {
    uint ret = 0;
    for(ulong i = start; i < end; ++i) {
        uint bit = (bs[(uint)(i / 8)] & (1 << (7 - i % 8))) > 0;
        ret += bit * (1 << (end - i - 1));
    }
    return ret;
}

std::tuple<uint, ulong> readGE(buf& bs, ulong pos) {
    uint count = 0;
    ulong size = bs.size() * 8;
    while(pos < size) {
        std::uint8_t bit = bs[(uint)(pos / 8)] & (char)(1 << (7 - pos % 8));
        if(bit == 0) {
            ++count;
            ++pos;
        } else
            break;
    }
    ++count;
    ulong start = pos;
    ulong end = pos + count;
    if(end > size) exit(30);
    return std::make_tuple(readUint(bs, start, end) - 1, end);
}

template <typename T>
std::vector<SliceType> H26xParser<T>::getTypes(buf& buffer, bool breakOnFirst) {
    T p;
    return p.parseBytestream(buffer, breakOnFirst);
}

template <typename T>
std::vector<SliceType> H26xParser<T>::parseBytestream(buf& bs, bool breakOnFirst) {
    uint pos = 0;
    uint size = bs.size();
    std::vector<SliceType> ret;
    while(pos < size) {
        uint start = findStart(bs, pos);
        uint end = findEnd(bs, start);
        if(start >= end) break;
        parseNal(bs, start, ret);
        pos = end;
        if(breakOnFirst && ret.size() > 0) break;
    }
    return ret;
}

void H264Parser::parseNal(buf& bs, uint start, std::vector<SliceType>& out) {
    uint pos = start;
    uint nalUnitType = bs[pos++] & 31;
    uint nalUnitHeaderBytes = 1;
    if(nalUnitType == 14 || nalUnitType == 20 || nalUnitType == 21) {
        uint avc3dExtensionFlag = 0;
        if(nalUnitType == 21) avc3dExtensionFlag = readUint(bs, pos * 8, pos * 8 + 1);
        if(avc3dExtensionFlag)
            nalUnitHeaderBytes += 2;
        else
            nalUnitHeaderBytes += 3;
    }
    pos = start + nalUnitHeaderBytes;
    if(nalUnitType == 1 || nalUnitType == 5) {
        const auto bpos1 = std::get<1>(readGE(bs, pos * 8));
        uint tyNum;
        ulong bpos2;
        std::tie(tyNum, bpos2) = readGE(bs, bpos1);
        pos = (uint)(bpos2 / 8) + (bpos2 % 8 > 0);
        out.push_back(getSliceType(tyNum, Profile::H264));
    }
}

void H265Parser::parseNal(buf& bs, uint start, std::vector<SliceType>& out) {
    nalUnitType = (bs[start] & 126) >> 1;
    uint pos = start + 2;
    if(nalUnitType == 33) {
        // Sequence parameter set
        uint spsMaxSubLayersMinus1 = (bs[pos] & 14) >> 1;
        ++pos;
        const auto bpos1 = std::get<1>(readGE(bs, pos * 8));  // We don't need this value
        uint chr;
        ulong bpos2;
        std::tie(chr, bpos2) = readGE(bs, bpos1);
        chromaFormatIdc = chr;
        if(chromaFormatIdc) ++bpos2;  // We don't need this value
        uint pw;
        ulong bpos3;
        std::tie(pw, bpos3) = readGE(bs, bpos2);
        uint ph;
        ulong bpos4;
        std::tie(ph, bpos4) = readGE(bs, bpos3);
        picWidthInLumaSamples = pw;
        picHeightInLumaSamples = ph;
        uint conformanceWindowFlag = readUint(bs, bpos4, bpos4 + 1);
        ulong bpos8 = ++bpos4;
        if(conformanceWindowFlag) {
            // We don't care about any of these values
            auto bpos5 = std::get<1>(readGE(bs, bpos4));
            auto bpos6 = std::get<1>(readGE(bs, bpos5));
            auto bpos7 = std::get<1>(readGE(bs, bpos6));
            auto tbpos8 = std::get<1>(readGE(bs, bpos7));
            bpos8 = tbpos8;
        }
        auto bpos9 = std::get<1>(readGE(bs, bpos8));
        auto bpos10 = std::get<1>(readGE(bs, bpos9));
        auto bpos11 = std::get<1>(readGE(bs, bpos10));
        uint spsSubLayerOrderingInfoPresentFlag = readUint(bs, bpos11, bpos11 + 1);
        ulong bpos12 = ++bpos11;
        for(uint i = spsSubLayerOrderingInfoPresentFlag ? 0 : spsMaxSubLayersMinus1; i <= spsMaxSubLayersMinus1; ++i) {
            auto tbpos1 = std::get<1>(readGE(bs, bpos12));
            auto tbpos2 = std::get<1>(readGE(bs, tbpos1));
            auto tbpos3 = std::get<1>(readGE(bs, tbpos2));
            bpos12 = tbpos3;
        }
        uint lm, ldm;
        ulong bpos13, bpos14;
        std::tie(lm, bpos13) = readGE(bs, bpos12);
        std::tie(ldm, bpos14) = readGE(bs, bpos13);
        log2MinLumaCodingBlockSizeMinus3 = lm;
        log2DiffMaxMinLumaCodingBlockSize = ldm;
        pos = (uint)(bpos14 / 8) + (bpos14 % 8 > 0);  // Update byte position with newest bit position
    } else if(nalUnitType == 34) {
        // Picture parameter set
        auto bpos1 = std::get<1>(readGE(bs, pos * 8));
        auto bpos2 = std::get<1>(readGE(bs, bpos1));
        dependentSliceSegmentsEnabledFlag = readUint(bs, bpos2, bpos2 + 1);
        bpos2 += 2;
        numExtraSliceHeaderBits = readUint(bs, bpos2, bpos2 + 3);
        bpos2 += 3;
        pos = (uint)(bpos2 / 8) + (bpos2 % 8 > 0);
    } else if(nalUnitType <= 9 || (16 <= nalUnitType && nalUnitType <= 21)) {
        // Coded slice segment
        ulong bpos1 = pos * 8;
        uint firstSliceSegmentInPicFlag = readUint(bs, bpos1, bpos1 + 1);
        ++bpos1;
        if(16 <= nalUnitType && nalUnitType <= 23) ++bpos1;
        auto bpos2 = std::get<1>(readGE(bs, bpos1));
        uint dependentSliceSegmentFlag = 0;
        if(!firstSliceSegmentInPicFlag) {
            if(dependentSliceSegmentsEnabledFlag) {
                dependentSliceSegmentFlag = readUint(bs, bpos2, bpos2 + 1);
                ++bpos2;
            }
            uint ctbLog2SizeY = log2MinLumaCodingBlockSizeMinus3 + 3 + log2DiffMaxMinLumaCodingBlockSize;
            uint ctbSizeY = 1 << ctbLog2SizeY;
            uint picWidthInCtbsY = ceil(picWidthInLumaSamples / ctbSizeY);
            uint picHeightInCtbsY = ceil(picHeightInLumaSamples / ctbSizeY);
            uint picSizeInCtbsY = picWidthInCtbsY * picHeightInCtbsY;
            uint vlen = ceil(log2(picSizeInCtbsY));
            bpos2 += vlen;
        }
        ulong bpos3 = bpos2;
        if(!dependentSliceSegmentFlag) {
            bpos2 += numExtraSliceHeaderBits;
            uint tyNum;
            ulong tbpos3;
            std::tie(tyNum, tbpos3) = readGE(bs, bpos2);
            bpos3 = tbpos3;
            out.push_back(getSliceType(tyNum, Profile::H265));
        }
        pos = (uint)(bpos3 / 8) + (bpos3 % 8 > 0);
    }
}

std::vector<SliceType> getTypesH264(buf& bs, bool breakOnFirst) {
    return H264Parser::getTypes(bs, breakOnFirst);
}
std::vector<SliceType> getTypesH265(buf& bs, bool breakOnFirst) {
    return H265Parser::getTypes(bs, breakOnFirst);
}

}  // namespace utility
}  // namespace dai
