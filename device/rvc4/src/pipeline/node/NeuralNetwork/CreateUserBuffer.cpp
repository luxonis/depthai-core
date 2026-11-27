#include "CreateUserBuffer.hpp"

#include <assert.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/IUserBufferFactory.hpp"
#include "DlSystem/StringList.hpp"
#include "DlSystem/TensorShape.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "SNPE/SNPE.hpp"
#include "SNPE/SNPEFactory.hpp"

static size_t calcSizeFromDims(const zdl::DlSystem::Dimension* dims, size_t rank, size_t elementSize) {
    if(rank == 0) return 0;
    size_t size = elementSize;
    while(rank--) {
        if(*dims == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        size *= *dims;
        dims++;
    }
    return size;
}

size_t createUserBuffer(zdl::DlSystem::UserBufferMap& userBufferMap,
                        std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                        std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                        std::unique_ptr<zdl::SNPE::SNPE>& snpe,
                        const char* name) {
    // get attributes of buffer by name
    auto bufferAttributesOpt = snpe->getInputOutputBufferAttributes(name);
    if(!bufferAttributesOpt) throw std::runtime_error(std::string("Error obtaining attributes for input tensor ") + name);

    // calculate the size of buffer required by the input tensor
    const zdl::DlSystem::TensorShape& bufferShape = (*bufferAttributesOpt)->getDims();

    size_t bufferElementSize = 1;
    bool isTfNBuffer = false;

    using ElementType = zdl::DlSystem::UserBufferEncoding::ElementType_t;
    switch((*bufferAttributesOpt)->getEncodingType()) {
        case DlSystem::UserBufferEncoding::ElementType_t::FLOAT16:
        case DlSystem::UserBufferEncoding::ElementType_t::TF16:
        case DlSystem::UserBufferEncoding::ElementType_t::INT16:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT16:
            bufferElementSize = 2;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::FLOAT:
        case DlSystem::UserBufferEncoding::ElementType_t::INT32:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT32:
            bufferElementSize = 4;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::INT64:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT64:
            bufferElementSize = 8;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::TF8:
            isTfNBuffer = true;
        case DlSystem::UserBufferEncoding::ElementType_t::UNSIGNED8BIT:
        case DlSystem::UserBufferEncoding::ElementType_t::INT8:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT8:
        case DlSystem::UserBufferEncoding::ElementType_t::BOOL8:
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::UNKNOWN:
            throw std::runtime_error("ERROR: unexpected buffer type");
            break;
    }

    // Calculate the stride based on buffer strides.
    // Note: Strides = Number of bytes to advance to the next element in each dimension.
    // For example, if a float tensor of dimension 2x4x3 is tightly packed in a buffer of 96 bytes, then the strides would be (48,12,4)
    // Note: Buffer stride is usually known and does not need to be calculated.
    // std::cerr << "BufferShape rank is: " << bufferShape.rank() << std::endl;
    std::vector<size_t> strides(bufferShape.rank());
    strides[strides.size() - 1] = bufferElementSize;
    size_t stride = strides[strides.size() - 1];
    for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
        if(bufferShape[i] == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        stride *= bufferShape[i];
        strides[i - 1] = stride;
    }
    size_t bufSize = calcSizeFromDims(bufferShape.getDimensions(), bufferShape.rank(), bufferElementSize);
    // set the buffer encoding type
    std::unique_ptr<zdl::DlSystem::UserBufferEncoding> userBufferEncoding;
    if(isTfNBuffer) {
        const zdl::DlSystem::UserBufferEncodingTfN* ubeTfN = dynamic_cast<const zdl::DlSystem::UserBufferEncodingTfN*>((*bufferAttributesOpt)->getEncoding());
        uint64_t stepEquivalentTo0 = ubeTfN->getStepExactly0();
        float quantizedStepSize = ubeTfN->getQuantizedStepSize();
        userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingTfN>(
            new zdl::DlSystem::UserBufferEncodingTfN(stepEquivalentTo0, quantizedStepSize, bufferElementSize * 8));
    } else {
        switch((*bufferAttributesOpt)->getEncodingType()) {
            case DlSystem::UserBufferEncoding::ElementType_t::FLOAT16:
                userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingFloatN>(new zdl::DlSystem::UserBufferEncodingFloatN(16));
                break;
            case DlSystem::UserBufferEncoding::ElementType_t::INT8:
            case DlSystem::UserBufferEncoding::ElementType_t::INT16:
            case DlSystem::UserBufferEncoding::ElementType_t::INT32:
            case DlSystem::UserBufferEncoding::ElementType_t::INT64:
                userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingIntN>(new zdl::DlSystem::UserBufferEncodingIntN(bufferElementSize * 8));
                break;
            case DlSystem::UserBufferEncoding::ElementType_t::UINT8:
            case DlSystem::UserBufferEncoding::ElementType_t::UINT16:
            case DlSystem::UserBufferEncoding::ElementType_t::UINT32:
            case DlSystem::UserBufferEncoding::ElementType_t::UINT64:
                userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingUintN>(new zdl::DlSystem::UserBufferEncodingUintN(bufferElementSize * 8));
                break;
            case DlSystem::UserBufferEncoding::ElementType_t::FLOAT:
                userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingFloat>(new zdl::DlSystem::UserBufferEncodingFloat());
                break;
            case DlSystem::UserBufferEncoding::ElementType_t::UNSIGNED8BIT:
            case DlSystem::UserBufferEncoding::ElementType_t::BOOL8:
                userBufferEncoding = std::unique_ptr<zdl::DlSystem::UserBufferEncodingUnsigned8Bit>(new zdl::DlSystem::UserBufferEncodingUnsigned8Bit());
                break;
            case DlSystem::UserBufferEncoding::ElementType_t::UNKNOWN:
            case DlSystem::UserBufferEncoding::ElementType_t::TF8:
            case DlSystem::UserBufferEncoding::ElementType_t::TF16:
                throw std::runtime_error("ERROR: unexpected buffer type");
                break;
        }
    }

    // create user-backed storage to load input data onto it
    applicationBuffers.emplace(name, std::vector<uint8_t>(bufSize));

    // create SNPE user buffer from the user-backed buffer
    zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
    snpeUserBackedBuffers.push_back(ubFactory.createUserBuffer(applicationBuffers.at(name).data(), bufSize, strides, userBufferEncoding.get()));
    if(snpeUserBackedBuffers.back() == nullptr) {
        throw std::runtime_error("Error while creating user buffer.");
    }
    // add the user-backed buffer to the inputMap, which is later on fed to the network for execution
    userBufferMap.add(name, snpeUserBackedBuffers.back().get());

    return bufSize;
}

size_t createInputBufferMap(zdl::DlSystem::UserBufferMap& inputMap,
                            std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                            std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                            std::unique_ptr<zdl::SNPE::SNPE>& snpe) {
    // get input tensor names of the network that need to be populated
    const auto& inputNamesOpt = snpe->getInputTensorNames();
    if(!inputNamesOpt) throw std::runtime_error("Error obtaining input tensor names");
    const zdl::DlSystem::StringList& inputNames = *inputNamesOpt;
    assert(inputNames.size() > 0);

    size_t bufSize = 0;

    // create SNPE user buffers for each application storage buffer
    for(const char* name : inputNames) {
        bufSize = createUserBuffer(inputMap, applicationBuffers, snpeUserBackedBuffers, snpe, name);
    }

    return bufSize;
}

void createOutputBufferMap(zdl::DlSystem::UserBufferMap& outputMap,
                           std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                           std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                           std::unique_ptr<zdl::SNPE::SNPE>& snpe) {
    // get input tensor names of the network that need to be populated
    const auto& outputNamesOpt = snpe->getOutputTensorNames();
    if(!outputNamesOpt) throw std::runtime_error("Error obtaining output tensor names");
    const zdl::DlSystem::StringList& outputNames = *outputNamesOpt;

    // create SNPE user buffers for each application storage buffer
    for(const char* name : outputNames) {
        createUserBuffer(outputMap, applicationBuffers, snpeUserBackedBuffers, snpe, name);
    }
}
