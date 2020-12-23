// Modified for internal use of depthai-core library
// Luxonis - December 2020

// Copyright (C) 2018-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <vector>
#include <utility>

#include "BlobFormat.hpp"

namespace dai {

class BlobReader {
public:
    BlobReader() = default;

    void parse(const std::vector<std::uint8_t>& blob);

    //const ie::InputsDataMap& getNetworkInputs() const { return _networkInputs; }
    //const ie::OutputsDataMap& getNetworkOutputs() const { return _networkOutputs; }

    //uint32_t getStageCount() const { return blobHeader.stages_count; }

    uint32_t getMagicNumber() const { return blobHeader.magic_number; }

    uint32_t getVersionMajor() const { return blobHeader.blob_ver_major; }
    uint32_t getVersionMinor() const { return blobHeader.blob_ver_minor; }

    uint32_t getNumberOfShaves() const { return blobHeader.number_of_shaves; }
    uint32_t getNumberOfSlices() const { return blobHeader.number_of_cmx_slices; }

    //const DataInfo& getInputInfo()  const { return _inputInfo; }
    //const DataInfo& getOutputInfo() const { return _outputInfo; }

    std::pair<const std::uint8_t*, size_t> getHeader() const { return {pBlob, sizeof(ElfN_Ehdr) + sizeof(mv_blob_header)};}

private:
    const std::uint8_t* pBlob = nullptr;

    mv_blob_header blobHeader = {};

    constexpr static std::uint32_t BLOB_MAGIC_NUMBER = 9709;

    // TODO(themarpe) - add additional network information
    //ie::InputsDataMap  _networkInputs;
    //ie::OutputsDataMap _networkOutputs;

    //DataInfo _inputInfo;
    //DataInfo _outputInfo;
};

}  // namespace dai
