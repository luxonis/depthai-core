#pragma once

#include <assert.h>
#include <string.h>
#include <inttypes.h>

#include <memory>
#include <numeric>
#include <vector>

#include "boost/optional.hpp"

#include "depthai-shared/timer.hpp"

#include "depthai-shared/metadata/frame_metadata.hpp"
#include "depthai-shared/object_tracker/object_tracker.hpp"
#include "depthai-shared/stream/stream_info.hpp"


struct HostDataPacket
{
    HostDataPacket(
        unsigned size,
        void* in_data,
        StreamInfo streamInfo
    )
        : stream_name(streamInfo.name)
        , elem_size(streamInfo.elem_size)
    {
        int frameSize = size;

        FrameMetadata metadata;
        // Copy metadata structure from end of packet
        memcpy( &metadata, ((uint8_t*) in_data) + size - sizeof(FrameMetadata), sizeof(FrameMetadata) );
        // Check if metadata is valid
        if(metadata.isValid()){
            
            // copy only frame data
            frameSize = metadata.frameSize;

            // frameSize must be less than or equal to size of the whole packet
            assert(frameSize <= size);

            //printf("Stream: %s (size: %d, frameSize: %d), metadata packet valid: w:%d, h:%d, t:%d, %6.3f\n", stream_name_.c_str(),size, frameSize, metadata.spec.width,metadata.spec.height, metadata.spec.type, metadata.getTimestamp());
            
            // set opt_metadata
            opt_metadata = metadata;

        } else {
            //printf("Stream: %s (size: %d), Metadata packet NOT valid\n",stream_name_.c_str(), size);
        }


        // set dimensions
        dimensions = streamInfo.getDimensionsForSize(frameSize);

        std::uint8_t* pData = (std::uint8_t*) in_data;

        // Regular copy (1 allocation only)
        data = std::make_shared<std::vector<std::uint8_t>>(pData, pData + frameSize);

        constructor_timer = Timer();
    }

    unsigned size()
    {
        return data->size();
    }

    const unsigned char* getData() const
    {
        return data->data();
    }

    std::string getDataAsString()
    {
        const unsigned char* data = getData();
        assert(data[size() - 1] == 0); // checking '\0'
        return reinterpret_cast<const char*>(&data[0]);
    }

    boost::optional<FrameMetadata> getMetadata(){
        return opt_metadata;
    }

    ObjectTracker getObjectTracker(){
        ObjectTracker ot_tracklets;
        assert(size() == sizeof(ObjectTracker));
        memcpy(&ot_tracklets, getData(), sizeof(ObjectTracker));
        return ot_tracklets;
    }

    boost::optional<FrameMetadata> opt_metadata;
    std::shared_ptr<std::vector<unsigned char>> data;
    std::string stream_name;
    std::vector<int> dimensions;
    int elem_size;

    Timer constructor_timer;
};
