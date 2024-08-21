#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

// project
#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/AudioHelpers.hpp"


namespace dai {

/**
 * AudioFrame message. Carries audio data and metadata.
 */
class AudioFrame : public Buffer, public dai::audio::AudioFile {
   public:
    /**
     * Construct AudioFrame message.
     * Timestamp is set to now
     */
    AudioFrame() = default;
    AudioFrame(SF_INFO info);
    virtual ~AudioFrame() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::AudioFrame;
    };

   public:
	sf_count_t getSize();
	sf_count_t seek(sf_count_t offset, int whence);
	sf_count_t read(void* ptr, sf_count_t count);
	sf_count_t write(const void* ptr, sf_count_t count);
	sf_count_t tell();

   private:
    sf_count_t position;
    std::mutex mtx;
    std::condition_variable cv;
    bool dataReady;
    SF_VIRTUAL_IO virtualIo;

   public:
//    DEPTHAI_SERIALIZE(AudioFrame); //, position, mtx, cv, dataReady);
};

}  // namespace dai
