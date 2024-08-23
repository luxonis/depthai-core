#pragma once

#include <vector>
#include <memory>

// project
#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/AudioHelpers.hpp"


namespace dai {

/**
 * AudioFrame message. Carries audio data and metadata.
 */
class AudioFrame : public Buffer {
   public:
    /**
     * Construct AudioFrame message.
     * Timestamp is set to now
     */
    AudioFrame() = default;
    AudioFrame(sf_count_t frames, unsigned int bitrate, unsigned int channels, int format);
    virtual ~AudioFrame() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::AudioFrame;
    };

	void setFrames(sf_count_t frames);
	void setBitrate(unsigned int bitrate);
	void setChannels(unsigned int channels);
	void setFormat(int format);

	sf_count_t getFrames() const;
	unsigned int getBitrate() const;
	unsigned int getChannels() const;
	int getFormat() const;

   private:
    sf_count_t frames;
    unsigned int bitrate;
    unsigned int channels;
    int format;

   public:
    DEPTHAI_SERIALIZE(AudioFrame, bitrate, channels, format);
};

}  // namespace dai
