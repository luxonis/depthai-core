#include "depthai/pipeline/node/AudioReplay.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/AudioHelpers.hpp"

#include <spdlog/async_logger.h>
#include <fstream>
#include <vector>

namespace dai {
namespace node {

void AudioReplay::run() {
	std::string pathStr = sourceFile.string();
	dai::audio::AudioFile file(pathStr.c_str(), SFM_READ);
	SF_INFO info = file.getInfo();

	std::vector<uint8_t> audioData;
	sf_count_t durationFrames = static_cast<sf_count_t>((1.0 / fps) * info.samplerate);

	if(info.format & SF_FORMAT_PCM_S8) {
		format = SF_FORMAT_PCM_S8;
		audioData.resize(durationFrames * info.channels * (16 / 8));
	} else 	if(info.format & SF_FORMAT_PCM_16) {
		format = SF_FORMAT_PCM_16;
		audioData.resize(durationFrames * info.channels * (16 / 8));
	} else 	if(info.format & SF_FORMAT_PCM_24) {
		format = SF_FORMAT_PCM_24;
		audioData.resize(durationFrames * info.channels * (32 / 8));
	} else 	if(info.format & SF_FORMAT_PCM_32) {
		format = SF_FORMAT_PCM_32;
		audioData.resize(durationFrames * info.channels * (32 / 8));
	} 
	

	std::cout << "Duration frames: " << durationFrames << std::endl;

	bool done = false;
	while (isRunning() && !done) {
		std::shared_ptr<Buffer> buf = std::make_shared<Buffer>();

		sf_count_t framesRead;
		switch(format) {
			case SF_FORMAT_PCM_S8:
			case SF_FORMAT_PCM_16:
				framesRead = file.readFrame((short*)audioData.data(), durationFrames);
				break;
			case SF_FORMAT_PCM_24:
			case SF_FORMAT_PCM_32:
				framesRead = file.readFrame((int*)audioData.data(), durationFrames);
				break;
		}
		std::cout << "Read frames: " << framesRead << std::endl;
		if (framesRead <= 0) {
			if (file.getError() == SF_ERR_NO_ERROR) {
				if(loop) {
					file.seek(0, SEEK_SET);
					continue;
				} else {
					audioData.resize(0);
					done = true;
				}
			} else {
				logger->error("AudioReplay {}: Failed to read audio file", __func__);
				done = true;
				break;
			}
		}

		std::cout << "Set data" << std::endl;
		buf->setData(audioData);
		std::cout << "Sending..." << std::endl;
		out.send(buf);

		std::cout << "Waiting" << std::endl;
		// Wait for the next period (1 second in this case)
		std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
	}
}

AudioReplay& AudioReplay::setSourceFile(const std::filesystem::path& sourceFile) {
    this->sourceFile = sourceFile;
    return *this;
}
AudioReplay& AudioReplay::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}
    
AudioReplay& AudioReplay::setFps(int fps) {
    this->fps = fps;
    return *this;
}

std::filesystem::path AudioReplay::getSourceFile() const {
    return sourceFile;
}

bool AudioReplay::getLoop() const {
    return loop;
}

int AudioReplay::getFps() const {
    return fps;
}

int AudioReplay::getFormat() const {
    return format;
}

}
}
