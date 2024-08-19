#include "depthai/pipeline/node/AudioReplay.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/AudioHelpers.hpp"

#include <fstream>
#include <vector>

namespace dai {
namespace node {

void AudioReplay::run() {
	dai::audio::AudioFile file(sourceFile.c_str(), SFM_READ);
	SF_INFO info = file.getInfo();

	std::vector<uint8_t> audioData(fps * info.channels * sizeof(int));

	sf_count_t durationFrames = static_cast<sf_count_t>((1.0 / fps) * info.samplerate);

	while (isRunning()) {
		std::shared_ptr<Buffer> buf = std::make_shared<Buffer>();

		sf_count_t framesRead = file.readFrame((int*)audioData.data(), durationFrames);
		if (framesRead <= 0) {
			//logger->warn("AudioReplay {}: End of file or error reading audio file", __func__);
			break;
		}

		buf->setData(audioData);
		out.send(buf);

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

}
}
