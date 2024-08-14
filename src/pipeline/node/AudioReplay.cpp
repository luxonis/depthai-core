#include "depthai/pipeline/node/AudioReplay.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

#include <fstream>
#include <vector>

namespace dai {
namespace node {

void AudioReplay::run() {
	std::ifstream file(sourceFile, std::ios::binary);

	if (file.bad() || file.eof()) {
		return;
	}

	// Stop eating new lines in binary mode!!!
	file.unsetf(std::ios::skipws);

	std::streampos fileSize;

	file.seekg(0, std::ios::end);
	fileSize = file.tellg();
	file.seekg(0, std::ios::beg);

	std::vector<uint8_t> fileData;
	fileData.reserve(fileSize);

	fileData.insert(fileData.begin(),
			std::istream_iterator<uint8_t>(file),
			std::istream_iterator<uint8_t>());

	std::shared_ptr<Buffer> buf = std::make_shared<Buffer>();
	buf->setData(fileData);

	do {
		out.send(buf);
	} while(isRunning() && loop);
}

AudioReplay& AudioReplay::setSourceFile(const std::filesystem::path& sourceFile) {
    this->sourceFile = sourceFile;
    return *this;
}
AudioReplay& AudioReplay::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

std::filesystem::path AudioReplay::getSourceFile() const {
    return sourceFile;
}

bool AudioReplay::getLoop() const {
    return loop;
}

}
}
