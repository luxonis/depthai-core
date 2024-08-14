#include "depthai/pipeline/node/AudioIn.hpp"

namespace dai {
namespace node {

AudioIn::AudioIn(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>(std::move(props)) { }

AudioIn::~AudioIn() {}

std::shared_ptr<AudioIn> AudioIn::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioIn>(shared_from_this());;
}

}
}
