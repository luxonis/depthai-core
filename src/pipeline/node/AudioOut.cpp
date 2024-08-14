#include "depthai/pipeline/node/AudioOut.hpp"

namespace dai {
namespace node {

AudioOut::AudioOut(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioOut, AudioOutProperties>(std::move(props)) { }

AudioOut::~AudioOut() {}

std::shared_ptr<AudioOut> AudioOut::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioOut>(shared_from_this());;
}

}
}
