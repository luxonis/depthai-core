#include "depthai/pipeline/node/AudioEncoder.hpp"

namespace dai {
namespace node {

AudioEncoder::AudioEncoder(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioEncoder, AudioEncoderProperties>(std::move(props)) { }
    
AudioEncoder::~AudioEncoder() {}

std::shared_ptr<AudioEncoder> AudioEncoder::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioEncoder>(shared_from_this());;
}

}
}
