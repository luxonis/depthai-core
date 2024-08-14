#include "depthai/pipeline/node/AudioMixer.hpp"

namespace dai {
namespace node {

AudioMixer::AudioMixer(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioMixer, AudioMixerProperties>(std::move(props)) { }

AudioMixer::~AudioMixer() {}

std::shared_ptr<AudioMixer> AudioMixer::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioMixer>(shared_from_this());;
}

}
}
