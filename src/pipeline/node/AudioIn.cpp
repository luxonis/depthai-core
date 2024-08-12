#include "depthai/pipeline/node/AudioIn.hpp"

#include <alsa/asoundlib.h>

namespace dai {
namespace node {

AudioIn::AudioIn(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, AudioIn, AudioInProperties>(std::move(props)) { }

std::shared_ptr<AudioIn> AudioIn::build() {
	isBuild = true;

	return std::static_pointer_cast<AudioIn>(shared_from_this());;
}

}
}
