#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/AudioProcProperties.hpp>

#include "depthai/pipeline/datatype/AudioInConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief AudioProc node.
 */
class AudioProc : public NodeCRTP<Node, AudioProc, AudioProcProperties> {
   public:
    constexpr static const char* NAME = "AudioProc";

    AudioProc(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    AudioProc(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use for audio/mic.
     */
    AudioInConfig initialConfig;

    /**
     * Input AudioInConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::AudioInConfig, false}}};

    /**
     * Audio input frames.
     * Default queue is blocking with size set by 'setNumFramesPool' (4).
     */
    Input input{*this, "input", Input::Type::SReceiver, true, 4, true, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Audio input reference frames.
     * Default queue is blocking with size set by 'setNumFramesPool' (4).
     */
    Input reference{*this, "reference", Input::Type::SReceiver, true, 4, true, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries processed audio data.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // node properties
    /**
     * Set number of frames in pool
     * @param frames Number of pool frames
     */
    void setNumFramesPool(int frames);

    /**
     * Get number of frames in pool
     * @returns Number of pool frames
     */
    int getNumFramesPool() const;

    /**
     * Set sample rate used for processing.
     * @param frames Sample rate in Hz
     */
    void setSampleRate(int rate);

    /**
     * Get sample rate used for processing.
     * @returns Sample rate in Hz
     */
    int getSampleRate() const;

};

}  // namespace node
}  // namespace dai
