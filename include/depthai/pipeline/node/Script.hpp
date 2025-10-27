#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai/properties/ScriptProperties.hpp>

namespace dai {
namespace node {

class Script : public DeviceNodeCRTP<DeviceNode, Script, ScriptProperties> {
   public:
    constexpr static const char* NAME = "Script";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   private:
    std::filesystem::path scriptPath;

   public:
    /**
     * Inputs to Script node. Can be accessed using subscript operator (Eg: inputs['in1'])
     * By default inputs are set to blocking with queue size 8
     */
    InputMap inputs{*this, "io", {DEFAULT_NAME, DEFAULT_GROUP, true, 8, {{{DatatypeEnum::Buffer, true}}}, false}};

    /**
     * Outputs from Script node. Can be accessed subscript operator (Eg: outputs['out1'])
     */
    OutputMap outputs{*this, "io", {DEFAULT_NAME, DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Specify local filesystem path to load the script
     * @param path Filesystem path to load the script
     * @param name Optionally set a name of this script, otherwise the name defaults to the path
     */
    void setScriptPath(const std::filesystem::path& path, const std::string& name = "");

    /**
     * Sets script data to be interpreted
     * @param script Script string to be interpreted
     * @param name Optionally set a name of this script
     */
    void setScript(const std::string& script, const std::string& name = "");

    /**
     * Sets script data to be interpreted
     * @param data Binary data that represents the script to be interpreted
     * @param name Optionally set a name of this script
     */
    void setScript(const std::vector<std::uint8_t>& data, const std::string& name = "");

    /**
     * @brief Get filesystem path from where script was loaded.
     *
     * @return std::filesystem::path from where script was loaded, otherwise returns empty path
     */
    std::filesystem::path getScriptPath() const;

    /**
     * @brief Get the script name in utf-8.
     *
     * When name set with setScript() or setScriptPath(), returns that name.
     * When script loaded with setScriptPath() with name not provided, returns the utf-8 string of that path.
     * Otherwise, returns "<script>"
     *
     * @return std::string of script name in utf-8
     */
    std::string getScriptName() const;

    /**
     * Set on which processor the script should run
     * @param type Processor type - Leon CSS or Leon MSS
     */
    void setProcessor(ProcessorType type);

    /**
     * Get on which processor the script should run
     * @returns Processor type - Leon CSS or Leon MSS
     */
    ProcessorType getProcessor() const;

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
