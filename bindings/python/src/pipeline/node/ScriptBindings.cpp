#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Script.hpp"

void bind_script(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<ScriptProperties> scriptProperties(m, "ScriptProperties", DOC(dai, ScriptProperties));
    auto script = ADD_NODE(Script);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    scriptProperties.def_readwrite("scriptUri", &ScriptProperties::scriptUri, DOC(dai, ScriptProperties, scriptUri))
        .def_readwrite("scriptName", &ScriptProperties::scriptName, DOC(dai, ScriptProperties, scriptName))
        .def_readwrite("processor", &ScriptProperties::processor, DOC(dai, ScriptProperties, processor));

    script.def("getScriptPath", &dai::node::Script::getScriptPath, DOC(dai, node, Script, getScriptPath))
        .def_readonly("inputs", &Script::inputs)
        .def_readonly("outputs", &Script::outputs)
#ifdef DEPTHAI_SCRIPT_NODE_ADD_IO
        .def_property_readonly(
            "io",
            [](Node& n) -> py::object {
                auto dict = py::dict();
                for(auto& output : n.getOutputRefs()) {
                    dict[py::str(output->getName())] = output;
                }
                for(auto& input : n.getInputRefs()) {
                    if(dict.contains(py::str(input->getName()))) {
                        throw std::runtime_error("Input with name '" + input->getName() + "' already exists in the node");
                    }
                    dict[py::str(input->getName())] = input;
                }
                return dict;
            },
            py::return_value_policy::reference_internal)
#endif
        .def("setScriptPath", &Script::setScriptPath, DOC(dai, node, Script, setScriptPath))
        .def("setScript",
             py::overload_cast<const std::string&, const std::string&>(&Script::setScript),
             py::arg("script"),
             py::arg("name") = "",
             DOC(dai, node, Script, setScript))
        .def("setScript",
             py::overload_cast<const std::vector<std::uint8_t>&, const std::string&>(&Script::setScript),
             py::arg("data"),
             py::arg("name") = "",
             DOC(dai, node, Script, setScript, 2))
        .def("setScriptPath", &Script::setScriptPath, py::arg("path"), py::arg("name") = "", DOC(dai, node, Script, setScriptPath))
        .def("getScriptName", &Script::getScriptName, DOC(dai, node, Script, getScriptName))
        .def("setProcessor", &Script::setProcessor, DOC(dai, node, Script, setProcessor))
        .def("getProcessor", &Script::getProcessor, DOC(dai, node, Script, getProcessor));
    daiNodeModule.attr("Script").attr("Properties") = scriptProperties;
}
