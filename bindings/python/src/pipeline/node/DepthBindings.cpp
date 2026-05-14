#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<Depth, DeviceNodeGroup>("Depth", DOC(dai, node, Depth));
    // Replace default factory: pipeline.create(Depth, Algorithm) or algorithm= keyword -> Pipeline::create<Depth>(...).
    pyNodeCreateMap.back().second = [](dai::Pipeline& p, py::object /*class_*/, const py::args& args, const py::kwargs& kwargs) -> std::shared_ptr<dai::Node> {
        py::object algKw = kwargs.attr("get")("algorithm", py::none());
        if(!algKw.is_none()) {
            return p.create<Depth>(algKw.cast<Depth::Algorithm>());
        }
        if(args.size() >= 1) {
            return p.create<Depth>(args[0].cast<Depth::Algorithm>());
        }
        return p.create<Depth>();
    };

    // Run nested binders first so Depth::Algorithm is registered before any method uses it as a default type.
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::enum_<Depth::Algorithm> depthAlgorithm(node, "Algorithm");
    depthAlgorithm.value("AUTO", Depth::Algorithm::AUTO)
        .value("STEREO", Depth::Algorithm::STEREO)
        .value("NEURAL", Depth::Algorithm::NEURAL)
        .value("NEURAL_ASSISTED_STEREO", Depth::Algorithm::NEURAL_ASSISTED_STEREO)
        .value("TOF", Depth::Algorithm::TOF)
        .value("GPU_STEREO", Depth::Algorithm::GPU_STEREO);

    node.def("getAlgorithm", &Depth::getAlgorithm)
        .def(
            "build",
            [](Depth& self, py::object fps) {
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                return self.build(std::move(optFps));
            },
            py::arg("fps") = py::none())
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal);
}
