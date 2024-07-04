#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"

extern py::handle daiNodeModule;
extern py::object messageQueueException; // Needed to be able to catch in C++ after it's raised on the Python side

using namespace dai;
using namespace dai::node;

class PyThreadedHostNode : public NodeCRTP<ThreadedHostNode, PyThreadedHostNode> {
   public:
    void run() override {
        try {
            PYBIND11_OVERRIDE_PURE(void, ThreadedHostNode, run);
        } catch(py::error_already_set& e) {
            if(e.matches(messageQueueException)) {
                logger->trace("Caught MessageQueue exception in ThreadedHostNode::run");
            } else {
                throw;
            }
        }
    }
};

class PyHostNode : public NodeCRTP<HostNode, PyHostNode> {
   public:
    std::shared_ptr<Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        PYBIND11_OVERRIDE_PURE(std::shared_ptr<Buffer>, HostNode, processGroup, in);
    }
};

void bind_hostnode(pybind11::module& m, void* pCallstack){
    // declare upfront
    auto threadedHostNode =
        py::class_<ThreadedHostNode, PyThreadedHostNode, ThreadedNode, std::shared_ptr<ThreadedHostNode>>(daiNodeModule, "ThreadedHostNode", DOC(dai, node, ThreadedHostNode));
    auto hostNode = py::class_<HostNode, PyHostNode, ThreadedHostNode, std::shared_ptr<HostNode>>(daiNodeModule, "HostNode", DOC(dai, node, HostNode));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    threadedHostNode
        .def(py::init<>([]() {
            auto node = std::make_shared<PyThreadedHostNode>();
            getImplicitPipeline()->add(node);
            return node;
        }))
        .def("run", &ThreadedHostNode::run);

    hostNode
        .def(py::init([]() {
            auto node = std::make_shared<PyHostNode>();
            getImplicitPipeline()->add(node);
            return node;
        }))
        .def("processGroup", &HostNode::processGroup)
        .def_property_readonly(
            "inputs", [](HostNode& node) { return &node.inputs; }, py::return_value_policy::reference_internal)
        .def_readonly("out", &HostNode::out, DOC(dai, node, HostNode, out))
        .def("runSyncingOnHost", &HostNode::runSyncingOnHost, DOC(dai, node, HostNode, runSyncingOnHost))
        .def("runSyncingOnDevice", &HostNode::runSyncingOnDevice, DOC(dai, node, HostNode, runSyncingOnDevice))
        .def("sendProcessingToPipeline", &HostNode::sendProcessingToPipeline, DOC(dai, node, HostNode, sendProcessingToPipeline));

    py::exec(R"(
        def __init_subclass__(cls):
            import inspect
            members = dict(inspect.getmembers(cls))
            assert "process" in members, "Subclass of HostNode must define method 'process'"
            sig = inspect.signature(members["process"])
            assert list(sig.parameters.keys())[0] == "self", \
                'Please use "self" as the first parameter for process method'

            cls.input_desc = {}
            for name, param in sig.parameters.items():
                if name == "self": continue
                annotation = param.annotation
                if annotation == inspect.Parameter.empty:
                    annotation = None
                cls.input_desc[name] = annotation

            cls.output_desc = sig.return_annotation
            if cls.output_desc == inspect.Signature.empty:
                cls.output_desc = None

            def processGroup(self, messageGroup):
                return members["process"](self,
                    *(messageGroup[argname] for argname in cls.input_desc.keys()))
            cls.processGroup = processGroup

            def link_args(self, *args):
                assert len(args) == len(cls.input_desc), "Number of arguments doesn't match the `process` method" 
                for (name, type), arg in zip(cls.input_desc.items(), args):
                    if type is not None:
                        assert type.__name__.isalpha(), "Security check failed"
                        type_enum = eval(f"DatatypeEnum.{type.__name__}")
                        for hierarchy in arg.getPossibleDatatypes():
                            # I believe this check isn't sound nor complete
                            # However, nether does the original in canConnect
                            # I belive it would be more confusing to have two
                            # different behaviours than one incorrect
                            if type_enum == hierarchy.datatype: break
                            if isDatatypeSubclassOf(type_enum, hierarchy.datatype): break
                        else:
                            raise TypeError(f"Input '{name}' cannot be linked due to incompatible message types. Input type: {type_enum} Output type: {hierarchy.datatype}")
                    arg.link(self.inputs[name])
                return self
            cls.link_args = link_args

            def __init__(self, *args):
                node.HostNode.__init__(self)
                self.link_args(*args)
            if not hasattr(cls, "__init__"):
                cls.__init__ = __init__

        node.HostNode.__init_subclass__ = classmethod(__init_subclass__)
    )",
             m.attr("__dict__"));
}
