#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

extern py::handle daiNodeModule;
extern py::object messageQueueException;  // Needed to be able to catch in C++ after it's raised on the Python side

using namespace dai;
using namespace dai::node;

class PyThreadedHostNode : public NodeCRTP<ThreadedHostNode, PyThreadedHostNode> {
   public:
    void run() override {
        try {
            PYBIND11_OVERRIDE_PURE(void, ThreadedHostNode, run);
        } catch(py::error_already_set& e) {
            if(e.matches(messageQueueException)) {
                pimpl->logger->trace("Caught MessageQueue exception in ThreadedHostNode::run");
            } else {
                throw;
            }
        }
    }
    void onStart() override {
        PYBIND11_OVERRIDE(void, ThreadedHostNode, onStart);
    }
    void onStop() override {
        PYBIND11_OVERRIDE(void, ThreadedHostNode, onStop);
    }
};

class PyHostNode : public NodeCRTP<HostNode, PyHostNode> {
   public:
    std::shared_ptr<Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        PYBIND11_OVERRIDE_PURE(std::shared_ptr<Buffer>, HostNode, processGroup, in);
    }
    void onStart() override {
        PYBIND11_OVERRIDE(void, HostNode, onStart);
    }
    void onStop() override {
        PYBIND11_OVERRIDE(void, HostNode, onStop);
    }
};

void bind_hostnode(pybind11::module& m, void* pCallstack) {
    // declare upfront
    auto threadedHostNode = py::class_<ThreadedHostNode, PyThreadedHostNode, ThreadedNode, std::shared_ptr<ThreadedHostNode>>(
        daiNodeModule, "ThreadedHostNode", DOC(dai, node, ThreadedHostNode));
    auto hostNode = py::class_<HostNode, PyHostNode, ThreadedHostNode, std::shared_ptr<HostNode>>(daiNodeModule, "HostNode", DOC(dai, node, HostNode));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
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
        .def("run", &ThreadedHostNode::run)
        .def("onStart", &ThreadedHostNode::onStart)
        .def("onStop", &ThreadedHostNode::onStop)
        .def(
            "createInput",
            [](ThreadedHostNode& node,
               std::string name,
               std::string group,
               bool blocking,
               int queueSize,
               std::vector<Node::DatatypeHierarchy> types,
               bool waitForMessage) {
                return std::make_shared<Node::Input>(node, Node::InputDescription{name, group, blocking, queueSize, types, waitForMessage});
            },
            py::arg("name") = Node::InputDescription{}.name,
            py::arg("group") = Node::InputDescription{}.group,
            py::arg("blocking") = Node::InputDescription{}.blocking,
            py::arg("queueSize") = Node::InputDescription{}.queueSize,
            py::arg("types") = Node::InputDescription{}.types,
            py::arg("waitForMessage") = Node::InputDescription{}.waitForMessage,
            py::keep_alive<1, 0>())
        .def(
            "createOutput",
            [](ThreadedHostNode& node, std::string name, std::string group, std::vector<Node::DatatypeHierarchy> types) {
                return std::make_shared<Node::Output>(node, Node::OutputDescription{name, group, types});
            },
            py::arg("name") = Node::OutputDescription{}.name,
            py::arg("group") = Node::OutputDescription{}.group,
            py::arg("possibleDatatypes") = Node::OutputDescription{}.types,
            py::keep_alive<1, 0>());

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
        .def("sendProcessingToPipeline", &HostNode::sendProcessingToPipeline, DOC(dai, node, HostNode, sendProcessingToPipeline))
        .def("onStart", &HostNode::onStart)
        .def("onStop", &HostNode::onStop);

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

        # Create a subnode as part of this user node
        def createSubnode(self, class_, *args, **kwargs):
            pipeline = self.getParentPipeline()
            child_node = pipeline.create(class_, *args, **kwargs)
            pipeline.remove(child_node) # necessary so that the node is only registered once
            self.add(child_node)
            return child_node

        node.HostNode.createSubnode = createSubnode
        node.ThreadedHostNode.createSubnode = createSubnode
    )",
             m.attr("__dict__"));
}
