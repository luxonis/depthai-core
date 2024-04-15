#include "NodeBindings.hpp"

#include <memory>

#include "Common.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/NodeGroup.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"

// Libraries
#include "hedley/hedley.h"

// pybind11
#include "pybind11/stl_bind.h"

// Map of python node classes and call to pipeline to create it
std::vector<std::pair<py::handle, std::function<std::shared_ptr<dai::Node>(dai::Pipeline&, py::object class_)>>> pyNodeCreateMap;
py::handle daiNodeModule;

std::vector<std::pair<py::handle, std::function<std::shared_ptr<dai::Node>(dai::Pipeline&, py::object class_)>>> NodeBindings::getNodeCreateMap(){
    return pyNodeCreateMap;
}

// Bind map - without init method
template <typename Map, typename holder_type = std::unique_ptr<Map>, typename... Args>
py::class_<Map, holder_type> bindNodeMap(py::handle scope, const std::string &name, Args&&... args) {
    using namespace py;
    using KeyType = typename Map::key_type;
    using MappedType = typename Map::mapped_type;
    using Class_ = class_<Map, holder_type>;

    // If either type is a non-module-local bound type then make the map binding non-local as well;
    // otherwise (e.g. both types are either module-local or converting) the map will be
    // module-local.
    auto tinfo = py::detail::get_type_info(typeid(MappedType));
    bool local = !tinfo || tinfo->module_local;
    if (local) {
        tinfo = py::detail::get_type_info(typeid(KeyType));
        local = !tinfo || tinfo->module_local;
    }

    Class_ cl(scope, name.c_str(), pybind11::module_local(local), std::forward<Args>(args)...);

    // Register stream insertion operator (if possible)
    detail::map_if_insertion_operator<Map, Class_>(cl, name);

    cl.def("__bool__",
        [](const Map &m) -> bool { return !m.empty(); },
        "Check whether the map is nonempty"
    );

    cl.def("__iter__",
           [](Map &m) { return make_key_iterator(m.begin(), m.end()); },
           keep_alive<0, 1>() /* Essential: keep list alive while iterator exists */
    );

    cl.def("items",
           [](Map &m) { return make_iterator(m.begin(), m.end()); },
           keep_alive<0, 1>() /* Essential: keep list alive while iterator exists */
    );

    // Warning - overload order matters!
   // Modified __getitem__. Uses operator[string] underneath
    cl.def("__getitem__",
        [](Map &m, const std::string &k) -> MappedType & {
            return m[k];
        },
        return_value_policy::reference_internal // ref + keepalive
    );
    // Modified __getitem__. Uses operator[pair<string,string>] underneath
    cl.def("__getitem__",
        [](Map &m, const KeyType &k) -> MappedType & {
            return m[k];
        },
        return_value_policy::reference_internal // ref + keepalive
    );

    // Warning - overload order matters!
    cl.def("__contains__",
        [](Map &m, const std::string &k) -> bool {
            auto it = m.find({m.name, k});
            if (it == m.end())
              return false;
           return true;
        }
    );
    cl.def("__contains__",
        [](Map &m, const KeyType &k) -> bool {
            auto it = m.find(k);
            if (it == m.end())
              return false;
           return true;
        }
    );

    // Assignment provided only if the type is copyable
    detail::map_assignment<Map, Class_>(cl);

    // Warning - overload order matters!
    cl.def("__delitem__",
           [](Map &m, const std::string &k) {
               auto it = m.find({m.name, k});
               if (it == m.end())
                   throw key_error();
               m.erase(it);
           }
    );
    cl.def("__delitem__",
           [](Map &m, const KeyType &k) {
               auto it = m.find(k);
               if (it == m.end())
                   throw key_error();
               m.erase(it);
           }
    );

    cl.def("__len__", &Map::size);

    return cl;
}


void bind_xlinkin(pybind11::module& m, void* pCallstack);
void bind_xlinkout(pybind11::module& m, void* pCallstack);
// void bind_benchmark(pybind11::module& m, void* pCallstack);
void bind_colorcamera(pybind11::module& m, void* pCallstack);
void bind_camera(pybind11::module& m, void* pCallstack);
void bind_monocamera(pybind11::module& m, void* pCallstack);
// void bind_stereodepth(pybind11::module& m, void* pCallstack);
// void bind_neuralnetwork(pybind11::module& m, void* pCallstack);
// void bind_videoencoder(pybind11::module& m, void* pCallstack);
// void bind_imagemanip(pybind11::module& m, void* pCallstack);
// void bind_warp(pybind11::module& m, void* pCallstack);
// void bind_spiout(pybind11::module& m, void* pCallstack);
// void bind_spiin(pybind11::module& m, void* pCallstack);
// void bind_detectionnetwork(pybind11::module& m, void* pCallstack);
// void bind_systemlogger(pybind11::module& m, void* pCallstack);
// void bind_script(pybind11::module& m, void* pCallstack);
// void bind_spatiallocationcalculator(pybind11::module& m, void* pCallstack);
// void bind_spatialdetectionnetwork(pybind11::module& m, void* pCallstack);
// void bind_objecttracker(pybind11::module& m, void* pCallstack);
// void bind_imu(pybind11::module& m, void* pCallstack);
// void bind_edgedetector(pybind11::module& m, void* pCallstack);
// void bind_featuretracker(pybind11::module& m, void* pCallstack);
// void bind_apriltag(pybind11::module& m, void* pCallstack);
// void bind_detectionparser(pybind11::module& m, void* pCallstack);
// void bind_uvc(pybind11::module& m, void* pCallstack);
// void bind_tof(pybind11::module& m, void* pCallstack);
// void bind_sync(pybind11::module& m, void* pCallstack);
// void bind_messagedemux(pybind11::module& m, void* pCallstack);
void bind_hostnode(pybind11::module& m, void* pCallstack);

void NodeBindings::addToCallstack(std::deque<StackFunction>& callstack) {
    // Bind Node et al
    callstack.push_front(NodeBindings::bind);

    // Bind all other nodes
    callstack.push_front(bind_xlinkin);
    callstack.push_front(bind_xlinkout);
    // callstack.push_front(bind_benchmark);
    callstack.push_front(bind_colorcamera);
    // callstack.push_front(bind_camera);
    callstack.push_front(bind_monocamera);
    // callstack.push_front(bind_stereodepth);
    // callstack.push_front(bind_neuralnetwork);
    // callstack.push_front(bind_videoencoder);
    // callstack.push_front(bind_imagemanip);
    // callstack.push_front(bind_warp);
    // callstack.push_front(bind_spiout);
    // callstack.push_front(bind_spiin);
    // callstack.push_front(bind_detectionnetwork);
    // callstack.push_front(bind_systemlogger);
    // callstack.push_front(bind_script);
    // callstack.push_front(bind_spatiallocationcalculator);
    // callstack.push_front(bind_spatialdetectionnetwork);
    // callstack.push_front(bind_objecttracker);
    // callstack.push_front(bind_imu);
    // callstack.push_front(bind_edgedetector);
    // callstack.push_front(bind_featuretracker);
    // callstack.push_front(bind_apriltag);
    // callstack.push_front(bind_detectionparser);
    // callstack.push_front(bind_uvc);
    // callstack.push_front(bind_tof);
    // callstack.push_front(bind_sync);
    // callstack.push_front(bind_messagedemux);
    callstack.push_front(bind_hostnode);
}

void NodeBindings::bind(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace std::chrono;

    //// Bindings for actual nodes
    // Move properties into nodes and nodes under 'node' submodule
    daiNodeModule = m.def_submodule("node");

    // Properties
    py::class_<Node, std::shared_ptr<Node>> pyNode(m, "Node", DOC(dai, Node));

    py::class_<Node::Input, MessageQueue, std::shared_ptr<Node::Input>> pyInput(pyNode, "Input", DOC(dai, Node, Input));
    py::enum_<Node::Input::Type> nodeInputType(pyInput, "Type");
    py::class_<Node::Output> pyOutput(pyNode, "Output", DOC(dai, Node, Output));
    py::enum_<Node::Output::Type> nodeOutputType(pyOutput, "Type");
    py::class_<Properties, std::shared_ptr<Properties>> pyProperties(m, "Properties", DOC(dai, Properties));
    py::class_<Node::DatatypeHierarchy> nodeDatatypeHierarchy(pyNode, "DatatypeHierarchy", DOC(dai, Node, DatatypeHierarchy));

    // Node::Id bindings
    py::class_<Node::Id>(pyNode, "Id", "Node identificator. Unique for every node on a single Pipeline");
    // Node::Connection bindings
    py::class_<Node::Connection> nodeConnection(pyNode, "Connection", DOC(dai, Node, Connection));
    // Node::InputMap bindings
    bindNodeMap<Node::InputMap>(pyNode, "InputMap");
    // Node::OutputMap bindings
    bindNodeMap<Node::OutputMap>(pyNode, "OutputMap");

    //     // NodeGroup
    py::class_<NodeGroup, Node, std::shared_ptr<NodeGroup>> pyNodeGroup(m, "NodeGroup", DOC(dai, NodeGroup));

    // // Threaded & Device nodes
    py::class_<ThreadedNode, Node, std::shared_ptr<ThreadedNode>> pyThreadedNode(m, "ThreadedNode", DOC(dai, ThreadedNode));
    py::class_<DeviceNode, ThreadedNode, std::shared_ptr<DeviceNode>> pyDeviceNode(m, "DeviceNode", DOC(dai, DeviceNode));

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


    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////
    // Node Bindings after properties, so types are resolved
    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////

    // Base 'Node' class binding

    nodeDatatypeHierarchy
        .def(py::init<DatatypeEnum, bool>())
        .def_readwrite("datatype", &Node::DatatypeHierarchy::datatype)
        .def_readwrite("descendants", &Node::DatatypeHierarchy::descendants)
    ;

    // Node::Input bindings
    nodeInputType
        .value("SReceiver", Node::Input::Type::SReceiver)
        .value("MReceiver", Node::Input::Type::MReceiver)
    ;

    pyInput
        .def(
            py::init([](Node& parent,
                        const std::string& name,
                        const std::string& group,
                        bool blocking,
                        int queueSize,
                        std::vector<Node::DatatypeHierarchy> types,
                        bool waitForMessage) {
                return std::unique_ptr<Node::Input>(new Node::Input(
                    parent,
                    {.name = name, .group = group, .blocking = blocking, .queueSize = queueSize, .types = std::move(types), .waitForMessage = waitForMessage}));
            }),
            py::arg("parent"),
            py::arg("name") = Node::InputDescription{}.name,
            py::arg("group") = Node::InputDescription{}.group,
            py::arg("blocking") = Node::InputDescription{}.blocking,
            py::arg("queueSize") = Node::InputDescription{}.queueSize,
            py::arg("types") = Node::InputDescription{}.types,
            py::arg("waitForMessage") = Node::InputDescription{}.waitForMessage,
            py::keep_alive<2, 1>())
        .def_readwrite("possibleDatatypes", &Node::Input::possibleDatatypes, DOC(dai, Node, Input, possibleDatatypes))
        .def("getParent",
             static_cast<const Node& (Node::Input::*)() const>(&Node::Input::getParent),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, Input, getParent))
        .def("getParent",
             static_cast<Node& (Node::Input::*)()>(&Node::Input::getParent),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, Input, getParent))
        .def("setWaitForMessage", &Node::Input::setWaitForMessage, py::arg("waitForMessage"), DOC(dai, Node, Input, setWaitForMessage))
        .def("getWaitForMessage", &Node::Input::getWaitForMessage, DOC(dai, Node, Input, getWaitForMessage))
        .def("setReusePreviousMessage", &Node::Input::setReusePreviousMessage, py::arg("reusePreviousMessage"), DOC(dai, Node, Input, setReusePreviousMessage))
        .def("getReusePreviousMessage", &Node::Input::getReusePreviousMessage, DOC(dai, Node, Input, getReusePreviousMessage));

    // Node::Output bindings
    nodeOutputType
        .value("MSender", Node::Output::Type::MSender)
        .value("SSender", Node::Output::Type::SSender)
    ;
    pyOutput
        .def(py::init([](Node& parent, const std::string& name, const std::string& group, std::vector<Node::DatatypeHierarchy> types) {
                 return std::unique_ptr<Node::Output>(new Node::Output(parent, {.name = name, .group = group, .types = std::move(types)}));
             }),
             py::arg("parent"),
             py::arg("name") = Node::OutputDescription{}.name,
             py::arg("group") = Node::OutputDescription{}.group,
             py::arg("possibleDatatypes") = Node::OutputDescription{}.types,
             py::keep_alive<2, 1>())
        .def("getPossibleDatatypes", &Node::Output::getPossibleDatatypes, DOC(dai, Node, Output, getPossibleDatatypes))
        .def("getParent",
             static_cast<const Node& (Node::Output::*)() const>(&Node::Output::getParent),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, Output, getParent))
        .def("getParent",
             static_cast<Node& (Node::Output::*)()>(&Node::Output::getParent),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, Output, getParent))
        .def("isSamePipeline", &Node::Output::isSamePipeline, py::arg("input"), DOC(dai, Node, Output, isSamePipeline))
        .def("canConnect", &Node::Output::canConnect, py::arg("input"), DOC(dai, Node, Output, canConnect))
        .def("link", static_cast<void (Node::Output::*)(Node::Input&)>(&Node::Output::link), py::arg("input"), DOC(dai, Node, Output, link))
        .def("unlink", static_cast<void (Node::Output::*)(Node::Input&)>(&Node::Output::unlink), py::arg("input"), DOC(dai, Node, Output, unlink))
        .def("send", &Node::Output::send, py::arg("msg"), DOC(dai, Node, Output, send))
        .def("trySend", &Node::Output::trySend, py::arg("msg"), DOC(dai, Node, Output, trySend));

    nodeConnection
        .def_readwrite("outputId", &Node::Connection::outputId, DOC(dai, Node, Connection, outputId))
        .def_readwrite("outputName", &Node::Connection::outputName, DOC(dai, Node, Connection, outputName))
        .def_readwrite("outputGroup", &Node::Connection::outputGroup, DOC(dai, Node, Connection, outputGroup))
        .def_readwrite("inputId", &Node::Connection::inputId, DOC(dai, Node, Connection, inputId))
        .def_readwrite("inputName", &Node::Connection::inputName, DOC(dai, Node, Connection, inputName))
        .def_readwrite("inputGroup", &Node::Connection::inputGroup, DOC(dai, Node, Connection, inputGroup))
    ;

    pyNode.def_readonly("id", &Node::id, DOC(dai, Node, id))
        .def("getName", &Node::getName, DOC(dai, Node, getName))
        .def("getOutputs", &Node::getOutputs, DOC(dai, Node, getOutputs))
        .def("getInputs", &Node::getInputs, DOC(dai, Node, getInputs))
        .def("stopPipeline", &Node::stopPipeline, DOC(dai, Node, stopPipeline), py::call_guard<py::gil_scoped_release>())
        .def("getOutputRefs",
             static_cast<std::vector<Node::Output*> (Node::*)()>(&Node::getOutputRefs),
             DOC(dai, Node, getOutputRefs),
             py::return_value_policy::reference_internal)
        .def("getInputRefs",
             static_cast<std::vector<Node::Input*> (Node::*)()>(&Node::getInputRefs),
             DOC(dai, Node, getInputRefs),
             py::return_value_policy::reference_internal)
        .def("getOutputRefs",
             static_cast<std::vector<const Node::Output*> (Node::*)() const>(&Node::getOutputRefs),
             DOC(dai, Node, getOutputRefs),
             py::return_value_policy::reference_internal)
        .def("getInputRefs",
             static_cast<std::vector<const Node::Input*> (Node::*)() const>(&Node::getInputRefs),
             DOC(dai, Node, getInputRefs),
             py::return_value_policy::reference_internal)
        .def("getInputMapRefs",
             static_cast<std::vector<Node::InputMap*> (Node::*)()>(&Node::getInputMapRefs),
             DOC(dai, Node, getInputMapRefs),
             py::return_value_policy::reference_internal)
        .def("getOutputMapRefs",
             static_cast<std::vector<Node::OutputMap*> (Node::*)()>(&Node::getOutputMapRefs),
             DOC(dai, Node, getOutputMapRefs),
             py::return_value_policy::reference_internal)
        .def("getParentPipeline", py::overload_cast<>(&Node::getParentPipeline), DOC(dai, Node, getParentPipeline))
        .def("getParentPipeline", py::overload_cast<>(&Node::getParentPipeline, py::const_), DOC(dai, Node, getParentPipeline))
        .def("getAssetManager",
             static_cast<const AssetManager& (Node::*)() const>(&Node::getAssetManager),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, getAssetManager))
        .def("getAssetManager",
             static_cast<AssetManager& (Node::*)()>(&Node::getAssetManager),
             py::return_value_policy::reference_internal,
             DOC(dai, Node, getAssetManager));
        // .def_property_readonly(  // TODO - This casting of the inputs/outputs might be illegal / cause bad behavior
        //     "io",
        //     [](Node& n) -> py::object {
        //         auto dict = py::dict();
        //         for(auto& output : n.getOutputRefs()) {
        //             // TODO - Revisit, pybind might try to release the output when refcount goes to zero
        //             dict[py::str(output->name)] = output;
        //         }
        //         for(auto& input : n.getInputRefs()) {
        //             // TODO - Revisit, pybind might try to release the input when refcount goes to zero
        //             dict[py::str(input->getName())] = input;
        //         }
        //         return dict;
        //     },
        //     py::return_value_policy::reference_internal);

    // TODO(themarpe) - refactor, threaded node could be separate from Node
    pyThreadedNode
        .def("trace", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->trace(msg);
        })
        .def("debug", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->debug(msg);
        })
        .def("info", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->info(msg);
        })
        .def("warn", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->warn(msg);
        })
        .def("error", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->error(msg);
        })
        .def("critical", [](dai::ThreadedNode& node, const std::string& msg) {
            node.logger->critical(msg);
        });
}

