#pragma once

#include <algorithm>
#include <condition_variable>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>

// project
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/AssetManager.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/utility/JoiningThread.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/copyable_unique_ptr.hpp"

// depthai
#include "depthai/capabilities/Capability.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/properties/Properties.hpp"

// libraries
#include <optional>

namespace dai {
// fwd declare Pipeline
class Pipeline;
class PipelineImpl;

// fwd declare input queue class
class InputQueue;

/**
 * @brief Abstract Node
 */
class Node : public std::enable_shared_from_this<Node> {
    friend class Pipeline;
    friend class PipelineImpl;
    friend class Device;

   public:
    // Nodes must always be managed
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;
    Node(Node&&) = delete;
    Node& operator=(Node&&) = delete;

    /// Node identificator. Unique for every node on a single Pipeline
    using Id = std::int64_t;
    struct Connection;
    struct ConnectionInternal;
    // fwd declare classes
    class Input;
    class Output;
    class InputMap;
    class OutputMap;

   protected:
    std::vector<Output*> outputRefs;
    std::vector<Input*> inputRefs;
    std::vector<OutputMap*> outputMapRefs;
    std::vector<InputMap*> inputMapRefs;
    std::vector<std::shared_ptr<Node>*> nodeRefs;

    // helpers for setting refs
    void setOutputRefs(std::initializer_list<Output*> l);
    void setOutputRefs(Output* outRef);
    void setInputRefs(std::initializer_list<Input*> l);
    void setInputRefs(Input* inRef);
    void setOutputMapRefs(std::initializer_list<OutputMap*> l);
    void setOutputMapRefs(OutputMap* outMapRef);
    void setInputMapRefs(std::initializer_list<InputMap*> l);
    void setInputMapRefs(InputMap* inMapRef);
    void setNodeRefs(std::initializer_list<std::pair<std::string, std::shared_ptr<Node>*>> l);
    void setNodeRefs(std::pair<std::string, std::shared_ptr<Node>*> nodeRef);
    void setNodeRefs(std::string alias, std::shared_ptr<Node>* nodeRef);

    // For record and replay
    virtual bool isSourceNode() const;
    virtual utility::NodeRecordParams getNodeRecordParams() const;
    virtual Output& getRecordOutput();
    virtual Input& getReplayInput();

    template <typename T>
    class Subnode {
        std::shared_ptr<Node> node;

       public:
        Subnode(Node& parent, std::string alias) {
            if(!parent.configureMode) {
                // Create node as well
                node = std::make_shared<T>();
                node->setAlias(alias);
                node->parentNode = node;
                // Add node to parents map
                parent.nodeMap.push_back(node);
            }
            // Add reference
            parent.nodeRefs.push_back(&node);
        }
        T& operator*() const noexcept {
            return *std::static_pointer_cast<T>(node).get();
        }
        T* operator->() const noexcept {
            return std::static_pointer_cast<T>(node).get();
        }
    };

   public:
    struct DatatypeHierarchy {
        DatatypeHierarchy(DatatypeEnum d, bool c) : datatype(d), descendants(c) {}
        DatatypeEnum datatype;
        bool descendants;
    };

    struct OutputDescription {
        std::string name{};                                                  // Name of the output
        std::string group{};                                                 // Group of the output
        std::vector<DatatypeHierarchy> types{{DatatypeEnum::Buffer, true}};  // Possible datatypes that can be sent
    };

    class Output {
        friend class PipelineImpl;

       public:
        struct QueueConnection {
            Output* output;
            std::shared_ptr<MessageQueue> queue;
            bool operator==(const QueueConnection& rhs) const {
                return output == rhs.output && queue == rhs.queue;
            }
        };
        enum class Type { MSender, SSender };
        virtual ~Output() = default;

       private:
        std::reference_wrapper<Node> parent;
        std::vector<MessageQueue*> connectedInputs;
        std::vector<QueueConnection> queueConnections;
        Type type = Type::MSender;  // Slave sender not supported yet
        OutputDescription desc;

       public:
        // std::vector<Capability> possibleCapabilities;

        Output(Node& par, OutputDescription desc, bool ref = true) : parent(par), desc(std::move(desc)) {
            // Place oneself to the parents references
            if(ref) {
                par.setOutputRefs(this);
            }
        }

        Node& getParent() {
            return parent;
        }
        const Node& getParent() const {
            return parent;
        }

        /// Output to string representation
        std::string toString() const;

        /**
         * Get name of the output
         */
        std::string getName() const {
            return desc.name;
        }

        /**
         * Get group of the output
         */
        std::string getGroup() const {
            return desc.group;
        }

        /**
         * Set group name for this output
         */
        void setGroup(std::string group) {
            desc.group = std::move(group);
        }

        /**
         * Set name for this output
         */
        void setName(std::string name) {
            desc.name = std::move(name);
        }

        /**
         * Get type of the output
         */
        Type getType() const {
            return type;
        }

        /**
         * Get possible datatypes that can be sent
         */
        std::vector<DatatypeHierarchy> getPossibleDatatypes() const {
            return desc.types;
        }

        /**
         * Check if this output and given input are on the same pipeline.
         * @see canConnect for checking if connection is possible
         * @returns True if output and input are on the same pipeline
         */
        bool isSamePipeline(const Input& in);

        /**
         * Check if connection is possible
         * @param in Input to connect to
         * @returns True if connection is possible, false otherwise
         */
        bool canConnect(const Input& in);

        /**
         * Retrieve all connections from this output
         * @returns Vector of connections
         */
        std::vector<ConnectionInternal> getConnections();

        /**
         * Retrieve all queue connections from this output
         * @returns Vector of queue connections
         */
        std::vector<QueueConnection> getQueueConnections() {
            return queueConnections;
        }

        /** Default value for the blocking argument in the createOutputQueue method */
        static constexpr bool OUTPUT_QUEUE_DEFAULT_BLOCKING = false;

        /** Default value for the maxSize argument in the createOutputQueue method */
        static constexpr unsigned int OUTPUT_QUEUE_DEFAULT_MAX_SIZE = 16;

        /**
         * @brief Construct and return a shared pointer to an output message queue
         *
         * @param maxSize: Maximum size of the output queue
         * @param blocking: Whether the output queue should block when full
         *
         * @return std::shared_ptr<dai::MessageQueue>: shared pointer to an output queue
         */
        std::shared_ptr<dai::MessageQueue> createOutputQueue(unsigned int maxSize = OUTPUT_QUEUE_DEFAULT_MAX_SIZE,
                                                             bool blocking = OUTPUT_QUEUE_DEFAULT_BLOCKING);

       private:
        void link(const std::shared_ptr<dai::MessageQueue>& queue) {
            connectedInputs.push_back(queue.get());
            queueConnections.push_back({this, queue});
        }

        void unlink(const std::shared_ptr<dai::MessageQueue>& queue) {
            connectedInputs.erase(std::remove(connectedInputs.begin(), connectedInputs.end(), queue.get()), connectedInputs.end());
            queueConnections.erase(std::remove(queueConnections.begin(), queueConnections.end(), QueueConnection{this, queue}), queueConnections.end());
        }

       public:
        /**
         * Link current output to input.
         *
         * Throws an error if this output cannot be linked to given input,
         * or if they are already linked
         *
         * @param in Input to link to
         */
        void link(Input& in);

        virtual void link(std::shared_ptr<Node> in);

        /**
         * Unlink a previously linked connection
         *
         * Throws an error if not linked.
         *
         * @param in Input from which to unlink from
         */
        void unlink(Input& in);

        /**
         * Sends a Message to all connected inputs
         * @param msg Message to send to all connected inputs
         */
        void send(const std::shared_ptr<ADatatype>& msg);

        /**
         * Try sending a message to all connected inputs
         * @param msg Message to send to all connected inputs
         * @returns True if ALL connected inputs got the message, false otherwise
         */
        bool trySend(const std::shared_ptr<ADatatype>& msg);
    };

    struct PairHash {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };
    /**
     * Output map which keeps track of extra outputs assigned to a node
     * Extends std::unordered_map<std::string, dai::Node::Output>
     */
    class OutputMap : public std::unordered_map<std::pair<std::string, std::string>, Output, PairHash> {
        OutputDescription defaultOutput;
        std::reference_wrapper<Node> parent;

       public:
        std::string name;
        OutputMap(Node& parent, std::string name, OutputDescription defaultOutput, bool ref = true);
        OutputMap(Node& parent, OutputDescription defaultOutput, bool ref = true);
        /// Create or modify an output
        Output& operator[](const std::string& key);
        /// Create or modify an output with specified group
        Output& operator[](std::pair<std::string, std::string> groupKey);
    };

    // Input extends the message queue with additional option that specifies whether to wait for message or not
    struct InputDescription {
        std::string name{};                                                  // Name of the input
        std::string group{};                                                 // Group of the input
        bool blocking{true};                                                 // Whether to block when input queue is full
        int queueSize{3};                                                    // Size of the queue
        std::vector<DatatypeHierarchy> types{{DatatypeEnum::Buffer, true}};  // Possible datatypes that can be received
        bool waitForMessage{false};
    };

    class Input : public MessageQueue {
       public:
        enum class Type { SReceiver, MReceiver };  // TODO(Morato) - refactor, make the MReceiver a separate class (shouldn't inherit from MessageQueue)

       private:
        std::reference_wrapper<Node> parent;
        std::vector<Output*> connectedOutputs;
        // Options - more information about the input
        bool waitForMessage{false};
        std::string group;
        Type type = Type::SReceiver;
        std::vector<std::shared_ptr<InputQueue>> connectedQueues;

       public:
        std::vector<DatatypeHierarchy> possibleDatatypes;
        explicit Input(Node& par, InputDescription desc, bool ref = true)
            : MessageQueue(std::move(desc.name), desc.queueSize, desc.blocking),
              parent(par),
              waitForMessage(desc.waitForMessage),
              possibleDatatypes(std::move(desc.types)) {
            if(ref) {
                par.setInputRefs(this);
            }
        }

        /**
         * Get the parent node
         */
        const Node& getParent() const {
            return parent;
        }
        /**
         * Get the parent node
         */
        Node& getParent() {
            return parent;
        }

        /**
         * Get type
         */
        Type getType() const {
            return type;
        }

        /**
         * Input to string representation
         */
        std::string toString() const;

        /**
         * Overrides default wait for message behavior.
         * Applicable for nodes with multiple inputs.
         * Specifies behavior whether to wait for this input when a Node processes certain data or not.
         * @param waitForMessage Whether to wait for message to arrive to this input or not
         */
        void setWaitForMessage(bool waitForMessage);

        /**
         * Get behavior whether to wait for this input when a Node processes certain data or not
         * @returns Whether to wait for message to arrive to this input or not
         */
        bool getWaitForMessage() const;

        /**
         * Equivalent to setWaitForMessage but with inverted logic.
         */
        void setReusePreviousMessage(bool reusePreviousMessage);

        /**
         * Equivalent to getWaitForMessage but with inverted logic.
         */
        bool getReusePreviousMessage() const;

        /**
         * Set group name for this input
         */
        void setGroup(std::string group);

        /**
         * Get group name for this input
         */
        std::string getGroup() const;

        /** Default value for the blocking argument in the createInputQueue method */
        static constexpr bool INPUT_QUEUE_DEFAULT_BLOCKING = false;

        /** Default value for the maxSize argument in the createInputQueue method */
        static constexpr unsigned int INPUT_QUEUE_DEFAULT_MAX_SIZE = 16;

        /**
         * @brief Create an shared pointer to an input queue that can be used to send messages to this input from onhost
         *
         * @param maxSize: Maximum size of the input queue
         * @param blocking: Whether the input queue should block when full
         *
         * @return std::shared_ptr<InputQueue>: shared pointer to an input queue
         */
        std::shared_ptr<InputQueue> createInputQueue(unsigned int maxSize = INPUT_QUEUE_DEFAULT_MAX_SIZE, bool blocking = INPUT_QUEUE_DEFAULT_BLOCKING);
    };

    /**
     * Input map which keeps track of inputs assigned to a node
     * Extends std::unordered_map<std::string, dai::Node::Input>
     */
    class InputMap : public std::unordered_map<std::pair<std::string, std::string>, Input, PairHash> {
        std::reference_wrapper<Node> parent;
        InputDescription defaultInput;

       public:
        std::string name;
        // InputMap(Input defaultInput);
        // InputMap(std::string name, Input defaultInput);
        InputMap(Node& parent, InputDescription defaultInput);
        InputMap(Node& parent, std::string name, InputDescription defaultInput);
        /// Create or modify an input
        Input& operator[](const std::string& key);
        /// Create or modify an input with specified group
        Input& operator[](std::pair<std::string, std::string> groupKey);
        // Check if the input exists
        bool has(const std::string& key) const;
    };

    /// Connection between an Input and Output internal
    struct ConnectionInternal {
        ConnectionInternal(Output& out, Input& in);
        std::weak_ptr<Node> outputNode;
        std::string outputName;
        std::string outputGroup;
        std::weak_ptr<Node> inputNode;
        std::string inputName;
        std::string inputGroup;
        Output* out;
        Input* in;
        bool operator==(const ConnectionInternal& rhs) const;
        struct Hash {
            size_t operator()(const dai::Node::ConnectionInternal& obj) const;
        };
    };

    /// Connection between an Input and Output
    struct Connection {
        friend struct std::hash<Connection>;
        Connection(Output out, Input in);
        Connection(ConnectionInternal c);
        Id outputId;
        std::string outputName;
        std::string outputGroup;
        Id inputId;
        std::string inputName;
        std::string inputGroup;
        bool operator==(const Connection& rhs) const;
    };

   protected:
    bool configureMode{false};

    // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
    std::weak_ptr<PipelineImpl> parent;
    std::weak_ptr<Node> parentNode;

    // used to improve error messages
    // when pipeline starts all nodes are checked
    virtual bool needsBuild() {
        return false;
    }

   public:
    // TODO(themarpe) - restrict access
    /// Id of node. Assigned after being placed on the pipeline
    Id id{-1};

    /// alias or name
    std::string alias;

   protected:
    AssetManager assetManager;

    virtual std::optional<OpenVINO::Version> getRequiredOpenVINOVersion();

    // Optimized for adding, searching and removing connections
    // using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    using NodeMap = std::vector<std::shared_ptr<Node>>;
    NodeMap nodeMap;

    // Connection map, NodeId represents id of node connected TO (input)
    // using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::Connection>>;
    using SetConnectionInternal = std::unordered_set<ConnectionInternal, ConnectionInternal::Hash>;
    SetConnectionInternal connections;
    using ConnectionMap = std::unordered_map<std::shared_ptr<Node>, SetConnectionInternal>;

   public:
    // access
    Pipeline getParentPipeline();
    const Pipeline getParentPipeline() const;

    /// Get alias
    std::string getAlias() const {
        return alias;
    }
    /// Set alias
    void setAlias(std::string alias) {
        this->alias = std::move(alias);
    }

    /// Retrieves nodes name
    virtual const char* getName() const = 0;

    /// Start node execution
    virtual void start(){};

    /// Wait for node to finish execution
    virtual void wait(){};

    /// Stop node execution
    virtual void stop(){};

    void stopPipeline();

    /// Build stages;
    virtual void buildStage1();
    virtual void buildStage2();
    virtual void buildStage3();

    /// Retrieves all nodes outputs
    std::vector<Output> getOutputs();

    /// Retrieves all nodes inputs
    std::vector<Input> getInputs();

    /// Retrieves reference to node outputs
    std::vector<Output*> getOutputRefs();

    /// Retrieves reference to node outputs
    std::vector<const Output*> getOutputRefs() const;

    /// Retrieves reference to node inputs
    std::vector<Input*> getInputRefs();

    /// Retrieves reference to node inputs
    std::vector<const Input*> getInputRefs() const;

    /// Retrieves reference to node outputs
    std::vector<OutputMap*> getOutputMapRefs();

    /// Retrieves reference to node inputs
    std::vector<InputMap*> getInputMapRefs();

    /// Retrieves reference to specific output
    Output* getOutputRef(std::string name);
    Output* getOutputRef(std::string group, std::string name);

    /// Retrieves reference to specific input
    Input* getInputRef(std::string name);
    Input* getInputRef(std::string group, std::string name);

    /// Retrieves reference to specific output map
    OutputMap* getOutputMapRef(std::string group);

    /// Retrieves reference to specific input map
    InputMap* getInputMapRef(std::string group);

   protected:
    Node() = default;
    Node(bool conf);
    void removeConnectionToNode(std::shared_ptr<Node> node);

   public:
    virtual ~Node() = default;

    /// Get node AssetManager as a const reference
    const AssetManager& getAssetManager() const;

    /// Get node AssetManager as a reference
    AssetManager& getAssetManager();

    /// Loads resource specified by URI and returns its data
    std::vector<uint8_t> loadResource(dai::Path uri);

    /// Create and place Node to this Node
    template <class N>
    std::shared_ptr<N> create() {
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        // Create and store the node in the map
        auto node = std::make_shared<N>();
        // Add
        add(node);
        // Return shared pointer to this node
        return node;
    }

    /// Add existing node to nodeMap
    void add(std::shared_ptr<Node> node);

    // Access to nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() const;
    std::shared_ptr<const Node> getNode(Node::Id id) const;
    std::shared_ptr<Node> getNode(Node::Id id);
    void remove(std::shared_ptr<Node> node);
    ConnectionMap getConnectionMap();
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    /// Get a reference to internal node map

    virtual void link(std::shared_ptr<Node> in);
    virtual Node::Output* requestOutput(const Capability& capability, bool onHost);
    virtual std::vector<std::pair<Input&, std::shared_ptr<Capability>>> getRequiredInputs();

    /**
     * @brief Returns true or false whether the node should be run on host or not
     */
    virtual bool runOnHost() const = 0;

    const NodeMap& getNodeMap() const {
        return nodeMap;
    }
};

// Node CRTP class
template <typename Base, typename Derived>
class NodeCRTP : public Base {
   public:
    virtual ~NodeCRTP() = default;

    const char* getName() const override {
        return Derived::NAME;
    };
    // std::unique_ptr<Node> clone() const override {
    //     return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    // };

    // No public constructor, only a factory function.
    template <typename... Args>
    [[nodiscard]] static std::shared_ptr<Derived> create(Args&&... args) {
        return std::make_shared<Derived>(std::forward<Args>(args)...);
    }
    [[nodiscard]] static std::shared_ptr<Derived> create(std::unique_ptr<Properties> props) {
        return std::shared_ptr<Derived>(new Derived(props));
    }

    friend Derived;
    friend Base;
};

}  // namespace dai

// Specialization of std::hash for Node::Connection
namespace std {
template <>
struct hash<dai::Node::Connection> {
    size_t operator()(const dai::Node::Connection& obj) const {
        size_t seed = 0;
        std::hash<dai::Node::Id> hId;
        std::hash<std::string> hStr;
        seed ^= hId(obj.outputId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hId(obj.inputId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

}  // namespace std
