#pragma once

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>

// project
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/AssetManager.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/utility/copyable_unique_ptr.hpp"

// depthai-shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/properties/Properties.hpp"

// libraries
#include "tl/optional.hpp"

namespace dai {
// fwd declare Pipeline
class Pipeline;
class PipelineImpl;

/**
 * @brief Abstract Node
 */
class Node : public std::enable_shared_from_this<Node> {
    friend class Pipeline;
    friend class PipelineImpl;

   public:
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
    std::unordered_map<std::string, Output*> outputRefs;
    std::unordered_map<std::string, Input*> inputRefs;
    std::unordered_map<std::string, OutputMap*> outputMapRefs;
    std::unordered_map<std::string, InputMap*> inputMapRefs;
    std::unordered_map<std::string, std::shared_ptr<Node>*> nodeRefs;

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
            parent.nodeRefs[alias] = &node;
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

    class Output {
        Node& parent;

       public:
        enum class Type { MSender, SSender };
        std::string group = "";
        std::string name;
        Type type;
        // Which types and do descendants count as well?
        std::vector<DatatypeHierarchy> possibleDatatypes;
        Output(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}
        Output(Node& par, std::string group, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), group(std::move(group)), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}
        Output(bool ref, Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setOutputRefs(this);
            }
        }
        Output(bool ref, Node& par, std::string group, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), group(std::move(group)), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setOutputRefs(this);
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
         * Link current output to input.
         *
         * Throws an error if this output cannot be linked to given input,
         * or if they are already linked
         *
         * @param in Input to link to
         */
        void link(Input& in);

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
        Output defaultOutput;

       public:
        std::string name;
        OutputMap(Node& parent, std::string name, Output defaultOutput);
        OutputMap(Node& parent, Output defaultOutput);
        OutputMap(bool ref, Node& parent, std::string name, Output defaultOutput);
        OutputMap(bool ref, Node& parent, Output defaultOutput);
        /// Create or modify an output
        Output& operator[](const std::string& key);
        /// Create or modify an output with specified group
        Output& operator[](std::pair<std::string, std::string> groupKey);
    };

    class Input {
        Node& parent;

       public:
        enum class Type { SReceiver, MReceiver };
        std::string group = "";
        std::string name;
        Type type;
        bool defaultBlocking{true};
        int defaultQueueSize{8};
        tl::optional<bool> blocking;
        tl::optional<int> queueSize;
        // Options - more information about the input
        tl::optional<bool> waitForMessage;
        bool defaultWaitForMessage{false};
        friend class Output;
        std::vector<DatatypeHierarchy> possibleDatatypes;

        MessageQueue queue;

        /// Constructs Input with default blocking and queueSize options
        Input(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize options
        Input(Node& par, std::string n, Type t, bool blocking, int queueSize, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), defaultBlocking(blocking), defaultQueueSize(queueSize), possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(Node& par, std::string n, Type t, bool blocking, int queueSize, bool waitForMessage, std::vector<DatatypeHierarchy> types)
            : parent(par),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(Node& par, std::string group, std::string n, Type t, bool blocking, int queueSize, bool waitForMessage, std::vector<DatatypeHierarchy> types)
            : parent(par),
              group(std::move(group)),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {}

        /// Constructs Input with default blocking and queueSize options
        Input(bool ref, Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setInputRefs(this);
            }
        }

        /// Constructs Input with specified blocking and queueSize options
        Input(bool ref, Node& par, std::string n, Type t, bool blocking, int queueSize, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), defaultBlocking(blocking), defaultQueueSize(queueSize), possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setInputRefs(this);
            }
        }

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(bool ref, Node& par, std::string n, Type t, bool blocking, int queueSize, bool waitForMessage, std::vector<DatatypeHierarchy> types)
            : parent(par),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setInputRefs(this);
            }
        }

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(bool ref,
              Node& par,
              std::string group,
              std::string n,
              Type t,
              bool blocking,
              int queueSize,
              bool waitForMessage,
              std::vector<DatatypeHierarchy> types)
            : parent(par),
              group(std::move(group)),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {
            // Place oneself to the parents references
            if(ref) {
                parent.setInputRefs(this);
            }
        }

        Node& getParent() {
            return parent;
        }
        const Node& getParent() const {
            return parent;
        }

        /// Input to string representation
        std::string toString() const;

        /**
         * Overrides default input queue behavior.
         * @param blocking True blocking, false overwriting
         */
        void setBlocking(bool blocking);

        /**
         * Get input queue behavior
         * @returns True blocking, false overwriting
         */
        bool getBlocking() const;

        /**
         * Overrides default input queue size.
         * If queue size fills up, behavior depends on `blocking` attribute
         * @param size Maximum input queue size
         */
        void setQueueSize(int size);

        /**
         * Get input queue size.
         * @returns Maximum input queue size
         */
        int getQueueSize() const;

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
    };

    /**
     * Input map which keeps track of inputs assigned to a node
     * Extends std::unordered_map<std::string, dai::Node::Input>
     */
    class InputMap : public std::unordered_map<std::pair<std::string, std::string>, Input, PairHash> {
        Input defaultInput;

       public:
        std::string name;
        InputMap(Node& parent, Input defaultInput);
        InputMap(Node& parent, std::string name, Input defaultInput);
        InputMap(bool ref, Node& parent, Input defaultInput);
        InputMap(bool ref, Node& parent, std::string name, Input defaultInput);
        /// Create or modify an input
        Input& operator[](const std::string& key);
        /// Create or modify an input with specified group
        Input& operator[](std::pair<std::string, std::string> groupKey);
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

   private:
    bool configureMode{false};

   protected:
    // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
    std::weak_ptr<PipelineImpl> parent;
    std::weak_ptr<Node> parentNode;

   public:
    // TODO(themarpe) - restrict access
    /// Id of node. Assigned after being placed on the pipeline
    Id id{-1};
    /// Marker if the node can run on host
    bool hostNode{false};
    /// alias or name
    std::string alias;

   protected:
    AssetManager assetManager;

    virtual tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion();

    // Optimized for adding, searching and removing connections
    // using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    using NodeMap = std::vector<std::shared_ptr<Node>>;
    NodeMap nodeMap;

    // Connection map, NodeId represents id of node connected TO (input)
    // using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::Connection>>;
    using SetConnectionInternal = std::unordered_set<ConnectionInternal, ConnectionInternal::Hash>;
    SetConnectionInternal connections;
    using ConnectionMap = std::unordered_map<std::shared_ptr<Node>, SetConnectionInternal>;

    // Properties
    copyable_unique_ptr<Properties> propertiesHolder;

   public:
    // access
    Pipeline getParentPipeline();
    const Pipeline getParentPipeline() const;

    // Get properties
    virtual Properties& getProperties();

    /// Get alias
    std::string getAlias() const {
        return alias;
    }
    /// Set alias
    void setAlias(std::string alias) {
        this->alias = std::move(alias);
    }

    /// Deep copy the node
    virtual std::unique_ptr<Node> clone() const = 0;

    /// Retrieves nodes name
    virtual const char* getName() const = 0;

    // TBD - might be an default impl instead
    /// Start node execution
    virtual void start() = 0;

    /// Wait for node to finish execution
    virtual void wait() = 0;

    /// Stop node execution
    virtual void stop() = 0;

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
    Node(std::unique_ptr<Properties> props, bool conf);
    void build();

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
    const NodeMap& getNodeMap() const {
        return nodeMap;
    }
};

// Node CRTP class
template <typename Base, typename Derived, typename Props>
class NodeCRTP : public Base {
   public:
    using Properties = Props;
    virtual ~NodeCRTP() = default;
    /// Underlying properties
    Properties& properties;
    const char* getName() const override {
        return Derived::NAME;
    };
    std::unique_ptr<Node> clone() const override {
        return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    };
    void build() {}

    // No public constructor, only a factory function.
    [[nodiscard]] static std::shared_ptr<Derived> create() {
        auto n = std::make_shared<Derived>();
        n->build();
        return n;
    }
    [[nodiscard]] static std::shared_ptr<Derived> create(std::unique_ptr<Properties> props) {
        auto n = std::shared_ptr<Derived>(new Derived(props));
        // Configure mode, don't build
        // n->build();
        return n;
    }

   protected:
    NodeCRTP() : Base(std::make_unique<Props>(), false), properties(static_cast<Properties&>(*Node::propertiesHolder)) {}
    NodeCRTP(std::unique_ptr<Properties> props) : Base(std::move(props), true), properties(static_cast<Properties&>(*Node::propertiesHolder)) {}
    NodeCRTP(std::unique_ptr<Properties> props, bool confMode)
        : Base(std::move(props), confMode), properties(static_cast<Properties&>(*Node::propertiesHolder)) {}

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
