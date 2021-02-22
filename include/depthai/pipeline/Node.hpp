#pragma once

#include <algorithm>
#include <set>
#include <string>
#include <tuple>

// project
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/AssetManager.hpp"

// depthai-shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"

// libraries
#include "nlohmann/json.hpp"
#include "tl/optional.hpp"

namespace dai {
// fwd declare Pipeline
class Pipeline;
class PipelineImpl;

class Node {
    friend class Pipeline;
    friend class PipelineImpl;

   public:
    using Id = std::int64_t;
    struct Connection;

   protected:
    struct DatatypeHierarchy {
        DatatypeHierarchy(DatatypeEnum d, bool c) : datatype(d), descendants(c) {}
        DatatypeEnum datatype;
        bool descendants;
    };

    // fwd declare Input class
    struct Input;

    struct Output {
        enum class Type { MSender, SSender };
        Node& parent;
        const std::string name;
        const Type type;
        // Which types and do descendants count as well?
        const std::vector<DatatypeHierarchy> possibleDatatypes;
        Output(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}
        bool isSamePipeline(const Input& in);

       public:
        bool canConnect(const Input& in);
        std::vector<Connection> getConnections();
        void link(const Input& in);
        void unlink(const Input& in);
    };

    struct Input {
        enum class Type { SReceiver, MReceiver };
        Node& parent;
        const std::string name;
        const Type type;
        bool defaultBlocking{true};
        int defaultQueueSize{8};
        tl::optional<bool> blocking;
        tl::optional<int> queueSize;
        friend struct Output;
        const std::vector<DatatypeHierarchy> possibleDatatypes;

        /// Constructs Input with default blocking and queueSize options
        Input(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize options
        Input(Node& par, std::string n, Type t, bool blocking, int queueSize, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), defaultBlocking(blocking), defaultQueueSize(queueSize), possibleDatatypes(std::move(types)) {}

       public:
        /**
         * Overrides default input queue behavior.
         * @param blocking True blocking, false overwriting
         */
        void setBlocking(bool blocking);

        /**
         * Get input queue behavior
         * @return True blocking, false overwriting
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
         * @return Maximum input queue size
         */
        int getQueueSize() const;
    };

    // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
    std::weak_ptr<PipelineImpl> parent;
    AssetManager assetManager;

    virtual nlohmann::json getProperties() = 0;
    virtual tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion();
    virtual std::shared_ptr<Node> clone() = 0;

    // access
    Pipeline getParentPipeline();
    const Pipeline getParentPipeline() const;

    /**
     *  Reads and stores asset into AssetManager
     * @param name Name of asset - will be prepended with node id to create final asset key
     * @param path Path of file to load
     * @param alignment Asset alignment
     * @returns Shared pointer to asset in AssetManager
     */
    std::shared_ptr<Asset> loadAsset(const std::string& name, const std::string& path, int alignment = 64);

   public:
    const Id id;
    virtual std::string getName() const = 0;
    virtual std::vector<Output> getOutputs() = 0;
    virtual std::vector<Input> getInputs() = 0;
    virtual std::vector<std::shared_ptr<Asset>> getAssets();
    struct Connection {
        friend struct std::hash<Connection>;
        Connection(Output out, Input in);
        Id outputId;
        std::string outputName;
        Id inputId;
        std::string inputName;
        bool operator==(const Connection& rhs) const;
    };

    Node(const std::shared_ptr<PipelineImpl>& p, Id nodeId);

    virtual ~Node() = default;
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