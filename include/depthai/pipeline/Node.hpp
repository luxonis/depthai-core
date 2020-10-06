#pragma once

#include <string>
#include <tuple>

// depthai-shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/AssetManager.hpp"
#include "nlohmann/json.hpp"

namespace dai {
// fwd declare Pipeline
class Pipeline;
class PipelineImpl;

class Node {
    friend class Pipeline;
    friend class PipelineImpl;

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
        std::vector<Input> conn;
        Output(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}

       public:
        bool canConnect(const Input& in);
        void link(Input in);
    };

    struct Input {
        enum class Type { SReceiver, MReceiver };
        Node& parent;
        const std::string name;
        const Type type;
        friend struct Output;
        // Which types and do descendants count as well?
        const std::vector<DatatypeHierarchy> possibleDatatypes;
        Input(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}
    };

    // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
    std::weak_ptr<PipelineImpl> parent;
    std::vector<Output> outputs;
    const int64_t id;

    virtual std::string getName() = 0;
    virtual std::vector<Output> getOutputs() = 0;
    virtual std::vector<Input> getInputs() = 0;
    virtual nlohmann::json getProperties() = 0;
    virtual std::shared_ptr<Node> clone() = 0;
    virtual void loadAssets(AssetManager& assetManager);

    // access
    Pipeline getParentPipeline();

   public:
    Node(const std::shared_ptr<PipelineImpl>& p, int64_t nodeId);

    virtual ~Node() = default;
};

}  // namespace dai
