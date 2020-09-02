#pragma once

#include <string>
#include <tuple>

// depthai-shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/Assets.hpp"

#include "nlohmann/json.hpp"

namespace dai
{
    // fwd declare Pipeline
    class Pipeline;
    class PipelineImpl;


    class Node {

        friend class Pipeline;
        friend class PipelineImpl;

    protected:

        struct DatatypeHierarchy {
            DatatypeHierarchy(DatatypeEnum d, bool c) : datatype(d), descendants(c){}
            DatatypeEnum datatype;
            bool descendants;
        };
        
        //fwd declare Input class
        class Input;

        struct Output{
            enum class Type{
                MSender, SSender
            };
            Node& parent;
            const std::string name;
            const Type type;
            // Which types and do descendants count as well?
            const std::vector<DatatypeHierarchy> possibleDatatypes;
            std::vector<Input> conn;
            Output(Node& par, std::string n, Type t, std::vector< DatatypeHierarchy > types) : parent(par), name(n), type(t), possibleDatatypes(types) {}
    
        public:
            bool canConnect(Input in){
                if(type == Type::MSender && in.type == Input::Type::MReceiver) return false;
                if(type == Type::SSender && in.type == Input::Type::SReceiver) return false;
                for(const auto& outHierarchy : possibleDatatypes){
                    for(const auto& inHierarchy : in.possibleDatatypes){
                        if(outHierarchy.datatype == inHierarchy.datatype) return true;
                        if(inHierarchy.descendants && isDatatypeSubclassOf(inHierarchy.datatype, outHierarchy.datatype)) return true;
                    }
                }
                return false;
            }

            void link(Input in) {
                if(!canConnect(in)){
                    std::string msg = "Cannot link '" + parent.getName() + "." + name + "' to '" + in.parent.getName() + in.name + "'";
                    throw std::runtime_error(msg);
                }

                conn.push_back(in);
            }        


        };

        struct Input {
            enum class Type{
                SReceiver, MReceiver
            };
            Node& parent;
            const std::string name;
            const Type type;
            friend class Output;
            // Which types and do descendants count as well?
            const std::vector< DatatypeHierarchy > possibleDatatypes;
            Input(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types) : parent(par), name(n), type(t), possibleDatatypes(types) {}
        };


        // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
        std::weak_ptr<PipelineImpl> parent;
        std::vector<Output> outputs;


        int64_t id;
        virtual std::string getName() = 0;
        virtual std::vector<Output> getOutputs() = 0;
        virtual std::vector<Input> getInputs() = 0;
        virtual nlohmann::json getProperties() = 0;
        virtual std::shared_ptr<Node> clone() = 0; 

        virtual void loadAssets(AssetManager& assetManager){
            (void) assetManager;
        }

    public:
        Node(const std::shared_ptr<PipelineImpl>& p) : parent(p) {}
        virtual ~Node(){};



    };

} // namespace dai
