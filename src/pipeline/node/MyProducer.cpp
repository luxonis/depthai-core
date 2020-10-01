#include "depthai/pipeline/node/MyProducer.hpp"

namespace dai
{
namespace node
{


std::string MyProducer::getName(){
    return "MyProducer";
}


std::vector<Node::Input> MyProducer::getInputs(){
    return {};
}

std::vector<Node::Output> MyProducer::getOutputs(){
    return {out};
}

nlohmann::json MyProducer::getProperties(){
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> MyProducer::clone(){
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}


MyProducer::MyProducer(const std::shared_ptr<PipelineImpl>& par) : Node(par) {}

void MyProducer::setMessage(std::string m){
    properties.message = m;
}

void MyProducer::setProcessor(ProcessorType proc){
    properties.processorPlacement = proc;
}



} // namespace node
} // namespace dai
