#include "depthai/pipeline/node/XLinkOut.hpp"

namespace dai
{
namespace node
{

std::string XLinkOut::getName(){
    return "XLinkOut";
}
        
std::vector<Node::Input> XLinkOut::getInputs(){
    return {in};
}

std::vector<Node::Output> XLinkOut::getOutputs(){
    return {};
}

nlohmann::json XLinkOut::getProperties(){
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> XLinkOut::clone(){
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

XLinkOut::XLinkOut(const std::shared_ptr<PipelineImpl>& par) : Node(par) {
    properties.maxFpsLimit = -1;
}

void XLinkOut::setStreamName(std::string name){
    properties.streamName = name;
}

void XLinkOut::setFpsLimit(double fps){
    properties.maxFpsLimit = fps;
}


} // namespace node
} // namespace dai
