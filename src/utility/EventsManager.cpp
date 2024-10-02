#include "depthai/utility/EventsManager.hpp"

#include <cpr/cpr.h>

#include <chrono>

#include "Environment.hpp"
#include "depthai/schemas/Event.pb.h"
namespace dai {

namespace utility {
EventsManager::EventsManager(const std::string& sessionToken, const std::string& agentToken, const std::string& deviceSerialNumber)
    : sessionToken(sessionToken), agentToken(agentToken), deviceSerialNumber(deviceSerialNumber) {}

std::string EventsManager::sendEvent(const std::string& name, const std::shared_ptr<ADatatype>& data, const std::vector<std::string>& tags) {
    // Create event
    auto event = std::make_unique<proto::Event>();
    event->set_name(name);
    event->set_created_at(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    for(const auto& tag : tags) {
        event->add_tags(tag);
    }
    proto::Extras* extras = event->add_extras();
    auto* extraData = extras->mutable_data();
    // bytes to stringstream
    std::stringstream ss;
    ss.write((const char*)data->data->getData().data(), data->data->getData().size());
    extraData->at(name) = ss.str();

    event->set_expect_files_num(0);

    event->set_source_serial_number(deviceSerialNumber);
    event->set_source_app_id(utility::getEnv("AGENT_APP_ID"));
    event->set_source_app_identified(utility::getEnv("AGENT_APP_IDENTIFIER"));

    std::string serializedEvent;
    event->SerializeToString(&serializedEvent);
    cpr::Response r = cpr::Post(cpr::Url{"http://0.0.0.0:80/post"}, cpr::Body{serializedEvent}, cpr::Authentication{sessionToken, agentToken});
	std::cout << r.text << std::endl;
    return r.text;
}

std::string EventsManager::sendSnap(const std::string& name, const std::shared_ptr<ADatatype>& data, const std::vector<std::string>& tags) {
    std::vector<std::string> tagsTmp = tags;
    tagsTmp.push_back("snap");
    return sendEvent(name, data, tagsTmp);
}
}  // namespace utility
}  // namespace dai
