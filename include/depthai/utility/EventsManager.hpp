#pragma once

#include <string>
#include <vector>
#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {
namespace utility {
class EventsManager {
   public:
	explicit EventsManager(const std::string& sessionToken, const std::string& agentToken, const std::string& deviceSerialNumber);
	virtual ~EventsManager() = default;

	std::string sendEvent(const std::string& name, const std::shared_ptr<ADatatype>& data, const std::vector<std::string>& tags);
	std::string sendSnap(const std::string& name, const std::shared_ptr<ADatatype>& data, const std::vector<std::string>& tags); 
private:
	std::string sessionToken;
	std::string agentToken;
	std::string deviceSerialNumber;
};
}  // namespace utility
}  // namespace dai
