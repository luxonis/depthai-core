#include <string>
#include <unordered_map>
#include <vector>

#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "SNPE/SNPE.hpp"

size_t createUserBuffer(zdl::DlSystem::UserBufferMap& userBufferMap,
                        std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                        std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                        std::unique_ptr<zdl::SNPE::SNPE>& snpe,
                        const char* name);

size_t createInputBufferMap(zdl::DlSystem::UserBufferMap& inputMap,
                            std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                            std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                            std::unique_ptr<zdl::SNPE::SNPE>& snpe);

void createOutputBufferMap(zdl::DlSystem::UserBufferMap& outputMap,
                           std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                           std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                           std::unique_ptr<zdl::SNPE::SNPE>& snpe);
