#include <string>
#include <unordered_map>
#include <vector>

#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "SNPE/SNPE.hpp"

// Helper function to fill a single entry of the UserBufferMap with the given user-backed buffer
size_t createUserBuffer(zdl::DlSystem::UserBufferMap& userBufferMap,
                        std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                        std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                        std::unique_ptr<zdl::SNPE::SNPE>& snpe,
                        const char* name);

// Create a UserBufferMap of the SNPE network inputs
size_t createInputBufferMap(zdl::DlSystem::UserBufferMap& inputMap,
                            std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                            std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                            std::unique_ptr<zdl::SNPE::SNPE>& snpe);

// Create a UserBufferMap of the SNPE network outputs
void createOutputBufferMap(zdl::DlSystem::UserBufferMap& outputMap,
                           std::unordered_map<std::string, std::vector<uint8_t>>& applicationBuffers,
                           std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                           std::unique_ptr<zdl::SNPE::SNPE>& snpe);
