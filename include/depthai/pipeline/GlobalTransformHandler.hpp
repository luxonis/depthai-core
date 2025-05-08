#pragma once

#include <memory>

#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {

namespace transforms {

enum class DaiCoordinateSystem { RDF = 0, FLU };


class DeviceTransformHandler {
   public:
    DeviceTransformHandler(const DeviceTransformHandler&) = default;
    DeviceTransformHandler(DeviceTransformHandler&&) = default;
    DeviceTransformHandler& operator=(const DeviceTransformHandler&) = default;
    DeviceTransformHandler& operator=(DeviceTransformHandler&&) = default;
    void initializeDeviceTransformTree();
    void setCoordinateSystem(DaiCoordinateSystem system);
  private:
    DaiCoordinateSystem coordSys;
};

class GlobalTransformHandler {
   public:

    GlobalTransformHandler(const GlobalTransformHandler&) = default;
    GlobalTransformHandler(GlobalTransformHandler&&) = default;
    GlobalTransformHandler& operator=(const GlobalTransformHandler&) = default;
    GlobalTransformHandler& operator=(GlobalTransformHandler&&) = default;
    void addTransform(std::shared_ptr<dai::TransformData> transform);
    void addStaticTransform(std::shared_ptr<dai::Transform> transform);
    void removeTransform(const std::string& transformName);
    std::shared_ptr<dai::TransformData> lookupTransform(std::shared_ptr<dai::TransformData> from,
                                                     std::shared_ptr<dai::TransformData> to,
                                                     dai::Timestamp maxInterval);
    std::shared_ptr<dai::TransformData> lookupTransform(const std::string& from, const std::string& to, dai::Timestamp maxInterval);
    std::shared_ptr<dai::TransformData> lookupTransform(std::shared_ptr<dai::TransformData> from, const std::string& to, dai::Timestamp maxInterval);
    std::shared_ptr<dai::TransformData> lookupTransform(const std::string& from, std::shared_ptr<dai::TransformData> to, dai::Timestamp maxInterval);
    std::vector<std::shared_ptr<dai::TransformData>> getTransformTree();
    void setCoordinateSystem(DaiCoordinateSystem system);

   private:
    std::vector<std::shared_ptr<dai::TransformData>> transforms;
    std::vector<std::shared_ptr<dai::TransformData>> staticTransforms;
    DaiCoordinateSystem coordSys;
};
}  // namespace transforms
}  // namespace dai
