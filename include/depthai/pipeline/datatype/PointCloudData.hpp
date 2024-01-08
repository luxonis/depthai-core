#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/build/config.hpp"
#include "depthai-shared/datatype/RawPointCloudData.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai-shared/common/Point3f.hpp"

// optional
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#endif

namespace dai {

/**
 * PointCloudData message. Carries ROI (region of interest) and threshold for depth calculation
 */
class PointCloudData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawPointCloudData& pcl;

   public:
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;
    using Buffer::getSequenceNum;

    std::vector<Point3f> points;

    /**
     * Construct PointCloudData message.
     */
    PointCloudData();
    explicit PointCloudData(std::shared_ptr<RawPointCloudData> ptr);
    virtual ~PointCloudData() = default;

    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves image width in pixels
     */
    unsigned int getWidth() const;

    /**
     * Retrieves image height in pixels
     */
    unsigned int getHeight() const;
      
    /**
     * Retrieves minimal x coordinate in milimeters
     */
    float getMinX() const;

    /**
     * Retrieves minimal y coordinate in milimeters
     */
    float getMinY() const;

    /**
     * Retrieves minimal z coordinate in milimeters
     */
    float getMinZ() const;

    /**
     * Retrieves maximal x coordinate in milimeters
     */
    float getMaxX() const;

    /**
     * Retrieves maximal y coordinate in milimeters
     */
    float getMaxY() const;

    /**
     * Retrieves maximal z coordinate in milimeters
     */
    float getMaxZ() const;

    // setters
    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    PointCloudData& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    PointCloudData& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Instance number relates to the origin of the frame (which camera)
     *
     * @param instance Instance number
     */
    PointCloudData& setInstanceNum(unsigned int instance);

    /**
     * Specifies sequence number
     *
     * @param seq Sequence number
     */
    PointCloudData& setSequenceNum(int64_t seq);

    /**
     * Specifies frame width
     *
     * @param width frame width
     */
    PointCloudData& setWidth(unsigned int width);

    /**
     * Specifies frame height
     *
     * @param height frame height
     */
    PointCloudData& setHeight(unsigned int height);

    /**
     * Specifies frame size
     *
     * @param height frame height
     * @param width frame width
     */
    PointCloudData& setSize(unsigned int width, unsigned int height);

    /**
     * Specifies frame size
     *
     * @param size frame size
     */
    PointCloudData& setSize(std::tuple<unsigned int, unsigned int> size);

    /**
     * Specifies minimal x coordinate in milimeters
	 * 
	 * @param val minimal x coordinate in milimeters
     */
    PointCloudData& setMinX(float val);

    /**
     * Specifies minimal y coordinate in milimeters
	 * 
	 * @param val minimal y coordinate in milimeters
     */
    PointCloudData& setMinY(float val);

    /**
     * Specifies minimal z coordinate in milimeters
	 * 
	 * @param val minimal z coordinate in milimeters
     */
    PointCloudData& setMinZ(float val);

    /**
     * Specifies maximal x coordinate in milimeters
	 * 
	 * @param val maximal x coordinate in milimeters
     */
    PointCloudData& setMaxX(float val);

    /**
     * Specifies maximal y coordinate in milimeters
	 * 
	 * @param val maximal y coordinate in milimeters
     */
    PointCloudData& setMaxY(float val);

    /**
     * Specifies maximal z coordinate in milimeters
	 * 
	 * @param val maximal z coordinate in milimeters
     */
    PointCloudData& setMaxZ(float val);

    #ifdef DEPTHAI_HAVE_PCL_SUPPORT
    /**
     * Converts PointCloudData to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPclData() const;

    #else
    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    void toPclData() const {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
    #endif
};

}  // namespace dai
