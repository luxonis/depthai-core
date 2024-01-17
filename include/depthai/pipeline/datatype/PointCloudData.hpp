#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/datatype/RawPointCloudData.hpp"
#include "depthai/build/config.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// optional
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    #include <pcl/point_types.h>
    #include <pcl/visualization/cloud_viewer.h>
#endif

namespace dai {

struct PointXYZRGB {
    float x, y, z;
    uint8_t r, g, b;
private:
    uint8_t padding[1];
};
struct PointXYZ {
    float x, y, z;
private:
    uint8_t padding[4];
};
static_assert(sizeof(PointXYZRGB) == 16, "PointXYZRGB size must be 16 bytes");
static_assert(sizeof(PointXYZ) == 16, "PointXYZ size must be 16 bytes");

/**
 * PointCloudData message. Carries ROI (region of interest) and threshold for depth calculation
 */
class PointCloudData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawPointCloudData& pcl;
    std::vector<PointXYZRGB> pointsXYZRGB;
    std::vector<PointXYZ> pointsXYZ;

   public:
    using Buffer::getSequenceNum;
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;


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
     * Retrieves a list of points containing rgb data.
     */
    std::vector<PointXYZRGB> getPointsXYZRGB();

    /**
     * Retrieves a list of points not containing rgb data.
     */
    std::vector<PointXYZ> getPointsXYZ();

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr toPclData();

    /**
     * Converts PointCloudData to pcl::PointCloud<pcl::PointXYZRGB>
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr toPclRGBData();

#else
    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    void toPclData() const {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
    template <typename... T>
    void toPclRGBData() const {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
#endif
};

}  // namespace dai
