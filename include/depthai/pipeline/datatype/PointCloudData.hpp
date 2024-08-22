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

/**
 * PointCloudData message. Carries point cloud data.
 */
class PointCloudData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawPointCloudData& pcl;
    std::vector<Point3f> points;

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

    std::vector<Point3f>& getPoints();

    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves the height in pixels - in case of a sparse point cloud, this represents the hight of the frame which was used to generate the point cloud
     */
    unsigned int getWidth() const;

    /**
     * Retrieves the height in pixels - in case of a sparse point cloud, this represents the hight of the frame which was used to generate the point cloud
     */
    unsigned int getHeight() const;

    /**
     * Retrieves minimal x coordinate in depth units (millimeter by default)
     */
    float getMinX() const;

    /**
     * Retrieves minimal y coordinate in depth units (millimeter by default)
     */
    float getMinY() const;

    /**
     * Retrieves minimal z coordinate in depth units (millimeter by default)
     */
    float getMinZ() const;

    /**
     * Retrieves maximal x coordinate in depth units (millimeter by default)
     */
    float getMaxX() const;

    /**
     * Retrieves maximal y coordinate in depth units (millimeter by default)
     */
    float getMaxY() const;

    /**
     * Retrieves maximal z coordinate in depth units (millimeter by default)
     */
    float getMaxZ() const;

    /**
     * Retrieves whether point cloud is sparse
     */
    bool isSparse() const;

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
     * Specifies minimal x coordinate in depth units (millimeter by default)
     *
     * @param val minimal x coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMinX(float val);

    /**
     * Specifies minimal y coordinate in depth units (millimeter by default)
     *
     * @param val minimal y coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMinY(float val);

    /**
     * Specifies minimal z coordinate in depth units (millimeter by default)
     *
     * @param val minimal z coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMinZ(float val);

    /**
     * Specifies maximal x coordinate in depth units (millimeter by default)
     *
     * @param val maximal x coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMaxX(float val);

    /**
     * Specifies maximal y coordinate in depth units (millimeter by default)
     *
     * @param val maximal y coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMaxY(float val);

    /**
     * Specifies maximal z coordinate in depth units (millimeter by default)
     *
     * @param val maximal z coordinate in depth units (millimeter by default)
     */
    PointCloudData& setMaxZ(float val);

#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    /**
     * Converts PointCloudData to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPclData() const;

#else
    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    void getPclData() const {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
#endif
};

}  // namespace dai
