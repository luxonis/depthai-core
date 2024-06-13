#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
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
    unsigned int width;        // width in pixels
    unsigned int height;       // height in pixels
    uint32_t instanceNum = 0;  // Which source created this frame (color, mono, ...)
    float minx, miny, minz;
    float maxx, maxy, maxz;
    bool sparse = false;

   public:
    using Buffer::getSequenceNum;
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;

    /**
     * Construct PointCloudData message.
     */
    PointCloudData() = default;
    virtual ~PointCloudData() = default;

    std::vector<Point3f> getPoints();

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

    /**
     * Specifies whether point cloud is sparse
     *
     * @param val whether point cloud is sparse
     */
    PointCloudData& setSparse(bool val);

    /**
     * Specifies instance number
     *
     * @param instanceNum instance number
     */
    PointCloudData& setInstanceNum(unsigned int instanceNum);

#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    /**
     * Converts PointCloudData to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPclData() const;
    void setPclData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void setPclData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
#else
    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    void getPclData() const {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
    template <typename... T>
    void setPclData(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with PCL support");
    }
#endif
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::PointCloudData;
    };
    DEPTHAI_SERIALIZE(
        PointCloudData, width, height, minx, miny, minz, maxx, maxy, maxz, sparse, instanceNum, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai
