#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/ProtoSerializable.hpp"

// optional
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
#endif

namespace dai {

/**
 * PointCloudData message. Carries point cloud data.
 */
class PointCloudData : public Buffer, public ProtoSerializable {
    unsigned int width;        // width in pixels (for organized) or number of points (for unorganized)
    unsigned int height;       // height in pixels (for organized) or 1 (for unorganized)
    uint32_t instanceNum = 0;  // Which source created this frame
    float minx, miny, minz;
    float maxx, maxy, maxz;
    
    bool color = false;

   public:
    using Buffer::getSequenceNum;
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;

    /**
     * Construct PointCloudData message.
     */
    PointCloudData() = default;
    virtual ~PointCloudData();

    std::vector<Point3f> getPoints();
    std::vector<Point3fRGBA> getPointsRGB();
    void setPoints(const std::vector<Point3f>& points);
    void setPointsRGB(const std::vector<Point3fRGBA>& points);

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
     * @deprecated Use isOrganized() instead. Sparse means height == 1
     */
    [[deprecated("isSparse is deprecated, use isOrganized() instead (isSparse == !isOrganized())")]]
    bool isSparse() const;

    /**
     * Retrieves whether point cloud is organized (height > 1)
     * Organized point clouds have width x height structure from the original image
     * Sparse point clouds have height == 1 and only contain valid points
     */
    bool isOrganized() const;

    /**
     * Retrieves whether point cloud is color
     */
    bool isColor() const;

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
     * @deprecated This setter is deprecated and will be removed in a future release
     */
    [[deprecated("setSparse is deprecated, width and height are set automatically based on organization")]]
    PointCloudData& setSparse(bool val);

    /**
     * Specifies whether point cloud is color
     *
     * @param val whether point cloud is color
     */
    PointCloudData& setColor(bool val);

    /**
     * Specifies instance number
     *
     * @param instanceNum instance number
     */
    PointCloudData& setInstanceNum(unsigned int instanceNum);

    /**
     * Recomputes the bounding box (min/max X, Y, Z) from the current point data.
     * Only considers valid points (z > 0). If no valid points exist, all bounds are set to 0.
     */
    PointCloudData& updateBoundingBox();

#ifdef DEPTHAI_ENABLE_PROTOBUF
    /**
     * Serialize message to proto buffer
     *
     * @returns serialized message
     */
    std::vector<std::uint8_t> serializeProto(bool metadataOnly = false) const override;

    /**
     * Serialize schema to proto buffer
     *
     * @returns serialized schema
     */
    ProtoSerializable::SchemaPair serializeSchema() const override;
#endif

#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    /**
     * Converts PointCloudData to pcl::PointCloud<pcl::PointXYZ>
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPclData() const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPclDataRGB() const;
    void setPclData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void setPclData(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void setPclDataRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
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
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PointCloudData;
    }
    DEPTHAI_SERIALIZE(
        PointCloudData, width, height, minx, miny, minz, maxx, maxy, maxz, instanceNum, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai
