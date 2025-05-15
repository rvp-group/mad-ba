#pragma once
#include "base_sensor_message.h"
#include "point_field.h"
#include "srrg_property/property_serializable.h"
#include "srrg_property/property_vector.h"
#include <srrg_pcl/point_types.h>
#include <srrg_property/vector_data.h>

namespace srrg2_core {

  using PointCloud2Data              = VectorData_<std::vector<uint8_t>>;
  using PointCloud2DataBLOBReference = BLOBReference<PointCloud2Data>;

  class PointCloud2Message : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloud2Message(const std::string& topic_    = "",
                       const std::string& frame_id_ = "",
                       const int& seq_              = -1,
                       const double& timestamp_     = -1.0);

    //! @brief 2D structure of the point cloud. If the cloud is unordered, height is 1 and width is
    //! the length of the point cloud.
    PropertyUnsignedInt height;
    PropertyUnsignedInt width;

    //! @brief flag that says if the data is bigEndian or not
    PropertyBool is_bigendian;

    //! @brief since the data is stored in a unique buffer and for each point data I have different
    //! attributes (that have different sizes), this values tells me of how much I should move in
    //! the buffer to get the next point
    PropertyUnsignedInt point_step;

    //! @brief same as the point step but for rows (if data is organized in a 2D matrix)
    PropertyUnsignedInt row_step;

    //! @brief available attibutes for each point
    PropertySerializableVector_<PointField> fields;

    //! @brief the actual raw buffer
    PropertySerializable_<PointCloud2DataBLOBReference> data;

    //! @brief this flag is true if there are no invalid points in the cloud
    PropertyBool is_dense;

    //! @brief memcopy of the buffer
    void setRawData(const std::vector<uint8_t>& data_);

    //! @brief given a point cloud fills all the attributes according to the cloud properties
    void setPointCloud(const Point3fVectorCloud& in_cloud_);
    void setPointCloud(const PointIntensity3fVectorCloud& in_cloud_);

    //! @brief given the complete copy of the raw message, read the data and puts in a intellegible
    //! srrg2 format. only coordinates are tranformed here
    void getPointCloud(Point3fVectorCloud& out_cloud_);

    //! @brief given the complete copy of the raw message, read the data and puts in a intellegible
    //!        srrg2 format. coordinates and intensities are tranformed here
    //! @param[out] out_cloud_: srrg2 point cloud with points and intensity
    //! @param[in] x_field_num_: the index in the field vector for the x-coordinate point attribute
    //!                          (might change depending on the driver implementation)
    //! @param[in] y_field_num_: the index in the field vector for the y-coordinate point attribute
    //!                          (might change depending on the driver implementation)
    //! @param[in] z_field_num_: the index in the field vector for the z-coordinate point attribute
    //!                          (might change depending on the driver implementation)
    //! @param[in] intensity_field_num_: the index in the field vector for the intensity point
    //!                                  attribute (might change depending on the driver
    //!                                  implementation)
    void getPointCloud(PointIntensity3fVectorCloud& out_cloud_,
                       const size_t& x_field_num_         = 0,
                       const size_t& y_field_num_         = 1,
                       const size_t& z_field_num_         = 2,
                       const size_t& intensity_field_num_ = 3);
  };

  using PointCloud2MessagePtr = std::shared_ptr<PointCloud2Message>;

} // namespace srrg2_core
