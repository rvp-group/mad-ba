#include "point_cloud2_message.h"

namespace srrg2_core {

  PointCloud2Message::PointCloud2Message(const std::string& topic_,
                                         const std::string& frame_id_,
                                         const int& seq_,
                                         const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(height, 0),
    SETUP_PROPERTY(width, 0),
    SETUP_PROPERTY(is_bigendian, false),
    SETUP_PROPERTY(point_step, 0),
    SETUP_PROPERTY(row_step, 0),
    SETUP_PROPERTY_NV(fields),
    SETUP_PROPERTY_NV(data),
    SETUP_PROPERTY(is_dense, false) {
  }

  void PointCloud2Message::setRawData(const std::vector<uint8_t>& data_) {
    srrg2_core::PointCloud2Data* data_out = new srrg2_core::PointCloud2Data();
    const int data_size                   = data_.size();
    const size_t out_size                 = data_size * sizeof(uint8_t);
    data_out->resize(out_size);
    std::memcpy(&(data_out->at(0)), &(data_[0]), out_size);
    data.value().set(data_out);
  }

  void PointCloud2Message::setPointCloud(const Point3fVectorCloud& in_cloud_) {
    using PointType = Point3fVectorCloud::PointType;
    height.setValue(in_cloud_.size());
    width.setValue(1);
    point_step.setValue(sizeof(Vector3f));
    row_step.setValue(point_step.value());
    is_dense.setValue(true);

    // srrg hardcode this for now
    fields.resize(3);
    for (size_t i = 0; i < fields.size(); ++i) {
      fields.value(i).offset.setValue(i * sizeof(float));
      fields.value(i).datatype.setValue(PointField::FLOAT32);
      fields.value(i).count.setValue(1);
    }

    std::vector<Vector3f> tmp;
    tmp.reserve(in_cloud_.size());
    for (const PointType& p : in_cloud_) {
      tmp.emplace_back(p.coordinates());
    }

    PointCloud2Data* data_out = new PointCloud2Data();
    const size_t data_size    = tmp.size() * point_step.value();
    const size_t out_size     = data_size * sizeof(uint8_t);
    data_out->resize(out_size);
    std::memcpy(&(data_out->at(0)), &(tmp[0]), out_size);
    data.value().set(data_out);
  }

  void PointCloud2Message::setPointCloud(const PointIntensity3fVectorCloud& in_cloud_) {
    using PointType         = PointIntensity3fVectorCloud::PointType;
    const size_t num_fields = 4; // ia x y z intensity
    const size_t step_size  = num_fields * sizeof(float);

    // ia easy
    height.setValue(in_cloud_.size());
    width.setValue(1);

    // ia the value of the step while reading this message
    point_step.setValue(step_size);
    row_step.setValue(point_step.value());
    is_dense.setValue(true);

    // ia harcoded fields ( x y z intensity ) - luckily all floats
    fields.resize(num_fields);
    for (size_t i = 0; i < fields.size(); ++i) {
      fields.value(i).offset.setValue(i * sizeof(float));
      fields.value(i).datatype.setValue(PointField::FLOAT32);
      fields.value(i).count.setValue(1);
    }

    // ia avoid Eigen alignement problems
    struct PointPODStruct {
      PointPODStruct(const float& x_, const float& y_, const float& z_, const float& i_) :
        x(x_),
        y(y_),
        z(z_),
        i(i_) {
      }
      const float x;
      const float y;
      const float z;
      const float i;
    };

    // ia copy the raw data (aka floats)
    std::vector<PointPODStruct> plain_data;
    plain_data.reserve(in_cloud_.size());
    for (const PointType& p : in_cloud_) {
      PointPODStruct pod(
        p.coordinates().x(), p.coordinates().y(), p.coordinates().z(), p.intensity());
      plain_data.emplace_back(pod);
    }
    assert(plain_data.size() == in_cloud_.size() &&
           "PointCloud2Message::setPointCloud|ERROR, pod size mismatch");

    // ia construct a propertycontainerdynamicidentifiebleregistereddynamicptrbossptrptrjesusptr
    PointCloud2Data* data_out = new PointCloud2Data(); // ia terrible name, it'a a buffer
    const size_t out_size     = step_size * plain_data.size() * sizeof(uint8_t);
    data_out->resize(out_size);
    std::memcpy(&(data_out->at(0)), &(plain_data[0]), out_size);
    data.value().set(data_out);
  }

  void PointCloud2Message::getPointCloud(Point3fVectorCloud& out_cloud_) {
    const size_t cloud_size = (height.value() * width.value());
    out_cloud_.resize(cloud_size);

    // ia get the srrg2 serializable object that contains the raw buffer
    PointCloud2Data* cloud_data = data.value().get();
    if (!cloud_data) {
      throw std::runtime_error("PointCloud2Message::getPointCloud|invalid cloud data");
    }
    // ia get the actual raw buffer
    uint8_t* raw_buffer = cloud_data->data();
    if (!raw_buffer) {
      throw std::runtime_error("PointCloud2Message::getPointCloud|invalid raw buffer");
    }

    // ia how many attributes do we have for each point?
    const size_t field_size = fields.size();

    size_t num_clean_points = 0;
    for (size_t i = 0; i < cloud_size; ++i) {
      Vector3f p = Vector3f::Zero();
      // bdc iterate over the fields
      for (size_t field = 0; field < field_size; ++field) {
        const size_t& current_offset = fields.value(field).offset.value();

        if (field < 3) {
          // bdc the datatype field may vary (float / uint16 / ...), so let's
          // use the field offset
          float* b = (float*) (raw_buffer + current_offset);
          p(field) = *b;
        }
      }

      // bdc move the buffer pointer using the provided point_step
      raw_buffer += point_step.value();
      const float p_norm = p.norm();
      if (p_norm < 1e3 && p_norm > 1e-3) {
        out_cloud_[num_clean_points].coordinates() = p;
        ++num_clean_points;
      }
    }
    out_cloud_.resize(num_clean_points);
  }

  void PointCloud2Message::getPointCloud(PointIntensity3fVectorCloud& out_cloud_,
                                         const size_t& x_field_num_,
                                         const size_t& y_field_num_,
                                         const size_t& z_field_num_,
                                         const size_t& intensity_field_num_) {
    // ia check message is well formed
    assert(data.value().get() && "PointCloud2Message::getPointCloud|invalid data vector");
    assert(data.value().get()->data() && "PointCloud2Message::getPointCloud|invalid raw buffer");

    // ia get the raw buffer
    uint8_t* raw_buffer = data.value().get()->data();

    //    // ia get the number of fields
    //    const size_t& num_fields = fields.size();
    //
    // // ia let's see whats in the inside
    // for (size_t i = 0; i < num_fields; ++i) {
    //   const auto& field = fields.value(i);
    //   std::cerr << "field [ " << i << " ] - " << field << std::endl;
    // }

    // ia allocate the point cloud
    const size_t num_raw_points = (height.value() * width.value());
    out_cloud_.reserve(num_raw_points);

    // ia cache field offsets
    const size_t& x_offset         = fields.value(x_field_num_).offset.value();
    const size_t& y_offset         = fields.value(y_field_num_).offset.value();
    const size_t& z_offset         = fields.value(z_field_num_).offset.value();
    const size_t& intensity_offset = fields.value(intensity_field_num_).offset.value();

    float* value_ptr = nullptr;
    for (size_t k = 0; k < num_raw_points; ++k) {
      // ia gather point attributes from buffer
      value_ptr     = (float*) (raw_buffer + x_offset);
      float x_coord = *value_ptr;

      value_ptr     = (float*) (raw_buffer + y_offset);
      float y_coord = *value_ptr;

      value_ptr     = (float*) (raw_buffer + z_offset);
      float z_coord = *value_ptr;

      value_ptr       = (float*) (raw_buffer + intensity_offset);
      float intensity = *value_ptr;

      // ia finished reading, move the buffer forward
      raw_buffer += point_step.value();

      // ia ensemble coordinates
      srrg2_core::Vector3f coordinates(x_coord, y_coord, z_coord);

      // ia skip invalid points
      const float coordinates_norm = coordinates.norm();
      if (coordinates_norm > 1e3 && coordinates_norm < 1e-3) {
        continue;
      }

      // ia build the point
      PointIntensity3f point;
      point.coordinates() = coordinates;
      point.intensity()   = intensity;

      // ia insert the point in our point cloud
      out_cloud_.emplace_back(point);
    }
  }

} // namespace srrg2_core
