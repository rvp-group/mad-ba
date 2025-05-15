#include <srrg_system_utils/system_utils.h>
#include "srrg_converters/converter.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;

#define LOG_VARIABLE(VARIABLE_) \
    std::cerr << #VARIABLE_ << ": " << VARIABLE_ << std::endl

int main(int argc, char** argv) {

  uint16_t seq = 0;

  std::cerr << std::fixed << std::setprecision(4);
  // imu conversion
  {

    IMUMessagePtr imu_message_srrg(new IMUMessage("/imu", "/imu", seq++, getTime()));
    imu_message_srrg->angular_velocity.setValue(Vector3f(1.0, 1.2, 1.4));
    imu_message_srrg->linear_acceleration.setValue(Vector3f(2.0, 2.2, 2.4));
    imu_message_srrg->orientation.setValue(Quaternionf(1, 2, 3, 4).normalized().matrix());
    Matrix3f covariance;
    covariance << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cerr << "mat size " << covariance.size() << std::endl;
    imu_message_srrg->orientation_covariance.setValue(covariance);
    imu_message_srrg->angular_velocity_covariance.setValue(covariance * 2);
    imu_message_srrg->linear_acceleration_covariance.setValue(covariance * 3);

    std::cerr << "\nIMUMessage SRRG to ROS\n" << std::endl;
    sensor_msgs::ImuPtr imu_message_ros = Converter::convert(imu_message_srrg);

    LOG_VARIABLE(imu_message_ros->header.frame_id);
    LOG_VARIABLE(imu_message_ros->header.seq);
    LOG_VARIABLE(imu_message_ros->header.stamp);
    LOG_VARIABLE(imu_message_ros->orientation.x);
    LOG_VARIABLE(imu_message_ros->orientation.y);
    LOG_VARIABLE(imu_message_ros->orientation.z);
    LOG_VARIABLE(imu_message_ros->orientation.w);

    LOG_VARIABLE(imu_message_ros->angular_velocity.x);
    LOG_VARIABLE(imu_message_ros->angular_velocity.y);
    LOG_VARIABLE(imu_message_ros->angular_velocity.z);
    LOG_VARIABLE(imu_message_ros->linear_acceleration.x);
    LOG_VARIABLE(imu_message_ros->linear_acceleration.y);
    LOG_VARIABLE(imu_message_ros->linear_acceleration.z);

    Matrix3d orientation_covariance(imu_message_ros->orientation_covariance.data());
    LOG_VARIABLE(orientation_covariance);
    Matrix3d angular_velocity_covariance(imu_message_ros->angular_velocity_covariance.data());
    LOG_VARIABLE(angular_velocity_covariance);
    Matrix3d linear_acceleration_covariance(imu_message_ros->linear_acceleration_covariance.data());
    LOG_VARIABLE(linear_acceleration_covariance);

    std::cerr << "\nIMUMessage ROS to SRRG\n" << std::endl;
    imu_message_srrg = Converter::convert<IMUMessage>(imu_message_ros);

    if (imu_message_srrg) {
      LOG_VARIABLE(imu_message_srrg->frame_id.value());
      LOG_VARIABLE(imu_message_srrg->seq.value());
      LOG_VARIABLE(imu_message_srrg->timestamp.value());
//      LOG_VARIABLE(imu_message_srrg->orientation.value().x());
//      LOG_VARIABLE(imu_message_srrg->orientation.value().y());
//      LOG_VARIABLE(imu_message_srrg->orientation.value().z());
//      LOG_VARIABLE(imu_message_srrg->orientation.value().w());

      LOG_VARIABLE(imu_message_srrg->angular_velocity.value().transpose());
      LOG_VARIABLE(imu_message_srrg->linear_acceleration.value().transpose());

      LOG_VARIABLE(imu_message_srrg->orientation_covariance.value());
      LOG_VARIABLE(imu_message_srrg->angular_velocity_covariance.value());
      LOG_VARIABLE(imu_message_srrg->linear_acceleration_covariance.value());
      std::cerr << std::endl;
    }
  }
  // image message
  {

    int rows = 3;
    int cols = 3;

    const float step = 1 / (float)(rows * cols);
    ImageVector4f* v4f = new ImageVector4f(rows, cols);
    const float max_float = 3568486;
    std::cerr << "Creating Image" << std::endl;
    float value = 0;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const float scaled_float = value * max_float;
        v4f->at(r, c) = ImageVector4f::CellType(scaled_float, scaled_float, scaled_float, scaled_float);
        value += step;
        std::cerr << scaled_float << " ";
      }
      std::cerr << std::endl;
    }

    ImageMessagePtr image_message_srrg(new ImageMessage("/pinhole_image", "/pinhole_image", seq++, getTime()));

    image_message_srrg->setImage(v4f);
    std::cerr << "\nImageMessage SRRG to ROS\n" << std::endl;
    sensor_msgs::ImagePtr image_message_ros = Converter::convert(image_message_srrg);
    delete v4f;

    LOG_VARIABLE(image_message_ros->header.frame_id);
    LOG_VARIABLE(image_message_ros->header.seq);
    LOG_VARIABLE(image_message_ros->header.stamp);
    LOG_VARIABLE(image_message_ros->height);
    LOG_VARIABLE(image_message_ros->width);
    LOG_VARIABLE(image_message_ros->encoding);
    LOG_VARIABLE(image_message_ros->step);

    std::cerr << "Converted image" << std::endl;
    float* d = (float*)image_message_ros->data.data();

    int k = 0;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        for (int chan = 0; chan < sensor_msgs::image_encodings::numChannels(image_message_ros->encoding); ++chan) {
          std::cerr << d[k] << " ";
          ++k;
        }
      }
      std::cerr << std::endl;
    }

    std::cerr << "\nImageMessage ROS to SRRG\n" << std::endl;
    image_message_srrg = Converter::convert<ImageMessage>(image_message_ros);

    if (image_message_srrg) {
      LOG_VARIABLE(image_message_srrg->frame_id.value());
      LOG_VARIABLE(image_message_srrg->seq.value());
      LOG_VARIABLE(image_message_srrg->timestamp.value());
      LOG_VARIABLE(image_message_srrg->topic.value());
      LOG_VARIABLE(image_message_srrg->image()->type());
      LOG_VARIABLE(image_message_srrg->image()->rows());
      LOG_VARIABLE(image_message_srrg->image()->cols());
      LOG_VARIABLE(image_message_srrg->image()->numChannels());
    }

    std::cerr << "Reconverted image" << std::endl;
    cv::Mat m;
    image_message_srrg->image()->toCv(m);
    std::cerr << m << std::endl;
  }

  // camera info conversion
  {
    CameraInfoMessagePtr camera_info_message_srrg(new CameraInfoMessage("/camera_info", "/camera_info", seq++, getTime()));
    camera_info_message_srrg->distortion_coefficients.setValue(Vector4d(0, 1, 2, 3));
    camera_info_message_srrg->depth_scale.setValue(5);
    camera_info_message_srrg->projection_model.setValue("acazzodicane");
    camera_info_message_srrg->distortion_model.setValue("napalla");
    Matrix3f cammat;
    cammat << 10, 0, 5, 0, 10, 5, 0, 0, 1;
    camera_info_message_srrg->camera_matrix.setValue(cammat);

    std::cerr << "\nIMUMessage SRRG to ROS\n" << std::endl;
    sensor_msgs::CameraInfoPtr camera_info_message_ros = Converter::convert(camera_info_message_srrg);

    LOG_VARIABLE(camera_info_message_ros->header.frame_id);
    LOG_VARIABLE(camera_info_message_ros->header.seq);
    LOG_VARIABLE(camera_info_message_ros->header.stamp);
    LOG_VARIABLE(camera_info_message_ros->distortion_model);
    Eigen::VectorXd distortion(camera_info_message_ros->D.size());
    memcpy(distortion.data(), camera_info_message_ros->D.data(), camera_info_message_ros->D.size() * sizeof(double));
    LOG_VARIABLE(distortion.transpose());
    Matrix3d cammad(camera_info_message_ros->K.data());
    LOG_VARIABLE(cammad);

    std::cerr << "\nImageMessage ROS to SRRG\n" << std::endl;
    camera_info_message_srrg = Converter::convert<CameraInfoMessage>(camera_info_message_ros);
    if (camera_info_message_srrg) {
      LOG_VARIABLE(camera_info_message_srrg->frame_id.value());
      LOG_VARIABLE(camera_info_message_srrg->seq.value());
      LOG_VARIABLE(camera_info_message_srrg->timestamp.value());
      LOG_VARIABLE(camera_info_message_srrg->topic.value());
      LOG_VARIABLE(camera_info_message_srrg->distortion_model.value());
      LOG_VARIABLE(camera_info_message_srrg->projection_model.value());
      std::wcerr << "depth scale is not a member of ros sensor_msgs::CameraInfo" << std::endl;
      LOG_VARIABLE(camera_info_message_srrg->depth_scale.value());
      LOG_VARIABLE(camera_info_message_srrg->distortion_coefficients.value().transpose());
      LOG_VARIABLE(camera_info_message_srrg->camera_matrix.value());
    }
  }

  // laser conversion
  {
    LaserMessagePtr laser_message_srrg(new LaserMessage("/scan", "/scan", seq++, getTime()));
    laser_message_srrg->angle_min.setValue(-100);
    laser_message_srrg->angle_max.setValue(100);
    laser_message_srrg->angle_increment.setValue(1);
    laser_message_srrg->time_increment.setValue(0.3);
    laser_message_srrg->scan_time.setValue(1);
    laser_message_srrg->range_min.setValue(0.5);
    laser_message_srrg->range_max.setValue(20);
    laser_message_srrg->ranges.setValue(std::vector<float>(5, 1.3));
    laser_message_srrg->intensities.setValue(std::vector<float>(5, 7));

    std::cerr << "\nIMUMessage SRRG to ROS\n" << std::endl;
    sensor_msgs::LaserScanPtr laser_message_ros = Converter::convert(laser_message_srrg);

    LOG_VARIABLE(laser_message_ros->header.frame_id);
    LOG_VARIABLE(laser_message_ros->header.seq);
    LOG_VARIABLE(laser_message_ros->header.stamp);
    LOG_VARIABLE(laser_message_ros->angle_min);
    LOG_VARIABLE(laser_message_ros->angle_max);
    LOG_VARIABLE(laser_message_ros->angle_increment);
    LOG_VARIABLE(laser_message_ros->time_increment);
    LOG_VARIABLE(laser_message_ros->scan_time);
    LOG_VARIABLE(laser_message_ros->range_min);
    LOG_VARIABLE(laser_message_ros->range_max);
    LOG_VARIABLE(laser_message_ros->ranges[0]);
    LOG_VARIABLE(laser_message_ros->intensities[0]);

    std::cerr << "\nImageMessage ROS to SRRG\n" << std::endl;
    laser_message_srrg = Converter::convert<LaserMessage>(laser_message_ros);
    if (laser_message_srrg) {
      LOG_VARIABLE(laser_message_srrg->frame_id.value());
      LOG_VARIABLE(laser_message_srrg->seq.value());
      LOG_VARIABLE(laser_message_srrg->timestamp.value());
      LOG_VARIABLE(laser_message_srrg->topic.value());
      LOG_VARIABLE(laser_message_srrg->angle_min.value());
      LOG_VARIABLE(laser_message_srrg->angle_max.value());
      LOG_VARIABLE(laser_message_srrg->angle_increment.value());
      LOG_VARIABLE(laser_message_srrg->time_increment.value());
      LOG_VARIABLE(laser_message_srrg->scan_time.value());
      LOG_VARIABLE(laser_message_srrg->range_min.value());
      LOG_VARIABLE(laser_message_srrg->range_max.value());
      LOG_VARIABLE(laser_message_srrg->ranges.value(0));
      LOG_VARIABLE(laser_message_srrg->intensities.value(0));
    }
  }

  // range conversion
  {
    RangeMessagePtr range_message_srrg(new RangeMessage("/uscan", "/uscan", seq++, getTime()));

    range_message_srrg->range_min.setValue(0.5);
    range_message_srrg->range_max.setValue(20);
    range_message_srrg->field_of_view.setValue(20);
    range_message_srrg->range.setValue(1.4);
    range_message_srrg->radiation_type.setValue(1);

    std::cerr << "\nIMUMessage SRRG to ROS\n" << std::endl;
    sensor_msgs::RangePtr range_message_ros = Converter::convert(range_message_srrg);

    LOG_VARIABLE(range_message_ros->header.frame_id);
    LOG_VARIABLE(range_message_ros->header.seq);
    LOG_VARIABLE(range_message_ros->header.stamp);
    LOG_VARIABLE(range_message_ros->field_of_view);
    LOG_VARIABLE(range_message_ros->min_range);
    LOG_VARIABLE(range_message_ros->max_range);
    LOG_VARIABLE(range_message_ros->range);
    LOG_VARIABLE((int)range_message_ros->radiation_type);

    std::cerr << "\nImageMessage ROS to SRRG\n" << std::endl;
    range_message_srrg = Converter::convert<RangeMessage>(range_message_ros);
    if (range_message_srrg) {
      LOG_VARIABLE(range_message_srrg->frame_id.value());
      LOG_VARIABLE(range_message_srrg->seq.value());
      LOG_VARIABLE(range_message_srrg->timestamp.value());
      LOG_VARIABLE(range_message_srrg->topic.value());
      LOG_VARIABLE(range_message_srrg->field_of_view.value());
      LOG_VARIABLE(range_message_srrg->range_min.value());
      LOG_VARIABLE(range_message_srrg->range_max.value());
      LOG_VARIABLE(range_message_srrg->range.value());
      LOG_VARIABLE(range_message_srrg->radiation_type.value());
    }
  }

  return 0;
}
