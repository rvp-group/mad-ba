#include <sensor_msgs/image_encodings.h>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_property/property_container.h>
#include <srrg_system_utils/system_utils.h>

#include "converter.h"

namespace srrg2_core_ros {

  // gg: mamma che merda
  inline void iso2rosPose(geometry_msgs::Pose& dest, const srrg2_core::Vector6f& src) {
    Eigen::Isometry3f iso = srrg2_core::geometry3d::v2t(src);
    Eigen::Quaternionf q(iso.linear());
    dest.position.x    = src(0);
    dest.position.y    = src(1);
    dest.position.z    = src(2);
    dest.orientation.x = q.x();
    dest.orientation.y = q.y();
    dest.orientation.z = q.z();
    dest.orientation.w = q.w();
  }

  inline void iso2rosPose(geometry_msgs::Pose& dest, const srrg2_core::Isometry3f& src) {
    iso2rosPose(dest, srrg2_core::geometry3d::t2v(src));
  }

  inline void rosPose2iso(srrg2_core::Vector6f& dest, const geometry_msgs::Pose& src) {
    dest(0) = src.position.x;
    dest(1) = src.position.y;
    dest(2) = src.position.z;
    dest(3) = src.orientation.x;
    dest(4) = src.orientation.y;
    dest(5) = src.orientation.z;
    if (src.orientation.w < 0.f) {
      dest.block<3, 1>(3, 0) = -dest.block<3, 1>(3, 0);
    }
  }

  inline void rosPose2iso(srrg2_core::Isometry3f& dest, const geometry_msgs::Pose& src) {
    srrg2_core::Vector6f dest_;
    rosPose2iso(dest_, src);
    dest = srrg2_core::geometry3d::v2t(dest_);
  }

  //--------------------------------------------------------------//
  //-------------------   ROS -> SRRG   --------------------------//
  //--------------------------------------------------------------//

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::CameraInfoConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::CameraInfoMessagePtr message_out(new srrg2_core::CameraInfoMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    message_out->distortion_model.setValue(message_in_->distortion_model);

    ROS_TO_SRRG_VECX(message_out->distortion_coefficients, message_in_->D);
    ROS_TO_SRRG_MAT3(message_out->camera_matrix, message_in_->K);

    // srrg these values are not stored in the ros message
    message_out->projection_model.setValue("pinhole");
    message_out->depth_scale.setValue(1e-3);
    message_out->rows.setValue(message_in_->height);
    message_out->cols.setValue(message_in_->width);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::ImageConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::ImageMessagePtr message_out(new srrg2_core::ImageMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(message_in_, message_in_->encoding);
    } catch (cv_bridge::Exception& e) {
      std::cerr << "cv_bridge exception: %s" << e.what() << std::endl;
      return 0;
    }

    srrg2_core::BaseImage* img_data = srrg2_core::BaseImage::newImagefromCv(cv_ptr->image);
    message_out->setImage(img_data);
    message_out->image_cols.setValue(img_data->cols());
    message_out->image_rows.setValue(img_data->rows());
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::ImuConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::IMUMessagePtr message_out(new srrg2_core::IMUMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    ROS_TO_SRRG_QUATERNION(message_out->orientation, message_in_->orientation);
    ROS_TO_SRRG_VEC3(message_out->angular_velocity, message_in_->angular_velocity);
    ROS_TO_SRRG_VEC3(message_out->linear_acceleration, message_in_->linear_acceleration);

    ROS_TO_SRRG_MAT3(message_out->orientation_covariance, message_in_->orientation_covariance);
    float ocn = message_out->orientation_covariance.value().norm();
    if (std::isnan(ocn) || std::isinf(ocn)) {
      return nullptr;
    }
    ROS_TO_SRRG_MAT3(message_out->angular_velocity_covariance,
                     message_in_->angular_velocity_covariance);
    float avcn = message_out->orientation_covariance.value().norm();
    if (std::isnan(avcn) || std::isinf(avcn)) {
      return nullptr;
    }

    ROS_TO_SRRG_MAT3(message_out->linear_acceleration_covariance,
                     message_in_->linear_acceleration_covariance);
    float lacn = message_out->orientation_covariance.value().norm();
    if (std::isnan(lacn) || std::isinf(lacn)) {
      return nullptr;
    }

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::LaserScanConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::LaserMessagePtr message_out(new srrg2_core::LaserMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    message_out->angle_min.setValue(message_in_->angle_min);
    message_out->angle_max.setValue(message_in_->angle_max);
    message_out->angle_increment.setValue(message_in_->angle_increment);
    message_out->time_increment.setValue(message_in_->time_increment);
    message_out->scan_time.setValue(message_in_->scan_time);
    message_out->range_max.setValue(message_in_->range_max);
    message_out->range_min.setValue(message_in_->range_min);

    message_out->ranges.setValue(message_in_->ranges);
    message_out->intensities.setValue(message_in_->intensities);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(nav_msgs::OdometryConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::OdometryMessagePtr message_out(new srrg2_core::OdometryMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    message_out->child_frame.setValue(message_in_->child_frame_id);

    ROS_TO_SRRG_POSE(message_out->pose, message_in_->pose);
    ROS_TO_SRRG_VEC3(message_out->linear_velocity, message_in_->twist.twist.linear);
    ROS_TO_SRRG_VEC3(message_out->angular_velocity, message_in_->twist.twist.angular);

    ROS_TO_SRRG_MAT6(message_out->pose_covariance, message_in_->pose.covariance);
    ROS_TO_SRRG_MAT6(message_out->twist_covariance, message_in_->twist.covariance);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(geometry_msgs::PointStampedConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::PointStampedMessagePtr message_out(new srrg2_core::PointStampedMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    ROS_TO_SRRG_VEC3(message_out->point, message_in_->point);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::RangeConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::RangeMessagePtr message_out(new srrg2_core::RangeMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    message_out->range.setValue(message_in_->range);
    message_out->radiation_type.setValue(message_in_->radiation_type);
    message_out->field_of_view.setValue(message_in_->field_of_view);
    message_out->range_max.setValue(message_in_->max_range);
    message_out->range_min.setValue(message_in_->min_range);
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(tf2_msgs::TFMessageConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }
    srrg2_core::TransformEventsMessagePtr message_out(new srrg2_core::TransformEventsMessage);
    message_out->events.resize(message_in_->transforms.size());

    double time = 0;
    // ds we can have multiple transform events per message (ROS)
    for (uint32_t u = 0; u < message_in_->transforms.size(); ++u) {
      // ds convert to transform event and store it
      srrg2_core::TransformEvent te = Converter::_convert(message_in_->transforms[u]);
      if (te.timeSeconds() > time) {
        time = te.timeSeconds();
      }
      message_out->events.setValue(u, te);
    }
    message_out->timestamp.setValue(time);
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(geometry_msgs::TwistStampedConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::TwistStampedMessagePtr message_out(new srrg2_core::TwistStampedMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    ROS_TO_SRRG_VEC3(message_out->linear, message_in_->twist.linear);
    ROS_TO_SRRG_VEC3(message_out->angular, message_in_->twist.angular);

    float twln = message_out->linear.value().norm();
    if (std::isnan(twln) || std::isinf(twln)) {
      return nullptr;
    }

    float twan = message_out->angular.value().norm();
    if (std::isnan(twan) || std::isinf(twan)) {
      return nullptr;
    }

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(geometry_msgs::TwistConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::TwistStampedMessagePtr message_out(new srrg2_core::TwistStampedMessage);

    ROS_TO_SRRG_VEC3(message_out->linear, message_in_->linear);
    ROS_TO_SRRG_VEC3(message_out->angular, message_in_->angular);

    float twln = message_out->linear.value().norm();
    if (std::isnan(twln) || std::isinf(twln)) {
      return nullptr;
    }

    float twan = message_out->angular.value().norm();
    if (std::isnan(twan) || std::isinf(twan)) {
      return nullptr;
    }

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(sensor_msgs::PointCloud2ConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::PointCloud2MessagePtr message_out(new srrg2_core::PointCloud2Message);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    message_out->height.setValue(message_in_->height);
    message_out->width.setValue(message_in_->width);
    message_out->fields.resize(message_in_->fields.size());
    for (size_t i = 0; i < message_in_->fields.size(); ++i) {
      message_out->fields.value(i).name.setValue(message_in_->fields[i].name);
      message_out->fields.value(i).datatype.setValue(message_in_->fields[i].datatype);
      message_out->fields.value(i).offset.setValue(message_in_->fields[i].offset);
      message_out->fields.value(i).count.setValue(message_in_->fields[i].count);
    }
    message_out->is_bigendian.setValue(message_in_->is_bigendian);
    message_out->point_step.setValue(message_in_->point_step);
    message_out->row_step.setValue(message_in_->row_step);

    message_out->setRawData(message_in_->data);

    message_out->is_dense.setValue(message_in_->is_dense);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::JointStateConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::JointsMessagePtr message_out(new srrg2_core::JointsMessage);

    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    const size_t joints_size = message_in_->position.size();
    message_out->joint_events.resize(joints_size);
    for (size_t i = 0; i < joints_size; ++i) {
      srrg2_core::JointEvent joint_event(message_in_->header.stamp.toSec(),
                                         message_in_->name[i],
                                         message_in_->position[i],
                                         message_in_->velocity[i],
                                         message_in_->effort[i]);
      message_out->joint_events.setValue(i, joint_event);
    }

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(sensor_msgs::NavSatFixConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    // ia check if message is well formed
    assert(message_in_->status.status < 3 && message_in_->status.status > -2 &&
           "invalid fix status");
    assert(message_in_->status.service > 0 && message_in_->status.service < 9 && "invalid service");
    assert(message_in_->position_covariance_type < 4 && "invalid covariance type");

    // ia create the message and populate the header
    srrg2_core::NavsatFixMessagePtr message_out(new srrg2_core::NavsatFixMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));

    // ia start to actually convert the message
    message_out->status.setValue(message_in_->status.status);
    message_out->service.setValue(message_in_->status.service);
    message_out->covariance_type.setValue(message_in_->position_covariance_type);
    message_out->latitude.setValue(message_in_->latitude);
    message_out->longitude.setValue(message_in_->longitude);

    // ia take altitude if present (otherwise leave default invalid values)
    if (!std::isnan(message_in_->altitude)) {
      message_out->has_altitude.setValue(true);
      message_out->altitude.setValue(message_in_->altitude);
    }

    // ia copy the goddamn position covariance
    ROS_TO_SRRG_MAT3(message_out->covariance, message_in_->position_covariance);

    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(geometry_msgs::PoseArrayConstPtr message_in_) {
    if (!message_in_)
      return nullptr;
    srrg2_core::PoseArrayMessagePtr message_out(new srrg2_core::PoseArrayMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    message_out->poses.resize(message_in_->poses.size());
    for (size_t i = 0; i < message_in_->poses.size(); ++i) {
      srrg2_core::PoseMessage& p_out  = message_out->poses.value(i);
      const geometry_msgs::Pose& p_in = message_in_->poses[i];
      rosPose2iso(p_out.pose_vector.value(), p_in);
    }
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(geometry_msgs::PoseStampedConstPtr message_in_) {
    if (!message_in_)
      return nullptr;
    srrg2_core::PoseStampedMessagePtr message_out(new srrg2_core::PoseStampedMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    rosPose2iso(message_out->pose.value().pose_vector.value(), message_in_->pose);
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(geometry_msgs::PoseWithCovarianceStampedConstPtr message_in_) {
    if (!message_in_)
      return nullptr;
    srrg2_core::PoseWithCovarianceStampedMessagePtr message_out(
      new srrg2_core::PoseWithCovarianceStampedMessage);
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    rosPose2iso(message_out->pose.value().pose_vector.value(), message_in_->pose.pose);

    int k = 0;
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c, ++k)
        message_out->covariance.value()(r, c) = message_in_->pose.covariance[k];
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(nav_msgs::PathConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }
    srrg2_core::PathMessagePtr message_out(new srrg2_core::PathMessage);
    srrg2_core::PoseStampedMessage pose_stamped;
    ROS_TO_SRRG_HEADER((*message_out), (*message_in_));
    for (auto it = message_in_->poses.begin(); it != message_in_->poses.end(); ++it) {
      ROS_TO_SRRG_HEADER(pose_stamped, (*it));
      rosPose2iso(pose_stamped.pose.value().pose_vector.value(), it->pose);
      message_out->poses.pushBack(pose_stamped);
    }
    return message_out;
  }

  srrg2_core::BaseSensorMessagePtr Converter::convert(grid_map_msgs::GridMapConstPtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }
    srrg2_core::GridMapMessagePtr message_out(new srrg2_core::GridMapMessage);
    message_out->timestamp.setValue(message_in_->info.header.stamp.toSec());
    message_out->frame_id.setValue(message_in_->info.header.frame_id);
    message_out->grid_map_struct.value().setResolution(message_in_->info.resolution);
    message_out->grid_map_struct.value().setSize(
      srrg2_core::Vector2i(message_in_->info.length_x, message_in_->info.length_y));
    Eigen::Quaternionf rotation_quaternion(srrg2_core::Matrix3f::Identity());
    rotation_quaternion.x()            = message_in_->info.pose.orientation.x;
    rotation_quaternion.y()            = message_in_->info.pose.orientation.y;
    rotation_quaternion.z()            = message_in_->info.pose.orientation.z;
    rotation_quaternion.w()            = message_in_->info.pose.orientation.w;
    srrg2_core::Isometry3f isometry_3D = srrg2_core::Isometry3f::Identity();
    isometry_3D.translation()          = srrg2_core::Vector3f(message_in_->info.pose.position.x,
                                                     message_in_->info.pose.position.y,
                                                     message_in_->info.pose.position.z);
    isometry_3D.linear()               = rotation_quaternion.normalized().toRotationMatrix();
    srrg2_core::Isometry2f isometry_2D = srrg2_core::Isometry2f::Identity();
    isometry_2D                        = srrg2_core::geometry3d::get2dFrom3dPose(isometry_3D);
    message_out->grid_map_struct.value().setOrigin(isometry_2D);
    if (message_in_->layers.size() != message_in_->data.size()) {
      std::cerr << FG_BBLUE("Different numbers of layers and data in grid map message.")
                << std::endl;
      return nullptr;
    }
    for (size_t i = 0; i < message_in_->layers.size(); ++i) {
      // bb check that the current layer is not empty
      // bb data has same dimension of layers and therefore data[i] exists
      const std_msgs::Float32MultiArray& current_layer_data = message_in_->data[i];
      const std::string& current_layer_name                 = message_in_->layers[i];
      if (message_in_->data[i].layout.dim.size() == 2) {
        size_t rows_ = current_layer_data.layout.dim[0].size;
        size_t cols_ = current_layer_data.layout.dim[1].size;
        // bb create a layer that has name layers[i] and as property a matrix of float
        srrg2_core::Property_<srrg2_core::Matrix_<float>>* property_layer =
          message_out->grid_map_struct.value()
            .property<srrg2_core::Property_<srrg2_core::Matrix_<float>>>(current_layer_name);
        // bb if it already exists do not do anything, else use new to instantiate it
        if (!property_layer) {
          property_layer = new srrg2_core::Property_<srrg2_core::Matrix_<float>>(
            current_layer_name,
            "float",
            &(message_out->grid_map_struct.value()),
            srrg2_core::Matrix_<float>(rows_, cols_),
            nullptr);
        }
        // bb create a reference to the float matrix
        srrg2_core::Matrix_<float>& property_matrix = property_layer->value();
        for (size_t idx = 0; idx < current_layer_data.data.size(); ++idx) {
          size_t col                   = idx % cols_;
          size_t row                   = (idx - col) / cols_;
          property_matrix.at(row, col) = current_layer_data.data[idx];
        }
        // bb analyze the matrix to find the biggest element
        float max = 0.f;
        for (size_t row = 0; row < property_layer->value().rows(); ++row) {
          for (size_t col = 0; col < property_layer->value().cols(); ++col) {
            if (max < property_layer->value().at(row, col)) {
              max = property_layer->value().at(row, col);
            }
          }
        }
      } else {
        std::cerr << "Layer " << current_layer_name << " cannot be converted." << std::endl
                  << "Only 2D matrices are currently handled." << std::endl;
      }
    }

    return message_out;
  }
  //--------------------------------------------------------------//
  //-------------------   SRRG -> ROS   --------------------------//
  //--------------------------------------------------------------//

  sensor_msgs::CameraInfoPtr
  Converter::convert(const srrg2_core::CameraInfoMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::CameraInfoPtr message_out(new sensor_msgs::CameraInfo());
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    message_out->distortion_model = message_in_->distortion_model.value();

    SRRG_TO_ROS_VECX(message_out->D, message_in_->distortion_coefficients);

    SRRG_TO_ROS_MAT(message_out->K, message_in_->camera_matrix);

    message_out->height = message_in_->rows.value();
    message_out->width  = message_in_->cols.value();

    return message_out;
  }

  sensor_msgs::ImagePtr Converter::convert(const srrg2_core::ImageMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    srrg2_core::ImageMessagePtr src_ptr =
      std::const_pointer_cast<srrg2_core::ImageMessage>(message_in_);
    srrg2_core::BaseImage* b_img = src_ptr->image();
    std::string encoding         = "";

    if (b_img->type() == srrg2_core::ImageType::TYPE_UNKNOWN) {
      encoding += "UKN";
    } else if ((b_img->type() & 0b111) == CV_8U) {
      encoding += "8U";
    } else if ((b_img->type() & 0b111) == CV_8S) {
      encoding += "8S";
    } else if ((b_img->type() & 0b111) == CV_16U) {
      encoding += "16U";
    } else if ((b_img->type() & 0b111) == CV_16S) {
      encoding += "16S";
    } else if ((b_img->type() & 0b111) == CV_32S) {
      encoding += "32S";
    } else if ((b_img->type() & 0b111) == CV_32F) {
      encoding += "32F";
    } else if ((b_img->type() & 0b111) == CV_64F) {
      encoding += "64F";
    } else {
      throw std::runtime_error("unable to get the encoding");
    }

    encoding += "C" + std::to_string(b_img->numChannels());

    sensor_msgs::ImagePtr message_out(new sensor_msgs::Image());

    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    message_out->height = static_cast<uint32_t>(b_img->rows());
    message_out->width  = static_cast<uint32_t>(b_img->cols());
    ;
    cv::Mat cv_img;
    b_img->toCv(cv_img);
    message_out->step = static_cast<uint32_t>(cv_img.elemSize() * message_out->width);
    message_out->data.resize(message_out->step * message_out->height);
    memcpy(message_out->data.data(), cv_img.data, message_out->step * message_out->height);

    message_out->encoding = encoding;

    //    std::wcerr << "unhandled big endian flag" << std::endl;

    return message_out;
  }

  sensor_msgs::ImuPtr Converter::convert(const srrg2_core::IMUMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::ImuPtr message_out(new sensor_msgs::Imu());

    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    SRRG_TO_ROS_QUATERNION(message_out->orientation, message_in_->orientation);
    SRRG_TO_ROS_VEC3(message_out->angular_velocity, message_in_->angular_velocity);
    SRRG_TO_ROS_VEC3(message_out->linear_acceleration, message_in_->linear_acceleration);
    SRRG_TO_ROS_MAT(message_out->orientation_covariance, message_in_->orientation_covariance);
    SRRG_TO_ROS_MAT(message_out->angular_velocity_covariance,
                    message_in_->angular_velocity_covariance);
    SRRG_TO_ROS_MAT(message_out->linear_acceleration_covariance,
                    message_in_->linear_acceleration_covariance);

    return message_out;
  }

  sensor_msgs::LaserScanPtr Converter::convert(const srrg2_core::LaserMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::LaserScanPtr message_out(new sensor_msgs::LaserScan);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    message_out->angle_min       = message_in_->angle_min.value();
    message_out->angle_max       = message_in_->angle_max.value();
    message_out->angle_increment = message_in_->angle_increment.value();
    message_out->time_increment  = message_in_->time_increment.value();
    message_out->scan_time       = message_in_->scan_time.value();
    message_out->range_max       = message_in_->range_max.value();
    message_out->range_min       = message_in_->range_min.value();

    message_out->ranges      = message_in_->ranges.value();
    message_out->intensities = message_in_->intensities.value();

    return message_out;
  }

  nav_msgs::OdometryPtr Converter::convert(srrg2_core::OdometryMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    nav_msgs::OdometryPtr message_out(new nav_msgs::Odometry);

    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    message_out->child_frame_id = message_in_->child_frame.value();

    SRRG_TO_ROS_POSE(message_out->pose, message_in_->pose);
    SRRG_TO_ROS_VEC3(message_out->twist.twist.linear, message_in_->linear_velocity);
    SRRG_TO_ROS_VEC3(message_out->twist.twist.angular, message_in_->angular_velocity);
    SRRG_TO_ROS_MAT(message_out->pose.covariance, message_in_->pose_covariance);
    SRRG_TO_ROS_MAT(message_out->twist.covariance, message_in_->twist_covariance);

    return message_out;
  }

  geometry_msgs::PointStampedPtr
  Converter::convert(srrg2_core::PointStampedMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    geometry_msgs::PointStampedPtr message_out(new geometry_msgs::PointStamped);

    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    SRRG_TO_ROS_VEC3(message_out->point, message_in_->point);

    return message_out;
  }

  sensor_msgs::RangePtr Converter::convert(const srrg2_core::RangeMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::RangePtr message_out(new sensor_msgs::Range);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    message_out->range          = message_in_->range.value();
    message_out->radiation_type = static_cast<uint8_t>(message_in_->radiation_type.value());
    message_out->field_of_view  = message_in_->field_of_view.value();
    message_out->max_range      = message_in_->range_max.value();
    message_out->min_range      = message_in_->range_min.value();

    return message_out;
  }

  geometry_msgs::TransformStamped
  Converter::convert(const srrg2_core::TransformEvent& message_in_) {
    geometry_msgs::TransformStamped message_out;

    // ds set header TODO check info
    message_out.header.frame_id = message_in_.identifierParent();
    message_out.header.seq      = 0;
    message_out.header.stamp    = ros::Time(message_in_.timeSeconds());

    // ds set data
    message_out.child_frame_id = message_in_.identifier();
    EIGEN_TO_ROS_QUATERNION(message_out.transform.rotation,
                            Eigen::Quaternionf(message_in_.transform().linear()));
    EIGEN_TO_ROS_VEC3(message_out.transform.translation, message_in_.transform().translation());

    return message_out;
  }

  tf2_msgs::TFMessagePtr
  Converter::convert(const srrg2_core::TransformEventsMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }
    tf2_msgs::TFMessagePtr message_out(new tf2_msgs::TFMessage());
    message_out->transforms.resize(message_in_->events.size());

    // ds we can have multiple transform events per message (ROS)
    for (std::size_t u = 0; u < message_in_->events.size(); ++u) {
      // ds convert to transform event and store it
      const srrg2_core::TransformEvent& ev = message_in_->events.value(u);
      message_out->transforms[u]           = convert(ev);
    }

    return message_out;
  }

  geometry_msgs::TwistStampedPtr
  Converter::convert(srrg2_core::TwistStampedMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    geometry_msgs::TwistStampedPtr message_out(new geometry_msgs::TwistStamped);

    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    SRRG_TO_ROS_VEC3(message_out->twist.linear, message_in_->linear);
    SRRG_TO_ROS_VEC3(message_out->twist.angular, message_in_->angular);

    return message_out;
  }

  sensor_msgs::PointCloud2Ptr
  Converter::convert(const srrg2_core::PointCloud2MessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::PointCloud2Ptr message_out(new sensor_msgs::PointCloud2);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    message_out->height = message_in_->height.value();
    message_out->width  = message_in_->width.value();
    message_out->fields.resize(message_in_->fields.size());
    for (size_t i = 0; i < message_in_->fields.size(); ++i) {
      message_out->fields[i].name     = message_in_->fields.value(i).name.value();
      message_out->fields[i].count    = message_in_->fields.value(i).count.value();
      message_out->fields[i].datatype = message_in_->fields.value(i).datatype.value();
      message_out->fields[i].offset   = message_in_->fields.value(i).offset.value();
    }
    message_out->is_bigendian            = message_in_->is_bigendian.value();
    message_out->point_step              = message_in_->point_step.value();
    message_out->row_step                = message_in_->row_step.value();
    srrg2_core::PointCloud2Data* data_in = message_in_->data.value().get();
    const size_t data_size               = message_out->height * message_out->row_step;
    message_out->data.resize(data_size);
    std::memcpy(&(message_out->data[0]), &(data_in->at(0)), data_size * sizeof(uint8_t));
    message_out->is_dense = message_in_->is_dense.value();

    return message_out;
  }

  sensor_msgs::JointStatePtr Converter::convert(const srrg2_core::JointsMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    sensor_msgs::JointStatePtr message_out(new sensor_msgs::JointState);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    const size_t joints_size = message_in_->joint_events.size();
    message_out->name.resize(joints_size);
    message_out->position.resize(joints_size);
    message_out->velocity.resize(joints_size);
    message_out->effort.resize(joints_size);
    for (size_t i = 0; i < joints_size; ++i) {
      const srrg2_core::JointEvent& joint_event = message_in_->joint_events.value(i);
      message_out->name[i]                      = joint_event.identifier();
      message_out->position[i]                  = joint_event.position();
      message_out->velocity[i]                  = joint_event.velocity();
      message_out->effort[i]                    = joint_event.effort();
    }

    return message_out;
  }

  sensor_msgs::NavSatFixPtr Converter::convert(const srrg2_core::NavsatFixMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    assert(message_in_->status.value() < 3 && message_in_->status.value() > -2 &&
           "invalid fix status");
    assert(message_in_->service.value() > 0 && message_in_->service.value() < 9 &&
           "invalid service");
    assert(message_in_->covariance_type.value() < 4 && "invalid covariance type");

    // ia construct and copy header
    sensor_msgs::NavSatFixPtr message_out(new sensor_msgs::NavSatFix);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));

    // ia convert the actual message
    message_out->status.status            = message_in_->status.value();
    message_out->status.service           = message_in_->service.value();
    message_out->position_covariance_type = message_in_->covariance_type.value();
    message_out->latitude                 = message_in_->latitude.value();
    message_out->longitude                = message_in_->longitude.value();

    // ia eye-disturbing altitude conversion
    if (message_in_->has_altitude.value()) {
      message_out->altitude = message_in_->altitude.value();
    } else {
      message_out->altitude = std::nan(""); // ia I hate ros
    }

    // ia finally the covariance
    SRRG_TO_ROS_MAT(message_out->position_covariance, message_in_->covariance);

    return message_out;
  }

  geometry_msgs::PoseArrayPtr
  Converter::convert(const srrg2_core::PoseArrayMessagePtr message_in_) {
    if (!message_in_)
      return nullptr;
    geometry_msgs::PoseArrayPtr message_out(new geometry_msgs::PoseArray);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    message_out->poses.resize(message_in_->poses.size());
    for (size_t i = 0; i < message_in_->poses.size(); ++i) {
      const srrg2_core::Vector6f& p_in = message_in_->poses.value(i).pose_vector.value();
      geometry_msgs::Pose& p_out       = message_out->poses[i];
      iso2rosPose(p_out, p_in);
    }
    return message_out;
  }

  geometry_msgs::PoseStampedPtr
  Converter::convert(const srrg2_core::PoseStampedMessagePtr message_in_) {
    if (!message_in_)
      return nullptr;
    geometry_msgs::PoseStampedPtr message_out(new geometry_msgs::PoseStamped);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    iso2rosPose(message_out->pose, message_in_->pose.value().pose_vector.value());
    return message_out;
  }

  geometry_msgs::PoseWithCovarianceStampedPtr
  Converter::convert(const srrg2_core::PoseWithCovarianceStampedMessagePtr message_in_) {
    if (!message_in_)
      return nullptr;
    geometry_msgs::PoseWithCovarianceStampedPtr message_out(
      new geometry_msgs::PoseWithCovarianceStamped);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    iso2rosPose(message_out->pose.pose, message_in_->pose.value().pose_vector.value());
    size_t k = 0;
    for (int r = 0; r < message_in_->covariance.value().rows(); ++r)
      for (int c = 0; c < message_in_->covariance.value().cols(); ++r, ++k)
        message_out->pose.covariance[k] = message_in_->covariance.value()(r, c);

    return message_out;
  }

  nav_msgs::PathPtr Converter::convert(const srrg2_core::PathMessagePtr message_in_) {
    if (!message_in_)
      return nullptr;
    nav_msgs::PathPtr message_out(new nav_msgs::Path);
    SRRG_TO_ROS_HEADER((*message_out), (*message_in_));
    message_out->poses.resize(message_in_->poses.size());
    for (size_t pose_num = 0; pose_num < message_in_->poses.size(); ++pose_num) {
      const srrg2_core::PoseStampedMessage& src = message_in_->poses.value(pose_num);
      geometry_msgs::PoseStamped& dest          = message_out->poses[pose_num];
      SRRG_TO_ROS_HEADER(dest, src);
      message_out->poses.resize(message_in_->poses.size());
      iso2rosPose(dest.pose, src.pose.value().pose_vector.value());
    }
    return message_out;
  }

  geometry_msgs::TwistPtr Converter::convert(const srrg2_core::CmdVelMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    geometry_msgs::TwistPtr message_out(new geometry_msgs::Twist);
    // bb alternatively use SRRG_TO_ROS_VEC3
    message_out->linear.x  = message_in_->linear.value();
    message_out->angular.z = message_in_->angular.value();

    return message_out;
  }

  grid_map_msgs::GridMapPtr Converter::convert(const srrg2_core::GridMapMessagePtr message_in_) {
    if (!message_in_) {
      return nullptr;
    }

    grid_map_msgs::GridMapPtr message_out(new grid_map_msgs::GridMap());
    message_out->info.header.stamp    = ros::Time(message_in_->timestamp.value());
    message_out->info.header.frame_id = message_in_->frame_id.value();
    message_out->info.resolution      = message_in_->grid_map_struct.value().resolution();
    message_out->info.length_x = message_in_->grid_map_struct.value().GridMap2DHeader::size().x();
    message_out->info.length_y = message_in_->grid_map_struct.value().GridMap2DHeader::size().y();
    srrg2_core::Isometry3f origin_isometry =
      srrg2_core::geometry3d::get3dFrom2dPose(message_in_->grid_map_struct.value().origin());
    message_out->info.pose.position.x = origin_isometry.translation().x();
    message_out->info.pose.position.y = origin_isometry.translation().y();
    message_out->info.pose.position.z = origin_isometry.translation().z();
    Eigen::Quaternionf rotation_quaternion(origin_isometry.linear());
    message_out->info.pose.orientation.x = rotation_quaternion.x();
    message_out->info.pose.orientation.y = rotation_quaternion.y();
    message_out->info.pose.orientation.z = rotation_quaternion.z();
    message_out->info.pose.orientation.w = rotation_quaternion.w();
    srrg2_core::StringPropertyPtrMap grid_map_properties =
      message_in_->grid_map_struct.value().properties();
    message_out->data.clear();
    for (const auto& layer : grid_map_properties) {
      message_out->layers.emplace_back(layer.first);
      std_msgs::Float32MultiArray data_array;
      if (layer.second->description() == "float") {
        srrg2_core::Matrix_<float> property_layer =
          message_in_->grid_map_struct.value()
            .property<srrg2_core::Property_<srrg2_core::Matrix_<float>>>(layer.first)
            ->value();
        // bb eigen storage order is by default column major
        // bb inner loop loops over columns
        for (size_t row = 0; row < property_layer.rows(); ++row) {
          for (size_t col = 0; col < property_layer.cols(); ++col) {
            data_array.data.emplace_back(property_layer(row, col));
          }
        }

        //        if (layer.first == "cost_map") {
        //          std::ofstream myfile;
        //          myfile.open("my_matrix_after_conversion_to_ros.txt");
        //          for (const auto& element : data_array.data) {
        //            std::cerr << "writing" << std::endl;
        //            myfile << element << std::endl;
        //          }
        //          myfile.close();
        //        }
        data_array.layout.dim.resize(2);
        data_array.layout.dim[0].size = property_layer.rows();
        data_array.layout.dim[1].size = property_layer.cols();
      } else {
        std::cerr << "DATA TYPE STILL NOT HANDLED!" << std::endl
                  << "Layer named " << layer.first << " will not be sent!" << std::endl;
      }

      message_out->data.emplace_back(std::move(data_array));
    }
    message_out->outer_start_index = 0;
    message_out->inner_start_index = 0;
    return message_out;
  }

  //--------------------------------------------------------------//
  //----------------------   HELPERS   ---------------------------//
  //--------------------------------------------------------------//

  srrg2_core::TransformEvent
  Converter::_convert(const geometry_msgs::TransformStamped& message_in_) {
    // srrg unable to construct a boost::shared_ptr from a reference
    srrg2_core::Quaternionf rot;
    srrg2_core::Vector3f transl;
    srrg2_core::Isometry3f pose = srrg2_core::Isometry3f::Identity();

    ROS_TO_EIGEN_QUATERNION(rot, message_in_.transform.rotation);
    ROS_TO_EIGEN_VEC3(transl, message_in_.transform.translation);
    pose.translation() = transl;
    pose.linear()      = rot.matrix();

    // ia remove / and \ from the identifier name
    const std::string event_identifier = srrg2_core::removeStringTokens(message_in_.child_frame_id);
    const std::string event_identifier_parent =
      srrg2_core::removeStringTokens(message_in_.header.frame_id);

    return srrg2_core::TransformEvent(
      message_in_.header.stamp.toSec(), event_identifier, pose, event_identifier_parent);
  }

  srrg2_core::TransformEvent
  Converter::_convert(const geometry_msgs::TransformStampedConstPtr& message_in_) {
    if (message_in_) {
      return _convert(*message_in_);
    }
    return srrg2_core::TransformEvent();
  }

  srrg2_core::BaseSensorMessagePtr
  Converter::convert(srrg2_core_ros::PlannerStatusMessageConstPtr message_in_) {
    srrg2_core::PlannerStatusMessagePtr msg_out(new srrg2_core::PlannerStatusMessage);
    ROS_TO_SRRG_HEADER((*msg_out), (*message_in_));
    msg_out->status.setValue(message_in_->status);
    msg_out->a_star_status.setValue(message_in_->a_star_status);
    msg_out->path_type.setValue(message_in_->path_type);
    msg_out->cost_to_global_goal.setValue(message_in_->cost_to_global_goal);
    msg_out->distance_to_global_goal.setValue(message_in_->distance_to_global_goal);
    msg_out->distance_to_local_goal.setValue(message_in_->distance_to_local_goal);
    return msg_out;
  }

  srrg2_core_ros::PlannerStatusMessagePtr
  Converter::convert(srrg2_core::PlannerStatusMessagePtr message_in_) {
    srrg2_core_ros::PlannerStatusMessagePtr msg_out(new srrg2_core_ros::PlannerStatusMessage);
    SRRG_TO_ROS_HEADER((*msg_out), (*message_in_));
    msg_out->status                  = message_in_->status.constValue();
    msg_out->a_star_status           = message_in_->a_star_status.constValue();
    msg_out->path_type               = message_in_->path_type.constValue();
    msg_out->cost_to_global_goal     = message_in_->cost_to_global_goal.constValue();
    msg_out->distance_to_global_goal = message_in_->distance_to_global_goal.constValue();
    msg_out->distance_to_local_goal  = message_in_->distance_to_local_goal.constValue();
    return msg_out;
  }

} // namespace srrg2_core_ros
