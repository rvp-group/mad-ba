#include <rosbag/bag.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PointStamped.h>
#include <srrg_converters/translator_utils.h>

#define UPDATE_HEADER(msg, frame)    \
  {                                     \
    h_msg.frame_id = frame;          \
    h_msg.seq = seq++;                  \
    h_msg.stamp = ros::Time::now();     \
    msg.header = h_msg;                 \
  }



int main(int argc, char** argv) {

  ros::Time::init();
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);

  uint32_t seq = 100;

  geometry_msgs::Vector3 tmp_v3;
  geometry_msgs::Quaternion tmp_q;

  // Header message
  std_msgs::Header h_msg;

  // IMU message
  sensor_msgs::Imu imu_msg;

  UPDATE_HEADER(imu_msg, "/imu");

  // fill the fields
  tmp_v3.x = 0;
  tmp_v3.y = 1;
  tmp_v3.z = 2;
  imu_msg.angular_velocity = tmp_v3;
  tmp_v3.x = 3;
  tmp_v3.y = 4;
  tmp_v3.z = 5;
  imu_msg.linear_acceleration = tmp_v3;
  tmp_v3.x = 6;
  tmp_v3.y = 7;
  tmp_v3.z = 8;
  tmp_q.w = 0.5;
  tmp_q.x = 0.6;
  tmp_q.y = 0.7;
  tmp_q.z = 0.8;
  imu_msg.orientation = tmp_q;

  imu_msg.angular_velocity_covariance.fill(0.5);
  imu_msg.linear_acceleration_covariance.fill(0.6);
  imu_msg.orientation_covariance.fill(0.7);

  // Laser message
  sensor_msgs::LaserScan l_msg;
  UPDATE_HEADER(l_msg, "/laser");
  l_msg.angle_min = -100;
  l_msg.angle_max = 100;
  l_msg.angle_increment = 0.3;
  l_msg.time_increment = 0.001;
  l_msg.range_max = 20;
  l_msg.range_min = 0.5;
  l_msg.ranges = std::vector<float>(10, 2);
  l_msg.intensities = std::vector<float>(10, 0.5);

  // range message
  sensor_msgs::Range r_msg;
  UPDATE_HEADER(r_msg, "/usound");
  r_msg.field_of_view = 10;
  r_msg.min_range = 20;
  r_msg.max_range = 0.5;
  r_msg.range = 33;
  r_msg.radiation_type = sensor_msgs::Range::INFRARED;

  bag.write("/imu", ros::Time::now(), imu_msg);
  bag.write("/laser", ros::Time::now(), l_msg);
  bag.write("/usound", ros::Time::now(), r_msg);


  // tf message
  tf2_msgs::TFMessage tf_msg;

  geometry_msgs::TransformStamped tf0, tf1, tf2, tf3, tf4, tf5;
  UPDATE_HEADER(tf0, "/odom");
  tf0.child_frame_id = "/base_link";
  EIGEN_TO_ROS_VEC3(tf0.transform.translation, srrg2_core::Vector3f(0, 0, 0));
  EIGEN_TO_ROS_QUATERNION(tf0.transform.rotation, srrg2_core::Quaternionf(1, 0, 0, 0));

  usleep(20);
  UPDATE_HEADER(tf1, "/base_link");
  tf1.child_frame_id = "/tf1";
  EIGEN_TO_ROS_VEC3(tf1.transform.translation, srrg2_core::Vector3f(0.1, 0.1, 0.1));
  EIGEN_TO_ROS_QUATERNION(tf1.transform.rotation, srrg2_core::Quaternionf(0.1, 0.1, 0.1, 0.1).normalized());
  usleep(20);
  UPDATE_HEADER(tf2, "/base_link");
  tf2.child_frame_id = "/tf2";
  EIGEN_TO_ROS_VEC3(tf2.transform.translation, srrg2_core::Vector3f(0.2, 0.2, 0.2));
  EIGEN_TO_ROS_QUATERNION(tf2.transform.rotation, srrg2_core::Quaternionf(0.2, 0.2, 0.2, 0.2).normalized());
  usleep(20);
  UPDATE_HEADER(tf3, "/tf2");
  tf3.child_frame_id = "/tf3";
  EIGEN_TO_ROS_VEC3(tf3.transform.translation, srrg2_core::Vector3f(0.3, 0.3, 0.3));
  EIGEN_TO_ROS_QUATERNION(tf3.transform.rotation, srrg2_core::Quaternionf(0.3, 0.3, 0.3, 0.3).normalized());
  usleep(20);
  UPDATE_HEADER(tf4, "/tf2");
  tf4.child_frame_id = "/tf4";
  EIGEN_TO_ROS_VEC3(tf4.transform.translation, srrg2_core::Vector3f(0.4, 0.4, 0.4));
  EIGEN_TO_ROS_QUATERNION(tf4.transform.rotation, srrg2_core::Quaternionf(0.4, 0.4, 0.4, 0.4).normalized());
  UPDATE_HEADER(tf5, "/tf4");
  tf5.child_frame_id = "/tf5";
  EIGEN_TO_ROS_VEC3(tf5.transform.translation, srrg2_core::Vector3f(0.4, 0.4, 0.4));
  EIGEN_TO_ROS_QUATERNION(tf5.transform.rotation, srrg2_core::Quaternionf(0.4, 0.4, 0.4, 0.4).normalized());

  tf_msg.transforms.push_back(tf1);
  tf_msg.transforms.push_back(tf3);
  tf_msg.transforms.push_back(tf0);
  tf_msg.transforms.push_back(tf2);
  tf_msg.transforms.push_back(tf4);
  tf_msg.transforms.push_back(tf5);

  bag.write("/tf", ros::Time::now(), tf_msg);

  float cum = 0.25;
  for (int i = 0; i < 10; ++i) {
    usleep(20);
    UPDATE_HEADER(tf2, "/base_link");
    tf2.child_frame_id = "/tf2";
    EIGEN_TO_ROS_VEC3(tf2.transform.translation, srrg2_core::Vector3f(cum, cum, cum));
    EIGEN_TO_ROS_QUATERNION(tf2.transform.rotation, srrg2_core::Quaternionf(cum, cum, cum, cum).normalized());
    cum += 0.05;
    tf_msg.transforms[3] = tf2;

    bag.write("/tf", ros::Time::now(), tf_msg);
  }

  bag.close();
}
