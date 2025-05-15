#include <iostream>

#include "srrg_boss/serializer.h"
#include "srrg_messages/instances.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/point_cloud2_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>

using namespace srrg2_core;

// clang-format off
const char* banner[] = {"This program converts EuRoC formatted stereo RGB files to SRRG format (json/bag)", 0};
// clang-format on

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& imu_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_);

void loadImageInformation(const std::string& folder_image_names_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_);

void loadIMUInformation(const std::string& folder_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        std::vector<double>& timestamps_seconds_,
                        double& rate_hz_,
                        double& gyroscope_noise_density_,
                        double& gyroscope_random_walk_,
                        double& accelerometer_noise_density_,
                        double& accelerometer_random_walk_,
                        Isometry3f& imu_in_base_link_);

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Isometry3f& imu_from_camera_left_);

void serializeCameraImageAndInfo(Serializer& serializer_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const bool& enable_rectification_,
                                 const cv::Mat* undistort_rectify_maps_,
                                 const Matrix3f& camera_matrix_rectified_);

void initializeRectification(const Matrix3f& camera_calibration_matrix_left_,
                             const Matrix3f& camera_calibration_matrix_right_,
                             const Vector5d& distortion_coefficients_left_,
                             const Vector5d& distortion_coefficients_right_,
                             const cv::Size& opencv_image_size_,
                             cv::Mat* undistort_rectify_maps_left_,
                             cv::Mat* undistort_rectify_maps_right_,
                             Matrix3f& camera_calibration_matrix_rectified_,
                             Isometry3f& camera_left_to_right_);

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_folder_camera_left(
    &command_line_parser, "cl",  "camera-left",
    "left camera messages folder (e.g. cam0) that contains: {data/, data.csv, sensor.yaml}", "");
  ArgumentString argument_folder_camera_right(
    &command_line_parser, "cr",  "camera-right",
    "right camera messages folder (e.g. cam1) that contains: {data/, data.csv, sensor.yaml}", "");
  ArgumentString argument_folder_imu(
    &command_line_parser, "imu",  "imu",
    "IMU messages folder (e.g. imu0) that contains: {data.csv, sensor.yaml}", "");
  ArgumentString argument_output_file(
    &command_line_parser, "o",  "output",
    "converted output file", "messages.json");
  ArgumentString argument_ground_truth_file(
     &command_line_parser, "gt",  "ground-truth",
     "ground truth file that contains IMU filtered prism poses w.r.t. world origin", "");
  ArgumentFlag param_enable_rectification(
      &command_line_parser, "r",  "enable-rectification",
      "performs stereo rectification (opencv) based on the provided calibration data");

  // clang-format on
  command_line_parser.parse();
  if (argument_folder_camera_left.value().empty()) {
    std::cerr << "ERROR: left camera folder is not set (-cl)" << std::endl;
    return 0;
  }
  if (argument_folder_camera_right.value().empty()) {
    std::cerr << "ERROR: right camera folder is not set (-cr)" << std::endl;
    return 0;
  }
  if (argument_output_file.value().empty()) {
    std::cerr << "ERROR: output filename is empty (set with -o)" << std::endl;
    return 0;
  }
  if (argument_ground_truth_file.value().empty()) {
    std::cerr << "ERROR: ground truth filename is empty (set with -gt)" << std::endl;
    return 0;
  }

  // ds buffer ground truth data
  StdVectorEigenIsometry3f imu_poses_in_world;
  std::vector<double> timestamps_seconds_gt;
  loadGroundTruthPosesAndStamps(
    argument_ground_truth_file.value(), imu_poses_in_world, timestamps_seconds_gt);
  if (timestamps_seconds_gt.empty()) {
    throw std::runtime_error("ERROR: no ground truth poses loaded");
  }

  // ds configure serializer for parallel in and out streaming
  Serializer serializer;
  serializer.setFilePath(argument_output_file.value());

  // ds read depth and RGB images and timestamps
  std::vector<std::string> image_file_paths_camera_left;
  std::vector<double> timestamps_camera_left;
  std::vector<std::string> image_file_paths_camera_right;
  std::vector<double> timestamps_camera_right;

  // ds parse image information
  loadImageInformation(
    argument_folder_camera_left.value(), image_file_paths_camera_left, timestamps_camera_left);
  if (timestamps_camera_left.empty()) {
    throw std::runtime_error("ERROR: no left camera image information retrieved");
  }
  loadImageInformation(
    argument_folder_camera_right.value(), image_file_paths_camera_right, timestamps_camera_right);
  if (timestamps_camera_right.empty()) {
    throw std::runtime_error("ERROR: no right camera image information retrieved");
  }

  // ds load camera calibration data from files
  Matrix3f camera_calibration_matrix_left(Matrix3f::Zero());
  Vector5d distortion_coefficients_left(Vector5d::Zero());
  Isometry3f imu_from_camera_left(Isometry3f::Identity());
  loadCameraCalibration(argument_folder_camera_left.value() + "/sensor.yaml",
                        camera_calibration_matrix_left,
                        distortion_coefficients_left,
                        imu_from_camera_left);
  Matrix3f camera_calibration_matrix_right(Matrix3f::Zero());
  Vector5d distortion_coefficients_right(Vector5d::Zero());
  Isometry3f imu_from_camera_right(Isometry3f::Identity());
  loadCameraCalibration(argument_folder_camera_right.value() + "/sensor.yaml",
                        camera_calibration_matrix_right,
                        distortion_coefficients_right,
                        imu_from_camera_right);
  Isometry3f camera_left_to_right(imu_from_camera_right.inverse() * imu_from_camera_left);
  std::cerr << "camera LEFT w.r.t. RIGHT: " << std::endl;
  std::cerr << camera_left_to_right.matrix() << std::endl;

  // ds common parameters for all sequences TODO verify
  const std::string distortion_model_name = "radial-tangential";
  const cv::Size opencv_image_size(752, 480);

  // ds check if rectification is desired
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];
  Matrix3f camera_calibration_matrix_rectified(Matrix3f::Zero());
  if (param_enable_rectification.isSet()) {
    initializeRectification(camera_calibration_matrix_left,
                            camera_calibration_matrix_right,
                            distortion_coefficients_left,
                            distortion_coefficients_right,
                            opencv_image_size,
                            undistort_rectify_maps_left,
                            undistort_rectify_maps_right,
                            camera_calibration_matrix_rectified,
                            camera_left_to_right);
    std::cerr << "camera LEFT (rectified) w.r.t. RIGHT (rectified): " << std::endl;
    std::cerr << camera_left_to_right.matrix() << std::endl;
  }

  camera_left_to_right = camera_left_to_right.inverse();
  std::cerr << "camera RIGHT in LEFT: " << std::endl;
  std::cerr << camera_left_to_right.matrix() << std::endl;

  // ds load IMU information
  std::vector<double> timestamps_imu;
  StdVectorEigenVector3d angular_velocities;
  StdVectorEigenVector3d linear_accelerations;
  double rate_hz;
  double gyroscope_noise_density;
  double gyroscope_random_walk;
  double accelerometer_noise_density;
  double accelerometer_random_walk;
  Isometry3f imu_in_base_link(Isometry3f::Identity());
  if (argument_folder_imu.isSet() && !argument_folder_imu.value().empty()) {
    loadIMUInformation(argument_folder_imu.value(),
                       angular_velocities,
                       linear_accelerations,
                       timestamps_imu,
                       rate_hz,
                       gyroscope_noise_density,
                       gyroscope_random_walk,
                       accelerometer_noise_density,
                       accelerometer_random_walk,
                       imu_in_base_link);
  }

  // ds stop before starting to allow for a quick configuration inspection
  //  std::cerr << "press [ENTER] to start conversion" << std::endl;
  //  getchar();

  // ds playback configuration UAGHS
  constexpr double timestamp_tolerance_seconds = 0.01;
  double timestamp_oldest                      = 0;
  size_t index_gt                              = 0;
  size_t index_camera_left                     = 0;
  size_t index_camera_right                    = 0;
  size_t index_imu                             = 0;

  // ds determine initial, smallest timestamp
  timestamp_oldest = timestamps_seconds_gt[0];
  if (timestamp_oldest > timestamps_camera_left[0]) {
    timestamp_oldest = timestamps_camera_left[0];
  }
  if (timestamp_oldest > timestamps_camera_right[0]) {
    timestamp_oldest = timestamps_camera_right[0];
  }
  if (!timestamps_imu.empty() && timestamp_oldest > timestamps_imu[0]) {
    timestamp_oldest = timestamps_imu[0];
  }
  std::cerr << std::setprecision(16);
  std::cerr << "initial timestamp (s): " << timestamp_oldest << std::endl;

  TransformEventsMessagePtr tf_static_message(
    new TransformEventsMessage("/tf_static", "/imu", index_gt, timestamp_oldest));
  tf_static_message->events.resize(3);

  TransformEvent event_imu_in_base_link(0, "imu", imu_in_base_link, "base_link");
  TransformEvent event_camera_left_in_base_link(
    0, "camera_left", imu_from_camera_left, "base_link");
  TransformEvent event_camera_right_in_left(0, "camera_right", camera_left_to_right, "camera_left");

  tf_static_message->events.setValue(0, event_imu_in_base_link);
  tf_static_message->events.setValue(1, event_camera_left_in_base_link);
  tf_static_message->events.setValue(2, event_camera_right_in_left);
  std::cerr << "T";
  serializer.writeObject(*tf_static_message);

  // ds playback all buffers
  while (index_gt < timestamps_seconds_gt.size() ||
         index_camera_left < timestamps_camera_left.size() ||
         index_camera_right < timestamps_camera_right.size() || index_imu < timestamps_imu.size()) {
    // ds if we still have ground truth information available
    if (index_gt < timestamps_seconds_gt.size()) {
      const double& timestamp_candidate = timestamps_seconds_gt[index_gt];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        // ds write message
        OdometryMessagePtr gt_imu_in_world(
          new OdometryMessage("/ground_truth", "/base_link", index_gt, timestamp_candidate));
        gt_imu_in_world->pose.setValue(imu_poses_in_world[index_gt]);
        serializer.writeObject(*gt_imu_in_world);
        std::cerr << "G";
        ++index_gt;
      }
    }

    // ds if we have left image data available
    if (index_camera_left < timestamps_camera_left.size()) {
      const double& timestamp_candidate = timestamps_camera_left[index_camera_left];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        serializeCameraImageAndInfo(serializer,
                                    index_camera_left,
                                    timestamp_candidate,
                                    "camera_left",
                                    image_file_paths_camera_left[index_camera_left],
                                    distortion_model_name,
                                    distortion_coefficients_left,
                                    camera_calibration_matrix_left,
                                    param_enable_rectification.isSet(),
                                    undistort_rectify_maps_left,
                                    camera_calibration_matrix_rectified);
      }
    }

    // ds if we have right image data available
    if (index_camera_right < timestamps_camera_right.size()) {
      const double& timestamp_candidate = timestamps_camera_right[index_camera_right];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        serializeCameraImageAndInfo(serializer,
                                    index_camera_right,
                                    timestamp_candidate,
                                    "camera_right",
                                    image_file_paths_camera_right[index_camera_right],
                                    distortion_model_name,
                                    distortion_coefficients_right,
                                    camera_calibration_matrix_right,
                                    param_enable_rectification.isSet(),
                                    undistort_rectify_maps_right,
                                    camera_calibration_matrix_rectified);
      }
    }

    // ds if we have IMU data available
    if (index_imu < timestamps_imu.size()) {
      const double& timestamp_candidate = timestamps_imu[index_imu];
      // ds if the timestamp is close enough to the currently smallest timestamp
      if (std::fabs(timestamp_candidate - timestamp_oldest) < timestamp_tolerance_seconds) {
        IMUMessagePtr message(new IMUMessage("/imu", "/imu", index_imu, timestamp_candidate));
        message->angular_velocity.setValue(angular_velocities[index_imu].cast<float>());
        //        message->angular_velocity_covariance.setValue(Matrix3f::Identity());
        message->linear_acceleration.setValue(linear_accelerations[index_imu].cast<float>());
        //        message->linear_acceleration_covariance.setValue(Matrix3f::Identity());
        //        message->orientation.setValue(Matrix3f::Identity());
        //        message->orientation_covariance.setValue(Matrix3f::Identity());
        message->rate_hz.setValue(rate_hz);
        message->accelerometer_noise_density.setValue(accelerometer_noise_density);
        message->accelerometer_random_walk.setValue(accelerometer_random_walk);
        message->gyroscope_noise_density.setValue(gyroscope_noise_density);
        message->gyroscope_random_walk.setValue(gyroscope_random_walk);
        serializer.writeObject(*message);
        std::cerr << "I";
        ++index_imu;
      }
    }

    // ds update timestamp in a cascade - lowest wins
    bool timestamp_updated = false;
    if (index_gt < timestamps_seconds_gt.size()) {
      // ds always move timestamp
      timestamp_oldest  = timestamps_seconds_gt[index_gt];
      timestamp_updated = true;
    }
    if (index_camera_left < timestamps_camera_left.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_camera_left[index_camera_left] || !timestamp_updated) {
        timestamp_oldest  = timestamps_camera_left[index_camera_left];
        timestamp_updated = true;
      }
    }
    if (index_camera_right < timestamps_camera_right.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_camera_right[index_camera_right] || !timestamp_updated) {
        timestamp_oldest  = timestamps_camera_right[index_camera_right];
        timestamp_updated = true;
      }
    }
    if (index_imu < timestamps_imu.size()) {
      // ds move timestamp further back if possible
      if (timestamp_oldest > timestamps_imu[index_imu] || !timestamp_updated) {
        timestamp_oldest  = timestamps_imu[index_imu];
        timestamp_updated = true;
      }
    }
  }
  std::cerr << std::endl;
  std::cerr << "# converted LEFT camera messages: " << index_camera_left << "/"
            << timestamps_camera_left.size() << std::endl;
  std::cerr << "# converted RIGHT camera messages: " << index_camera_right << "/"
            << timestamps_camera_right.size() << std::endl;
  std::cerr << "# converted IMU messages: " << index_imu << "/" << timestamps_imu.size()
            << std::endl;
  std::cerr << "# converted GT messages (IMU filtered): " << index_gt << "/"
            << timestamps_seconds_gt.size() << std::endl;
  return 0;
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& imu_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_) {
  std::ifstream ground_truth_stream(ground_truth_file_);
  if (!ground_truth_stream.good() || !ground_truth_stream.is_open()) {
    throw std::runtime_error("ERROR: unable to open ground truth file");
  }

  // ds skip the first line (EuRoC format header information)
  std::cerr << "loadGroundTruthPosesAndStamps|<header>" << std::endl;
  std::string buffer;
  std::getline(ground_truth_stream, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadGroundTruthPosesAndStamps|</header>" << std::endl;

  // ds parse string into tokens split by custom delimiter comma UAGH
  while (std::getline(ground_truth_stream, buffer)) {
    std::replace(buffer.begin(), buffer.end(), ',', ' ');
    std::stringstream stream(buffer);

    // ds ground truth values
    double timestamp_nanoseconds, p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z,
      v_RS_R_x, v_RS_R_y, v_RS_R_z, b_w_RS_S_x, b_w_RS_S_y, b_w_RS_S_z, b_a_RS_S_x, b_a_RS_S_y,
      b_a_RS_S_z;
    while (stream >> timestamp_nanoseconds >> p_RS_R_x >> p_RS_R_y >> p_RS_R_z >> q_RS_w >>
           q_RS_x >> q_RS_y >> q_RS_z >> v_RS_R_x >> v_RS_R_y >> v_RS_R_z >> b_w_RS_S_x >>
           b_w_RS_S_y >> b_w_RS_S_z >> b_a_RS_S_x >> b_a_RS_S_y >> b_a_RS_S_z) {
      if (timestamp_nanoseconds == 0) {
        throw std::runtime_error(
          "loadGroundTruthPosesAndStamps|ERROR: GT timestamps are expected to start at > 0");
      }
      Isometry3f imu_in_world;
      imu_in_world.translation() = Vector3f(p_RS_R_x, p_RS_R_y, p_RS_R_z);
      imu_in_world.linear()      = Quaternionf(q_RS_w, q_RS_x, q_RS_y, q_RS_z).toRotationMatrix();
      imu_poses_in_world_.push_back(imu_in_world);
      timestamps_seconds_gt_.push_back(timestamp_nanoseconds / 1e9);
    }
  }
  ground_truth_stream.close();
  std::cerr << "loadGroundTruthPosesAndStamps|loaded ground truth poses: "
            << timestamps_seconds_gt_.size() << " from file '" << ground_truth_file_ << "'"
            << std::endl;
}

void _setValueByKey(const std::string& buffer_, const std::string& key_, double& value_) {
  const size_t index_begin  = buffer_.find(key_);
  const size_t query_length = std::strlen(key_.c_str());
  const size_t index_end    = buffer_.find_first_of(' ', query_length);
  if (index_begin != std::string::npos) {
    value_ = std::stod(buffer_.substr(index_begin + query_length, index_end - query_length));
  }
}

void loadIMUInformation(const std::string& folder_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        std::vector<double>& timestamps_seconds_,
                        double& rate_hz_,
                        double& gyroscope_noise_density_,
                        double& gyroscope_random_walk_,
                        double& accelerometer_noise_density_,
                        double& accelerometer_random_walk_,
                        Isometry3f& imu_in_base_link_) {
  const std::string file_path_imu_data(folder_imu_data_ + "/data.csv");
  std::ifstream stream_imu_data(file_path_imu_data);
  if (!stream_imu_data.good() || !stream_imu_data.is_open()) {
    throw std::runtime_error("ERROR: unable to open IMU data file: '" + file_path_imu_data + "'");
  }

  // ds skip the first line (EuRoC format header information)
  std::cerr << "loadIMUInformation|<header>" << std::endl;
  std::string buffer;
  std::getline(stream_imu_data, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadIMUInformation|</header>" << std::endl;

  // ds parse string into tokens split by custom delimiter comma UAGH
  while (std::getline(stream_imu_data, buffer)) {
    std::replace(buffer.begin(), buffer.end(), ',', ' ');
    std::stringstream stream(buffer);

    // ds ground truth values
    double timestamp_nanoseconds, w_RS_S_x, w_RS_S_y, w_RS_S_z, a_RS_S_x, a_RS_S_y, a_RS_S_z;
    while (stream >> timestamp_nanoseconds >> w_RS_S_x >> w_RS_S_y >> w_RS_S_z >> a_RS_S_x >>
           a_RS_S_y >> a_RS_S_z) {
      if (timestamp_nanoseconds == 0) {
        throw std::runtime_error(
          "loadIMUInformation|ERROR: GT timestamps are expected to start at > 0");
      }
      angular_velocities_.push_back(Vector3d(w_RS_S_x, w_RS_S_y, w_RS_S_z));
      linear_accelerations_.push_back(Vector3d(a_RS_S_x, a_RS_S_y, a_RS_S_z));
      timestamps_seconds_.push_back(timestamp_nanoseconds / 1e9);
    }
  }
  stream_imu_data.close();
  std::cerr << "loadIMUInformation|loaded IMU data entries: " << timestamps_seconds_.size()
            << " from file '" << file_path_imu_data << "'" << std::endl;

  // ds load IMU parameters
  const std::string file_path_imu_parameters(folder_imu_data_ + "/sensor.yaml");
  std::ifstream stream_imu_parameters(file_path_imu_parameters);
  if (!stream_imu_parameters.good() || !stream_imu_parameters.is_open()) {
    throw std::runtime_error("ERROR: unable to open IMU parameters file: '" +
                             file_path_imu_parameters + "'");
  }

  // ds evil parsing - skip lines until we hit the transform 'data' line
  for (size_t i = 0; i < 8; ++i) {
    std::getline(stream_imu_parameters, buffer);
  }

  // ds parse transform by tokens
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      char delimiter = ',';
      if (r == 3 && c == 3) {
        delimiter = ']';
      }
      std::getline(stream_imu_parameters, buffer, delimiter);
      const size_t index_begin = buffer.find_last_of('[');
      if (index_begin != std::string::npos) {
        buffer = buffer.substr(index_begin + 1);
      }
      imu_in_base_link_.matrix()(r, c) = std::stod(buffer);
    }
  }

  while (getline(stream_imu_parameters, buffer)) {
    _setValueByKey(buffer, "rate_hz: ", rate_hz_);
    _setValueByKey(buffer, "gyroscope_noise_density: ", gyroscope_noise_density_);
    _setValueByKey(buffer, "gyroscope_random_walk: ", gyroscope_random_walk_);
    _setValueByKey(buffer, "accelerometer_noise_density: ", accelerometer_noise_density_);
    _setValueByKey(buffer, "accelerometer_random_walk: ", accelerometer_random_walk_);
  }
  stream_imu_parameters.close();
}

void loadImageInformation(const std::string& folder_image_names_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_) {
  const std::string file_path_image_names = folder_image_names_ + "/data.csv";
  std::ifstream stream_image_names(file_path_image_names);
  if (!stream_image_names.good() || !stream_image_names.is_open()) {
    throw std::runtime_error("loadImageInformation|ERROR: unable to open file: " +
                             file_path_image_names);
  }
  image_filenames_.clear();
  timestamps_seconds_.clear();

  // ds skip the first 3 lines (TUM format header information)
  std::cerr << "loadImageInformation|<header>" << std::endl;
  std::string buffer;
  std::getline(stream_image_names, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadImageInformation|</header>" << std::endl;

  // ds parse string into tokens split by custom delimiter comma UAGH
  while (std::getline(stream_image_names, buffer)) {
    std::replace(buffer.begin(), buffer.end(), ',', ' ');
    std::stringstream stream(buffer);

    // ds parse image information by tokens
    double timestamp_nanoseconds;
    std::string image_file_name;
    while (stream >> timestamp_nanoseconds >> image_file_name) {
      image_filenames_.push_back(folder_image_names_ + "/data/" + image_file_name);
      timestamps_seconds_.push_back(timestamp_nanoseconds / 1e9);
    }
  }
  stream_image_names.close();
  std::cerr << "loadImageInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << file_path_image_names << "'" << std::endl;
}

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Isometry3f& imu_from_camera_left_) {
  std::cerr << "loadCameraCalibration|calibration file: '" << file_path_camera_calibration_ << "'"
            << std::endl;
  std::ifstream stream(file_path_camera_calibration_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadCameraCalibration|ERROR: unable to open file: " +
                             file_path_camera_calibration_);
  }

  // ds evil parsing - skip lines until we hit the transform 'data' line
  std::string buffer;
  for (size_t i = 0; i < 8; ++i) {
    std::getline(stream, buffer);
  }

  // ds parse transform by tokens
  for (size_t r = 0; r < 4; ++r) {
    for (size_t c = 0; c < 4; ++c) {
      char delimiter = ',';
      if (r == 3 && c == 3) {
        delimiter = ']';
      }
      std::getline(stream, buffer, delimiter);
      const size_t index_begin = buffer.find_last_of('[');
      if (index_begin != std::string::npos) {
        buffer = buffer.substr(index_begin + 1);
      }
      imu_from_camera_left_.matrix()(r, c) = std::stod(buffer);
    }
  }

  // ds skip more lines to get to intrinsics and distortion coefficients
  for (size_t i = 0; i < 6; ++i) {
    std::getline(stream, buffer);
  }

  // ds parse camera focal length and principal point by tokens (fu, fv, cu, cv)
  double camera_intrinsic_parameters[4];
  for (size_t i = 0; i < 4; ++i) {
    char delimiter = ',';
    if (i == 3) {
      delimiter = ']';
    }
    std::getline(stream, buffer, delimiter);
    const size_t index_begin = buffer.find_last_of('[');
    if (index_begin != std::string::npos) {
      buffer = buffer.substr(index_begin + 1);
    }
    camera_intrinsic_parameters[i] = std::stod(buffer);
  }
  camera_calibration_matrix_left_(0, 0) = camera_intrinsic_parameters[0];
  camera_calibration_matrix_left_(0, 2) = camera_intrinsic_parameters[2];
  camera_calibration_matrix_left_(1, 1) = camera_intrinsic_parameters[1];
  camera_calibration_matrix_left_(1, 2) = camera_intrinsic_parameters[3];

  // ds skip distortion model
  std::getline(stream, buffer);

  // ds parse distortion coefficients (in order)
  for (size_t i = 0; i < 4; ++i) {
    char delimiter = ',';
    if (i == 3) {
      delimiter = ']';
    }
    std::getline(stream, buffer, delimiter);
    const size_t index_begin = buffer.find_last_of('[');
    if (index_begin != std::string::npos) {
      buffer = buffer.substr(index_begin + 1);
    }
    distortion_coefficients_left_(i) = std::stod(buffer);
  }
  stream.close();
  std::cerr << "loadCameraCalibration|camera matrix: " << std::endl;
  std::cerr << camera_calibration_matrix_left_ << std::endl;
  std::cerr << "loadCameraCalibration|distortion coefficients: " << std::endl;
  std::cerr << distortion_coefficients_left_.transpose() << std::endl;
  std::cerr << "loadCameraCalibration|camera w.r.t. IMU: " << std::endl;
  std::cerr << imu_from_camera_left_.matrix() << std::endl;
}

void serializeCameraImageAndInfo(Serializer& serializer_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const bool& enable_rectification_,
                                 const cv::Mat* undistort_rectify_maps_,
                                 const Matrix3f& camera_matrix_rectified_) {
  // ds create image message
  ImageMessagePtr image_message(
    new ImageMessage("/" + label_ + "/image_raw", "/" + label_, index_, timestamp_));

  // ds load RGB image from disk and convert
  cv::Mat image_opencv = cv::imread(file_path_image_, CV_LOAD_IMAGE_GRAYSCALE);

  // ds rectify image
  if (enable_rectification_) {
    cv::remap(image_opencv,
              image_opencv,
              undistort_rectify_maps_[0],
              undistort_rectify_maps_[1],
              cv::INTER_LINEAR);
  }
  if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
    throw std::runtime_error(
      "serializeCameraImageAndInfo|ERROR: invalid camera image: " + file_path_image_ +
      " with dimensions: " + std::to_string(image_opencv.rows) + " x " +
      std::to_string(image_opencv.cols));
  }

  // ds map to srrg
  ImageUInt8* base_image(new ImageUInt8());
  base_image->fromCv(image_opencv);
  image_message->setImage(base_image);
  image_message->image_cols.setValue(image_opencv.cols);
  image_message->image_rows.setValue(image_opencv.rows);

  // ds create camera info message
  CameraInfoMessagePtr camera_info_message(
    new CameraInfoMessage(image_message->topic.value() + "/info",
                          image_message->frame_id.value(),
                          image_message->seq.value(),
                          image_message->timestamp.value()));
  camera_info_message->projection_model.setValue("pinhole");
  if (enable_rectification_) {
    camera_info_message->distortion_model.setValue("undistorted-rectified");
    camera_info_message->camera_matrix.setValue(camera_matrix_rectified_);
  } else {
    camera_info_message->distortion_model.setValue(distortion_model_name_);
    camera_info_message->distortion_coefficients.setValue(distortion_coefficients_);
    camera_info_message->camera_matrix.setValue(camera_matrix_);
  }
  camera_info_message->cols.setValue(image_opencv.cols);
  camera_info_message->rows.setValue(image_opencv.rows);

  // ds write messages related to this image
  serializer_.writeObject(*camera_info_message);
  serializer_.writeObject(*image_message);
  if (label_ == "camera_left") {
    std::cerr << "L";
  } else if (label_ == "camera_right") {
    std::cerr << "R";
  } else {
    throw std::runtime_error("serializeCameraImageAndInfo|ERROR: unknown camera label");
  }

  // ds display converted images
  cv::imshow("processed image: " + label_, image_opencv);
  cv::waitKey(1);
  ++index_;
}

void initializeRectification(const Matrix3f& camera_calibration_matrix_left_,
                             const Matrix3f& camera_calibration_matrix_right_,
                             const Vector5d& distortion_coefficients_left_,
                             const Vector5d& distortion_coefficients_right_,
                             const cv::Size& opencv_image_size_,
                             cv::Mat* undistort_rectify_maps_left_,
                             cv::Mat* undistort_rectify_maps_right_,
                             Matrix3f& camera_calibration_matrix_rectified_,
                             Isometry3f& camera_left_to_right_) {
  // ds rectification
  cv::Mat opencv_camera_calibration_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat opencv_camera_calibration_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat opencv_distortion_coefficients_left(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat opencv_distortion_coefficients_right(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat opencv_projection_matrix_left(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat opencv_projection_matrix_right(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat opencv_rectification_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat opencv_rectification_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat opencv_rotation_camera_left_to_right(3, 3, CV_64F, cv::Scalar(0));
  cv::Mat opencv_translation_camera_left_to_right(3, 1, CV_64F, cv::Scalar(0));
  cv::Mat opencv_depth_mapping(4, 4, CV_64F, cv::Scalar(0));

  // ds buffer matrices to opencv
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      opencv_camera_calibration_matrix_left.at<double>(r, c) =
        camera_calibration_matrix_left_(r, c);
      opencv_camera_calibration_matrix_right.at<double>(r, c) =
        camera_calibration_matrix_right_(r, c);
      opencv_rotation_camera_left_to_right.at<double>(r, c) = camera_left_to_right_.linear()(r, c);
    }
    opencv_translation_camera_left_to_right.row(r) = camera_left_to_right_.translation()(r);
  }
  for (size_t i = 0; i < 4; ++i) {
    opencv_distortion_coefficients_left.at<double>(i)  = distortion_coefficients_left_(i);
    opencv_distortion_coefficients_right.at<double>(i) = distortion_coefficients_right_(i);
  }

  // ds compute rectification parameters
  cv::stereoRectify(opencv_camera_calibration_matrix_left,
                    opencv_distortion_coefficients_left,
                    opencv_camera_calibration_matrix_right,
                    opencv_distortion_coefficients_right,
                    opencv_image_size_,
                    opencv_rotation_camera_left_to_right,
                    opencv_translation_camera_left_to_right,
                    opencv_rectification_matrix_left,
                    opencv_rectification_matrix_right,
                    opencv_projection_matrix_left,
                    opencv_projection_matrix_right,
                    opencv_depth_mapping,
                    CV_CALIB_ZERO_DISPARITY,
                    0);

  // ds check if failed
  if (cv::norm(opencv_projection_matrix_left) == 0 ||
      cv::norm(opencv_projection_matrix_right) == 0) {
    throw std::runtime_error("ERROR: rectification parameter retrieval failed");
  }

  // ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(opencv_camera_calibration_matrix_left,
                              opencv_distortion_coefficients_left,
                              opencv_rectification_matrix_left,
                              opencv_projection_matrix_left,
                              opencv_image_size_,
                              CV_16SC2,
                              undistort_rectify_maps_left_[0],
                              undistort_rectify_maps_left_[1]);
  cv::initUndistortRectifyMap(opencv_camera_calibration_matrix_right,
                              opencv_distortion_coefficients_right,
                              opencv_rectification_matrix_right,
                              opencv_projection_matrix_right,
                              opencv_image_size_,
                              CV_16SC2,
                              undistort_rectify_maps_right_[0],
                              undistort_rectify_maps_right_[1]);

  // ds check if rectification failed
  if (cv::norm(undistort_rectify_maps_left_[0]) == 0 ||
      cv::norm(undistort_rectify_maps_left_[1]) == 0) {
    throw std::runtime_error("ERROR: unable to undistort and rectify camera left");
  }
  if (cv::norm(undistort_rectify_maps_right_[0]) == 0 ||
      cv::norm(undistort_rectify_maps_right_[1]) == 0) {
    throw std::runtime_error("ERROR: unable to undistort and rectify camera right");
  }

  std::cerr << "initializeRectification|LEFT projection matrix: " << std::endl;
  std::cerr << opencv_projection_matrix_left << std::endl;
  std::cerr << "initializeRectification|RIGHT projection matrix: " << std::endl;
  std::cerr << opencv_projection_matrix_right << std::endl;

  // ds get right projection matrix to eigen space
  Matrix3_4f projection_matrix_right_eigen(Matrix3_4f::Zero());
  for (uint32_t row = 0; row < 3; ++row) {
    for (uint32_t col = 0; col < 4; ++col) {
      // ds for camera calibration parameters
      if (col < 3) {
        // ds consistency check
        if (std::fabs(opencv_projection_matrix_left.at<double>(row, col) -
                      opencv_projection_matrix_right.at<double>(row, col)) > 1e-5) {
          throw std::runtime_error(
            "initializeRectification|ERROR: inconsistent projection matrices");
        }
        // ds pick calibration parameters
        camera_calibration_matrix_rectified_(row, col) =
          opencv_projection_matrix_right.at<double>(row, col);
      }

      // ds convert whole projection matrix
      projection_matrix_right_eigen(row, col) = opencv_projection_matrix_right.at<double>(row, col);
    }
  }

  // ds compute offset for right camera in order to reconstruct projection matrix form txt_io
  // message
  const Vector3f offset(camera_calibration_matrix_rectified_.fullPivLu().solve(
    projection_matrix_right_eigen.block<3, 1>(0, 3)));
  camera_left_to_right_.setIdentity();
  camera_left_to_right_.translation() = offset;
  std::cerr << "initializeRectification|rectified camera calibration matrix: " << std::endl;
  std::cerr << camera_calibration_matrix_rectified_ << std::endl;
  std::cerr << "initializeRectification|rectified stereo baseline (m): "
            << camera_left_to_right_.translation().transpose() << std::endl;
}
