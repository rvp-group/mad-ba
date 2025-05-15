#include <dirent.h>
#include <experimental/filesystem>
#include <iostream>
#include <iterator>

#include <srrg_boss/serializer.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/range_message.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_file_sink.h>
#include <srrg_messages/message_handlers/message_sorted_source.h>
#include <srrg_messages/message_handlers/message_synchronized_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/instances.h>

using namespace srrg2_core;
namespace fs = std::experimental::filesystem;

bool verbose = false;
const std::string exe_name("converter_kitti_to_srrg");
#define LOG std::cerr << exe_name + "|"
#define VLOG(X)  \
  if (verbose) { \
    X            \
  }

using TimestampPoseMap = std::map<double,
                                  Isometry3f,
                                  std::less<double>,
                                  Eigen::aligned_allocator<std::pair<const double, Isometry3f>>>;
using DoubleRangesMap  = std::map<double, std::vector<float>>;

// clang-format off
const char* banner[] = {"This program converts Malaga formatted stereo RGB files to SRRG format (json/bag)", 0};
// clang-format on

void printProgress(const float& progress) {
  const int bar_width = 80;

  std::cerr << FG_BWHITE("[ ");
  int pos = bar_width * progress;
  for (int i = 0; i < bar_width; ++i) {
    if (i < pos)
      std::cerr << FG_YELLOW("=");
    else if (i == pos)
      std::cerr << FG_BYELLOW("|");
    else
      std::cerr << " ";
  }
  std::cerr << FG_BWHITE(" ] ") << int(progress * 100.0) << " %\r";
  std::cerr.flush();
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_);

void loadImageInformation(const std::string& folder_images_,
                          std::vector<std::string>& image_filenames_left_,
                          std::vector<double>& timestamps_seconds_left_,
                          std::vector<std::string>& image_filenames_right_,
                          std::vector<double>& timestamps_seconds_right_);

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        StdVectorEigenVector3d& orientations_,
                        std::vector<double>& timestamps_seconds_);

void loadLaserInformation(DoubleRangesMap& laser_data_, const std::string& filename_prefix_);

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Matrix3f& camera_calibration_matrix_right_,
                           Vector5d& distortion_coefficients_right_,
                           Isometry3f& camera_right_from_left_);

void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_);

void serializeLaser(MessageFileSink& sink_,
                    const size_t& index_,
                    const double& timestamp_,
                    const std::string& topic_name_,
                    const std::vector<float>& ranges_,
                    const float& fov_,
                    const float& max_range_,
                    const float& scan_time_);

// srrg based on the setup information about Malaga
// [http://ingmec.ual.es/~jlblanco/papers/blanco2013malaga_urban_dataset_IJRR_draft.pdf] I
// arbitrarly set the baselink in 0
const TransformEventsMessagePtr buildStaticTfTree(const double& initial_timestamp_,
                                                  const Isometry3f& camera_right_in_left_);

static const std::string prefix_name("malaga-urban-dataset-extract-");
static const std::string img_size("1024x768");
static const std::string image_dir_postfix("_rectified_" + img_size + "_Images/");
static const std::string calib_filename("camera_params_rectified_a=0_" + img_size + ".txt");
static const std::string sensor_innerfix("_all-sensors_");
static const std::string times_postfix("_times.txt");

static std::string sequence("");

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_sequence_number(
    &command_line_parser,
    "s",
    "sequence",
    "sequence number of the MALAGA dataset to convert [between 00 and 15]",
    "");
  ArgumentString argument_output_file(
    &command_line_parser, "o", "output", "converted output file", "messages.json");
  ArgumentFlag arg_verbose(&command_line_parser, "v", "verbose", "logs something", true);

  command_line_parser.parse();
  if (argument_sequence_number.value().empty()) {
    std::cerr << "ERROR: sequence number not inserted (-s)" << std::endl;
    return 0;
  }

  verbose = arg_verbose.isSet();
  if (argument_output_file.value().empty()) {
    std::cerr << "ERROR: output filename is empty (set with -o)" << std::endl;
    return 0;
  }

  sequence = argument_sequence_number.value();
  size_t sequence_number(std::stoi(sequence));
  // srrg check correct sequence
  if (sequence_number < 0 || sequence_number > 15) {
    throw std::runtime_error(exe_name + "|ERROR handled MALAGA sequences goes from 00 to 15");
  }

  // srrg cache the files
  std::string dataset_name(prefix_name + sequence);
  std::string dataset_folder(dataset_name + "/");
  std::string sensor_file_prefix(dataset_folder + dataset_name + sensor_innerfix);
  std::string calib_file(dataset_folder + calib_filename);
  std::string gps_file(sensor_file_prefix + "GPS.txt");
  std::string imu_file(sensor_file_prefix + "IMU.txt");
  std::string images_folder(dataset_folder + dataset_name + image_dir_postfix);

  if (!isAccessible(dataset_folder)) {
    throw std::runtime_error(exe_name + "|ERROR cannot dataset_folder file [ " + dataset_folder +
                             " ]");
  }

  if (!isAccessible(calib_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot calib_file file [ " + calib_file + " ]");
  }

  if (!isAccessible(gps_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot gps_file file [ " + gps_file + " ]");
  }

  if (!isAccessible(imu_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot imu_file file [ " + imu_file + " ]");
  }

  if (!isAccessible(images_folder)) {
    throw std::runtime_error(exe_name + "|ERROR cannot images_folder file [ " + images_folder +
                             " ]");
  }

  // ds buffer ground truth data
  StdVectorEigenIsometry3f gps_poses_in_world;
  std::vector<double> timestamps_gt;
  loadGroundTruthPosesAndStamps(gps_file, gps_poses_in_world, timestamps_gt);
  if (timestamps_gt.empty()) {
    throw std::runtime_error("ERROR: no ground truth poses loaded");
  }

  // ds save ground truth in kitti format (for benchmarking)
  std::ofstream ground_truth_kitti_format("gt.txt", std::ofstream::out);
  std::ofstream ground_truth_timestamps_kitti_format("times.txt", std::ofstream::out);
  ground_truth_kitti_format << std::fixed;
  ground_truth_kitti_format << std::setprecision(9);
  ground_truth_timestamps_kitti_format << std::fixed;
  ground_truth_timestamps_kitti_format << std::setprecision(9);
  Isometry3f gps_from_camera(Isometry3f::Identity());
  gps_from_camera.linear() << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  for (size_t i = 0; i < timestamps_gt.size(); ++i) {
    // ds move to kitti origin uagh
    const Isometry3f world_from_camera(gps_from_camera * gps_poses_in_world[i]);
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        ground_truth_kitti_format << world_from_camera.matrix()(r, c) << " ";
      }
    }
    ground_truth_kitti_format << std::endl;
    ground_truth_timestamps_kitti_format << timestamps_gt[i] << std::endl;
  }
  ground_truth_kitti_format.close();
  ground_truth_timestamps_kitti_format.close();
  std::cerr << "saved GT information to 'gt.txt' and 'times.txt' (KITTI style)" << std::endl;

  // ds read depth and RGB images and timestamps
  std::vector<std::string> image_file_paths_camera_left;
  std::vector<double> timestamps_camera_left;
  std::vector<std::string> image_file_paths_camera_right;
  std::vector<double> timestamps_camera_right;

  // ds parse image information
  loadImageInformation(images_folder,
                       image_file_paths_camera_left,
                       timestamps_camera_left,
                       image_file_paths_camera_right,
                       timestamps_camera_right);
  if (timestamps_camera_left.empty()) {
    throw std::runtime_error("ERROR: no left camera image information retrieved");
  }

  // ds load camera calibration data from files
  Matrix3f camera_calibration_matrix_left(Matrix3f::Zero());
  Vector5d distortion_coefficients_left(Vector5d::Zero());
  Matrix3f camera_calibration_matrix_right(Matrix3f::Zero());
  Vector5d distortion_coefficients_right(Vector5d::Zero());
  Isometry3f camera_left_to_right(Isometry3f::Identity());
  loadCameraCalibration(calib_file,
                        camera_calibration_matrix_left,
                        distortion_coefficients_left,
                        camera_calibration_matrix_right,
                        distortion_coefficients_right,
                        camera_left_to_right);

  std::cerr << "camera LEFT w.r.t. RIGHT: " << std::endl;
  std::cerr << camera_left_to_right.matrix() << std::endl;

  // ds common parameters for all sequences
  const std::string distortion_model_name = "undistorted";

  // ds TODO load IMU information
  std::vector<double> timestamps_imu;
  StdVectorEigenVector3d angular_velocities;
  StdVectorEigenVector3d linear_accelerations;
  StdVectorEigenVector3d orientations_rpy;
  const double rate_hz = 100;
  loadIMUInformation(
    imu_file, angular_velocities, linear_accelerations, orientations_rpy, timestamps_imu);

  DoubleRangesMap sick_front;
  DoubleRangesMap sick_rear;
  DoubleRangesMap hokuyo_1;
  DoubleRangesMap hokuyo_2;
  DoubleRangesMap hokuyo_3;
  std::vector<float> data_sick_front;
  std::vector<float> data_sick_rear;
  std::vector<float> data_hokuyo_1;
  std::vector<float> data_hokuyo_2;
  std::vector<float> data_hokuyo_3;
  loadLaserInformation(sick_front, sensor_file_prefix + "LASER2");
  loadLaserInformation(sick_rear, sensor_file_prefix + "LASER1");
  loadLaserInformation(hokuyo_1, sensor_file_prefix + "HOKUYO1");
  loadLaserInformation(hokuyo_2, sensor_file_prefix + "HOKUYO2");
  loadLaserInformation(hokuyo_3, sensor_file_prefix + "HOKUYO3");
  LOG << "mismatching size: gt timestamps others: " << FG_BLUE(timestamps_gt.size()) << std::endl;
  std::cerr << "\ttimestamps_sick_front:  " << sick_front.size() << std::endl;
  std::cerr << "\ttimestamps_sick_rear:   " << sick_rear.size() << std::endl;
  std::cerr << "\ttimestamps_hokuyo_1:    " << hokuyo_1.size() << std::endl;
  std::cerr << "\ttimestamps_hokuyo_2:    " << hokuyo_2.size() << std::endl;
  std::cerr << "\ttimestamps_hokuyo_3:    " << hokuyo_3.size() << std::endl;
  std::cerr << "\ttimestamps_imu:         " << timestamps_imu.size() << std::endl;
  std::cerr << "\ttimestamps_camera_l:    " << timestamps_camera_left.size() << std::endl;
  std::cerr << "\ttimestamps_camera_r:    " << timestamps_camera_right.size() << std::endl;

  const size_t tot_messages = timestamps_gt.size() + sick_front.size() + sick_rear.size() +
                              hokuyo_1.size() + hokuyo_2.size() + hokuyo_3.size() +
                              timestamps_imu.size() + timestamps_camera_left.size() +
                              timestamps_camera_right.size();

  // ds stop before starting to allow for a quick configuration inspection
  //  std::cerr << "press [ENTER] to start conversion" << std::endl;
  //  getchar();

  // ds playback configuration UAGHS
  size_t index_gt           = 0;
  size_t index_camera_left  = 0;
  size_t index_camera_right = 0;
  size_t index_imu          = 0;
  size_t index_sick_front   = 0;
  size_t index_sick_rear    = 0;
  size_t index_hokuyo_1     = 0;
  size_t index_hokuyo_2     = 0;
  size_t index_hokuyo_3     = 0;

  // ds determine initial, smallest timestamp
  double timestamp_oldest = std::numeric_limits<double>::max();
  timestamp_oldest        = std::min(timestamp_oldest, timestamps_gt[0]);
  timestamp_oldest        = std::min(timestamp_oldest, sick_front.begin()->first);
  timestamp_oldest        = std::min(timestamp_oldest, sick_rear.begin()->first);
  timestamp_oldest        = std::min(timestamp_oldest, hokuyo_1.begin()->first);
  timestamp_oldest        = std::min(timestamp_oldest, hokuyo_2.begin()->first);
  timestamp_oldest        = std::min(timestamp_oldest, hokuyo_3.begin()->first);
  timestamp_oldest        = std::min(timestamp_oldest, timestamps_imu[0]);
  timestamp_oldest        = std::min(timestamp_oldest, timestamps_camera_left[0]);
  timestamp_oldest        = std::min(timestamp_oldest, timestamps_camera_right[0]);

  std::cerr << std::setprecision(16);
  std::cerr << "initial timestamp (s): " << timestamp_oldest << std::endl;
  LOG << "writing output to [ " << FG_YELLOW(argument_output_file.value()) << " ]\n";
  MessageFileSink sink;
  sink.open(argument_output_file.value());
  SystemUsageCounter::tic();

  const TransformEventsMessagePtr tf_static_message =
    buildStaticTfTree(timestamp_oldest, camera_left_to_right);
  sink.putMessage(tf_static_message);

  // ds playback all buffers
  size_t processed_msgs = 0;
  while (true) {
    double first_timestamp = std::numeric_limits<double>::max();
    if (timestamps_gt.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_gt[0]);
    }
    if (sick_front.size()) {
      first_timestamp = std::min(first_timestamp, sick_front.begin()->first);
    }
    if (sick_rear.size()) {
      first_timestamp = std::min(first_timestamp, sick_rear.begin()->first);
    }
    if (hokuyo_1.size()) {
      first_timestamp = std::min(first_timestamp, hokuyo_1.begin()->first);
    }
    if (hokuyo_2.size()) {
      first_timestamp = std::min(first_timestamp, hokuyo_2.begin()->first);
    }
    if (hokuyo_3.size()) {
      first_timestamp = std::min(first_timestamp, hokuyo_3.begin()->first);
    }
    if (timestamps_imu.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_imu[0]);
    }
    if (timestamps_camera_left.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_camera_left[0]);
    }
    if (timestamps_camera_right.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_camera_right[0]);
    }

    if (first_timestamp == std::numeric_limits<double>::max()) {
      break;
    }
    if (timestamps_gt.front() == first_timestamp) {
      OdometryMessagePtr gt_imu_in_world(
        new OdometryMessage("/ground_truth", "gps", index_gt, first_timestamp));
      gt_imu_in_world->pose.setValue(gps_poses_in_world.front());
      sink.putMessage(gt_imu_in_world);
      gps_poses_in_world.erase(gps_poses_in_world.begin());
      timestamps_gt.erase(timestamps_gt.begin());
      ++index_gt;
      ++processed_msgs;
    }
    if (sick_front.begin()->first == first_timestamp) {
      serializeLaser(sink,
                     index_sick_front,
                     first_timestamp,
                     "sick_front",
                     sick_front.begin()->second,
                     M_PI,
                     80.0,
                     1 / 75);
      sick_front.erase(sick_front.begin());
      ++index_sick_front;
      ++processed_msgs;
    }
    if (sick_rear.begin()->first == first_timestamp) {
      serializeLaser(sink,
                     index_sick_rear,
                     first_timestamp,
                     "sick_rear",
                     sick_rear.begin()->second,
                     M_PI,
                     80.0,
                     1 / 75);
      sick_rear.erase(sick_rear.begin());
      ++index_sick_rear;
      ++processed_msgs;
    }
    if (hokuyo_1.begin()->first == first_timestamp) {
      serializeLaser(sink,
                     index_hokuyo_1,
                     first_timestamp,
                     "hokuyo_1",
                     hokuyo_1.begin()->second,
                     3 * M_PI_2,
                     30.0,
                     1 / 40);
      hokuyo_1.erase(hokuyo_1.begin());
      ++index_hokuyo_1;
      ++processed_msgs;
    }
    if (hokuyo_2.begin()->first == first_timestamp) {
      serializeLaser(sink,
                     index_hokuyo_2,
                     first_timestamp,
                     "hokuyo_2",
                     hokuyo_2.begin()->second,
                     3 * M_PI_2,
                     30.0,
                     1 / 40);
      hokuyo_2.erase(hokuyo_2.begin());
      ++index_hokuyo_2;
      ++processed_msgs;
    }
    if (hokuyo_3.begin()->first == first_timestamp) {
      serializeLaser(sink,
                     index_hokuyo_3,
                     first_timestamp,
                     "hokuyo_3",
                     hokuyo_3.begin()->second,
                     3 * M_PI_2,
                     30.0,
                     1 / 40);
      hokuyo_3.erase(hokuyo_3.begin());
      ++index_hokuyo_3;
      ++processed_msgs;
    }

    if (timestamps_imu.front() == first_timestamp) {
      IMUMessagePtr message(new IMUMessage("/imu", "imu", index_imu, first_timestamp));
      message->angular_velocity.setValue(angular_velocities.front().cast<float>());
      //        message->angular_velocity_covariance.setValue(Matrix3f::Identity());
      message->linear_acceleration.setValue(linear_accelerations.front().cast<float>());
      //        message->linear_acceleration_covariance.setValue(Matrix3f::Identity());
      message->orientation.setValue(geometry3d::a2r<float>(orientations_rpy.front().cast<float>()));
      //        message->orientation_covariance.setValue(Matrix3f::Identity());
      message->rate_hz.setValue(rate_hz);

      angular_velocities.erase(angular_velocities.begin());
      linear_accelerations.erase(linear_accelerations.begin());
      orientations_rpy.erase(orientations_rpy.begin());
      timestamps_imu.erase(timestamps_imu.begin());
      ++index_imu;
      ++processed_msgs;
      sink.putMessage(message);
    }

    if (timestamps_camera_left.front() == first_timestamp) {
      serializeCameraImageAndInfo(sink,
                                  index_camera_left,
                                  first_timestamp,
                                  "camera_left",
                                  image_file_paths_camera_left.front(),
                                  distortion_model_name,
                                  distortion_coefficients_left,
                                  camera_calibration_matrix_left);
      image_file_paths_camera_left.erase(image_file_paths_camera_left.begin());
      timestamps_camera_left.erase(timestamps_camera_left.begin());
      ++index_camera_left;
      ++processed_msgs;
    }
    if (timestamps_camera_right.front() == first_timestamp) {
      serializeCameraImageAndInfo(sink,
                                  index_camera_right,
                                  first_timestamp,
                                  "camera_right",
                                  image_file_paths_camera_right.front(),
                                  distortion_model_name,
                                  distortion_coefficients_right,
                                  camera_calibration_matrix_right);
      image_file_paths_camera_right.erase(image_file_paths_camera_right.begin());
      timestamps_camera_right.erase(timestamps_camera_right.begin());
      ++index_camera_right;
      ++processed_msgs;
    }

    const double progress_percentage = (double) processed_msgs * 1.f / (double) (tot_messages);
    printProgress(progress_percentage);
  }
  double t = SystemUsageCounter::toc();

  LOG << "converted [ " << processed_msgs << " ] messages in [ " << t << " ] s -- FPS [ "
      << (float) processed_msgs / t << " ] Hz\n";

  sink.close();

  return 0;
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_seconds_gt_) {
  gps_poses_in_world_.clear();
  timestamps_seconds_gt_.clear();

  // ds load ground truth text file
  std::ifstream stream_ground_truth(ground_truth_file_, std::ifstream::in);

  // ds check failure
  if (!stream_ground_truth.is_open() || !stream_ground_truth.good()) {
    throw std::runtime_error("loadGroundTruthPosesAndStamps|ERROR: unable to open file: '" +
                             ground_truth_file_ + "'");
  }

  // ds reading buffers - skip the first line (header line)
  double timestamp_seconds      = 0;
  double latitude_radians       = 0;
  double longitude_radians      = 0;
  double altitude_meters        = 0;
  uint32_t fix                  = 0;
  uint32_t number_of_satellites = 0;
  double speed_knots            = 0;
  double heading_degrees        = 0;
  std::string line_buffer("");
  std::getline(stream_ground_truth, line_buffer);

  // ds start parsing all poses to the last line
  Isometry3d pose_gps_previous(Isometry3d::Identity());
  const Vector3d orientation_car(0, 0, 1);

  // ds reading buffers
  line_buffer = "";
  while (std::getline(stream_ground_truth, line_buffer) && !line_buffer.empty()) {
    std::istringstream stream(line_buffer);

    // ds parse in fixed order
    stream >> timestamp_seconds;
    stream >> latitude_radians;
    stream >> longitude_radians;
    stream >> altitude_meters;
    stream >> fix;
    stream >> number_of_satellites;
    stream >> speed_knots;
    stream >> heading_degrees;

    // ds parse pose matrix directly from file
    Isometry3d pose(Isometry3d::Identity());
    for (uint8_t u = 0; u < 3; ++u) {
      stream >> pose(u, 3);
    }

    // ds compute relative orientation (have nothing else)
    const Vector3d translation_delta = pose.translation() - pose_gps_previous.translation();
    Vector3d translation_delta_normalized(Vector3d::Zero());
    if (translation_delta.norm() > 0) {
      translation_delta_normalized = translation_delta / translation_delta.norm();
      Quaterniond orientation =
        Quaterniond::FromTwoVectors(orientation_car, translation_delta_normalized);
      pose.linear() = orientation.toRotationMatrix();

      // ds check if we can update the initial orientation
      if (gps_poses_in_world_.size() == 1) {
        gps_poses_in_world_.front().linear() = pose.linear().cast<float>();
      }
    }

    // ds set pose
    gps_poses_in_world_.push_back(pose.cast<float>());
    pose_gps_previous = pose;

    // ds sanity check
    if (!timestamps_seconds_gt_.empty() && timestamp_seconds < timestamps_seconds_gt_.back()) {
      throw std::runtime_error("loadGroundTruthPosesAndStamps|invalid timestamp (s): " +
                               std::to_string(timestamp_seconds));
    }

    // ds keep timestamp
    timestamps_seconds_gt_.push_back(timestamp_seconds);
  }
  stream_ground_truth.close();
  std::cerr << "loadGroundTruthPosesAndStamps|loaded GPS poses: " << gps_poses_in_world_.size()
            << std::endl;
}

void loadImageInformation(const std::string& folder_images_,
                          std::vector<std::string>& image_filenames_left_,
                          std::vector<double>& timestamps_seconds_left_,
                          std::vector<std::string>& image_filenames_right_,
                          std::vector<double>& timestamps_seconds_right_) {
  image_filenames_left_.clear();
  timestamps_seconds_left_.clear();
  image_filenames_right_.clear();
  timestamps_seconds_right_.clear();

  // ds obtain list of image filenames in the folder
  std::vector<std::string> image_file_names;
  image_file_names.reserve(10000);
  DIR* directory;
  struct dirent* entry;
  if ((directory = opendir(folder_images_.c_str()))) {
    while ((entry = readdir(directory))) {
      // ds ignore linux fake directories
      if (entry->d_name[0] != '.') {
        image_file_names.emplace_back(entry->d_name);
      }
    }
  }
  closedir(directory);

  // ds order input by timestamps
  std::sort(image_file_names.begin(), image_file_names.end());

  // ds parse string into tokens
  // ds parse image information by tokens
  double timestamp_previous_seconds = 0;
  for (const std::string& image_file_name : image_file_names) {
    // ds parse timestamp TODO less hardcoding hehe
    const double timestamp_seconds = std::stod(image_file_name.substr(12, 17));

    // ds sanity check
    if (timestamp_seconds < timestamp_previous_seconds) {
      throw std::runtime_error("loadImageInformation|ERROR: inconsistent timestamp (s): " +
                               std::to_string(timestamp_seconds) + " < " +
                               std::to_string(timestamp_previous_seconds));
    } else {
      timestamp_previous_seconds = timestamp_seconds;
    }

    // ds parse based on tag
    if (image_file_name.find("left") != std::string::npos) {
      image_filenames_left_.push_back(folder_images_ + "/" + image_file_name);
      timestamps_seconds_left_.push_back(timestamp_seconds);
    } else {
      image_filenames_right_.push_back(folder_images_ + "/" + image_file_name);
      timestamps_seconds_right_.push_back(timestamp_seconds);
    }
  }
  std::cerr << "loadImageInformation|# loaded entries LEFT: " << timestamps_seconds_left_.size()
            << " RIGHT: " << timestamps_seconds_right_.size() << " for folder '" << folder_images_
            << "'" << std::endl;
}

void loadCameraCalibration(const std::string& file_path_camera_calibration_,
                           Matrix3f& camera_calibration_matrix_left_,
                           Vector5d& distortion_coefficients_left_,
                           Matrix3f& camera_calibration_matrix_right_,
                           Vector5d& distortion_coefficients_right_,
                           Isometry3f& camera_right_from_left_) {
  std::cerr << "loadCameraCalibration|calibration file: '" << file_path_camera_calibration_ << "'"
            << std::endl;
  std::ifstream stream(file_path_camera_calibration_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadCameraCalibration|ERROR: unable to open file: " +
                             file_path_camera_calibration_);
  }

  // ds evil parsing - skip lines until we hit the transform 'data' line
  std::string buffer;
  for (size_t i = 0; i < 7; ++i) {
    std::getline(stream, buffer);
  }

  // ds read left camera parameters
  camera_calibration_matrix_left_(0, 2) = std::stod(buffer.substr(3)); // ds c_x
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(1, 2) = std::stod(buffer.substr(3)); // ds c_y
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(0, 0) = std::stod(buffer.substr(3)); // ds f_x
  std::getline(stream, buffer);
  camera_calibration_matrix_left_(1, 1) = std::stod(buffer.substr(3)); // ds f_y
  camera_calibration_matrix_left_(2, 2) = 1;
  distortion_coefficients_left_.setZero();

  // ds skip lines
  for (size_t i = 0; i < 8; ++i) {
    std::getline(stream, buffer);
  }

  // ds read right camera parameters
  camera_calibration_matrix_right_(0, 2) = std::stod(buffer.substr(3)); // ds c_x
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(1, 2) = std::stod(buffer.substr(3)); // ds c_y
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(0, 0) = std::stod(buffer.substr(3)); // ds f_x
  std::getline(stream, buffer);
  camera_calibration_matrix_right_(1, 1) = std::stod(buffer.substr(3)); // ds f_y
  camera_calibration_matrix_right_(2, 2) = 1;
  distortion_coefficients_right_.setZero();

  // ds skip lines
  for (size_t i = 0; i < 7; ++i) {
    std::getline(stream, buffer);
  }

  const size_t index_begin = buffer.find_last_of('[') + 1;
  const size_t index_end   = buffer.find_last_of(']');

  // ds parse transform by tokens
  camera_right_from_left_.setIdentity();
  std::stringstream stream_trans(buffer.substr(index_begin, index_end - index_begin));
  Vector_<float, 7> translation_orientation(Vector_<float, 7>::Zero());
  for (size_t i = 0; i < 7; ++i) {
    stream_trans >> translation_orientation(i);
  }
  camera_right_from_left_.translation() = translation_orientation.head<3>();

  if (camera_right_from_left_.translation()[0] < 0) {
    throw std::runtime_error("baseline bx must be positive");
  }

  //  // ds TODO implement actual transforms
  //  imu_from_camera_left_.setIdentity();
  //  imu_from_camera_right_.setIdentity();
  //  imu_from_camera_left_.translation()  = -camera_right_from_left.translation() / 2.0;
  //  imu_from_camera_right_.translation() = camera_right_from_left.translation() / 2.0;

  stream.close();
  std::cerr << "loadCameraCalibration|camera matrix LEFT: " << std::endl;
  std::cerr << camera_calibration_matrix_left_ << std::endl;
  std::cerr << "loadCameraCalibration|camera matrix RIGHT: " << std::endl;
  std::cerr << camera_calibration_matrix_right_ << std::endl;
  std::cerr << "loadCameraCalibration|distortion coefficients LEFT: " << std::endl;
  std::cerr << distortion_coefficients_left_.transpose() << std::endl;
  std::cerr << "loadCameraCalibration|distortion coefficients RIGHT: " << std::endl;
  std::cerr << distortion_coefficients_right_.transpose() << std::endl;
  std::cerr << "camera right in camera left [t component in meters]\n"
            << camera_right_from_left_.matrix() << "\n";

  //  std::cerr << "loadCameraCalibration|LEFT camera w.r.t. IMU: " << std::endl;
  //  std::cerr << imu_from_camera_left_.matrix() << std::endl;
  //  std::cerr << "loadCameraCalibration|RIGHT camera w.r.t. IMU: " << std::endl;
  //  std::cerr << imu_from_camera_right_.matrix() << std::endl;
}

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& angular_velocities_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        StdVectorEigenVector3d& orientations_,
                        std::vector<double>& timestamps_seconds_) {
  angular_velocities_.clear();
  linear_accelerations_.clear();
  orientations_.clear();
  std::cerr << "loadIMUInformation|file: '" << file_imu_data_ << "'" << std::endl;
  std::ifstream stream(file_imu_data_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadIMUInformation|ERROR: unable to open file: " + file_imu_data_);
  }

  // ds skip first line (header)
  std::string buffer;
  std::getline(stream, buffer);
  std::cerr << "loadIMUInformation|<header>" << std::endl;
  std::cerr << buffer << std::endl;
  std::cerr << "loadIMUInformation|<header>" << std::endl;

  // ds parse by tokens
  angular_velocities_.reserve(100000);
  linear_accelerations_.reserve(100000);
  orientations_.reserve(100000);
  double timestamp_seconds = 0;
  double IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC, IMU_YAW_VEL, IMU_PITCH_VEL, IMU_ROLL_VEL, IMU_X_VEL,
    IMU_Y_VEL, IMU_Z_VEL, IMU_YAW, IMU_PITCH, IMU_ROLL, IMU_X, IMU_Y, IMU_Z;
  while (stream >> timestamp_seconds >> IMU_X_ACC >> IMU_Y_ACC >> IMU_Z_ACC >> IMU_YAW_VEL >>
         IMU_PITCH_VEL >> IMU_ROLL_VEL >> IMU_X_VEL >> IMU_Y_VEL >> IMU_Z_VEL >> IMU_YAW >>
         IMU_PITCH >> IMU_ROLL >> IMU_X >> IMU_Y >> IMU_Z) {
    angular_velocities_.emplace_back(Vector3d(IMU_YAW_VEL, IMU_PITCH_VEL, IMU_ROLL_VEL));
    linear_accelerations_.emplace_back(Vector3d(IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC));
    orientations_.emplace_back(Vector3d(IMU_YAW, IMU_PITCH, IMU_ROLL));
    timestamps_seconds_.emplace_back(timestamp_seconds);
  }
  std::cerr << "loadIMUInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << file_imu_data_ << "'" << std::endl;
  stream.close();
}

void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_) {
  // ds create image message
  ImageMessagePtr image_message(
    new ImageMessage("/" + label_ + "/image_raw", "/" + label_, index_, timestamp_));

  // ds load RGB image from disk and convert
  cv::Mat image_opencv = cv::imread(file_path_image_, CV_LOAD_IMAGE_GRAYSCALE);
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
  camera_info_message->distortion_model.setValue(distortion_model_name_);
  camera_info_message->distortion_coefficients.setValue(distortion_coefficients_);
  camera_info_message->camera_matrix.setValue(camera_matrix_);
  camera_info_message->cols.setValue(image_opencv.cols);
  camera_info_message->rows.setValue(image_opencv.rows);

  // ds write messages related to this image
  sink_.putMessage(image_message);
  sink_.putMessage(camera_info_message);
}

void loadLaserInformation(DoubleRangesMap& data_, const std::string& filename_prefix_) {
  data_.clear();
  const std::string timestamps_file(filename_prefix_ + times_postfix);
  const std::string filename(filename_prefix_ + ".txt");

  if (!isAccessible(filename)) {
    throw std::runtime_error(exe_name + "|ERROR cannot laser file [ " + filename + " ]");
  }

  if (!isAccessible(timestamps_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot timestamps file [ " + timestamps_file +
                             " ]");
  }

  std::ifstream stream(timestamps_file, std::ifstream::in);
  std::string buffer("");
  std::vector<double> ts;
  while (std::getline(stream, buffer)) {
    std::stringstream ss(buffer);
    double timestamp = -1.0;
    ss >> timestamp;
    ts.push_back(timestamp);
  }
  stream.close();

  stream.open(filename, std::ifstream::in);
  buffer.clear();

  std::vector<std::vector<float>> d;
  while (std::getline(stream, buffer)) {
    std::stringstream ss(buffer);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<float> asd;
    while (begin != end) {
      asd.push_back(std::stod(*begin));
      ++begin;
    }
    d.push_back(asd);
  }
  stream.close();

  if (ts.size() != d.size()) {
    throw std::runtime_error("loadLaserInformation| mismatching sizes");
  }

  for (size_t i = 0; i < ts.size(); ++i) {
    data_.insert(std::make_pair(ts[i], d[i]));
  }
}

void serializeLaser(MessageFileSink& sink_,
                    const size_t& index_,
                    const double& timestamp_,
                    const std::string& topic_name_,
                    const std::vector<float>& ranges_,
                    const float& fov_,
                    const float& max_range_,
                    const float& scan_time_) {
  LaserMessagePtr laser_message(
    new LaserMessage("/" + topic_name_, topic_name_, index_, timestamp_));

  float num_scans       = (float) ranges_.size();
  float angle_increment = (float) fov_ / num_scans;
  laser_message->angle_increment.setValue(angle_increment);
  laser_message->angle_min.setValue(-3 * M_PI_4);
  laser_message->angle_min.setValue(+3 * M_PI_4);
  laser_message->range_min.setValue(0.1);
  laser_message->range_max.setValue(max_range_);
  laser_message->ranges.setValue(std::move(ranges_));
  laser_message->scan_time.setValue(scan_time_);
  laser_message->time_increment.setValue(scan_time_ / num_scans);
  sink_.putMessage(laser_message);
}

const TransformEventsMessagePtr buildStaticTfTree(const double& initial_timestamp_,
                                                  const Isometry3f& camera_right_in_left_) {
  const double& timestamp(initial_timestamp_);

  TransformEventsMessagePtr tf_static_message(
    new TransformEventsMessage("/tf_static", "/base_link", 0, timestamp));
  tf_static_message->events.resize(10);

  {
    Isometry3f base_link(Isometry3f::Identity());
    TransformEvent event_base_link(timestamp, "base_link", base_link);
    tf_static_message->events.setValue(0, event_base_link);
  }
  {
    Isometry3f camera_left_in_base_link(Isometry3f::Identity());
    camera_left_in_base_link.translation() = Vector3f(0.785, 0, 0.057);
    camera_left_in_base_link.linear() =
      geometry3d::rotationY(srrg2_core::deg2Rad(-8.2f)).cast<float>();
    TransformEvent event_camera_left_in_base_link(
      timestamp, "camera_left", camera_left_in_base_link, "base_link");
    tf_static_message->events.setValue(1, event_camera_left_in_base_link);
  }
  {
    TransformEvent event_camera_right_in_left(
      timestamp, "camera_right", camera_right_in_left_, "camera_left");
    tf_static_message->events.setValue(2, event_camera_right_in_left);
  }
  {
    Isometry3f imu_in_base_link(Isometry3f::Identity());
    imu_in_base_link.translation() = Vector3f(0.400, 0.040, 0.000);
    TransformEvent event_imu_in_base_link(timestamp, "imu", imu_in_base_link, "base_link");
    tf_static_message->events.setValue(3, event_imu_in_base_link);
  }
  {
    Isometry3f gps_in_base_link(Isometry3f::Identity());
    gps_in_base_link.translation() = Vector3f(0.155, 0.069, 0.004);
    TransformEvent event_gps_in_base_link(timestamp, "gps", gps_in_base_link, "base_link");
    tf_static_message->events.setValue(4, event_gps_in_base_link);
  }
  {
    Isometry3f sick_rear_in_base_link(Isometry3f::Identity());
    sick_rear_in_base_link.translation() = Vector3f(-0.023, 0, 0.097);
    sick_rear_in_base_link.linear()      = geometry3d::rotationZ((float) M_PI);
    TransformEvent event_sick_rear_in_base_link(
      timestamp, "sick_rear", sick_rear_in_base_link, "base_link");
    tf_static_message->events.setValue(5, event_sick_rear_in_base_link);
  }

  {
    Isometry3f sick_front_in_base_link(Isometry3f::Identity());
    sick_front_in_base_link.translation() = Vector3f(0.536, 0, 0.093);
    TransformEvent event_sick_front_in_base_link(
      timestamp, "sick_front", sick_front_in_base_link, "base_link");
    tf_static_message->events.setValue(6, event_sick_front_in_base_link);
  }

  {
    Isometry3f hokuyo_1_in_base_link(Isometry3f::Identity());
    hokuyo_1_in_base_link.translation() = Vector3f(0.536, 0, 0.273);
    hokuyo_1_in_base_link.linear()      = geometry3d::rotationY(srrg2_core::deg2Rad(21.4f));
    TransformEvent event_hokuyo_1_in_base_link(
      timestamp, "hokuyo_1", hokuyo_1_in_base_link, "base_link");
    tf_static_message->events.setValue(7, event_hokuyo_1_in_base_link);
  }

  {
    Isometry3f hokuyo_2_in_base_link(Isometry3f::Identity());
    hokuyo_2_in_base_link.translation() = Vector3f(0.075, -0.489, 0.055);
    hokuyo_2_in_base_link.linear() << 0, -1, 0, 0, 0, -1, 1, 0, 0;

    TransformEvent event_hokuyo_2_in_base_link(
      timestamp, "hokuyo_2", hokuyo_2_in_base_link, "base_link");
    tf_static_message->events.setValue(8, event_hokuyo_2_in_base_link);
  }
  {
    Isometry3f hokuyo_3_in_base_link(Isometry3f::Identity());
    hokuyo_3_in_base_link.translation() = Vector3f(0.075, 0.489, 0.055);
    hokuyo_3_in_base_link.linear() << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    TransformEvent event_hokuyo_3_in_base_link(
      timestamp, "hokuyo_3", hokuyo_3_in_base_link, "base_link");
    tf_static_message->events.setValue(9, event_hokuyo_3_in_base_link);
  }

  return tf_static_message;
}
