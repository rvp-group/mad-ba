#include <dirent.h>
#include <experimental/filesystem>
#include <iostream>

#include <srrg_boss/serializer.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_geometry/geometry3d.h>
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
using IntStringMap     = std::map<size_t, std::string>;

// clang-format off
const char* banner[] = {"This program converts KITTI formatted stereo RGB + velodyne files to SRRG format (json/bag)", 0};
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

void loadTimestamps(std::vector<double>& timestamps_, const std::string& timestamps_file_);
void loadGroundTruthPoses(TimestampPoseMap& gt_poses,
                          const std::string& ground_truth_file_,
                          const std::vector<double>& timestamps_);
void loadCalibrationParameters(Matrix3f& K_matrix_left_,
                               Matrix3f& K_matrix_right_,
                               Isometry3f& camera_right_in_left_,
                               Isometry3f& camera_left_in_lidar_,
                               const std::string& calibration_file_);

// srrg based on the setup information about KITTI [http://www.cvlibs.net/datasets/kitti/setup.php]
// I arbitrarly set the baselink in between the two rear weels
const TransformEventsMessagePtr buildStaticTfTree(const double& initial_timestamp_,
                                                  const Isometry3f& camera_right_in_left_,
                                                  const Isometry3f& camera_left_in_lidar_);

void retrieveFilenames(IntStringMap& files_,
                       const std::string& sequence_path_,
                       const std::string& sensor_);

void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 const size_t& idx_,
                                 const double& timestamp_,
                                 const std::string& topic_name_,
                                 const std::string& file_name_,
                                 const std::string& distortion_model_name,
                                 const Matrix3f& calibration_matrix_);

void serializeVelodyne(MessageFileSink& sink_,
                       const size_t& idx_,
                       const double& timestamp_,
                       const std::string& topic_name_,
                       const std::string& file_name_,
                       const bool normalize_intensities_);

static const std::string prefix_name("data_odometry_");
static const std::string inner_path("dataset/sequences/");
static const std::string calib_path(prefix_name + "calib/" + inner_path);
static const std::string gray_path(prefix_name + "gray/" + inner_path);
static const std::string poses_path(prefix_name + "poses/dataset/poses/");
static const std::string velodyne_path(prefix_name + "velodyne/" + inner_path);

static std::string sequence("");
bool gt_present = false;
int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_sequence_number(
    &command_line_parser,
    "s",
    "sequence",
    "sequence number of the KITTI dataset to convert [between 00 and 21]",
    "");
  ArgumentString argument_output_file(
    &command_line_parser, "o", "output", "converted output file", "messages.json");
  ArgumentFlag arg_verbose(&command_line_parser, "v", "verbose", "logs something", true);
  ArgumentFlag arg_normalize_intensity(
    &command_line_parser,
    "ni",
    "normalize-intensity",
    "normalize intensity by the maximum intensity value ever read in this run [optional]",
    false);
  command_line_parser.parse();
  if (argument_sequence_number.value().empty()) {
    std::cerr << "ERROR: sequence number not inserted (-s)" << std::endl;
    return 0;
  }

  sequence = argument_sequence_number.value();
  verbose  = arg_verbose.isSet();
  size_t sequence_number(std::stoi(sequence));

  // srrg check correct sequence
  if (sequence_number < 0 || sequence_number > 21) {
    throw std::runtime_error(exe_name + "|ERROR handled KITTI sequences goes from 00 to 21");
  }

  // srrg check if sequence has gt
  if (sequence_number >= 0 && sequence_number <= 10) {
    gt_present = true;
  }
  // srrg cache the files
  std::string calib_file(calib_path + sequence + "/calib.txt");
  std::string gray_sequence_path(gray_path + sequence + "/");
  std::string camera_calib_file(gray_sequence_path + "calib.txt");
  std::string timestamps_file(gray_sequence_path + "times.txt");
  //  std::string gray_left_path(gray_sequence_path + "image_0/");
  //  std::string gray_right_path(gray_sequence_path + "image_1/");
  std::string gt_file(poses_path + sequence + ".txt");
  std::string velodyne_sequence_path(velodyne_path + sequence + "/");

  if (!isAccessible(calib_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot find calib file [ " + calib_file + " ]");
  }

  if (!isAccessible(camera_calib_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot find camera calib file [ " +
                             camera_calib_file + " ]");
  }

  if (!isAccessible(timestamps_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot find timestamp file [ " + timestamps_file +
                             " ]");
  }

  //  if (!isAccessible(gray_left_path)) {
  //    throw std::runtime_error(exe_name + "|ERROR cannot find image left folder [ " +
  //    gray_left_path +
  //                             " ]");
  //  }
  //
  //  if (!isAccessible(gray_right_path)) {
  //    throw std::runtime_error(exe_name + "|ERROR cannot find image right folder [ " +
  //                             gray_right_path + " ]");
  //  }

  if (gt_present && !isAccessible(gt_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot find groundtruth file[ " + gt_file + " ]");
  }

  if (!isAccessible(velodyne_sequence_path)) {
    throw std::runtime_error(exe_name + "|ERROR cannot find image right folder [ " +
                             velodyne_sequence_path + " ]");
  }

  std::vector<double> timestamps;
  loadTimestamps(timestamps, timestamps_file);

  TimestampPoseMap gt_poses;
  if (gt_present) {
    loadGroundTruthPoses(gt_poses, gt_file, timestamps);
  }

  Matrix3f K_matrix_left(Matrix3f::Zero());
  Matrix3f K_matrix_right(Matrix3f::Zero());
  Isometry3f camera_right_in_left(Isometry3f::Identity());
  Isometry3f camera_left_in_lidar(Isometry3f::Identity());
  loadCalibrationParameters(
    K_matrix_left, K_matrix_right, camera_right_in_left, camera_left_in_lidar, calib_file);

  IntStringMap image_left_files;
  IntStringMap image_right_files;
  IntStringMap lidar_files;

  retrieveFilenames(image_left_files, gray_sequence_path, "image_0");
  retrieveFilenames(image_right_files, gray_sequence_path, "image_1");
  retrieveFilenames(lidar_files, velodyne_sequence_path, "velodyne");

  if (timestamps.size() != image_left_files.size()) {
    LOG << FG_RED("mismatching size: timestamps vs image_left_files"
                  << timestamps.size() << " != " << image_left_files.size() << " ]")
        << std::endl;
    throw std::runtime_error("");
  }
  if (timestamps.size() != image_right_files.size()) {
    LOG << FG_RED("mismatching size: timestamps vs image_right_files"
                  << timestamps.size() << " != " << image_right_files.size() << " ]")
        << std::endl;
    throw std::runtime_error("");
  }
  if (timestamps.size() != lidar_files.size()) {
    LOG << FG_RED("mismatching size: timestamps vs lidar_files"
                  << timestamps.size() << " != " << lidar_files.size() << " ]")
        << std::endl;
    throw std::runtime_error("");
  }

  LOG << FG_BBLUE("Starting conversion...") << std::endl;
  LOG << FG_BBLUE("messages: " << timestamps.size() + 1) << std::endl;
  std::cerr << std::setprecision(16);
  // ia start to create messages and write them to file
  LOG << "writing output to [ " << FG_YELLOW(argument_output_file.value()) << " ]\n";
  MessageFileSink sink;
  sink.open(argument_output_file.value());

  SystemUsageCounter::tic();
  const double& initial_timestamp = timestamps[0];

  size_t reading_idx        = 0;
  size_t index_gt           = 0;
  size_t index_camera_left  = 0;
  size_t index_camera_right = 0;
  size_t index_velodyne     = 0;

  auto it_image_left_files  = image_left_files.begin();
  auto it_image_right_files = image_right_files.begin();
  auto it_velodyne_files    = lidar_files.begin();

  const std::string distortion_model_name("undistorted-rectified");
  const TransformEventsMessagePtr tf_static_message =
    buildStaticTfTree(initial_timestamp, camera_right_in_left, camera_left_in_lidar);
  sink.putMessage(tf_static_message);
  ++reading_idx;

  for (size_t i = 0; i < timestamps.size(); ++i) {
    const double& timestamp(timestamps[i]);

    if (gt_present) {
      OdometryMessagePtr gt_imu_in_world(
        new OdometryMessage("/ground_truth", "kitti_link", index_gt, timestamp));
      gt_imu_in_world->pose.setValue(gt_poses[timestamp]);
      sink.putMessage(gt_imu_in_world);
      ++index_gt;
    }

    serializeCameraImageAndInfo(sink,
                                index_camera_left,
                                timestamp,
                                "camera_left",
                                it_image_left_files->second,
                                distortion_model_name,
                                K_matrix_left);
    ++it_image_left_files;
    ++index_camera_left;

    serializeCameraImageAndInfo(sink,
                                index_camera_right,
                                timestamp,
                                "camera_right",
                                it_image_right_files->second,
                                distortion_model_name,
                                K_matrix_right);
    ++it_image_right_files;
    ++index_camera_right;

    serializeVelodyne(sink,
                      index_velodyne,
                      timestamp,
                      "velodyne",
                      it_velodyne_files->second,
                      arg_normalize_intensity.isSet());
    ++it_velodyne_files;
    ++index_velodyne;

    const double progress_percentage =
      (double) reading_idx * 1.f / (double) (timestamps.size() + 1);
    printProgress(progress_percentage);

    ++reading_idx;
  }
  double t = SystemUsageCounter::toc();

  LOG << "converted [ " << reading_idx << " ] messages in [ " << t << " ] s -- FPS [ "
      << (float) reading_idx / t << " ] Hz\n";

  // ia done
  sink.close();
  return 0;
}

void loadTimestamps(std::vector<double>& timestamps_, const std::string& timestamps_file_) {
  VLOG(LOG << "loading timestamps from file [ " << FG_YELLOW(timestamps_file_) << " ]\n";)
  std::string gt_line;
  std::ifstream gt_stream(timestamps_file_, std::ifstream::in);
  const double init_time = srrg2_core::getTime();
  while (std::getline(gt_stream, gt_line)) {
    std::stringstream ss(gt_line);
    double timestamp = -1.0;
    ss >> timestamp;
    timestamps_.push_back(init_time + timestamp);
  }
  gt_stream.close();
  VLOG(LOG << "sequence " << sequence << " has " << timestamps_.size() << " frames\n";)
}

void loadGroundTruthPoses(TimestampPoseMap& gt_poses,
                          const std::string& ground_truth_file_,
                          const std::vector<double>& timestamps_) {
  VLOG(LOG << "loading gt from file [ " << FG_YELLOW(ground_truth_file_) << " ]\n";)
  std::string gt_line;
  std::ifstream gt_stream(ground_truth_file_, std::ifstream::in);
  size_t k = 0;
  while (std::getline(gt_stream, gt_line)) {
    std::stringstream ss(gt_line);
    Isometry3f gt_T = Isometry3f::Identity();
    for (size_t r = 0; r < 4; ++r) {
      for (size_t c = 0; c < 4; ++c) {
        ss >> gt_T.matrix()(r, c);
      }
    }
    gt_poses.insert(std::make_pair(timestamps_[k], gt_T));
    k++;
  }
  if (k != timestamps_.size()) {
    LOG << FG_RED("mismatch between gt_poses and timestamps (" << k << " != " << timestamps_.size()
                                                               << "\n");
    throw std::runtime_error("");
  }

  gt_stream.close();
}

void loadCalibrationParameters(Matrix3f& K_matrix_left_,
                               Matrix3f& K_matrix_right_,
                               Isometry3f& camera_right_in_left_,
                               Isometry3f& camera_left_in_lidar_,
                               const std::string& calibration_file_) {
  VLOG(LOG << "loading cameras calibration params from file [ " << FG_YELLOW(calibration_file_)
           << " ]\n";)

  std::string calib_line;
  std::ifstream calib_stream(calibration_file_, std::ifstream::in);
  while (std::getline(calib_stream, calib_line)) {
    std::stringstream ss(calib_line);
    std::string label;
    ss >> label;
    if (label == "P0:") {
      Matrix3_4f P0 = Matrix3_4f::Zero();
      for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 4; ++c) {
          ss >> P0.matrix()(r, c);
        }
      }

      K_matrix_left_ = P0.block<3, 3>(0, 0);
    } else if (label == "P1:") {
      Matrix3_4f P1 = Matrix3_4f::Zero();
      for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 4; ++c) {
          ss >> P1.matrix()(r, c);
        }
      }

      K_matrix_right_ = P1.block<3, 3>(0, 0);
      Vector3f t_px   = P1.block<3, 1>(0, 3);
      camera_right_in_left_.setIdentity();
      camera_right_in_left_.translation() = K_matrix_left_.inverse() * -t_px;
    } else if (label == "Tr:") {
      camera_left_in_lidar_.setIdentity();
      for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 4; ++c) {
          ss >> camera_left_in_lidar_.matrix()(r, c);
        }
      }
      camera_left_in_lidar_ = camera_left_in_lidar_.inverse();
    }
  }
  LOG << "calibration matrix left \n" << K_matrix_left_ << "\n";
  LOG << "calibration matrix right \n" << K_matrix_right_ << "\n";
  LOG << "camera right in camera left [t component in meters]\n"
      << camera_right_in_left_.matrix() << "\n";
  LOG << "camera left in camera lidar [t component in meters]\n"
      << camera_left_in_lidar_.matrix() << "\n";

  if (camera_right_in_left_.translation()[0] < 0) {
    throw std::runtime_error("baseline bx must be positive");
  }

  calib_stream.close();
}

const TransformEventsMessagePtr buildStaticTfTree(const double& initial_timestamp_,
                                                  const Isometry3f& camera_right_in_left_,
                                                  const Isometry3f& camera_left_in_lidar_) {
  const double& timestamp(initial_timestamp_);

  TransformEventsMessagePtr tf_static_message(
    new TransformEventsMessage("/tf_static", "/base_link", 0, timestamp));
  tf_static_message->events.resize(5);

  Isometry3f base_link(Isometry3f::Identity());
  base_link.translation()[2] = 0.3;
  TransformEvent event_base_link(timestamp, "base_link", base_link);
  tf_static_message->events.setValue(0, event_base_link);

  Isometry3f gps_in_base_link(Isometry3f::Identity());
  gps_in_base_link.translation() = Vector3f(-0.05, +0.32, 0.93 - 0.3);
  TransformEvent event_gps_in_base_link(timestamp, "gps", gps_in_base_link, "base_link");
  tf_static_message->events.setValue(1, event_gps_in_base_link);

  Isometry3f lidar_in_gps(Isometry3f::Identity());
  lidar_in_gps.translation() = Vector3f(0.81, -0.32, 1.73 - 0.93);
  Isometry3f lidar_in_base_link(gps_in_base_link * lidar_in_gps);
  TransformEvent event_lidar_in_base_link(timestamp, "velodyne", lidar_in_base_link, "base_link");
  tf_static_message->events.setValue(2, event_lidar_in_base_link);

  Isometry3f camera_left_in_lidar(camera_left_in_lidar_);

  Isometry3f dummy_identity(Isometry3f::Identity());
  if (camera_left_in_lidar.matrix() == dummy_identity.matrix()) {
    camera_left_in_lidar.translation() = Vector3f(+0.27, 0, 1.65 - 1.73);
    Matrix3f rot_camera_left_in_lidar;
    // clang-format off
    rot_camera_left_in_lidar <<  0,  0, 1,
                                -1,  0, 0,
                                 0, -1, 0;
    // clang-format on
    camera_left_in_lidar.linear() = rot_camera_left_in_lidar;
  }
  Isometry3f camera_left_in_base_link(lidar_in_base_link * camera_left_in_lidar);
  TransformEvent event_camera_left_in_base_link(
    timestamp, "camera_left", camera_left_in_base_link, "base_link");
  tf_static_message->events.setValue(3, event_camera_left_in_base_link);

  Isometry3f camera_right_in_left(camera_right_in_left_);
  if (camera_right_in_left.matrix() == dummy_identity.matrix()) {
    camera_right_in_left.translation() = Vector3f(+0.54, 0, 0);
  }
  TransformEvent event_camera_right_in_left(
    timestamp, "camera_right", camera_right_in_left, "camera_left");
  tf_static_message->events.setValue(4, event_camera_right_in_left);

  return tf_static_message;
}

void retrieveFilenames(IntStringMap& files_,
                       const std::string& sequence_path_,
                       const std::string& sensor_) {
  LOG << "loading filenames from directory [ " << FG_YELLOW(sequence_path_ + sensor_) << " ]\n";

  for (const auto& vel_entry : fs::directory_iterator(sequence_path_ + sensor_)) {
    const std::string filepath = vel_entry.path();
    const std::string filename = vel_entry.path().filename();
    const std::string file_no  = filename.substr(0, filename.find_last_of("."));
    files_.insert(std::make_pair(std::atoi(file_no.c_str()), filepath));
    VLOG(LOG << "reading file [ " << filepath << " ]\n";)
  }
}

void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 const size_t& index_,
                                 const double& timestamp_,
                                 const std::string& topic_name_,
                                 const std::string& file_name_,
                                 const std::string& distortion_model_name_,
                                 const Matrix3f& calibration_matrix_) {
  // ds create image message
  ImageMessagePtr image_message(
    new ImageMessage("/" + topic_name_ + "/image_raw", topic_name_, index_, timestamp_));

  cv::Mat image_opencv = cv::imread(file_name_, CV_LOAD_IMAGE_GRAYSCALE);
  if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
    throw std::runtime_error("serializeCameraImageAndInfo|ERROR: invalid camera image: " +
                             file_name_ + " with dimensions: " + std::to_string(image_opencv.rows) +
                             " x " + std::to_string(image_opencv.cols));
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
  camera_info_message->distortion_model.setValue("undistorted-rectified");
  camera_info_message->camera_matrix.setValue(calibration_matrix_);
  camera_info_message->cols.setValue(image_opencv.cols);
  camera_info_message->rows.setValue(image_opencv.rows);

  sink_.putMessage(image_message);
  sink_.putMessage(camera_info_message);
}

void serializeVelodyne(MessageFileSink& sink_,
                       const size_t& idx_,
                       const double& timestamp_,
                       const std::string& topic_name_,
                       const std::string& file_name_,
                       const bool normalize_intensities_) {
  PointCloud2MessagePtr lidar_msg(
    new PointCloud2Message("/" + topic_name_ + "/pointcloud", topic_name_, idx_, timestamp_));

  std::ifstream stream(file_name_, std::ios::binary);
  if (!stream.is_open()) {
    throw std::runtime_error(exe_name + "|ERROR, invalid filename [ " + file_name_ + " ]");
  }

  // ia read the bulky binary file
  stream.seekg(0, std::ios::end);
  const size_t num_points = stream.tellg() / (4 * sizeof(float));
  stream.seekg(0, std::ios::beg);
  std::vector<float> values(4 * num_points);
  stream.read((char*) &values[0], 4 * num_points * sizeof(float));
  stream.close();

  // ia now organize data
  PointIntensity3fVectorCloud cloud;
  cloud.reserve(num_points);

  float max_intensity = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    PointIntensity3f point;
    point.coordinates().x() = values[4 * i];
    point.coordinates().y() = values[4 * i + 1];
    point.coordinates().z() = values[4 * i + 2];
    point.intensity()       = values[4 * i + 3];
    max_intensity           = std::max(point.intensity(), max_intensity);
    cloud.emplace_back(point);
  }

  if (cloud.size() != num_points) {
    throw std::runtime_error(exe_name + "|ERROR, cloud size mismatch while reading [ " +
                             file_name_ + " ]");
  }

  // ia TODO do we need this normalization?
  if (normalize_intensities_) {
    const float inv_max_intensity = 1.f / max_intensity;
    for (auto& p : cloud) {
      p.intensity() *= inv_max_intensity;
    }
  }

  // ia now we have to create the message
  lidar_msg->setPointCloud(cloud);
  sink_.putMessage(lidar_msg);
}
