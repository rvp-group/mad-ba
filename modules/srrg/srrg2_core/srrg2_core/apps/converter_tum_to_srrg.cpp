#include <dirent.h>
#include <experimental/filesystem>
#include <iostream>

#include <srrg_boss/serializer.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
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
namespace fs = std::experimental::filesystem;

bool verbose = false;
const std::string exe_name("converter_kitti_to_srrg");
#define LOG std::cerr << exe_name + "|"
#define VLOG(X)  \
  if (verbose) { \
    X            \
  }

// clang-format off
const char* banner[] = {"This program converts TUM formatted RGB and depth files to SRRG format (json/bag)", 0};
// clang-format on

using TimestampPoseMap = std::map<double,
                                  Isometry3f,
                                  std::less<double>,
                                  Eigen::aligned_allocator<std::pair<const double, Isometry3f>>>;

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

void loadImageInformation(const std::string& infile_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_);

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        std::vector<double>& timestamps_seconds_);

template <typename ImageType_>
void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const cv::Mat* undistort_rectify_maps_,
                                 const float& depth_factor_);

void initializeUndistortion(const Matrix3f& camera_calibration_matrix_,
                            const Vector5d& distortion_coefficients_,
                            const cv::Size& opencv_image_size_,
                            cv::Mat* undistort_rectify_maps_);

static const std::string prefix_name("rgbd_dataset_");
static const std::string imu_filename("accelerometer.txt");
static const std::string depth_filename("depth.txt");
static const std::string gt_filename("groundtruth.txt");
static const std::string rgb_filename("rgb.txt");

static std::string sequence("");

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_sequence_name(
    &command_line_parser, "s", "sequence", "sequence name from TUM dataset", "");
  ArgumentString argument_output_file(
    &command_line_parser, "o", "output", "converted output file", "messages.json");
  ArgumentFlag arg_verbose(&command_line_parser, "v", "verbose", "logs something", true);
  command_line_parser.parse();

  if (argument_sequence_name.value().empty()) {
    std::cerr << "ERROR: input sequence is empty (set with -s)" << std::endl;
    return 0;
  }
  if (argument_output_file.value().empty()) {
    std::cerr << "ERROR: output filename is empty (set with -o)" << std::endl;
    return 0;
  }
  verbose = arg_verbose.isSet();

  sequence = argument_sequence_name.value();

  std::string dataset_name(prefix_name + sequence);
  std::string dataset_folder(dataset_name + "/");
  std::string gt_file(dataset_folder + gt_filename);
  std::string imu_file(dataset_folder + imu_filename);
  std::string rgb_file(dataset_folder + rgb_filename);
  std::string depth_file(dataset_folder + depth_filename);

  if (!isAccessible(dataset_folder)) {
    throw std::runtime_error(exe_name + "|ERROR cannot dataset_folder [ " + dataset_folder + " ]");
  }

  if (!isAccessible(gt_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot gt_file file [ " + gt_file + " ]");
  }
  void serializeCameraDepthImageAndInfo(MessageFileSink & sink_,
                                        size_t & index_,
                                        const double& timestamp_,
                                        const std::string& label_,
                                        const std::string& file_path_image_,
                                        const std::string& distortion_model_name_,
                                        const Vector5d& distortion_coefficients_,
                                        const Matrix3f& camera_matrix_,
                                        const cv::Mat* undistort_rectify_maps_,
                                        const float& depth_factor);

  if (!isAccessible(imu_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot imu_file file [ " + imu_file + " ]");
  }

  if (!isAccessible(rgb_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot rgb_file file [ " + rgb_file + " ]");
  }

  if (!isAccessible(depth_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot depth_file file [ " + depth_file + " ]");
  }

  // ds buffer ground truth data
  StdVectorEigenIsometry3f camera_poses_in_world;
  std::vector<double> timestamps_gt;
  loadGroundTruthPosesAndStamps(gt_file, camera_poses_in_world, timestamps_gt);
  if (timestamps_gt.empty()) {
    throw std::runtime_error("ERROR: no ground truth poses loaded");
  }
  std::cerr << "loaded ground truth poses: " << timestamps_gt.size() << std::endl;

  // ds read depth and RGB images and timestamps
  std::vector<std::string> image_filenames_rgb;
  std::vector<double> timestamps_rgb;
  std::vector<std::string> image_filenames_depth;
  std::vector<double> timestamps_depth;

  // ds parse image information
  loadImageInformation(rgb_file, image_filenames_rgb, timestamps_rgb);
  if (timestamps_rgb.empty()) {
    throw std::runtime_error("ERROR: no RGB image information retrieved");
  }
  loadImageInformation(depth_file, image_filenames_depth, timestamps_depth);
  if (timestamps_depth.empty()) {
    throw std::runtime_error("ERROR: no Depth image information retrieved");
  }

  // ds TUM calibration TODO load from disk instead of hardcoded switch
  Matrix3f camera_calibration_matrix;
  Vector_<double, 5> distortion_coefficients;

  if (sequence.find("freiburg1") != std::string::npos) {
    camera_calibration_matrix << 517.3, 0, 318.6, 0, 516.5, 255.3, 0, 0, 1;
    distortion_coefficients << 0.2624, -0.9531, -0.0054, 0.0026, 1.1633;
  } else if (sequence.find("freiburg2") != std::string::npos) {
    camera_calibration_matrix << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    distortion_coefficients << 0.2312, -0.7849, -0.0033, -0.0001, 0.9172;
  } else if (sequence.find("freiburg3") != std::string::npos) {
    camera_calibration_matrix << 535.4, 0, 320.1, 0, 539.2, 247.6, 0, 0, 1;
    distortion_coefficients.setZero();
  } else {
    throw std::runtime_error("unrecognied camera");
  }

  std::cerr << "camera matrix: " << std::endl;
  std::cerr << camera_calibration_matrix << std::endl;
  std::cerr << "distortion coefficients: " << std::endl;
  std::cerr << distortion_coefficients.transpose() << std::endl;

  const cv::Size opencv_image_size(640, 480);
  cv::Mat undistort_rectify_maps[2];
  Matrix3f camera_calibration_matrix_rectified(Matrix3f::Zero());

  //  initializeUndistortion(
  //    camera_calibration_matrix, distortion_coefficients, opencv_image_size,
  //    undistort_rectify_maps);

  //  std::cerr << undistort_rectify_maps[0] << std::endl;
  //  std::cerr << undistort_rectify_maps[1] << std::endl;
  // ds common parameters for all TUM sequences TODO verify
  const std::string distortion_model_name = "radial_tangential";
  const float factor_to_srrg_depth        = 1.0f / 5.0f;

  std::vector<double> timestamps_imu;
  StdVectorEigenVector3d linear_accelerations;

  loadIMUInformation(imu_file, linear_accelerations, timestamps_imu);

  std::cerr << "depth factor: " << factor_to_srrg_depth << std::endl;
  //  std::cerr << "press [ENTER] to start conversion" << std::endl;
  //  getchar();

  const size_t tot_messages =
    timestamps_gt.size() + timestamps_imu.size() + timestamps_rgb.size() + timestamps_depth.size();

  size_t index_gt    = 0;
  size_t index_rgb   = 0;
  size_t index_depth = 0;
  size_t index_imu   = 0;

  // ds determine initial, smallest timestamp
  double timestamp_oldest = std::numeric_limits<double>::max();

  if (timestamps_gt.size()) {
    timestamp_oldest = std::min(timestamp_oldest, timestamps_gt[0]);
  }
  if (timestamps_imu.size()) {
    timestamp_oldest = std::min(timestamp_oldest, timestamps_imu[0]);
  }
  if (timestamps_rgb.size()) {
    timestamp_oldest = std::min(timestamp_oldest, timestamps_rgb[0]);
  }
  if (timestamps_depth.size()) {
    timestamp_oldest = std::min(timestamp_oldest, timestamps_depth[0]);
  }
  if (timestamp_oldest == std::numeric_limits<double>::max()) {
    throw std::runtime_error("gesu' cristo");
  }

  std::cerr << std::setprecision(16);
  std::cerr << "initial timestamp (s): " << timestamp_oldest << " tot_msgs: " << tot_messages
            << std::endl;
  LOG << "writing output to [ " << FG_YELLOW(argument_output_file.value()) << " ]\n";
  MessageFileSink sink;
  sink.open(argument_output_file.value());
  SystemUsageCounter::tic();

  // ds playback all buffers
  size_t processed_msgs = 0;
  while (true) {
    // ds if we still have ground truth information available
    double first_timestamp = std::numeric_limits<double>::max();
    if (timestamps_gt.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_gt[0]);
    }
    if (timestamps_imu.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_imu[0]);
    }
    if (timestamps_rgb.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_rgb[0]);
    }
    if (timestamps_depth.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_depth[0]);
    }
    if (first_timestamp == std::numeric_limits<double>::max()) {
      break;
    }

    if (timestamps_gt.size() && timestamps_gt[0] == first_timestamp) {
      OdometryMessagePtr gt_in_world(
        new OdometryMessage("/ground_truth", "gps", index_gt, first_timestamp));
      gt_in_world->pose.setValue(camera_poses_in_world.front());
      sink.putMessage(gt_in_world);
      camera_poses_in_world.erase(camera_poses_in_world.begin());
      timestamps_gt.erase(timestamps_gt.begin());
      ++index_gt;
      ++processed_msgs;
    }

    // ds if we have RGB data available
    if (timestamps_rgb.size() && timestamps_rgb.front() == first_timestamp) {
      serializeCameraImageAndInfo<ImageUInt8>(sink,
                                              index_rgb,
                                              first_timestamp,
                                              "rgb",
                                              dataset_folder + image_filenames_rgb.front(),
                                              distortion_model_name,
                                              distortion_coefficients,
                                              camera_calibration_matrix,
                                              undistort_rectify_maps,
                                              1.);
      image_filenames_rgb.erase(image_filenames_rgb.begin());
      timestamps_rgb.erase(timestamps_rgb.begin());
      ++index_rgb;
      ++processed_msgs;
    }

    if (timestamps_depth.size() && timestamps_depth.front() == first_timestamp) {
      serializeCameraImageAndInfo<ImageUInt16>(sink,
                                               index_depth,
                                               first_timestamp,
                                               "depth",
                                               dataset_folder + image_filenames_depth.front(),
                                               distortion_model_name,
                                               distortion_coefficients,
                                               camera_calibration_matrix,
                                               undistort_rectify_maps,
                                               factor_to_srrg_depth);
      image_filenames_depth.erase(image_filenames_depth.begin());
      timestamps_depth.erase(timestamps_depth.begin());
      ++index_depth;
      ++processed_msgs;
    }

    if (timestamps_imu.size() && timestamps_imu.front() == first_timestamp) {
      IMUMessagePtr message(new IMUMessage("/imu", "imu", index_imu, first_timestamp));
      message->angular_velocity.setValue(Vector3f::Zero());
      //        message->angular_velocity_covariance.setValue(Matrix3f::Identity());
      message->linear_acceleration.setValue(linear_accelerations.front().cast<float>());
      //        message->linear_acceleration_covariance.setValue(Matrix3f::Identity());
      message->orientation.setValue(Matrix3f::Zero());
      //        message->orientation_covariance.setValue(Matrix3f::Identity());
      message->rate_hz.setValue(-1);

      linear_accelerations.erase(linear_accelerations.begin());
      timestamps_imu.erase(timestamps_imu.begin());
      ++index_imu;
      ++processed_msgs;
      sink.putMessage(message);
    }

    const double progress_percentage = (double) processed_msgs * 1.f / (double) (tot_messages);
    printProgress(progress_percentage);
  }
  double t = SystemUsageCounter::toc();

  LOG << "converted [ " << processed_msgs << " ] messages in [ " << t << " ] s -- FPS [ "
      << (float) processed_msgs / t << " ] Hz\n";

  sink.close();

  std::cerr << std::endl;
  std::cerr << "# converted RGB messages: " << index_rgb << std::endl;
  std::cerr << "# converted Depth messages: " << index_depth << std::endl;
  std::cerr << "# converted GT messages: " << index_gt << std::endl;
  return 0;
}

void loadImageInformation(const std::string& infile_name_,
                          std::vector<std::string>& image_filenames_,
                          std::vector<double>& timestamps_seconds_) {
  std::ifstream infile(infile_name_);
  if (!infile.good() || !infile.is_open()) {
    throw std::runtime_error("loadImageInformation|ERROR: unable to open file: " + infile_name_);
  }
  image_filenames_.clear();
  timestamps_seconds_.clear();

  // ds skip the first 3 lines (TUM format header information)
  std::cerr << "loadImageInformation|<header>" << std::endl;
  std::string buffer;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::getline(infile, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadImageInformation|</header>" << std::endl;

  // ds parse image information by tokens
  double timestamp_seconds;
  std::string filename;
  while (infile >> timestamp_seconds >> filename) {
    image_filenames_.push_back(filename);
    timestamps_seconds_.push_back(timestamp_seconds);
  }
  std::cerr << "loadImageInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << infile_name_ << "'" << std::endl;
  infile.close();
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_gt_) {
  std::ifstream ground_truth_file(ground_truth_file_);
  if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
    throw std::runtime_error("ERROR: unable to open ground truth file");
  }

  // ds skip the first 3 lines (TUM format header information)
  std::cerr << "<header>" << std::endl;
  std::string buffer;
  std::getline(ground_truth_file, buffer);
  std::cerr << buffer << std::endl;
  std::getline(ground_truth_file, buffer);
  std::cerr << buffer << std::endl;
  std::getline(ground_truth_file, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "</header>" << std::endl;

  double timestamp_seconds;
  double t_x, t_y, t_z;
  double q_x, q_y, q_z, q_w;
  while (ground_truth_file >> timestamp_seconds >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w) {
    if (timestamp_seconds == 0) {
      throw std::runtime_error("ERROR: GT timestamps are expected to start at > 0");
    }
    Isometry3f camera_in_world;
    camera_in_world.translation() = Vector3f(t_x, t_y, t_z);
    camera_in_world.linear()      = Quaternionf(q_w, q_x, q_y, q_z).toRotationMatrix();
    gps_poses_in_world_.push_back(camera_in_world);
    timestamps_gt_.push_back(timestamp_seconds);
  }
  ground_truth_file.close();
}

void loadIMUInformation(const std::string& file_imu_data_,
                        StdVectorEigenVector3d& linear_accelerations_,
                        std::vector<double>& timestamps_seconds_) {
  linear_accelerations_.clear();
  std::cerr << "loadIMUInformation|file: '" << file_imu_data_ << "'" << std::endl;
  std::ifstream stream(file_imu_data_);
  if (!stream.good() || !stream.is_open()) {
    throw std::runtime_error("loadIMUInformation|ERROR: unable to open file: " + file_imu_data_);
  }

  // ds skip first line (header)
  std::string buffer;
  std::cerr << "loadIMUInformation|<header>" << std::endl;
  std::getline(stream, buffer);
  std::cerr << buffer << std::endl;
  std::getline(stream, buffer);
  std::cerr << buffer << std::endl;
  std::getline(stream, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "loadIMUInformation|<header>" << std::endl;

  // ds parse by tokens
  linear_accelerations_.reserve(100000);
  double timestamp_seconds = 0;
  double IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC;
  while (stream >> timestamp_seconds >> IMU_X_ACC >> IMU_Y_ACC >> IMU_Z_ACC) {
    linear_accelerations_.emplace_back(Vector3d(IMU_X_ACC, IMU_Y_ACC, IMU_Z_ACC));
    timestamps_seconds_.emplace_back(timestamp_seconds);
  }
  std::cerr << "loadIMUInformation|# loaded entries: " << timestamps_seconds_.size()
            << " for file '" << file_imu_data_ << "'" << std::endl;
  stream.close();
}

template <typename ImageType_>
void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const std::string& distortion_model_name_,
                                 const Vector5d& distortion_coefficients_,
                                 const Matrix3f& camera_matrix_,
                                 const cv::Mat* undistort_rectify_maps_,
                                 const float& depth_factor_) {
  // ds create image message
  ImageMessagePtr image_message(
    new ImageMessage("/camera/" + label_ + "/image_raw", label_, index_, timestamp_));

  // ds load RGB image from disk and convert
  cv::Mat image_opencv;
  if (depth_factor_ != 1.) {
    image_opencv = cv::imread(file_path_image_, CV_LOAD_IMAGE_ANYDEPTH);
    image_opencv *= depth_factor_;
  } else {
    image_opencv = cv::imread(file_path_image_, CV_LOAD_IMAGE_GRAYSCALE);
  }

  if (image_opencv.rows <= 0 || image_opencv.cols <= 0) {
    throw std::runtime_error(
      "serializeCameraImageAndInfo|ERROR: invalid camera image: " + file_path_image_ +
      " with dimensions: " + std::to_string(image_opencv.rows) + " x " +
      std::to_string(image_opencv.cols));
  }

  // ds map to srrg
  ImageType_* base_image(new ImageType_());
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
  if (depth_factor_ != 1.) {
    camera_info_message->depth_scale.setValue(1e-3f);
  }
  camera_info_message->projection_model.setValue("pinhole");
  camera_info_message->distortion_model.setValue(distortion_model_name_);
  camera_info_message->camera_matrix.setValue(camera_matrix_);
  camera_info_message->distortion_coefficients.setValue(distortion_coefficients_);
  camera_info_message->cols.setValue(image_opencv.cols);
  camera_info_message->rows.setValue(image_opencv.rows);

  // ds write messages related to this image
  sink_.putMessage(image_message);
  sink_.putMessage(camera_info_message);
}

void initializeUndistortion(const Matrix3f& camera_calibration_matrix_,
                            const Vector5d& distortion_coefficients_,
                            const cv::Size& opencv_image_size_,
                            cv::Mat* undistort_rectify_maps_) {
  // ds rectification
  cv::Mat opencv_camera_calibration_matrix(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat opencv_distortion_coefficients(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat opencv_projection_matrix(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat opencv_rectification_matrix(cv::Mat::eye(3, 3, CV_64F));

  // ds buffer matrices to opencv
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      opencv_camera_calibration_matrix.at<double>(r, c) = camera_calibration_matrix_(r, c);
    }
  }
  for (size_t i = 0; i < 4; ++i) {
    opencv_distortion_coefficients.at<double>(i) = distortion_coefficients_(i);
  }

  // ds check if failed
  if (cv::norm(opencv_projection_matrix) == 0) {
    throw std::runtime_error("ERROR: rectification parameter retrieval failed");
  }

  // ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(opencv_camera_calibration_matrix,
                              opencv_distortion_coefficients,
                              opencv_rectification_matrix,
                              opencv_projection_matrix,
                              opencv_image_size_,
                              CV_16SC2,
                              undistort_rectify_maps_[0],
                              undistort_rectify_maps_[1]);

  // ds check if rectification failed
  if (cv::norm(undistort_rectify_maps_[0]) == 0 || cv::norm(undistort_rectify_maps_[1]) == 0) {
    throw std::runtime_error("ERROR: unable to undistort and rectify camera left");
  }
}
