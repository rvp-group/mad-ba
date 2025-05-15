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

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& gps_poses_in_world_,
                                   std::vector<double>& timestamps_gt_);

void loadImageInformation(const std::string& associations_file_,
                          std::vector<std::string>& rgb_image_filenames_,
                          std::vector<double>& rgb_timestamps_seconds_,
                          std::vector<std::string>& depth_image_filenames_,
                          std::vector<double>& depth_timestamps_seconds_);

template <typename ImageType_>
void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const Matrix3f& camera_matrix_,
                                 const float& depth_factor_);

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

static const double frame_rate = 1. / 30.;
static const std::string postfix_name("_frei_png");
static const std::string gt_filename("groundtruth.txt");
static const std::string associations_filename("associations.txt");
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

  std::string dataset_folder(sequence + postfix_name + "/");
  std::string gt_file(dataset_folder + gt_filename);
  std::string associations_file(dataset_folder + associations_filename);

  if (!isAccessible(dataset_folder)) {
    throw std::runtime_error(exe_name + "|ERROR cannot dataset_folder [ " + dataset_folder + " ]");
  }

  if (!isAccessible(gt_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot gt file [ " + gt_file + " ]");
  }

  if (!isAccessible(associations_file)) {
    throw std::runtime_error(exe_name + "|ERROR cannot associations file [ " + associations_file +
                             " ]");
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
  loadImageInformation(associations_file,
                       image_filenames_rgb,
                       timestamps_rgb,
                       image_filenames_depth,
                       timestamps_depth);

  // ds TUM camera matrix and depth factor TODO load from disk
  Matrix3f camera_calibration_matrix;
  camera_calibration_matrix << 481.2, 0, 319.5, 0, -480, 239.5, 0, 0, 1;
  std::cerr << "current camera matrix: " << std::endl;
  std::cerr << camera_calibration_matrix << std::endl;
  const float factor_to_srrg_depth = 1.0 / 5;

  const size_t tot_messages =
    timestamps_gt.size() + timestamps_rgb.size() + timestamps_depth.size();

  const double init_time = srrg2_core::getTime();

  size_t index_gt    = 0;
  size_t index_rgb   = 0;
  size_t index_depth = 0;

  //  std::cerr << "depth factor: [TODO automate this]: " << factor_to_srrg_depth << std::endl;
  //  std::cerr << "press [ENTER] to start conversion" << std::endl;
  //  getchar();

  // ds determine initial, smallest timestamp
  double timestamp_oldest = std::numeric_limits<double>::max();

  if (timestamps_gt.size()) {
    timestamp_oldest = std::min(timestamp_oldest, timestamps_gt[0]);
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
  std::cerr << "initial timestamp (s): " << timestamp_oldest * frame_rate
            << " tot_msgs: " << tot_messages << std::endl;

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
    if (timestamps_rgb.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_rgb[0]);
    }
    if (timestamps_depth.size()) {
      first_timestamp = std::min(first_timestamp, timestamps_depth[0]);
    }
    if (first_timestamp == std::numeric_limits<double>::max()) {
      break;
    }

    const double converted_timestamp = first_timestamp + init_time;

    if (timestamps_gt.size() && timestamps_gt[0] == first_timestamp) {
      OdometryMessagePtr gt_in_world(
        new OdometryMessage("/ground_truth", "gps", index_gt, converted_timestamp));
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
                                              converted_timestamp,
                                              "rgb",
                                              dataset_folder + image_filenames_rgb.front(),
                                              camera_calibration_matrix,
                                              1.);
      image_filenames_rgb.erase(image_filenames_rgb.begin());
      timestamps_rgb.erase(timestamps_rgb.begin());
      ++index_rgb;
      ++processed_msgs;
    }

    if (timestamps_depth.size() && timestamps_depth.front() == first_timestamp) {
      serializeCameraImageAndInfo<ImageUInt16>(sink,
                                               index_depth,
                                               converted_timestamp,
                                               "depth",
                                               dataset_folder + image_filenames_depth.front(),
                                               camera_calibration_matrix,
                                               factor_to_srrg_depth);
      image_filenames_depth.erase(image_filenames_depth.begin());
      timestamps_depth.erase(timestamps_depth.begin());
      ++index_depth;
      ++processed_msgs;
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
  //
  //  // ds parse associations by tokens
  //  while (input_file >> timestamp_depth >> path_image_depth >> timestamp_rgb >> path_image_rgb) {
  //    // ds image messages
  //    ImageMessagePtr image_message_depth(new ImageMessage(
  //      "/camera/depth/image", "/openni_rgb_optical_frame", timestamp_depth, timestamp_depth));
  //    ImageMessagePtr image_message_rgb(new ImageMessage(
  //      "/camera/rgb/image_color", "/openni_rgb_optical_frame", timestamp_rgb, timestamp_rgb));
  //
  //    // ds load Depth image from disk and convert
  //    cv::Mat image_opencv_depth = cv::imread(path_image_depth, CV_LOAD_IMAGE_ANYDEPTH);
  //    if (image_opencv_depth.rows <= 0 || image_opencv_depth.cols <= 0) {
  //      std::cerr << "\nWARNING: skipping invalid image: " << path_image_depth
  //                << " with dimensions: " << image_opencv_depth.rows << " x "
  //                << image_opencv_depth.cols << std::endl;
  //      continue;
  //    }
  //    cv::Mat image_opencv_depth_scaled(image_opencv_depth);
  //    image_opencv_depth_scaled *= factor_to_srrg_depth;
  //    ImageUInt16* image_depth(new ImageUInt16());
  //    image_depth->fromCv(image_opencv_depth_scaled);
  //    image_message_depth->setImage(image_depth); // ImageFloat takes up double the space
  //    image_message_depth->image_cols.setValue(image_opencv_depth.cols);
  //    image_message_depth->image_rows.setValue(image_opencv_depth.rows);
  //
  //    // ds load RGB image from disk and convert
  //    const cv::Mat image_opencv_rgb = cv::imread(path_image_rgb, CV_LOAD_IMAGE_GRAYSCALE);
  //    if (image_opencv_rgb.rows <= 0 || image_opencv_rgb.cols <= 0) {
  //      std::cerr << "\nWARNING: skipping invalid image: " << path_image_rgb
  //                << " with dimensions: " << image_opencv_rgb.rows << " x " <<
  //                image_opencv_rgb.cols
  //                << std::endl;
  //      continue;
  //    }
  //    ImageUInt8* base_image(new ImageUInt8());
  //    base_image->fromCv(image_opencv_rgb);
  //    image_message_rgb->setImage(base_image);
  //    image_message_rgb->image_cols.setValue(image_opencv_rgb.cols);
  //    image_message_rgb->image_rows.setValue(image_opencv_rgb.rows);
  //
  //    // ds display converted images
  //    image_opencv_depth *= 5; // ds visibility
  //    cv::imshow("processed image Depth", image_opencv_depth);
  //    cv::imshow("processed image RGB", image_opencv_rgb);
  //    cv::waitKey(1);
  //
  //    // ds camera calibration messages
  //    CameraInfoMessagePtr camera_info_message_depth(
  //      new CameraInfoMessage(image_message_depth->topic.value() + "/info",
  //                            image_message_depth->frame_id.value(),
  //                            image_message_depth->seq.value(),
  //                            image_message_depth->timestamp.value()));
  //    camera_info_message_depth->depth_scale.setValue(1e-3f);
  //    camera_info_message_depth->projection_model.setValue("pinhole");
  //    camera_info_message_depth->distortion_model.setValue("undistorted");
  //    camera_info_message_depth->camera_matrix.setValue(camera_calibration_matrix);
  //    CameraInfoMessagePtr camera_info_message_rgb(
  //      new CameraInfoMessage(image_message_rgb->topic.value() + "/info",
  //                            image_message_rgb->frame_id.value(),
  //                            image_message_rgb->seq.value(),
  //                            image_message_rgb->timestamp.value()));
  //    camera_info_message_rgb->depth_scale.setValue(1e-3f);
  //    camera_info_message_rgb->projection_model.setValue("pinhole");
  //    camera_info_message_rgb->distortion_model.setValue("undistorted");
  //    camera_info_message_rgb->camera_matrix.setValue(camera_calibration_matrix);
  //
  //    // ds write messages
  //    serializer.writeObject(*image_message_depth);
  //    serializer.writeObject(*image_message_rgb);
  //    serializer.writeObject(*camera_info_message_depth);
  //    serializer.writeObject(*camera_info_message_rgb);
  //    std::cerr << "ID";
  //
  //    // ds look for ground truth data (in RGB camera frame)
  //    auto iterator = camera_poses_in_world.find(timestamp_rgb);
  //    if (iterator != camera_poses_in_world.end()) {
  //      TransformEventsMessagePtr transform_message(
  //        new TransformEventsMessage("/tf",
  //                                   image_message_rgb->frame_id.value(),
  //                                   image_message_rgb->seq.value(),
  //                                   image_message_rgb->timestamp.value()));
  //      transform_message->events.resize(1);
  //      TransformEvent event_gt(
  //        image_message_rgb->timestamp.value(), "camera_rgb", iterator->second, "world");
  //      transform_message->events.setValue(0, event_gt);
  //      serializer.writeObject(*transform_message);
  //      std::cerr << "G";
  //    } else {
  //      std::cerr << "_";
  //    }
  //    ++number_of_converted_messages;
  //  }
  //  std::cerr << std::endl;
  //  input_file.close();
  //
  //  std::cerr << "# converted RGBD messages: " << number_of_converted_messages << std::endl;
  //  return 0;
}

void loadGroundTruthPosesAndStamps(const std::string& ground_truth_file_,
                                   StdVectorEigenIsometry3f& poses_in_world_,
                                   std::vector<double>& timestamps_gt_) {
  std::ifstream ground_truth_file(ground_truth_file_);
  if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
    throw std::runtime_error("ERROR: unable to open ground truth file");
  }

  double timestamp_seconds;
  double t_x, t_y, t_z;
  double q_x, q_y, q_z, q_w;
  while (ground_truth_file >> timestamp_seconds >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w) {
    if (timestamp_seconds == 0) {
      throw std::runtime_error("ERROR: GT timestamps are expected to start at 1 for ICL");
    }
    Isometry3f camera_in_world;
    camera_in_world.translation() = Vector3f(t_x, t_y, t_z);
    camera_in_world.linear()      = Quaternionf(q_w, q_x, q_y, q_z).toRotationMatrix();
    poses_in_world_.push_back(camera_in_world);
    timestamps_gt_.push_back(timestamp_seconds * frame_rate);
  }

  ground_truth_file.close();
}

void loadImageInformation(const std::string& associations_file_,
                          std::vector<std::string>& image_filenames_rgb_,
                          std::vector<double>& timestamps_rgb_,
                          std::vector<std::string>& image_filenames_depth_,
                          std::vector<double>& timestamps_depth_) {
  // ds open file
  std::ifstream input_file(associations_file_);
  if (!input_file.good() || !input_file.is_open()) {
    throw std::runtime_error("ERROR: unable to open input file [" + associations_file_ + "]");
  }

  image_filenames_rgb_.clear();
  timestamps_rgb_.clear();
  image_filenames_depth_.clear();
  timestamps_depth_.clear();

  size_t timestamp_rgb         = 0;
  std::string path_image_rgb   = "";
  size_t timestamp_depth       = 0;
  std::string path_image_depth = "";

  while (input_file >> timestamp_depth >> path_image_depth >> timestamp_rgb >> path_image_rgb) {
    image_filenames_rgb_.push_back(path_image_rgb);
    timestamps_rgb_.push_back(timestamp_rgb * frame_rate);
    image_filenames_depth_.push_back(path_image_depth);
    timestamps_depth_.push_back(timestamp_depth * frame_rate);
  }
  std::cerr << "loadImageInformation|# loaded entries: " << timestamps_rgb_.size() << " for file '"
            << associations_file_ << "'" << std::endl;

  input_file.close();
}

template <typename ImageType_>
void serializeCameraImageAndInfo(MessageFileSink& sink_,
                                 size_t& index_,
                                 const double& timestamp_,
                                 const std::string& label_,
                                 const std::string& file_path_image_,
                                 const Matrix3f& camera_matrix_,
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
  camera_info_message->camera_matrix.setValue(camera_matrix_);
  camera_info_message->cols.setValue(image_opencv.cols);
  camera_info_message->rows.setValue(image_opencv.rows);

  // ds write messages related to this image
  sink_.putMessage(image_message);
  sink_.putMessage(camera_info_message);
}
