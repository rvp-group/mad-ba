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
const char* banner[] = {"This program computes the BF feature matches between two images of proportional size", 0};
// clang-format on

int main(int argc_, char** argv_) {
  messages_registerTypes();

  // ds set up CLI parameters
  // clang-format off
  ParseCommandLine command_line_parser(argv_, banner);
  ArgumentString argument_query_image_file (
    &command_line_parser, "q",  "query", "query image", "");
  ArgumentString argument_reference_image_file (
    &command_line_parser, "r",  "reference", "reference image", "out.json");
  constexpr float maximum_descriptor_distance = 50;

  // clang-format on
  command_line_parser.parse();
  if (!argument_query_image_file.isSet() || argument_query_image_file.value().empty()) {
    std::cerr << "ERROR: query image file not set" << std::endl;
    return -1;
  }
  if (!argument_reference_image_file.isSet() || argument_reference_image_file.value().empty()) {
    std::cerr << "ERROR: reference image file not set" << std::endl;
    return -1;
  }

  // ds load images from disk in grayscale
  cv::Mat display_query_image(
    cv::imread(argument_query_image_file.value(), CV_LOAD_IMAGE_ANYCOLOR));
  cv::Mat display_reference_image(
    cv::imread(argument_reference_image_file.value(), CV_LOAD_IMAGE_ANYCOLOR));
  cv::Mat query_image;
  cv::Mat reference_image;
  cv::cvtColor(display_query_image, query_image, cv::COLOR_BGR2GRAY);
  cv::cvtColor(display_reference_image, reference_image, cv::COLOR_BGR2GRAY);

  // ds verify size
  if (query_image.rows != reference_image.rows || query_image.cols != reference_image.cols) {
    // ds check if we can scale the images to be the same size
    const double scaling_factor = static_cast<double>(query_image.rows) / reference_image.rows;
    if (scaling_factor == static_cast<double>(query_image.cols) / reference_image.cols) {
      assert(scaling_factor != 1);
      if (scaling_factor < 1) {
        std::cerr << "REFERENCE image is scaled down by: " << scaling_factor << std::endl;
        cv::resize(reference_image, reference_image, cv::Size(query_image.cols, query_image.rows));
        cv::resize(display_reference_image,
                   display_reference_image,
                   cv::Size(query_image.cols, query_image.rows));
      } else {
        std::cerr << "QUERY image is scaled down by: " << scaling_factor << std::endl;
        cv::resize(query_image, query_image, cv::Size(reference_image.cols, reference_image.rows));
        cv::resize(
          display_query_image, display_query_image, cv::Size(query_image.cols, query_image.rows));
      }
    } else {
      std::cerr << "ERROR: images have unequal sizes ";
      std::cerr << "QUERY < " << query_image.rows << " x " << query_image.cols << ", ";
      std::cerr << "REFERENCE < " << reference_image.rows << " x " << reference_image.cols
                << std::endl;
      return -1;
    }
  }

  // ds detect keypoints for both images
  std::vector<cv::KeyPoint> query_keypoints;
  std::vector<cv::KeyPoint> reference_keypoints;
  cv::Ptr<cv::FeatureDetector> detector(cv::ORB::create(500));
  detector->detect(query_image, query_keypoints);
  detector->detect(reference_image, reference_keypoints);

  // ds compute descriptors for keypoints
  cv::Ptr<cv::DescriptorExtractor> extractor(cv::ORB::create(500));
  cv::Mat query_descriptors;
  cv::Mat reference_descriptors;
  extractor->compute(query_image, query_keypoints, query_descriptors);
  extractor->compute(reference_image, reference_keypoints, reference_descriptors);

  // ds draw detected keypoints
  for (const cv::KeyPoint& keypoint : query_keypoints) {
    cv::circle(display_query_image, keypoint.pt, 2, cv::Scalar(0, 255, 0), -1);
  }
  for (const cv::KeyPoint& keypoint : reference_keypoints) {
    cv::circle(display_reference_image, keypoint.pt, 2, cv::Scalar(255, 0, 0), -1);
  }

  // ds compute matches
  cv::BFMatcher matcher(cv::NORM_HAMMING, true /*crosscheck*/);
  std::vector<cv::DMatch> matches;
  matcher.match(query_descriptors, reference_descriptors, matches);

  // ds filter matches by hamming distance
  size_t number_of_verified_matches = 0;
  for (const cv::DMatch& match : matches) {
    if (match.distance < maximum_descriptor_distance) {
      ++number_of_verified_matches;
      cv::circle(
        display_query_image, query_keypoints[match.queryIdx].pt, 2, cv::Scalar(0, 255, 0), -1);
      cv::circle(display_reference_image,
                 reference_keypoints[match.trainIdx].pt,
                 2,
                 cv::Scalar(0, 255, 0),
                 -1);
    }
  }
  std::cerr << "number of matches: " << number_of_verified_matches << std::endl;
  std::cerr << "matching ratio: "
            << static_cast<double>(number_of_verified_matches) / query_keypoints.size()
            << std::endl;
  std::cerr << "query keypoints: " << query_keypoints.size() << std::endl;
  std::cerr << "reference keypoints: " << reference_keypoints.size() << std::endl;

  // ds show images
  cv::imshow("query", display_query_image);
  cv::imshow("reference", display_reference_image);
  cv::imwrite("query.png", display_query_image);
  cv::imwrite("reference.png", display_reference_image);
  return cv::waitKey(0);
}
