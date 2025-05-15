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
const char* banner[] = {"This program computes the Frobenius distance norm between two images of proportional size", 0};
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
  cv::Mat query_image(cv::imread(argument_query_image_file.value(), CV_LOAD_IMAGE_GRAYSCALE));
  cv::Mat reference_image(
    cv::imread(argument_reference_image_file.value(), CV_LOAD_IMAGE_GRAYSCALE));

  // ds verify size
  if (query_image.rows != reference_image.rows || query_image.cols != reference_image.cols) {
    // ds check if we can scale the images to be the same size
    const double scaling_factor = static_cast<double>(query_image.rows) / reference_image.rows;
    if (scaling_factor == static_cast<double>(query_image.cols) / reference_image.cols) {
      assert(scaling_factor != 1);
      if (scaling_factor < 1) {
        std::cerr << "REFERENCE image is scaled down by: " << scaling_factor << std::endl;
        cv::resize(reference_image, reference_image, cv::Size(query_image.cols, query_image.rows));
      } else {
        std::cerr << "QUERY image is scaled down by: " << scaling_factor << std::endl;
        cv::resize(query_image, query_image, cv::Size(reference_image.cols, reference_image.rows));
      }
    } else {
      std::cerr << "ERROR: images have unequal sizes ";
      std::cerr << "QUERY < " << query_image.rows << " x " << query_image.cols << ", ";
      std::cerr << "REFERENCE < " << reference_image.rows << " x " << reference_image.cols
                << std::endl;
      return -1;
    }
  }

  // ds compute sum of squared intensity deltas
  size_t sum_of_squared_intensity_deltas = 0;
  for (size_t r = 0; r < static_cast<size_t>(query_image.rows); ++r) {
    for (size_t c = 0; c < static_cast<size_t>(query_image.cols); ++c) {
      const int32_t intensity_delta = query_image.at<uchar>(r, c) - reference_image.at<uchar>(r, c);
      sum_of_squared_intensity_deltas += intensity_delta * intensity_delta;
    }
  }
  std::cerr << "sum of squared intensity distances: " << sum_of_squared_intensity_deltas
            << std::endl;
  std::cerr << "total number of distances: " << query_image.rows * query_image.cols << std::endl;
  std::cerr << "Frobenius norm: " << std::sqrt(sum_of_squared_intensity_deltas) << std::endl;

  // ds show images
  cv::imshow("query", query_image);
  cv::imshow("reference", reference_image);
  return cv::waitKey(0);
}
