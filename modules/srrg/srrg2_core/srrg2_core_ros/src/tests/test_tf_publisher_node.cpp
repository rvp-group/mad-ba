#include <iostream>

//ds srrg
#include "srrg_converters/converter.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;



int32_t main(int32_t argc_, char** argv_) {

  //ds initialize roscpp and allocate a ROS node
  ros::init(argc_, argv_, "test_tf_publisher");
  ros::NodeHandle node;

  //ds configure publisher
  ros::Publisher publisher = node.advertise<tf2_msgs::TFMessage>("/tf", 1000);
  ros::Rate publication_rate(10);

  //ds start publishing
  size_t number_of_published_messages = 0;
  while (ros::ok()) {

    //ds create a bunch of transforms to publish over ROS
    TransformEventsMessagePtr transform_events(new TransformEventsMessage());
    transform_events->events.resize(10);
    for (size_t u = 0; u < transform_events->events.size(); ++u) {
      Isometry3f pose(Isometry3f::Identity());
      pose.translation().x() = u;
      transform_events->events.value(u) = TransformEvent(ros::Time::now().toSec(), "/link_"+std::to_string(u), pose, "/world");
    }

    //ds obtain ROS message from converter and publish message
    publisher.publish(Converter::convert(transform_events));

    //ds give control to ROS
    ros::spinOnce();
    publication_rate.sleep();
    ++number_of_published_messages;
    std::cerr << "published messages: " << number_of_published_messages << "\r";
  }
  return 0;
}
