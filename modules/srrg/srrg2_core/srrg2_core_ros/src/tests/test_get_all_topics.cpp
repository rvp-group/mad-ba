#include <ros/ros.h>

int main(int argc, char** argv) {

  //ds initialize roscpp
  ros::init(argc, argv, "test_get_all_topics");

  //ds start node
  ros::master::V_TopicInfo topics;
  std::cerr << "waiting for new topics" << std::endl;
  ros::Time::init();
  ros::Rate r(10);
  while (topics.size() < 2) {
    ros::master::getTopics(topics);
    r.sleep();
  }
  r.sleep();
  ros::master::getTopics(topics);

  std::cerr << std::endl;
  std::cerr << topics.size() << std::endl;
  for (auto topic: topics) {
    std::cerr << topic.name << std::endl;
    const std::string& message_type = topic.datatype;
    std::size_t split_idx = message_type.find('/');
    std::string ros_namespace = message_type.substr(0, split_idx);
    std::string ros_class = message_type.substr(split_idx+1);
    std::cerr << ros_namespace << "::" << ros_class << std::endl;
    std::cerr << std::endl;
  }


  return 0;
}
