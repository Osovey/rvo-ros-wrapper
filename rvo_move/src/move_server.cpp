#include <ros/ros.h>

#include "rvo_wrapper.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_server");
  rvo::MoveServer ms(ros::this_node::getName());
  ms.start();
  ros::spin();
}
