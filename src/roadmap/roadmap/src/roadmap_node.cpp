#include "roadmap.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, ros::this_node::getName());

  Roadmap roadmap;
  ros::spin();

  return 0;
}

void Poll()
{
}