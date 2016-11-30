#include <stdlib.h>
#include "RosOopExampleNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "swarm_controller_node");
  RosOopExampleNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
