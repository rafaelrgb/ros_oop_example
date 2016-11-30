#ifndef _ROS_OOP_EXAMPLE_NODE_H_
#define _ROS_OOP_EXAMPLE_NODE_H_

#include <string>
#include "Node.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <swarm_control/UavPosition.h>

#define FACTOR  60.0

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

#define MAXVEL   5.0
#define MINVEL   -5.0

class RosOopExampleNode : public Node
{
public:
  RosOopExampleNode(ros::NodeHandle *nh);
  virtual ~RosOopExampleNode();

private:
  virtual void controlLoop();

  // Position of elements in the system
  geometry_msgs::Point32 position_;
  geometry_msgs::Point32 migrationPoint_;

  // Drone commands
  double roll_;
  double pitch_;

  // Flight mode
  std::string mode_;
  bool guided_;
  bool armed_;

  // ROS objects
  ros::Subscriber mavros_state_sub_;    // Subscriber to flight mode
  ros::Subscriber migration_point_sub_; // Subscriber to migration point
  ros::Subscriber odom_sub_;            // Subscriber to odometry
  ros::Publisher rc_override_pub_;      // RC publisher

  // Member functions
  void mavrosStateCb( const mavros_msgs::StateConstPtr &msg );
  void migrationPointCb( const geometry_msgs::Point32ConstPtr &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void publishRCOverride();

};

#endif // _ROS_OOP_EXAMPLE_NODE_H_
