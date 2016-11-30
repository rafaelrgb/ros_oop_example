#include "RosOopExampleNode.h"


RosOopExampleNode::RosOopExampleNode(ros::NodeHandle *nh)
  : Node(nh, 30)
{
    migrationPoint_.x = 0.0;
    migrationPoint_.y = 0.0;
    position_.x = 0.0;
    position_.y = 0.0;
    mavros_state_sub_ = nh->subscribe("/mavros/state", 1, &RosOopExampleNode::mavrosStateCb, this);
    migration_point_sub_ = nh->subscribe("/migration_point", 1, &RosOopExampleNode::migrationPointCb, this);
    odom_sub_ = nh->subscribe("/mavros/local_position/odom", 1, &RosOopExampleNode::odomCb, this);
    rc_override_pub_ = nh->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
}

RosOopExampleNode::~RosOopExampleNode()
{
    mavros_state_sub_.shutdown();
    migration_point_sub_.shutdown();
    odom_sub_.shutdown();
    rc_override_pub_.shutdown();
}

void RosOopExampleNode::controlLoop()
{
    // Print information on screen
    char tab2[1024];
    strncpy(tab2, mode_.c_str(), sizeof(tab2));
    tab2[sizeof(tab2) - 1] = 0;
    ROS_INFO("Objective = (%f , %f) \n Roll = %f | Pitch = %f\n Mode = %s \n UavX = %f | UavY = %f\n", migrationPoint_.x, migrationPoint_.y, roll_, pitch_, tab2, position_.x, position_.y);

    // Error between pose and objective
    float ErX = 0.0;
    float ErY = 0.0;

    // Get objective center and calculate error
    ErX = position_.x - migrationPoint_.x;
    ErY = position_.y - migrationPoint_.y;

    // Calculate Roll and Pitch depending on the mode
    if (mode_ == "LOITER"){
        roll_ = BASERC + ErX * FACTOR;
        pitch_ = BASERC - ErY * FACTOR;
    }else{
        roll_ = BASERC;
        pitch_ = BASERC;
    }

    // Limit the Roll
    if (roll_ > MAXRC)
    {
        roll_ = MAXRC;
    } else if (roll_ < MINRC)
    {
        roll_ = MINRC;
    }

    // Limit the Pitch
    if (pitch_ > MAXRC)
    {
        pitch_ = MAXRC;
    } else if (pitch_ < MINRC)
    {
        pitch_ = MINRC;
    }

    publishRCOverride();
}

void RosOopExampleNode::publishRCOverride()
{
    // Create RC msg
    mavros_msgs::OverrideRCIn msg;

    msg.channels[0] = roll_;     //Roll
    msg.channels[1] = pitch_;    //Pitch
    msg.channels[2] = BASERC;   //Throttle
    msg.channels[3] = 0;        //Yaw
    msg.channels[4] = 0;
    msg.channels[5] = 0;
    msg.channels[6] = 0;
    msg.channels[7] = 0;

    rc_override_pub_.publish(msg);
}

void RosOopExampleNode::mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    mode_ = msg->mode;
    guided_ = msg->guided==128;
    armed_ = msg->armed==128;
}

void RosOopExampleNode::migrationPointCb( const geometry_msgs::Point32ConstPtr &msg )
{
    migrationPoint_.x = msg->x;
    migrationPoint_.y = msg->y;
}

void RosOopExampleNode::odomCb( const nav_msgs::OdometryConstPtr &msg )
{
    position_.x = msg->pose.pose.position.x;
    position_.y = msg->pose.pose.position.y;
}
