#pragma once

#include "wild_point_filter.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"




class IMU_wild_point_filter_Node
{
public:
  IMU_wild_point_filter_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  


private:
  ros::NodeHandle nh_;
  

  IMUWildPointFilter wild_point_filter_;
  //ros::Timer pubTImer_;
  ros::Subscriber subscribeIMU_;
  //void publishPoseState(const ros::TimerEvent&);
 

  // ROS publisher
  ros::Publisher publishIMU_filtered;

  // Callbacks
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_Message_data);



};