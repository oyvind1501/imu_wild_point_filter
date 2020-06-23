#include "ros_node.h"
#include <Eigen/Core>

using namespace Eigen;

IMU_wild_point_filter_Node::IMU_wild_point_filter_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_{ pnh }
{
  //int publish_rate{ 125 };
  // Subscribe to IMU
  ROS_INFO("Subscribing to imu/data_raw");
  subscribeIMU_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1000, &IMU_wild_point_filter_Node::imuCallback, this,
                                                  ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("Publishing State");
  publishIMU_filtered = nh_.advertise<sensor_msgs::Imu>("/imu/data_filtered", 1000);

  //pubTImer_ = nh_.createTimer(ros::Duration(1.0f / publish_rate), &IMU_wild_point_filter_Node::publishPoseState, this);
}

// IMU Subscriber
void IMU_wild_point_filter_Node::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_Message_data)
{
  Vector3d raw_acceleration_measurements = Vector3d::Zero();
  Vector3d raw_gyro_measurements = Vector3d::Zero();

  raw_acceleration_measurements << imu_Message_data->linear_acceleration.x, imu_Message_data->linear_acceleration.y,
    imu_Message_data->linear_acceleration.z;


  raw_gyro_measurements << imu_Message_data->angular_velocity.x, imu_Message_data->angular_velocity.y,
    imu_Message_data->angular_velocity.z;

  wild_point_filter_.filterWildPointAccX(raw_acceleration_measurements(0));
  wild_point_filter_.filterWildPointAccY(raw_acceleration_measurements(1));
  wild_point_filter_.filterWildPointAccZ(raw_acceleration_measurements(2));
  wild_point_filter_.filterWildPointGyroX(raw_gyro_measurements(0));
  wild_point_filter_.filterWildPointGyroY(raw_gyro_measurements(1));
  wild_point_filter_.filterWildPointGyroZ(raw_gyro_measurements(2));

  sensor_msgs::Imu imu_msg_filtered;
  static size_t trace_id{ 0 };
 
  imu_msg_filtered.header.frame_id = "/imu_wild_point_filter_link";
  imu_msg_filtered.header.seq = trace_id++;
  imu_msg_filtered.header.stamp = ros::Time::now();
  imu_msg_filtered.linear_acceleration.x = wild_point_filter_.getFilteredWildPointAccX();
  imu_msg_filtered.linear_acceleration.y = wild_point_filter_.getFilteredWildPointAccY();
  imu_msg_filtered.linear_acceleration.z = wild_point_filter_.getFilteredWildPointAccZ();
  imu_msg_filtered.angular_velocity.x = wild_point_filter_.getFilteredWildPointGyroX();
  imu_msg_filtered.angular_velocity.y = wild_point_filter_.getFilteredWildPointGyroY();
  imu_msg_filtered.angular_velocity.z = wild_point_filter_.getFilteredWildPointGyroZ();

 

  if(wild_point_filter_.flag_ == true)
  {
    publishIMU_filtered.publish(imu_msg_filtered);
  }

}




int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_wild_point_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  IMU_wild_point_filter_Node imu_wild_point_filter_node(nh, pnh);
  ros::spin();
  return 0;
}