#include "wild_point_filter.h"
#include <cmath>
#include <iostream>

using namespace std;

IMUWildPointFilter::IMUWildPointFilter()
:dropped_acceleration_x_msg_{0}
,dropped_acceleration_y_msg_{0}
,dropped_acceleration_z_msg_{0}
,filtered_gyro_x_msg_{0}
,filtered_gyro_y_msg_{0}
,filtered_gyro_z_msg_{0}
,filtered_acc_x_msg_{0}
,filtered_acc_y_msg_{0}
,filtered_acc_z_msg_{0}
,flag_{true}
{
    acceleration_buffer_x_.clear();
    acceleration_buffer_y_.clear();
    acceleration_buffer_z_.clear();
    gyro_buffer_x_.clear();
    gyro_buffer_y_.clear();
    gyro_buffer_z_.clear();
}

void IMUWildPointFilter::filterWildPointGyroX(const double& gyro_x_msg)
{

    gyro_buffer_x_.push_back(gyro_x_msg);

    if(gyro_buffer_x_.size() == 2)
    {
        if(abs(gyro_buffer_x_.back() - gyro_buffer_x_.front()) < max(2*abs(gyro_buffer_x_.front()),GYRO_X_PEAK_TO_PEAK_NOISE) || dropped_gyro_x_msg_ > MAX_DROPPED_GYRO_X_MSG)
        {
            filtered_gyro_x_msg_ = gyro_buffer_x_.back();
            dropped_gyro_x_msg_ = 0;
        }
        else
        {

            //ROS_DEBUG("GYRO_X_MSG wild point rejected");
            dropped_gyro_x_msg_ += 1;
            flag_ = false;
        }
        gyro_buffer_x_ = std::vector<double>{filtered_gyro_x_msg_}; // Previous: gyro_buffer_x_.back()
    }
    else 
    {
        filtered_gyro_x_msg_ = gyro_x_msg;
    }
}

void IMUWildPointFilter::filterWildPointGyroY(const double& gyro_y_msg)
{
    gyro_buffer_y_.push_back(gyro_y_msg);

     if(gyro_buffer_y_.size() == 2)
    {
        if(abs(gyro_buffer_y_.back() - gyro_buffer_y_.front()) < max(2*abs(gyro_buffer_y_.front()),GYRO_Y_PEAK_TO_PEAK_NOISE) || dropped_gyro_y_msg_ > MAX_DROPPED_GYRO_Y_MSG)
        {
            filtered_gyro_y_msg_ = gyro_buffer_y_.back();
            dropped_gyro_y_msg_ = 0;
        }
        else
        {
            //ROS_DEBUG("GYRO_X_MSG wild point rejected");
            dropped_gyro_y_msg_ += 1;
        }
        gyro_buffer_y_ = std::vector<double>{gyro_buffer_x_.back()};
    }
    else 
    {
        filtered_gyro_y_msg_ = gyro_y_msg;
    }
}

void IMUWildPointFilter::filterWildPointGyroZ(const double& gyro_z_msg)
{
    gyro_buffer_z_.push_back(gyro_z_msg);

    if(gyro_buffer_z_.size() == 2)
    {
        if(abs(gyro_buffer_z_.back() - gyro_buffer_z_.front()) < max(2*abs(gyro_buffer_z_.front()),GYRO_Z_PEAK_TO_PEAK_NOISE) || dropped_gyro_z_msg_ > MAX_DROPPED_GYRO_Z_MSG)
        {
            filtered_gyro_z_msg_ = gyro_buffer_z_.back();
            dropped_gyro_z_msg_ = 0;
        }
        else
        {
            //ROS_DEBUG("GYRO_X_MSG wild point rejected");

            dropped_gyro_z_msg_ += 1;
        }
        gyro_buffer_z_ = std::vector<double>{gyro_buffer_z_.back()};
    }
    else 
    {
        filtered_gyro_z_msg_ = gyro_z_msg;
    }
}

void IMUWildPointFilter::filterWildPointAccX(const double& acc_x_msg)
{
    acceleration_buffer_x_.push_back(acc_x_msg);

    if (acceleration_buffer_x_.size() == 2)
    {
        if(std::abs(acceleration_buffer_x_.back() - acceleration_buffer_x_.front()) < ACC_TOLERANCE || dropped_acceleration_x_msg_ > MAX_DROPPED_ACC_X_MSG )
        {
            filtered_acc_x_msg_ = acceleration_buffer_x_.back();
            dropped_acceleration_x_msg_ = 0;
            flag_=true;
        }
        else
        {
            std::cout<<"dropped_filtered_acc_x_msg: "<<filtered_acc_x_msg_<<std::endl;
            
            //ROS_DEBUG("ACC_X_MSG wild point rejected");
            dropped_acceleration_x_msg_ +=1;
            flag_ = false;
        }
        //acceleration_buffer_x_ = std::vector<double>{acceleration_buffer_x_.back()};
        acceleration_buffer_x_ = std::vector<double>{filtered_acc_x_msg_};
    }
    else
    {
        filtered_acc_x_msg_ = acc_x_msg;
    }
    
}

void IMUWildPointFilter::filterWildPointAccY(const double& acc_y_msg)
{
    acceleration_buffer_y_.push_back(acc_y_msg);

    if (acceleration_buffer_y_.size() == 2)
    {
        if(std::abs(acceleration_buffer_y_.back() - acceleration_buffer_y_.front()) < ACC_TOLERANCE || dropped_acceleration_y_msg_ > MAX_DROPPED_ACC_Y_MSG )
        {
            filtered_acc_y_msg_ = acceleration_buffer_y_.back();
            dropped_acceleration_y_msg_ = 0;
        }
        else
        {
            //ROS_DEBUG("ACC_X_MSG wild point rejected");
            dropped_acceleration_y_msg_ +=1;
        }
        acceleration_buffer_y_ = std::vector<double>{acceleration_buffer_y_.back()};
    }
    else
    {
        filtered_acc_y_msg_ = acc_y_msg;
    }
    
}

void IMUWildPointFilter::filterWildPointAccZ(const double& acc_z_msg)
{
    acceleration_buffer_z_.push_back(acc_z_msg);

    if (acceleration_buffer_z_.size() == 2)
    {
        if(std::abs(acceleration_buffer_z_.back() - acceleration_buffer_z_.front()) < ACC_TOLERANCE || dropped_acceleration_z_msg_ > MAX_DROPPED_ACC_Z_MSG )
        {
            filtered_acc_z_msg_ = acceleration_buffer_z_.back();
            dropped_acceleration_z_msg_ = 0;
        }
        else
        {
            //ROS_DEBUG("ACC_X_MSG wild point rejected");
            std::cout<<"ACC_Z_MSG_wild_point_rejected"<<std::endl;
            dropped_acceleration_z_msg_ +=1;
        }
        acceleration_buffer_z_ = std::vector<double>{acceleration_buffer_z_.back()};
    }
    else
    {
        filtered_acc_z_msg_ = acc_z_msg;
    }
    
}


