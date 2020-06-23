#include <vector>

constexpr double ACC_TOLERANCE{0.6};
constexpr double MAX_DROPPED_ACC_X_MSG{2};
constexpr double MAX_DROPPED_ACC_Y_MSG{2};
constexpr double MAX_DROPPED_ACC_Z_MSG{5};
constexpr double MAX_DROPPED_GYRO_X_MSG{5};
constexpr double MAX_DROPPED_GYRO_Y_MSG{5};
constexpr double MAX_DROPPED_GYRO_Z_MSG{5};
constexpr double GYRO_X_PEAK_TO_PEAK_NOISE{0.0002};
constexpr double GYRO_Y_PEAK_TO_PEAK_NOISE{0.0002};
constexpr double GYRO_Z_PEAK_TO_PEAK_NOISE{0.0002};


class IMUWildPointFilter
{
private:

    
      // Acc wild point filter
    std::vector<double> acceleration_buffer_x_;
    std::vector<double> acceleration_buffer_y_;
    std::vector<double> acceleration_buffer_z_;
    double dropped_acceleration_x_msg_;
    double dropped_acceleration_y_msg_;
    double dropped_acceleration_z_msg_;

    // Gyro wild point filter
    std::vector<double> gyro_buffer_x_;
    std::vector<double> gyro_buffer_y_;
    std::vector<double> gyro_buffer_z_;
    double dropped_gyro_x_msg_;
    double dropped_gyro_y_msg_;
    double dropped_gyro_z_msg_;

    double filtered_gyro_x_msg_;
    double filtered_gyro_y_msg_;
    double filtered_gyro_z_msg_;

    double filtered_acc_x_msg_;
    double filtered_acc_y_msg_;
    double filtered_acc_z_msg_;


public:
    IMUWildPointFilter();
    void filterWildPointGyroX(const double& gyro_x_msg);
    void filterWildPointGyroY(const double& gyro_y_msg);
    void filterWildPointGyroZ(const double& gyro_z_msg);
    void filterWildPointAccX(const double& acc_x_msg);
    void filterWildPointAccY(const double& acc_y_msg);
    void filterWildPointAccZ(const double& acc_z_msg);

    inline double getFilteredWildPointGyroX() const
    {
        return filtered_gyro_x_msg_;
    }
    inline double getFilteredWildPointGyroY() const
    {
        return filtered_gyro_y_msg_;
    }
    inline double getFilteredWildPointGyroZ() const
    {
        return filtered_gyro_z_msg_;
    }
    inline double getFilteredWildPointAccX() const
    {
        return filtered_acc_x_msg_;
    }
    inline double getFilteredWildPointAccY() const
    {
        return filtered_acc_y_msg_;
    }
    inline double getFilteredWildPointAccZ() const
    {
        return filtered_acc_z_msg_;
    }

    // Flag
    bool flag_;
};




