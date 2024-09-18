#pragma once

// #define HRPTUNING

//#include <inertial_sense_ros2.h>
#include <termios.h>
#include <unistd.h>

#include "inertial_sense_ros.h"

class cTestNode : public rclcpp::Node
{
public:
    cTestNode() : Node("test_node"){}
    void init();
    bool step();
    void cbWheelEncoder(const sensor_msgs::msg::JointState &msg);
    void cbPIMU(const inertial_sense_ros2::msg::PIMU::SharedPtr pimu);
    void cbIMU(const  sensor_msgs::msg::Imu &imu);
    void cbINS(const nav_msgs::msg::Odometry &ins);
    void cbGPS(const inertial_sense_ros2::msg::GPS &gps);

    int get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out);
    double get_avg_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_min_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_max_deviation(std::vector<double> &a, std::vector<double> &b);

    bool quiet = true;
    bool got_gps_tow = false;
    bool did_rx_pimu_ = false;

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;
    std::vector<double> ins_ts;

private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel_encoder_;
    rclcpp::Subscription<inertial_sense_ros2::msg::PIMU>::SharedPtr sub_pimu_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<inertial_sense_ros2::msg::GPS>::SharedPtr sub_gps1_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ins_;


};