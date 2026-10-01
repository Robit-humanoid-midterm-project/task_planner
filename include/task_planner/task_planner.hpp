#pragma once

#include "rclcpp/rclcpp.hpp"

#include "humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "humanoid_interfaces/msg/vision_data.hpp"
#include "humanoid_interfaces/srv/global_scan_data.hpp"

using M2Ik = humanoid_interfaces::msg::Master2IkMsg;
using GlobalData = humanoid_interfaces::srv::GlobalScanData;
using VisionData = humanoid_interfaces::msg::VisionData;

using ServiceFuture = rclcpp::Client<GlobalData>::SharedFuture;

class TaskPlanner : public rclcpp::Node
{
  public:
    TaskPlanner();
    ~TaskPlanner();

  private:
    rclcpp::Client<GlobalData>::SharedPtr GlobalData_client;
    void request_global_data();
    void global_data_response_callback(ServiceFuture future);
    std::array<double, 9> global_data{0.0}; // GlobalData_srv

    rclcpp::Subscription<VisionData>::SharedPtr VisionData_sub;
    void vision_data_topic_callback(const VisionData::SharedPtr msg);

    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
    void state_change_callback();

    rclcpp::TimerBase::SharedPtr timer_; // test
    void timer_callback();

    int i = 0;

    // ---------
    // Master2Ik
    double x_length = 0.0;
    double y_length = 0.0;
    double yaw = 0.0;
    double flag = 0.0;
    // Master2Ik
    // ---------

    // -----------
    // Vision Data
    double timestamp = 0.0;
    double frame_drop = 0.0;

    double camera_x = 0.0;
    double camera_y = 0.0;

    double left_x_1_dist = 0.0;
    double right_x_2_dist = 0.0;
    double theta = 0.0;

    std::array<double, 3> section_1{0.0, 0.0, 0.0};
    std::array<double, 3> section_2{0.0, 0.0, 0.0};
    std::array<double, 3> section_3{0.0, 0.0, 0.0};

    double section_1_detected = 0.0;
    double section_2_detected = 0.0;
    double section_3_detected = 0.0;

    std::array<double, 4> nearest_line{0.0, 0.0, 0.0, 0.0};

    std::array<double, 2> obstacle_1{0.0, 0.0};
    std::array<double, 2> obstacle_2{0.0, 0.0};
    std::array<double, 2> obstacle_3{0.0, 0.0};

    double confidence = 0.0;
    // Vision Data
    // -----------
};
