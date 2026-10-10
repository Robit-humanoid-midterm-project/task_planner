#pragma once

#include "rclcpp/rclcpp.hpp"

#include "gamecontroller/msg/gamecontroldata.hpp"

#include "humanoid_interfaces/msg/master2_ik_msg.hpp"
#include "humanoid_interfaces/msg/vision_data.hpp"
#include "humanoid_interfaces/srv/global_scan_data.hpp"

#include <algorithm>
#include <deque>
#include <string>
#include <vector>

using controlData = gamecontroller::msg::Gamecontroldata;

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

    rclcpp::Subscription<controlData>::SharedPtr ControlData_sub;
    void control_data_callback(const controlData::SharedPtr msg);

    int state = 0; // play: 3, finish: 4

    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    void get_position();
    std::deque<double> buffer;
    long unsigned int window_size = 5;
    std::array<std::array<double, 2>, 3> get_closest_obstacle();
    void test_vision_walk(std::array<std::array<double, 2>, 3> dist);
    int last_dir = 0; // -1 0 1 (L, 0, R)

    double speed = 20.0;
    void update_state();
    void forward();
    void backward();
    void left();
    void right();
    void turn_left();
    void turn_right();
    void standstill();
    void play();
    void stop();

    std::string cur_state = "F"; // F, B, L, R, SS, S

    double dist_x = 0;
    double dist_second_x = 0;
    int detectived = 0;

    int ob_undetective_cnt = 0;

    double map_width = 1.4;
    double close_std = 0.20;

    // yaml에서 받아오기, +x, y, yaw값
    double F_Max_Test_X = 40.0;
    double F_Min_Test_X = 30.0;

    double B_Test_x = -15.0;

    double L_Test_x = 4.50;
    double L_Test_side = 10.00;

    double R_Test_x = 4.50;
    double R_Test_side = -10.00;

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
    // double frame_drop = 0.0;

    double camera_x = 0.7;
    double camera_y = 0.0;

    double left_x_1_dist = 0.7;
    // double right_x_2_dist = 0.7;
    // double theta = 0.0;

    // std::array<double, 3> section_1{0.0, 0.0, 0.0};
    // std::array<double, 3> section_2{0.0, 0.0, 0.0};
    // std::array<double, 3> section_3{0.0, 0.0, 0.0};

    // double section_1_detected = 0.0;
    // double section_2_detected = 0.0;
    // double section_3_detected = 0.0;

    // std::array<double, 4> nearest_line{0.0, 0.0, 0.0, 0.0};

    std::array<double, 2> obstacle_1{0.0, 0.0};
    std::array<double, 2> obstacle_2{0.0, 0.0};
    std::array<double, 2> obstacle_3{0.0, 0.0};

    // double confidence = 0.0;
    std::array<double, 3> obstacle_ratio{0.0, 0.0, 0.0};
    // Vision Data
    // -----------
};
