#pragma once

#include "rclcpp/rclcpp.hpp"

#include "humanoid_interfaces/msg/master2_ik_msg.hpp"

using M2Ik = humanoid_interfaces::msg::Master2IkMsg;

class TaskPlanner : public rclcpp::Node
{
  public:
    TaskPlanner();
    ~TaskPlanner();

  private:
    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
    void state_change_callback();

    rclcpp::TimerBase::SharedPtr timer_; // test
    void timer_callback();

    int i = 0;

    double x_length = 0.0;
    double y_length = 0.0;
    double yaw = 0.0;
    double flag = 0.0;
};
