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
    void state_change_callback();
    rclcpp::Publisher<M2Ik>::SharedPtr Master2Ik_pub;
};
