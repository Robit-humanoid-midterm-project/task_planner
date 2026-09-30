#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    Master2Ik_pub = this->create_publisher<M2Ik>("master2ik", 10);
}

TaskPlanner::~TaskPlanner()
{
}

void TaskPlanner::state_change_callback()
{
    M2Ik msg;

    // temp
    msg.x_length = x_length;
    msg.y_length = y_length;
    msg.yaw = yaw;
    msg.flag = flag;

    Master2Ik_pub->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}