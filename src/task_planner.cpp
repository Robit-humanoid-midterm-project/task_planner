#include "task_planner/task_planner.hpp"

#include <sstream>

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    Master2Ik_pub = this->create_publisher<M2Ik>("master2ik", 10);
}

TaskPlanner::~TaskPlanner()
{
}

void TaskPlanner::state_change_callback()
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}