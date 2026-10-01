#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    GlobalData_client = this->create_client<GlobalData>("vision_node"); // vision node 이름

    VisionData_sub = this->create_subscription<VisionData>(
        "vision2master", 10, std::bind(&TaskPlanner::vision_data_topic_callback, this, std::placeholders::_1));

    Master2Ik_pub = this->create_publisher<M2Ik>("master2ik", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&TaskPlanner::timer_callback, this));

    request_global_data();
}

TaskPlanner::~TaskPlanner()
{
}

void TaskPlanner::request_global_data()
{
    // 켜질 때까지 대기
    while (!GlobalData_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
            return;
    }

    // 빈 request 생성 (보내는 값 없음)
    auto request = std::make_shared<GlobalData::Request>();

    GlobalData_client->async_send_request(
        request, std::bind(&TaskPlanner::global_data_response_callback, this, std::placeholders::_1));
}

void TaskPlanner::global_data_response_callback(ServiceFuture future)
{
    auto response = future.get();

    // 데이터 받기
    for (int i = 0; i < 9; ++i)
    {
        global_data = response->object_detective;
    }
}

void TaskPlanner::vision_data_topic_callback(const VisionData::SharedPtr msg)
{
    timestamp = msg->timestamp;
    frame_drop = msg->frame_drop;

    camera_x = msg->camera_x;
    camera_y = msg->camera_y;

    left_x_1_dist = msg->left_x_1_dist;
    right_x_2_dist = msg->right_x_2_dist;
    theta = msg->theta;

    section_1 = msg->section_1;
    section_2 = msg->section_2;
    section_3 = msg->section_3;

    section_1_detected = msg->section_1_detected;
    section_2_detected = msg->section_2_detected;
    section_3_detected = msg->section_3_detected;

    nearest_line = msg->nearest_line;

    obstacle_1 = msg->obstacle_1;
    obstacle_2 = msg->obstacle_2;
    obstacle_3 = msg->obstacle_3;

    confidence = msg->confidence;
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
void TaskPlanner::timer_callback() // test
{
    if (i % 2 == 0)
    {
        x_length += 15;
        flag = 1;
        i++;
    }
    else
    {
        x_length -= 15;
        flag = 0;
        i = 0;
    }
    state_change_callback();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}