#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    GlobalData_client = this->create_client<GlobalData>("vision_node"); // vision node 이름

    VisionData_sub = this->create_subscription<VisionData>(
        "vision2master", 10, std::bind(&TaskPlanner::vision_data_topic_callback, this, std::placeholders::_1));

    Master2Ik_pub = this->create_publisher<M2Ik>("master2ik", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&TaskPlanner::timer_callback, this));

    // request_global_data();
}

TaskPlanner::~TaskPlanner()
{
}

// ---
// srv
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
// srv
// ---

// ---------------------
// vision data subscribe
void TaskPlanner::vision_data_topic_callback(const VisionData::SharedPtr msg)
{
    timestamp = msg->timestamp;
    frame_drop = msg->frame_drop;

    camera_x = msg->camera_x;
    camera_y = msg->camera_y;

    left_x_1_dist = msg->left_x_1_dist;
    right_x_2_dist = msg->right_x_2_dist;
    // theta = msg->theta;

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
// vision data subscribe
// ---------------------

void TaskPlanner::timer_callback() // temp
{
    auto start_time = std::chrono::high_resolution_clock::now(); // delay 체크용
    //----------------------------------------------------------

    test_vision_walk(get_closest_obstacle());
    update_state();

    //----------------------------------------------------------
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (duration > 5000) // 5ms
        RCLCPP_WARN(this->get_logger(), "Timer callback delay: %ld us", duration);
}

// vision 내에 인식되는 장애물 중 가장 가까운 obstacle 반환
std::array<double, 2> TaskPlanner::get_closest_obstacle()
{
    std::array<std::array<double, 2>, 3> raw_obstacles = {obstacle_1, obstacle_2, obstacle_3};

    double min_distance = 9999.0;
    std::array<double, 2> closest_obstacle = {-1000.0, -1000.0};

    for (const auto &obs : raw_obstacles)
    {
        if (obs[0] <= -999.0 || obs[1] <= -999.0)
            continue;

        // hypot(x, y): 유클라디안(피타고라스) 거리
        double distance = std::hypot(obs[1], obs[0]);

        if (distance < min_distance)
        {
            min_distance = distance;
            closest_obstacle = obs;
        }
    }
    return closest_obstacle;
}

// void TaskPlanner::get_position(double dist_y, double dist_x)
// {
// }

void TaskPlanner::test_vision_walk(std::array<double, 2> dist)
{
    // 장애물이 연속 2개 있는 경우 장애물 사이에서 진동할 수 있음(최근접 obstable이 바뀌면서)
    // 1. 멀리서 판별이 가능한 경우 -> 지정된 곳으로 이동
    // 2. 멀리서 판별하지 못한 경우 -> 일단 가까운 obstacle을 피하고, 만약 해당 방향이 막힌 곳이면 반대 방향을 고정함

    // 장애물이 양쪽 사이드에 있는 경우 가운데로 어떻게 갈지
    // 1. 사이드 라인 인식 가능한 경우 -> 사이드 라인과 멀어지는 방향으로 이동
    // 2. 사이드 라인 인식 불가능한 경우 -> 멀리서 (1, 0, 1)을 인식해야만 함 -> 가운데로 이동

    // 장애물이 가운데 하나 있으면서 다음 구간 장애물이 왼쪽 또는 오른쪽만 뚫려있는 경우
    // 다음 장애물 인식이 필요함 -> 뚫려있는 양쪽 구간 뒤에 인식되는 장애물이 있는지 확인?

    // 현재 위치를 알 수 있는 경우 / 없는 경우 고려

    int is_detectived = dist[0] >= 0.01 && dist[0] <= 1.25;

    if (!is_detectived)
    {
        detective_count = 0;
        undetective_count++;

        if (undetective_count < real_undetective_std)
        {
            if (cur_state == "L")
                left();
            else if (cur_state == "R")
                right();
        }
        else
            forward();

        return;
    }

    undetective_count = 0;
    detective_count++;

    if (detective_count > real_detective_std)
    {
        if (dist[1] >= 0 && dist[1] < 0.58)
            left();
        else if (dist[1] < 0 && dist[1] > -0.58)
            right();
    }
    else
        forward();
}

// -----
// state
void TaskPlanner::update_state()
{
    M2Ik msg;

    // temp
    msg.x_length = x_length;
    msg.y_length = y_length;
    msg.yaw = yaw;
    msg.flag = flag;

    RCLCPP_INFO(this->get_logger(), "Publishing M2Ik -> x: %.2f, y: %.2f, yaw: %.2f, flag: %d", msg.x_length,
                msg.y_length, msg.yaw, msg.flag);

    Master2Ik_pub->publish(msg);
}

void TaskPlanner::forward()
{
    x_length = speed; // 20
    y_length = 0;
    yaw = 0;
    flag = 1;
    cur_state = "F";
}
void TaskPlanner::backward()
{
    x_length = -speed;
    y_length = 0;
    yaw = 0;
    flag = 1;
    cur_state = "B";
}
void TaskPlanner::left()
{
    x_length = 0;
    y_length = speed;
    yaw = 0;
    flag = 1;
    cur_state = "L";
}
void TaskPlanner::right()
{
    x_length = 0;
    y_length = -speed;
    yaw = 0;
    flag = 1;
    cur_state = "R";
}
void TaskPlanner::turn_left()
{
    x_length = 0;
    y_length = 0;
    yaw = speed;
    flag = 1;
    cur_state = "TL";
}
void TaskPlanner::turn_right()
{
    x_length = 0;
    y_length = 0;
    yaw = -speed;
    flag = 1;
    cur_state = "TR";
}
void TaskPlanner::standstill() // play
{
    x_length = 0;
    y_length = 0;
    yaw = 0;
    flag = 1;
    cur_state = "SS";
}
void TaskPlanner::play()
{
    standstill();
}
void TaskPlanner::stop()
{
    x_length = 0;
    y_length = 0;
    yaw = 0;
    flag = 0;
    cur_state = "S";
}
// state
// -----

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}