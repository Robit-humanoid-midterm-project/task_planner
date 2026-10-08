#include "task_planner/task_planner.hpp"

TaskPlanner::TaskPlanner() : Node("task_planner")
{
    speed = declare_parameter<double>("speed", 20);
    left_x_1_dist = declare_parameter<double>("start_x", 0.7);
    camera_x = left_x_1_dist;
    camera_y = declare_parameter<double>("start_y", 0.0);
    map_width = declare_parameter<double>("map_width", 1.4);
    close_std = declare_parameter<double>("close_std", 0.20);

    F_Max_Test_X = declare_parameter<double>("F_Max_Test_X", 40.0);
    F_Min_Test_X = declare_parameter<double>("F_Min_Test_X", 30.0);

    B_Test_x = declare_parameter<double>("B_Test_x", -15.0);

    L_Test_x = declare_parameter<double>("L_Test_x", 4.5);
    L_Test_side = declare_parameter<double>("L_Test_side", 10.0);

    R_Test_x = declare_parameter<double>("R_Test_x", 4.5);
    R_Test_side = declare_parameter<double>("R_Test_side", -10.0);

    GlobalData_client = this->create_client<GlobalData>("vision_node"); // vision node 이름

    VisionData_sub = this->create_subscription<VisionData>(
        "vision2master", 10, std::bind(&TaskPlanner::vision_data_topic_callback, this, std::placeholders::_1));

    ControlData_sub = this->create_subscription<controlData>(
        "gamecontroldata", 10, std::bind(&TaskPlanner::control_data_callback, this, std::placeholders::_1));

    Master2Ik_pub = this->create_publisher<M2Ik>("master2ik", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000 / 50), std::bind(&TaskPlanner::timer_callback, this));

    // request_global_data();
}

TaskPlanner::~TaskPlanner()
{
    stop();
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
    // frame_drop = msg->frame_drop;

    // camera_x = msg->camera_x;
    // camera_y = msg->camera_y;

    left_x_1_dist = msg->left_x_1_dist;
    // right_x_2_dist = msg->right_x_2_dist;
    // theta = msg->theta;

    // section_1 = msg->section_1;
    // section_2 = msg->section_2;
    // section_3 = msg->section_3;

    // section_1_detected = msg->section_1_detected;
    // section_2_detected = msg->section_2_detected;
    // section_3_detected = msg->section_3_detected;

    // earest_line = msg->nearest_line;

    obstacle_1 = msg->obstacle_1;
    obstacle_2 = msg->obstacle_2;
    obstacle_3 = msg->obstacle_3;

    // confidence = msg->confidence;
}
// vision data subscribe
// ---------------------

// ------------------------------
// game controller data subscribe
void TaskPlanner::control_data_callback(const controlData::SharedPtr msg)
{
    state = msg->state;
    // RCLCPP_INFO(this->get_logger(), "gamecontroller state: %d", state);
}
// game controller data subscribe
// ------------------------------

void TaskPlanner::timer_callback() // temp
{
    // RCLCPP_INFO(this->get_logger(), "cur state: %s", cur_state.c_str());
    if (state != 3)
    {
        stop();
        update_state();
        return;
    }

    auto start_time = std::chrono::high_resolution_clock::now(); // delay 체크용
    //----------------------------------------------------------

    get_position();
    test_vision_walk(get_closest_obstacle());
    update_state();

    //----------------------------------------------------------
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    if (duration > 5000) // 5ms
        RCLCPP_WARN(this->get_logger(), "Timer callback delay: %ld us", duration);
}

// lastest position get
void TaskPlanner::get_position()
{
    buffer.push_back(left_x_1_dist);
    if (buffer.size() > window_size)
        buffer.pop_front();

    if (buffer.size() <= window_size)
    {
        camera_x = left_x_1_dist;
        return;
    }

    std::vector<double> temp(buffer.begin(), buffer.end());
    std::nth_element(temp.begin(), temp.begin() + 2, temp.end());

    camera_x = temp[2];
}

// vision 내에 인식되는 장애물 중 가장 가까운 순서대로 obstacle 반환
std::array<std::array<double, 2>, 3> TaskPlanner::get_closest_obstacle()
{
    std::array<std::array<double, 2>, 3> obstacles = {obstacle_1, obstacle_2, obstacle_3};

    std::sort(obstacles.begin(), obstacles.end(), [](const std::array<double, 2> &a, const std::array<double, 2> &b) {
        // 유효한 데이터인지 확인
        bool valid_a = (a[0] > -999.0 && a[1] > -999.0);
        bool valid_b = (b[0] > -999.0 && b[1] > -999.0);

        if (valid_a && valid_b)
            return std::hypot(a[0], a[1]) < std::hypot(b[0], b[1]);
        else if (valid_a)
            return true;
        else if (valid_b)
            return false;
        else
            return false;
    });

    return obstacles;
}

void TaskPlanner::test_vision_walk(std::array<std::array<double, 2>, 3> dist)
{
    // 최단 경로로 가기 위해서 다음 구간 장애물 인식 필요
    // 현재 구간 뒤에 있는 빨강+파랑 픽셀 수가 적은쪽으로 이동하는 방법
    // TODO: vision 화면 최하단 중앙에 장애물 색이 있는지 없는지 bool값

    int is_detectived = dist[0][0] >= 0.01 && dist[0][0] <= 1.10;

    detectived = is_detectived;
    dist_x = dist[0][1];
    dist_second_x = dist[1][1];

    if (camera_x < close_std && camera_x >= 0.0)
    {
        right();
        if (is_detectived)
            last_dir = 1;
        return;
    }
    else if (camera_x > map_width - close_std && camera_x <= map_width)
    {
        left();
        if (is_detectived)
            last_dir = -1;
        return;
    }

    if (!is_detectived)
    {
        last_dir = 0;
        if (cur_state == "L")
        {
            if (camera_x < map_width - close_std && camera_x >= 0)
            {
                forward();
                return;
            }
            left();
            return;
        }
        else if (cur_state == "R")
        {
            if (camera_x > close_std && camera_x <= map_width)
            {
                forward();
                return;
            }
            right();
            return;
        }

        forward();
        return;
    }

    if (is_detectived)
    {
        float escape_std = 0.22;
        float last_dir_escape_std = 0.28;
        if (last_dir == -1)
        {
            if (dist_x <= map_width && dist_x > last_dir_escape_std)
            {
                last_dir = 0;
                forward();
                return;
            }
            left();
            return;
        }
        if (last_dir == 1)
        {
            if (dist_x >= -map_width && dist_x < -last_dir_escape_std)
            {
                last_dir = 0;
                forward();
                return;
            }
            right();
            return;
        }

        double two_std = 0.55;
        if (dist_x >= 0 && dist_x < escape_std)
        {
            // 2개 연속 && 2개 다 인식된 경우 빈 곳으로 이동
            if (dist_second_x >= -two_std && dist_second_x < 0)
            {
                right();
                last_dir = 1;
                return;
            }
            left();
        }
        else if (dist_x < 0 && dist_x >= -escape_std)
        {
            if (dist_second_x <= two_std && dist_second_x > 0)
            {
                right();
                last_dir = -1;
                return;
            }
            right();
        }
    }
    else
        forward();
}

// -----
// state
void TaskPlanner::update_state()
{
    // 상태 변경 시 정지
    static std::string prev_state = "NONE";
    static int pause_cnt = 0;
    const int pause_period = 20;

    if (prev_state == "NONE")
        prev_state = cur_state;

    if (cur_state != prev_state)
    {
        pause_cnt = pause_period;
        prev_state = cur_state;
    }

    M2Ik msg;

    if (pause_cnt > 0)
    {
        pause_cnt--;
        msg.x_length = 0.0;
        msg.y_length = 0.0;
        msg.yaw = 0.0;
        msg.flag = 0;
    }
    else
    {
        msg.x_length = x_length;
        msg.y_length = y_length;
        msg.yaw = yaw;
        msg.flag = flag;
    }

    RCLCPP_INFO(this->get_logger(), "x_speed: %.2f, y_speed: %.2f || x_position: %.3f, dist_x: %.2f || is_d: %d",
                msg.x_length, msg.y_length, camera_x, dist_x, detectived);

    Master2Ik_pub->publish(msg);
}

void TaskPlanner::forward()
{
    // x_length = speed; // 20
    x_length = F_Min_Test_X;
    y_length = 0;
    yaw = 0;
    flag = 1;
    cur_state = "F";
}
void TaskPlanner::backward()
{
    // x_length = -speed;
    x_length = B_Test_x;
    y_length = 0;
    yaw = 0;
    flag = 1;
    cur_state = "B";
}
void TaskPlanner::left()
{
    x_length = -0.2;
    // y_length = speed;
    // x_length = L_Test_x;
    y_length = L_Test_side;
    yaw = 0;
    flag = 1;
    cur_state = "L";
}
void TaskPlanner::right()
{
    x_length = -0.2;
    // y_length = -speed;
    // x_length = R_Test_x;
    y_length = R_Test_side;
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