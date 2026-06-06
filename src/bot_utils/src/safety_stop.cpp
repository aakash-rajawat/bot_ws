#include "bot_utils/safety_stop.hpp"


SafetyStop::SafetyStop(const std::string& name)
    : Node(name),
    m_state(State::FREE),
    m_prev_state(State::FREE),
    m_is_first_msg(true)
{
    declare_parameter<double>("distance_danger", 0.3);
    m_dist_danger = get_parameter("distance_danger").as_double();

    declare_parameter<double>("distance_warning", 0.6);
    m_dist_warn = get_parameter("distance_warning").as_double();

    declare_parameter<std::string>("scan_topic", "scan");
    m_scan_topic = get_parameter("scan_topic").as_string();

    declare_parameter<std::string>("safety_stop_topic", "safety_stop");
    m_safety_stop_topic = get_parameter("safety_stop_topic").as_string();

    constexpr int subQOS {10};
    m_laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
        m_scan_topic,
        subQOS,
        std::bind(&SafetyStop::lidarCallback, this, std::placeholders::_1)
    );

    constexpr int pubQOS {10};
    m_safety_stop_pub = create_publisher<std_msgs::msg::Bool>(
        m_safety_stop_topic,
        pubQOS
    );

    m_decrease_speed_client = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(
        this,
        "joy_turbo_decrease"
    );

    m_increase_speed_client = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(
        this,
        "joy_turbo_increase"
    );

    while(!m_decrease_speed_client->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok())
    {
        RCLCPP_WARN(get_logger(), "Action /joy_turbo_decrease not available. Waiting for server...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    while(!m_increase_speed_client->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok())
    {
        RCLCPP_WARN(get_logger(), "Action /joy_turbo_increase not available. Waiting for server...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    m_zones_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lidar_zones",
        pubQOS
    );

    visualization_msgs::msg::Marker zone_warning {};
    zone_warning.id = 0;
    zone_warning.action = visualization_msgs::msg::Marker::ADD;
    zone_warning.type = visualization_msgs::msg::Marker::CYLINDER;
    zone_warning.scale.z = 0.001;
    zone_warning.scale.x = m_dist_warn * 2;
    zone_warning.scale.y = m_dist_warn * 2;
    zone_warning.color.r = 0.72;
    zone_warning.color.g = 0.60;
    zone_warning.color.b = 0.20;
    zone_warning.color.a = 0.40;
    m_zones.markers.push_back(zone_warning);

    visualization_msgs::msg::Marker zone_danger {};
    zone_danger.id = 1;
    zone_danger.action = visualization_msgs::msg::Marker::ADD;
    zone_danger.type = visualization_msgs::msg::Marker::CYLINDER;
    zone_danger.scale.z = 0.001;
    zone_danger.scale.x = m_dist_danger * 2;
    zone_danger.scale.y = m_dist_danger * 2;
    zone_danger.color.r = 0.55;
    zone_danger.color.g = 0.18;
    zone_danger.color.b = 0.18;
    zone_danger.color.a = 0.40;
    m_zones.markers.push_back(zone_danger);
}


void SafetyStop::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    m_state = State::FREE;
    for(const auto & range : msg->ranges)
    {
        if(!std::isinf(range) && range <= m_dist_warn)
        {
            m_state = State::WARNING;
            if(range <= m_dist_danger)
            {
                m_state = State::DANGER;
                break;
            }
            
        }
    }

    if(m_state != m_prev_state)
    {
        std_msgs::msg::Bool is_safety_stop {};
        if(m_state == State::WARNING)
        {
            is_safety_stop.data = false;
            m_zones.markers.at(0).color.a = 0.8;
            m_zones.markers.at(1).color.a = 0.4;
            m_decrease_speed_client->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
        }
        else if(m_state == State::DANGER)
        {
            m_zones.markers.at(0).color.a = 0.8;
            m_zones.markers.at(1).color.a = 0.8;
            is_safety_stop.data = true;

        }
        else if (m_state == State::FREE)
        {
            m_zones.markers.at(0).color.a = 0.4;
            m_zones.markers.at(1).color.a = 0.4;
            is_safety_stop.data = false;
            m_increase_speed_client->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
        }

        m_prev_state = m_state;
        m_safety_stop_pub->publish(is_safety_stop);
    }

    if(m_is_first_msg)
    {
        for(auto & zone : m_zones.markers)
        {
            zone.header.frame_id = msg->header.frame_id;
        }
        m_is_first_msg = false;
    }
    m_zones_pub->publish(m_zones);
}

    


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyStop>("safety_stop"));
    rclcpp::shutdown();

    return 0;
}
