#ifndef BATTERY_FAKER_H
#define BATTERY_FAKER_H

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include <string>
#include <map>

#include "mission_planner/classes.hpp"
#include "mission_planner/msg/battery_control.hpp"

#include "as2_msgs/msg/platform_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include <stdexcept> 
#include <ament_index_cpp/get_package_share_directory.hpp>  

class BatteryFaker : public rclcpp::Node
{
  private:
    std::string id_;

    std::string battery_mode_;
    int mode_;
    float battery_increase_;
    float battery_decrease_;
    int state_;

    classes::Position position_;

    std::string pose_topic_;
    std::string state_topic_;
    std::string config_file_;
    std::map<std::string, std::map<std::string, classes::Position>> known_positions_;

    // Subscribers
    rclcpp::Subscription<mission_planner::msg::BatteryControl>::SharedPtr control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr state_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    sensor_msgs::msg::BatteryState battery_;
    rclcpp::Rate loop_rate_;

    void update_battery();
    void readConfigFile(const std::string& config_file);
    void controlCallback(const mission_planner::msg::BatteryControl::SharedPtr control);
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info);

  public:
    BatteryFaker();
    ~BatteryFaker() = default;
};

#endif