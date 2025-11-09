#include "mission_planner/battery_faker.hpp"


#include <chrono>
#include <thread>

BatteryFaker::BatteryFaker() 
  : Node("battery_faker"),
    loop_rate_(0.2),
    battery_increase_(0.001f),
    battery_decrease_(0.001f)
{
    // Declarar todos los parámetros
    this->declare_parameter<std::string>("battery_mode", "static");
    this->declare_parameter<std::string>("id", "uav0");  // Valor por defecto específico
    this->declare_parameter<std::string>("pose_topic", "");
    this->declare_parameter<std::string>("state_topic", "");
    this->declare_parameter<std::string>("config_file", "");

    // Obtener parámetros PRIMERO
    this->get_parameter("battery_mode", battery_mode_);
    this->get_parameter("id", id_);
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("state_topic", state_topic_);
    this->get_parameter("config_file", config_file_);

    // Si los topics están vacíos, configurar valores por defecto
    if (pose_topic_.empty()) {
        pose_topic_ = "/" + id_ + "/self_localization/pose";
    }
    if (state_topic_.empty()) {
        state_topic_ = "/" + id_ + "/platform/info";
    }

    // Determinar modo de batería
    if (battery_mode_ == "recharge_anywhere")
        mode_ = 1;
    else if (battery_mode_ == "recharge_in_base")
        mode_ = 2;
    else if (battery_mode_ == "only_discharge")
        mode_ = 3;
    else // static
        mode_ = 0;

    // Cargar archivo de configuración
    if (config_file_.empty()) {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("mission_planner");
            config_file_ = package_share_dir + "/config/conf.yaml";
            RCLCPP_INFO(this->get_logger(), "Found package share directory: %s", package_share_dir.c_str());
        } catch (const std::exception& e)  {
            RCLCPP_ERROR(this->get_logger(), "Package 'mission_planner' not found: %s", e.what());
        }
    }

    RCLCPP_INFO(this->get_logger(), "Reading config file: %s", config_file_.c_str());
    readConfigFile(config_file_);

    // Publishers
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/" + id_ + "/battery_fake", 1);

    // Subscribers
    control_sub_ = this->create_subscription<mission_planner::msg::BatteryControl>(
        "/" + id_ + "/battery_fake/control", 1,
        std::bind(&BatteryFaker::controlCallback, this, std::placeholders::_1));

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 1,
        std::bind(&BatteryFaker::positionCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
        state_topic_, 1,
        std::bind(&BatteryFaker::platformInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Battery Faker READY for UAV: %s", id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "State topic: %s", state_topic_.c_str());
    
    battery_.percentage = 1.0f;

    // Timer para actualizar la batería
    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BatteryFaker::update_battery, this));
}

void BatteryFaker::update_battery() {
    int flag;
    switch(mode_) {
        case 1: // Recharge anywhere when landed
            switch(state_) {
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                    battery_.percentage = battery_.percentage + battery_increase_;
                    if(battery_.percentage > 1.0f)
                        battery_.percentage = 1.0f;
                    break;
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                default:
                    break;
            }
            break;
        case 2: // Recharge only in recharging base
            switch(state_) {
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                    flag = 0;
                    for(auto& charging_station : known_positions_["charging_stations"]) {
                        if(classes::distance(position_, charging_station.second) < 0.5f)
                            flag = 1;
                    }
                    if(flag) {
                        battery_.percentage = battery_.percentage + battery_increase_;
                        if(battery_.percentage > 1.0f)
                            battery_.percentage = 1.0f;
                    }
                    break;
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                default:
                    break;
            }
            break;
        case 3: // Recharge disabled
            switch(state_) {
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                default:
                    break;
            }
            break;
        case 0: // Battery Static
        default:
            break;
    }
    
    battery_.header.stamp = this->now();
    battery_pub_->publish(battery_);
    
    // RCLCPP_DEBUG(this->get_logger(), "Mode: %d\tUAV State: %d\tPercentage: %.3f", 
    //              mode_, state_, battery_.percentage);
}

void BatteryFaker::readConfigFile(const std::string& config_file) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file);
        if(yaml_config["positions"]) {
            for(auto const& group : yaml_config["positions"]) {
                for(auto const& position : group.second) {
                    known_positions_[group.first.as<std::string>()][position.first.as<std::string>()] = 
                        classes::Position(position.first.as<std::string>(),
                                        position.second['x'].as<float>(),
                                        position.second['y'].as<float>(),
                                        position.second['z'].as<float>());
                }
            }
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading config file: %s", e.what());
    }
}

void BatteryFaker::controlCallback(const mission_planner::msg::BatteryControl::SharedPtr control) {
    if(control->percentage != -1.0f)
        battery_.percentage = control->percentage;
    mode_ = control->mode;
    if(control->battery_increase >= 0.0f)
        battery_increase_ = control->battery_increase;
    if(control->battery_decrease >= 0.0f)
        battery_decrease_ = control->battery_decrease;
}

void BatteryFaker::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    position_.update(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
}

void BatteryFaker::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info) {
    // Mapear estados de Aerostack2 a los estados originales
    switch (info->status.state) {
        case as2_msgs::msg::PlatformStatus::LANDED:
            state_ = 1; // LANDED_DISARMED
            break;
        case as2_msgs::msg::PlatformStatus::TAKING_OFF:
            state_ = 3; // TAKING_OFF
            break;
        case as2_msgs::msg::PlatformStatus::FLYING:
            state_ = 4; // FLYING_AUTO
            break;
        case as2_msgs::msg::PlatformStatus::LANDING:
            state_ = 6; // LANDING
            break;
        case as2_msgs::msg::PlatformStatus::EMERGENCY:
            state_ = 5; // FLYING_MANUAL
            break;
        default:
            state_ = 0; // UNINITIALIZED
            break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryFaker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}