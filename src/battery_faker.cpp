#include "mission_planner/battery_faker.hpp"


#include <chrono>
#include <thread>

BatteryFaker::BatteryFaker() 
  : Node("battery_faker"),
    loop_rate_(0.2),
    // battery_increase_(0.001f),
    // battery_decrease_(0.001f)
    battery_increase_(0.05f),
    battery_decrease_(0.02f)
{
    // Declare parameters
    this->declare_parameter<std::string>("battery_mode", "static");
    // Rates per 500 ms tick. With the defaults a drone goes from 100% to the
    // 30% low-battery threshold after ~87 s of flight, and back up in ~18 s.
    this->declare_parameter<double>("battery_decrease", 0.004);
    this->declare_parameter<double>("battery_increase", 0.020);
    // Trickle charge for a drone stranded away from a station. At 0 it is
    // disabled and such a drone can never recover.
    this->declare_parameter<double>("rescue_trickle", 0.0015);
    this->declare_parameter<std::string>("id", "drone0");
    this->declare_parameter<std::string>("pose_topic", "");
    this->declare_parameter<std::string>("state_topic", "");
    this->declare_parameter<std::string>("config_file", "");

    // Read the parameters first
    this->get_parameter("battery_mode", battery_mode_);
    {
      double dec = 0.004, inc = 0.020;
      double resc = 0.0015;
      this->get_parameter("battery_decrease", dec);
      this->get_parameter("battery_increase", inc);
      this->get_parameter("rescue_trickle", resc);
      battery_decrease_ = static_cast<float>(dec);
      battery_increase_ = static_cast<float>(inc);
      rescue_trickle_ = static_cast<float>(resc);
    }
    this->get_parameter("id", id_);
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("state_topic", state_topic_);
    this->get_parameter("config_file", config_file_);

    // Fall back to the default topics when none are given
    if (pose_topic_.empty()) {
        pose_topic_ = "/" + id_ + "/self_localization/pose";
    }
    if (state_topic_.empty()) {
        state_topic_ = "/" + id_ + "/platform/info";
    }

    // Battery mode
    if (battery_mode_ == "recharge_anywhere")
        mode_ = 1;
    else if (battery_mode_ == "recharge_in_base")
        mode_ = 2;
    else if (battery_mode_ == "only_discharge")
        mode_ = 3;
    else // static
        mode_ = 0;

    // Load the configuration file
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
    "/" + id_ + "/sensor_measurements/battery", 1);

    // Subscribers
    control_sub_ = this->create_subscription<mission_planner::msg::BatteryControl>(
        "/" + id_ + "/battery_fake/control", 1,
        std::bind(&BatteryFaker::controlCallback, this, std::placeholders::_1));

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(), 
        std::bind(&BatteryFaker::positionCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
        state_topic_, 1,
        std::bind(&BatteryFaker::platformInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Battery Faker READY for UAV: %s", id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "State topic: %s", state_topic_.c_str());
    
    battery_.percentage = 1.0f;

    // Held in a member: as a local it died with the constructor and
    // update_battery() was never called.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BatteryFaker::update_battery, this));

    RCLCPP_INFO(this->get_logger(),
        "[battery] faker running (mode=%s): discharge %.4f/tick, charge %.4f/tick (500 ms tick)",
        battery_mode_.c_str(), battery_decrease_, battery_increase_);
}

// Rescue charge ceiling: just enough to fly back to a station.
static constexpr float kRescueThreshold = 0.55f;

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
                    } else if(rescue_trickle_ > 0.0f && battery_.percentage < kRescueThreshold) {
                        // A drone stranded away from a station would never
                        // recharge, and so could never reach one. A slow
                        // trickle lets it recover enough range to fly back.
                        battery_.percentage += rescue_trickle_;
                        if(battery_.percentage > kRescueThreshold)
                            battery_.percentage = kRescueThreshold;
                        if(!rescue_announced_) {
                            RCLCPP_WARN(this->get_logger(),
                                "[battery] %s is grounded away from a station; trickle charging to %.0f%% so it can fly back",
                                id_.c_str(), kRescueThreshold * 100.0f);
                            rescue_announced_ = true;
                        }
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
    // Map AEROSTACK2 states onto the internal numbering. DISARMED counts as
    // on the ground: drones disarm after landing, and it is the only state
    // they sit in while parked at a station.
    switch (info->status.state) {
        case as2_msgs::msg::PlatformStatus::DISARMED:
            state_ = 1; // on the ground, disarmed
            break;
        case as2_msgs::msg::PlatformStatus::LANDED:
            state_ = info->armed ? 2 : 1; // on the ground
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