#include "mission_planner/ist_ugv_faker.hpp"

#include <memory>
#include <string>

ISTugvFaker::ISTugvFaker() 
    : Node("ist_ugv_faker"),
      atrvjr_direction_("right"),
      jackal_direction_("up"),
      current_angle_(0.0f)
{
    // Initialize action servers
    this->mobile_station_as_ = rclcpp_action::create_server<RequestMobileChargingStationAction>(
        this,
        "/jackal0/cooperation_use/request_mobile_charging_station",
        std::bind(&ISTugvFaker::handle_mobile_station_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ISTugvFaker::handle_mobile_station_cancel, this, std::placeholders::_1),
        std::bind(&ISTugvFaker::handle_mobile_station_accepted, this, std::placeholders::_1));

    this->closer_inspection_as_ = rclcpp_action::create_server<DoCloserInspectionAction>(
        this,
        "/atrvjr/cooperation_use/do_closer_inspection",
        std::bind(&ISTugvFaker::handle_closer_inspection_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ISTugvFaker::handle_closer_inspection_cancel, this, std::placeholders::_1),
        std::bind(&ISTugvFaker::handle_closer_inspection_accepted, this, std::placeholders::_1));

    // Initialize publishers
    atrvjr_pose_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/atrvjr/geopose", 1);
    jackal_pose_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/jackal0/geopose", 1);

    // Define north and south points for jackal trajectory
    // Latitude  - y
    // Longitude - x
    // Altitude  - z
    north_position_.pose.position.latitude = 38.54121161489917;
    north_position_.pose.position.longitude = -7.961711602856107;
    north_position_.pose.position.altitude = 230.1384181595744;
    
    south_position_.pose.position.latitude = 38.54106489217978;
    south_position_.pose.position.longitude = -7.961711602856107;
    south_position_.pose.position.altitude = 230.1384181595744;

    // Define center coordinates for atrvjr circular trajectory
    center_.pose.position.latitude = 38.5410856346297;
    center_.pose.position.longitude = -7.961538763126298;
    center_.pose.position.altitude = 230.0598269717272;

    // Compute the circular trajectory parameters
    angular_velocity_ = 2 * 3.141592654f / 30.0f; // w = 2 * pi / T
    angle_increment_ = angular_velocity_ * 0.333f; // Δθ = w * Δt (0.333 seconds for 3Hz)

    // Compute the latitude increment per position update interval
    // 15 meters, let's say 7.5 seconds to travel those 15 meters, 0.333 seconds per position update
    // 22.5 updates per direction (7.5 / 0.333)
    latitude_increment_ = (north_position_.pose.position.latitude - south_position_.pose.position.latitude) / 22.5f;

    // Initial position: south
    jackal_pose_.pose.position.latitude = south_position_.pose.position.latitude;
    jackal_pose_.pose.position.longitude = south_position_.pose.position.longitude;
    jackal_pose_.pose.position.altitude = south_position_.pose.position.altitude;

    atrvjr_pose_.pose.position.altitude = center_.pose.position.altitude;

    // Timer for position updates (3Hz = 0.333 seconds)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(333),
        std::bind(&ISTugvFaker::publishPositions, this));

    RCLCPP_INFO(this->get_logger(), "IST UGV Faker initialized");
}

void ISTugvFaker::publishPositions()
{
    UGVPositionUpdater();
    atrvjr_pose_pub_->publish(atrvjr_pose_);
    jackal_pose_pub_->publish(jackal_pose_);
}

void ISTugvFaker::UGVPositionUpdater()
{
    // Jackal will move in a vertical line in the central path at the north of the origin
    if (jackal_direction_ == "up") {
        jackal_pose_.pose.position.latitude = jackal_pose_.pose.position.latitude + latitude_increment_;
        if (jackal_pose_.pose.position.latitude > north_position_.pose.position.latitude) {
            jackal_direction_ = "down";
        }
    }
    else {
        jackal_pose_.pose.position.latitude = jackal_pose_.pose.position.latitude - latitude_increment_;
        if (jackal_pose_.pose.position.latitude < south_position_.pose.position.latitude) {
            jackal_direction_ = "up";
        }
    }

    // atrvjr will move in a circle around a coordinate center point
    current_angle_ += angle_increment_;
    if (current_angle_ > 2 * 3.141592654f) {
        current_angle_ = 0.0f;
    }
    atrvjr_pose_.pose.position.latitude = center_.pose.position.latitude + 0.00005f * std::sin(current_angle_);
    atrvjr_pose_.pose.position.longitude = center_.pose.position.longitude + 0.00005f * std::cos(current_angle_);
}

// Mobile Station Action Server Callbacks
rclcpp_action::GoalResponse ISTugvFaker::handle_mobile_station_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RequestMobileChargingStationAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received mobile charging station request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ISTugvFaker::handle_mobile_station_cancel(
    const std::shared_ptr<GoalHandleMobileStation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel mobile charging station");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ISTugvFaker::handle_mobile_station_accepted(
    const std::shared_ptr<GoalHandleMobileStation> goal_handle)
{
    // This function should execute the action, but since we're just faking success...
    auto result = std::make_shared<RequestMobileChargingStationAction::Result>();
    result->success = true;
    
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Mobile charging station request completed successfully");
}

// Closer Inspection Action Server Callbacks
rclcpp_action::GoalResponse ISTugvFaker::handle_closer_inspection_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DoCloserInspectionAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received closer inspection request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ISTugvFaker::handle_closer_inspection_cancel(
    const std::shared_ptr<GoalHandleCloserInspection> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel closer inspection");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ISTugvFaker::handle_closer_inspection_accepted(
    const std::shared_ptr<GoalHandleCloserInspection> goal_handle)
{
    // This function should execute the action, but since we're just faking success...
    auto result = std::make_shared<DoCloserInspectionAction::Result>();
    result->success = true;
    
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Closer inspection request completed successfully");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ISTugvFaker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}