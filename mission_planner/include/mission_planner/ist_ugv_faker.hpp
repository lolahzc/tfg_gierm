#ifndef IST_UGV_FAKER_HPP
#define IST_UGV_FAKER_HPP

#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mission_planner/action/request_mobile_charging_station.hpp"
#include "mission_planner/action/do_closer_inspection.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"

class ISTugvFaker : public rclcpp::Node
{
public:
    using RequestMobileChargingStationAction = mission_planner::action::RequestMobileChargingStation;
    using GoalHandleMobileStation = rclcpp_action::ServerGoalHandle<RequestMobileChargingStationAction>;
    
    using DoCloserInspectionAction = mission_planner::action::DoCloserInspection;
    using GoalHandleCloserInspection = rclcpp_action::ServerGoalHandle<DoCloserInspectionAction>;

    ISTugvFaker();
    ~ISTugvFaker() = default;

private:
    // Action servers
    rclcpp_action::Server<RequestMobileChargingStationAction>::SharedPtr mobile_station_as_;
    rclcpp_action::Server<DoCloserInspectionAction>::SharedPtr closer_inspection_as_;

    // Publishers
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr atrvjr_pose_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr jackal_pose_pub_;
    
    // Messages
    geographic_msgs::msg::GeoPoseStamped atrvjr_pose_;
    geographic_msgs::msg::GeoPoseStamped jackal_pose_;
    geographic_msgs::msg::GeoPoseStamped north_position_;
    geographic_msgs::msg::GeoPoseStamped south_position_;
    geographic_msgs::msg::GeoPoseStamped center_;

    // Movement parameters
    float latitude_increment_;
    float angular_velocity_;
    float angle_increment_;
    float current_angle_;
    std::string atrvjr_direction_;
    std::string jackal_direction_;

    // Timer for position updates
    rclcpp::TimerBase::SharedPtr timer_;

    void UGVPositionUpdater();
    void publishPositions();

    // Action server callbacks
    rclcpp_action::GoalResponse handle_mobile_station_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const RequestMobileChargingStationAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_mobile_station_cancel(
        const std::shared_ptr<GoalHandleMobileStation> goal_handle);
    
    void handle_mobile_station_accepted(
        const std::shared_ptr<GoalHandleMobileStation> goal_handle);

    rclcpp_action::GoalResponse handle_closer_inspection_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DoCloserInspectionAction::Goal> goal);
    
    rclcpp_action::CancelResponse handle_closer_inspection_cancel(
        const std::shared_ptr<GoalHandleCloserInspection> goal_handle);
    
    void handle_closer_inspection_accepted(
        const std::shared_ptr<GoalHandleCloserInspection> goal_handle);
};

#endif // IST_UGV_FAKER_HPP