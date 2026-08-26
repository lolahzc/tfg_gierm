#ifndef GESTURE_RECOGNITION_FAKER_HPP
#define GESTURE_RECOGNITION_FAKER_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mission_planner/action/new_task.hpp"
#include "mission_planner/msg/waypoint.hpp"

class GestureRecognitionFaker : public rclcpp::Node
{
public:
    using NewTaskAction = mission_planner::action::NewTask;
    using GoalHandleNewTask = rclcpp_action::ClientGoalHandle<NewTaskAction>;

    explicit GestureRecognitionFaker(const std::string& task_id, int argc, char** argv);
    void execute();

private:
    std::string task_id_;
    int argc_;
    char** argv_;
    
    rclcpp_action::Client<NewTaskAction>::SharedPtr action_client_;
    
    void send_goal();
  
};

#endif // GESTURE_RECOGNITION_FAKER_HPP