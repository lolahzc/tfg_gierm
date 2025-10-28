#include "mission_planner/gesture_recognition_faker.hpp"

#include <chrono>
#include <future>
#include <memory>

GestureRecognitionFaker::GestureRecognitionFaker(const std::string& task_id, int argc, char** argv)
    : Node("gesture_recognition_faker_" + task_id),
      task_id_(task_id),
      argc_(argc),
      argv_(argv)
{
    this->action_client_ = rclcpp_action::create_client<NewTaskAction>(
        this, "incoming_task_action");
}

void GestureRecognitionFaker::execute()
{
    // Wait for action server
    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    send_goal();
}

void GestureRecognitionFaker::send_goal()
{
    auto goal_msg = NewTaskAction::Goal();
    bool error = true;
    int i;

    if (argc_ > 3) {
        goal_msg.task.id = task_id_;
        goal_msg.task.type = argv_[2][0];  // Get first character
        
        switch(goal_msg.task.type) {
            case 'M':
            case 'm':
                if (argc_ == 6) {
                    goal_msg.task.monitor.human_target_id = argv_[3];
                    goal_msg.task.monitor.distance = std::atof(argv_[4]);
                    goal_msg.task.monitor.number = std::atoi(argv_[5]);
                    error = false;
                }
                break;
            case 'F':
            case 'f':
                if (argc_ == 5) {
                    goal_msg.task.monitor_ugv.ugv_id = argv_[3];
                    goal_msg.task.monitor_ugv.height = std::atof(argv_[4]);
                    error = false;
                }
                break;
            case 'I':
            case 'i':
                i = 3;
                while (i + 2 < argc_) {
                    mission_planner::msg::Waypoint waypoint;
                    waypoint.x = std::atof(argv_[i]);
                    waypoint.y = std::atof(argv_[i + 1]);
                    waypoint.z = std::atof(argv_[i + 2]);

                    goal_msg.task.inspect.waypoints.push_back(waypoint);
                    i += 3;
                }
                if (!goal_msg.task.inspect.waypoints.empty()) {
                    error = false;
                }
                break;
            case 'A':
            case 'a':
                i = 3;
                while (i + 2 < argc_) {
                    mission_planner::msg::Waypoint waypoint;
                    waypoint.x = std::atof(argv_[i]);
                    waypoint.y = std::atof(argv_[i + 1]);
                    waypoint.z = std::atof(argv_[i + 2]);

                    goal_msg.task.inspect.waypoints.push_back(waypoint);
                    i += 3;
                }
                if (!goal_msg.task.inspect.waypoints.empty()) {
                    error = false;
                }
                break;
            case 'D':
            case 'd':
                if (argc_ == 5) {
                    goal_msg.task.deliver.tool_id = argv_[3];
                    goal_msg.task.deliver.human_target_id = argv_[4];
                    error = false;
                }
                break;
            default:
                break;
        }
    }

    if (error) {
        RCLCPP_INFO(this->get_logger(), "Usage: gesture_recognition_faker task_id task_type task_params");
        RCLCPP_INFO(this->get_logger(), "task_params defined in msg/{monitor, inspect, deliver_tool}_params");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal to action server");

    auto send_goal_options = rclcpp_action::Client<NewTaskAction>::SendGoalOptions();
    
    // Configurar los callbacks usando lambdas
    send_goal_options.goal_response_callback =
        [this](auto future) {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

    send_goal_options.feedback_callback =
        [this](auto goal_handle, auto feedback) {
            RCLCPP_DEBUG(this->get_logger(), "Received feedback");
        };

    send_goal_options.result_callback =
        [this](const auto & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "New Task sent successfully");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
            }
            rclcpp::shutdown();
        };

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " task_id task_type task_params" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    
    std::string task_id = argv[1];
    auto node = std::make_shared<GestureRecognitionFaker>(task_id, argc, argv);
    
    node->execute();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}