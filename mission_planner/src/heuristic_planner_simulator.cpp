#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mission_planner/action/heuristic_planning.hpp>
#include <mission_planner/msg/task_queue.hpp>
#include <mission_planner/msg/task.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <map>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;
using HeuristicPlanning = mission_planner::action::HeuristicPlanning;
using GoalHandleHeuristicPlanning = rclcpp_action::ServerGoalHandle<HeuristicPlanning>;

namespace
{
double euclideanDistance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// How many meters of extra travel one already-queued task is "worth" when
// weighing a candidate agent. Keeps assignment from piling every task on
// whichever single agent happens to start closest to all of them.
constexpr double kLoadPenaltyMeters = 15.0;
}  // namespace

class HeuristicPlannerSimulator : public rclcpp::Node
{
public:
    HeuristicPlannerSimulator()
    : Node("heuristic_planner_simulator")
    {
        // Create the action server
        this->action_server_ = rclcpp_action::create_server<HeuristicPlanning>(
            this,
            "heuristic_planning",
            std::bind(&HeuristicPlannerSimulator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&HeuristicPlannerSimulator::handle_cancel, this, std::placeholders::_1),
            std::bind(&HeuristicPlannerSimulator::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Heuristic Planner Simulator initialized and waiting for goals...");
    }

private:
    rclcpp_action::Server<HeuristicPlanning>::SharedPtr action_server_;
    std::map<std::string, std::string> task_to_agent_map_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const HeuristicPlanning::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received heuristic planning request with %zu agents and %zu tasks",
                   goal->available_agents.size(), goal->remaining_tasks.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleHeuristicPlanning> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel heuristic planning");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleHeuristicPlanning> goal_handle)
    {
        std::thread{std::bind(&HeuristicPlannerSimulator::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleHeuristicPlanning> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing heuristic planning simulation...");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<HeuristicPlanning::Result>();
        
        // Simular tiempo de procesamiento
        std::this_thread::sleep_for(1s);
        
        // Check whether the action was cancelled
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Heuristic planning canceled");
            return;
        }
        
        // Publicar feedback
        auto feedback = std::make_shared<HeuristicPlanning::Feedback>();
        feedback->status = "Processing tasks...";
        goal_handle->publish_feedback(feedback);
        
        // Produce the simulated plan
        if (simulate_heuristic_planning(goal, result)) {
            RCLCPP_INFO(this->get_logger(), "Heuristic planning completed successfully");
            goal_handle->succeed(result);
        } else {
            RCLCPP_WARN(this->get_logger(), "Heuristic planning failed");
            result->success = false;
            goal_handle->succeed(result);
        }
    }

    bool simulate_heuristic_planning(
        const std::shared_ptr<const HeuristicPlanning::Goal> goal,
        std::shared_ptr<HeuristicPlanning::Result> result)
    {
        try {
            std::vector<std::string> agents = goal->available_agents;
            std::vector<std::string> tasks = goal->remaining_tasks;
            
            RCLCPP_INFO(this->get_logger(), "Planning for %zu agents and %zu tasks",
                       agents.size(), tasks.size());

            // ESCUDO 1: No hay agentes
            if (agents.empty()) {
                RCLCPP_WARN(this->get_logger(), "No agents available for planning");
                return false;
            }

            // ESCUDO 2: No hay tareas (Asignar Wait)
            if (tasks.empty()) {
                for (const auto& agent : agents) {
                    mission_planner::msg::TaskQueue queue;
                    queue.agent_id = agent;
                    mission_planner::msg::Task wait_task;
                    wait_task.id = "t_W";
                    wait_task.type = 'W';
                    queue.queue.push_back(wait_task);
                    result->planning_result.push_back(queue);
                }
                return true;
            }

            // --- REPARTO: MEMORIA + DISTANCIA + EQUILIBRIO DE CARGA ---
            std::map<std::string, mission_planner::msg::TaskQueue> agent_queues;
            std::map<std::string, int> agent_task_count;
            std::map<std::string, geometry_msgs::msg::Point> agent_position;
            std::map<std::string, geometry_msgs::msg::Point> task_position;

            for (const auto& agent_id : agents) {
                agent_queues[agent_id].agent_id = agent_id;
                agent_task_count[agent_id] = 0; // everyone starts with no tasks
            }
            for (size_t i = 0; i < agents.size() && i < goal->agent_positions.size(); ++i) {
                agent_position[agents[i]] = goal->agent_positions[i];
            }
            for (size_t i = 0; i < tasks.size() && i < goal->task_positions.size(); ++i) {
                task_position[tasks[i]] = goal->task_positions[i];
            }

            // 1. Reuse earlier assignments, but only while that agent is
            //    still available; otherwise the task is treated as new below,
            //    so it cannot stay bound to a disconnected agent.
            for (const auto& task_id : tasks) {
                auto remembered = task_to_agent_map_.find(task_id);
                if (remembered != task_to_agent_map_.end() && agent_task_count.count(remembered->second)) {
                    agent_task_count[remembered->second]++;
                }
            }

            // 2. Assign new or orphaned tasks by cost = distance + load
            for (const auto& task_id : tasks) {
                auto remembered = task_to_agent_map_.find(task_id);
                bool has_live_owner = remembered != task_to_agent_map_.end() &&
                                       agent_task_count.count(remembered->second);

                if (!has_live_owner) {
                    bool have_task_pos = task_position.count(task_id) > 0;
                    double best_cost = std::numeric_limits<double>::max();
                    std::vector<std::string> best_agents;

                    for (const auto& agent_id : agents) {
                        double cost = static_cast<double>(agent_task_count[agent_id]) * kLoadPenaltyMeters;
                        if (have_task_pos && agent_position.count(agent_id)) {
                            cost += euclideanDistance(agent_position[agent_id], task_position[task_id]);
                        }
                        if (cost < best_cost - 1e-6) {
                            best_cost = cost;
                            best_agents = {agent_id};
                        } else if (std::abs(cost - best_cost) <= 1e-6) {
                            best_agents.push_back(agent_id);
                        }
                    }

                    // Break exact ties deterministically (same task always picks the
                    // same agent among equally-good candidates).
                    std::seed_seq seed(task_id.begin(), task_id.end());
                    std::mt19937 gen(seed);
                    std::uniform_int_distribution<> distrib(0, static_cast<int>(best_agents.size()) - 1);
                    std::string chosen_agent = best_agents[distrib(gen)];

                    task_to_agent_map_[task_id] = chosen_agent;
                    agent_task_count[chosen_agent]++;

                    RCLCPP_INFO(this->get_logger(), "New task '%s' assigned to '%s' (cost %.2f)",
                                task_id.c_str(), chosen_agent.c_str(), best_cost);
                }

                // 3. Push the task onto that drone's queue
                std::string current_agent = task_to_agent_map_[task_id];
                mission_planner::msg::Task t;
                t.id = task_id;
                t.type = determine_task_type(task_id);
                agent_queues[current_agent].queue.push_back(t);
            }

            // 4. Pack the result
            for (const auto& agent_id : agents) {
                if (agent_queues[agent_id].queue.empty()) {
                    mission_planner::msg::Task wait_task;
                    wait_task.id = "t_W";
                    wait_task.type = 'W';
                    agent_queues[agent_id].queue.push_back(wait_task);
                }
                result->planning_result.push_back(agent_queues[agent_id]);
            }

            result->success = true;
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return false;
        }
    }

    char determine_task_type(const std::string& task_id)
    {
        // Infer the task type from its id
        if (task_id.find("monitor") != std::string::npos || task_id.find("M_") != std::string::npos) {
            return 'M';
        } else if (task_id.find("inspect") != std::string::npos || task_id.find("I_") != std::string::npos) {
            return 'I';
        } else if (task_id.find("deliver") != std::string::npos || task_id.find("D_") != std::string::npos) {
            return 'D';
        } else if (task_id.find("array") != std::string::npos || task_id.find("A_") != std::string::npos) {
            return 'A';
        } else if (task_id.find("recharge") != std::string::npos || task_id.find("R_") != std::string::npos) {
            return 'R';
        } else {
            return 'M'; // Por defecto Monitor
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeuristicPlannerSimulator>();
    RCLCPP_INFO(node->get_logger(), "Heuristic Planner Simulator started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}