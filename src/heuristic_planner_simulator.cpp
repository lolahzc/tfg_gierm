#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mission_planner/action/heuristic_planning.hpp>
#include <mission_planner/msg/task_queue.hpp>
#include <mission_planner/msg/task.hpp>
#include <map>
#include <vector>
#include <algorithm>
#include <random>

using namespace std::chrono_literals;
using HeuristicPlanning = mission_planner::action::HeuristicPlanning;
using GoalHandleHeuristicPlanning = rclcpp_action::ServerGoalHandle<HeuristicPlanning>;

class HeuristicPlannerSimulator : public rclcpp::Node
{
public:
    HeuristicPlannerSimulator()
    : Node("heuristic_planner_simulator")
    {
        // Crear servidor de acción
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

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const HeuristicPlanning::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "🎯 Received heuristic planning request with %zu agents and %zu tasks", 
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
        RCLCPP_INFO(this->get_logger(), "🔄 Executing heuristic planning simulation...");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<HeuristicPlanning::Result>();
        
        // Simular tiempo de procesamiento
        std::this_thread::sleep_for(1s);
        
        // Verificar si la acción fue cancelada
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
        
        // Generar planificación simulada
        if (simulate_heuristic_planning(goal, result)) {
            RCLCPP_INFO(this->get_logger(), "✅ Heuristic planning completed successfully");
            goal_handle->succeed(result);
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Heuristic planning failed");
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
            
            RCLCPP_INFO(this->get_logger(), "📊 Planning for %zu agents and %zu tasks", 
                       agents.size(), tasks.size());

            // CASO 1: No hay agentes disponibles
            if (agents.empty()) {
                RCLCPP_WARN(this->get_logger(), "❌ No agents available for planning");
                return false;
            }

            // CASO 2: No hay tareas pendientes - asignar tareas de espera
            if (tasks.empty()) {
                RCLCPP_INFO(this->get_logger(), "📭 No tasks available, assigning wait tasks");
                for (const auto& agent : agents) {
                    mission_planner::msg::TaskQueue queue;
                    queue.agent_id = agent;
                    
                    mission_planner::msg::Task wait_task;
                    wait_task.id = "t_W";
                    wait_task.type = 'W';
                    queue.queue.push_back(wait_task);
                    
                    result->planning_result.push_back(queue);
                    RCLCPP_INFO(this->get_logger(), "⏳ Assigned WAIT task to agent: %s", agent.c_str());
                }
                return true;
            }

            // CASO 3: Hay agentes y tareas - asignar estratégicamente
            RCLCPP_INFO(this->get_logger(), "🎯 Assigning %zu tasks to %zu agents", tasks.size(), agents.size());
            
            // Estrategia simple: asignar tareas round-robin a los agentes
            size_t tasks_assigned = 0;
            
            for (size_t i = 0; i < agents.size() && tasks_assigned < tasks.size(); ++i) {
                mission_planner::msg::TaskQueue queue;
                queue.agent_id = agents[i];
                
                // Asignar al menos una tarea a cada agente
                mission_planner::msg::Task task;
                task.id = tasks[tasks_assigned];
                task.type = determine_task_type(tasks[tasks_assigned]);
                queue.queue.push_back(task);
                
                result->planning_result.push_back(queue);
                RCLCPP_INFO(this->get_logger(), "✅ Assigned task '%s' to agent '%s'", 
                           tasks[tasks_assigned].c_str(), agents[i].c_str());
                
                tasks_assigned++;
            }

            // Si sobran tareas, asignarlas a los agentes de forma balanceada
            while (tasks_assigned < tasks.size()) {
                for (size_t i = 0; i < agents.size() && tasks_assigned < tasks.size(); ++i) {
                    mission_planner::msg::Task additional_task;
                    additional_task.id = tasks[tasks_assigned];
                    additional_task.type = determine_task_type(tasks[tasks_assigned]);
                    
                    result->planning_result[i].queue.push_back(additional_task);
                    RCLCPP_INFO(this->get_logger(), "➕ Additional task '%s' to agent '%s'", 
                               tasks[tasks_assigned].c_str(), agents[i].c_str());
                    
                    tasks_assigned++;
                }
            }

            // Si sobran agentes, asignarles tarea de espera
            for (size_t i = tasks.size(); i < agents.size(); ++i) {
                mission_planner::msg::TaskQueue queue;
                queue.agent_id = agents[i];
                
                mission_planner::msg::Task wait_task;
                wait_task.id = "t_W";
                wait_task.type = 'W';
                queue.queue.push_back(wait_task);
                
                result->planning_result.push_back(queue);
                RCLCPP_INFO(this->get_logger(), "⏳ Assigned WAIT task to extra agent: %s", agents[i].c_str());
            }

            result->success = true;
            RCLCPP_INFO(this->get_logger(), "🎉 Generated plan with %zu agent queues", 
                       result->planning_result.size());
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "💥 Exception in heuristic planning: %s", e.what());
            return false;
        }
    }

    char determine_task_type(const std::string& task_id)
    {
        // Determinar el tipo de tarea basado en el ID
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
    RCLCPP_INFO(node->get_logger(), "🚀 Heuristic Planner Simulator started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}