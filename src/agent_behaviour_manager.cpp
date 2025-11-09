#include "mission_planner/agent_behaviour_manager.hpp"

#include <chrono>
#include <thread>

// Behavior Tree structure xml definition
static const char* behaviour_tree_xml = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
    </BehaviorTree>
</root>
 )";

// Behavior Tree Nodes implementation ***********************************************************************************

// ******************************* Actions {

// GoNearChargingStation {
GoNearChargingStation::GoNearChargingStation(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
GoNearChargingStation::~GoNearChargingStation(){halt();}
void GoNearChargingStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearChargingStation::providedPorts() {return{};}
BT::NodeStatus GoNearChargingStation::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(agent_->get_logger(), "Failed to call stop");

  classes::Task* task;
  classes::Position assigned_charging_station;

  std::string nearest_station;
  float distance = -1;
  float tmp_distance;

  std::string aux = "";

  /***************************************** TODO: TO BE IMPROVED ***************************************************/
  nearest_station = "charging_station_" + agent_->id_;

  // Emergency Recharging
  if(agent_->task_queue_.empty())
    assigned_charging_station = agent_->known_positions_["charging_stations"][nearest_station];
  // Recharge Task
  else
  {
    task = agent_->task_queue_.front();
    if(task->getType() != 'R')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(agent_->get_logger(), "[GoNearChargingStation] First task of the queue isn't type Recharge");
      return BT::NodeStatus::FAILURE;
    }
    assigned_charging_station = task->getChargingStation();

    // Assign and reserve this charging station for this Agent
    if(assigned_charging_station.getID().empty())
    {
      task->setChargingStation(&(agent_->known_positions_["charging_stations"][nearest_station]));
      assigned_charging_station = task->getChargingStation();
    }
  }

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: // LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(agent_->get_logger(), "[GoNearChargingStation] Calling take_off");
        // Aerostack2 take_off action call
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(agent_->get_logger(), "[GoNearChargingStation] Failed to call take_off");
          return BT::NodeStatus::FAILURE;
        }
        // take_off action result waiting loop
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: // FLYING_AUTO
        // Go to recharging station
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO_STREAM(agent_->get_logger(), "[GoNearChargingStation] Moving to " << aux << "recharging station (" <<
            assigned_charging_station.getX() << ", " << assigned_charging_station.getY() << ")[" <<
            agent_->pose_frame_id_.c_str() << "]");
        if(agent_->go_to_waypoint(assigned_charging_station.getX(), assigned_charging_station.getY(),
              assigned_charging_station.getZ() + 1, false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(assigned_charging_station.getX(),
                assigned_charging_station.getY(), assigned_charging_station.getZ() + 1))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          RCLCPP_INFO(agent_->get_logger(), "[GoNearChargingStation] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(agent_->get_logger(), "[GoNearChargingStation] Failed to call go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: // UNINITIALIZED
      case 1: // LANDED_DISARMED
      case 3: // TAKING_OFF
      case 5: // FLYING_MANUAL
      case 6: // LANDING
      default:
        break;
    }
  }
  return BT::NodeStatus::IDLE;
}
void GoNearChargingStation::halt(){
  RCLCPP_INFO(agent_->get_logger(), "[GoNearChargingStation] halt requested");
  BT::AsyncActionNode::halt();
}
// }

// Recharge {
Recharge::Recharge(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
Recharge::~Recharge(){halt();}
void Recharge::init(AgentNode* agent){agent_ = agent;}
BT::PortsList Recharge::providedPorts() {return{};}
BT::NodeStatus Recharge::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(agent_->get_logger(), "Failed to call stop");

  bool recharge_task = false;
  classes::Task* task;
  float final_percentage;

  auto task_result_ac_ = rclcpp_action::create_client<mission_planner::action::TaskResult>(
    agent_->get_node_base_interface(),
    agent_->get_node_graph_interface(),
    agent_->get_node_logging_interface(),
    agent_->get_node_waitables_interface(),
    "/" + agent_->beacon_.id + "/task_result");
  
  mission_planner::action::TaskResult::Goal goal;
  
  // Emergency Recharging
  if(agent_->task_queue_.empty())
    final_percentage = 0.99;
  // Recharge Task
  else
  {
    task = agent_->task_queue_.front();
    if(task->getType() != 'R')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(agent_->get_logger(), "[Recharge] First task of the queue isn't type Recharge");
      return BT::NodeStatus::FAILURE;
    }
    final_percentage = task->getFinalPercentage();
    recharge_task = true;
    goal.task.id = task->getID();
    goal.task.type = task->getType();
  }

  // TODO: Calling Recharge lower level controllers (faked) 
  RCLCPP_INFO(agent_->get_logger(), "[Recharge] Calling Lower-level controllers...");
  
  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 1: // LANDED_DISARMED
      case 2: // LANDED_ARMED
        if(isHaltRequested())
        {
          if(recharge_task)
          {
            if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
            {
              auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
            }
          }
          return BT::NodeStatus::IDLE;
        }
        RCLCPP_INFO(agent_->get_logger(), "[Recharge] Recharging...");
        while(!isHaltRequested())
        {
          if(agent_->battery_ < final_percentage)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          else
          {
            if(recharge_task)
            {
              if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
              {
                auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
              }
            }
            return BT::NodeStatus::SUCCESS;
          }
        }
        if(recharge_task)
        {
          if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
          {
            auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
          }
        }
        return BT::NodeStatus::IDLE;
        break;
      case 4: // FLYING_AUTO
        if(isHaltRequested())
        {
          if(recharge_task)
          {
            if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
            {
              auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
            }
          }
          return BT::NodeStatus::IDLE;
        }
        if(!agent_->land(false))
        {
          if(isHaltRequested())
          {
            if(recharge_task)
            {
              if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
              {
                auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
              }
            }
            return BT::NodeStatus::IDLE;
          }
          RCLCPP_ERROR(agent_->get_logger(), "[Recharge] Failed to call land");
          if(recharge_task)
          {
            if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
            {
              auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
            }
          }
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 1 && agent_->state_ != 2)
          {
            if(isHaltRequested())
            {
              if(recharge_task)
              {
                if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
                {
                  auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
                }
              }
              return BT::NodeStatus::IDLE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 0: // UNINITIALIZED
      case 3: // TAKING_OFF
      case 5: // FLYING_MANUAL
      case 6: // LANDING
      default:
        break;
    }
  }

  if(recharge_task)
  {
    if(agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1)))
    {
      auto goal_handle_future = agent_->task_result_ac_->async_send_goal(goal);
    }
  }
  return BT::NodeStatus::IDLE;
}
void Recharge::halt(){
  RCLCPP_INFO(agent_->get_logger(), "[Recharge] halt requested");
  BT::AsyncActionNode::halt();
}
// }

// BackToStation {
BackToStation::BackToStation(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
BackToStation::~BackToStation(){halt();}
void BackToStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList BackToStation::providedPorts() {return{};}
BT::NodeStatus BackToStation::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(agent_->get_logger(), "Failed to call stop");
    
  RCLCPP_INFO(agent_->get_logger(), "[BackToStation] Emptying the Queue...");
  agent_->emptyTheQueue();

  int flag = 0;
  std::string nearest_station = "station_" + agent_->id_;

  while(!isHaltRequested())
  {
    // If Agent is already in station. Land if needed and back_to_station
    for(auto& station : agent_->known_positions_["stations"])
    {
      if(classes::distance2D(agent_->position_, station.second) < agent_->distance_error_)
        flag = 1;
    }
    if(flag)
    {
      switch(agent_->state_)
      {
        case 1: // LANDED_DISARMED
        case 2: // LANDED_ARMED
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          return BT::NodeStatus::SUCCESS;
          break;
        case 4: // FLYING_AUTO
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(agent_->land(false))
          {
            while(agent_->state_ != 1 && agent_->state_ != 2)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            RCLCPP_INFO(agent_->get_logger(), "[BackToStation] LANDED");
            return BT::NodeStatus::SUCCESS;
          }
          else
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            RCLCPP_ERROR(agent_->get_logger(), "[BackToStation] Failed to call land");
            return BT::NodeStatus::FAILURE;
          }
          break;
        case 0: // UNINITIALIZED
        case 3: // TAKING_OFF
        case 5: // FLYING_MANUAL
        case 6: // LANDING
        default:
          break;
      }
    }
    // If Agent is not in station, call go_to_waypoint
    else
    {
      switch(agent_->state_)
      {
        case 2: // LANDED_ARMED
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(!agent_->take_off(agent_->take_off_height_, false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            RCLCPP_ERROR(agent_->get_logger(), "[BackToStation] Failed to call take_off");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            while(agent_->state_ != 4)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
          break;
        case 4: // FLYING_AUTO
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(!agent_->go_to_waypoint(agent_->known_positions_["stations"][nearest_station].getX(),
                agent_->known_positions_["stations"][nearest_station].getY(),
                agent_->known_positions_["stations"][nearest_station].getZ() + 1, false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            RCLCPP_ERROR(agent_->get_logger(), "[BackToStation] Failed to call go_to_waypoint");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            while(!agent_->checkIfGoToServiceSucceeded(agent_->known_positions_["stations"][nearest_station].getX(),
                  agent_->known_positions_["stations"][nearest_station].getY(),
                  agent_->known_positions_["stations"][nearest_station].getZ() + 1))
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
          // Land in station
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(!agent_->land(false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            RCLCPP_ERROR(agent_->get_logger(), "[BackToStation] Failed to call land");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            while(agent_->state_ != 1 && agent_->state_ != 2)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return BT::NodeStatus::SUCCESS;
          }
          break;
        case 0: // UNINITIALIZED
        case 1: // LANDED_DISARMED
        case 3: // TAKING_OFF
        case 5: // FLYING_MANUAL
        case 6: // LANDING
        default:
          break;
      }
    }
  }
  return BT::NodeStatus::IDLE;
}
void BackToStation::halt(){
  RCLCPP_INFO(agent_->get_logger(), "[BackToStation] halt requested");
  BT::AsyncActionNode::halt();
}
// }

//GoNearHumanTarget
GoNearHumanTarget::GoNearHumanTarget(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
GoNearHumanTarget::~GoNearHumanTarget(){halt();}
void GoNearHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearHumanTarget::providedPorts() {return{};}
BT::NodeStatus GoNearHumanTarget::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("go_near_human_target"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  float distance;
  switch(task->getType())
  {
    case 'D':
    case 'd':
      distance = 1.5;
      break;
    case 'M':
    case 'm':
      distance = task->getDistance();
      break;
    default:
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] First task of the queue isn't type Monitor");
      return BT::NodeStatus::FAILURE;
      break;
  }

  classes::Position human_position = task->getHumanPosition();
  classes::Position near_human_pose = classes::closePose2D(agent_->position_, human_position, distance);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] Moving near HT...");
        if(agent_->go_to_waypoint(near_human_pose.getX(), near_human_pose.getY(), near_human_pose.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_human_pose.getX(), near_human_pose.getY(),
                near_human_pose.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          RCLCPP_INFO(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_ERROR(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] Failed to call service go_to_waypoint");
        return BT::NodeStatus::FAILURE;
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
    //MAYBE PUT SOME SLEEP TIME HERE ********************************************************************************
  }
  return BT::NodeStatus::IDLE;
}
void GoNearHumanTarget::halt(){
  RCLCPP_INFO(rclcpp::get_logger("go_near_human_target"), "[GoNearHumanTarget] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//MonitorHumanTarget
MonitorHumanTarget::MonitorHumanTarget(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
MonitorHumanTarget::~MonitorHumanTarget(){halt();}
void MonitorHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::PortsList MonitorHumanTarget::providedPorts() {return{};}
BT::NodeStatus MonitorHumanTarget::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("monitor_human_target"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("monitor_human_target"), "[MonitorHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'M')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("monitor_human_target"), "[MonitorHumanTarget] First task of the queue isn't type Monitor");
    return BT::NodeStatus::FAILURE;
  }

  // ROS2: Usar el action client existente en AgentNode en lugar de crear uno nuevo
  auto goal = mission_planner::action::TaskResult::Goal();

  //TODO: Calling Safety Monitoring lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("monitor_human_target"), "[MonitorHumanTarget] Calling Lower-level controllers..."); 
  //********************************************* FAKED *************************************************************
  while(!isHaltRequested())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  //TODO: As this task ends when an operator requested, its result should be either true or false according to this
  // ROS2: Esperar que el servidor esté disponible
  if (!agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("monitor_human_target"), "Action server not available after waiting");
  } else {
    goal.task.id = task_id;
    goal.task.type = 'M';
    goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
    agent_->task_result_ac_->async_send_goal(goal);
  }
  
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'M');
  RCLCPP_INFO(rclcpp::get_logger("monitor_human_target"), "[MonitorHumanTarget] MONITOR TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();
  
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void MonitorHumanTarget::halt(){
  RCLCPP_INFO(rclcpp::get_logger("monitor_human_target"), "[MonitorHumanTarget] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//GoNearUGV
GoNearUGV::GoNearUGV(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
GoNearUGV::~GoNearUGV(){halt();}
void GoNearUGV::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearUGV::providedPorts() {return{};}
BT::NodeStatus GoNearUGV::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("go_near_ugv"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  float height;
  switch(task->getType())
  {
    case 'F':
    case 'f':
      height = task->getHeight();
      break;
    default:
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] First task of the queue isn't type MonitorUGV");
      return BT::NodeStatus::FAILURE;
      break;
  }

  // Extract the current position of the UGV
  classes::Position near_waypoint(agent_->atrvjr_pose_.getX(), agent_->atrvjr_pose_.getY(), height);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Moving near UGV...");
        if(agent_->go_to_waypoint(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          if (classes::distance2D(near_waypoint, agent_->position_) < agent_->distance_error_)
          {
            RCLCPP_INFO(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Returning SUCCESS...");
            return BT::NodeStatus::SUCCESS;
          }
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }
  return BT::NodeStatus::IDLE;
}
void GoNearUGV::halt(){
  RCLCPP_INFO(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//MonitorUGV
MonitorUGV::MonitorUGV(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
MonitorUGV::~MonitorUGV(){halt();}
void MonitorUGV::init(AgentNode* agent){agent_ = agent;}
BT::PortsList MonitorUGV::providedPorts() {return{};}
BT::NodeStatus MonitorUGV::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("monitor_ugv"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'F')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] First task of the queue isn't type MonitorUGV");
    return BT::NodeStatus::FAILURE;
  }
  float height = task->getHeight();

  // ROS2: Usar el action client existente en AgentNode
  auto goal = mission_planner::action::TaskResult::Goal();

  RCLCPP_INFO(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] Calling Lower-level controllers..."); 
  while(!isHaltRequested())
  {
    //RCLCPP_INFO_STREAM(rclcpp::get_logger("monitor_ugv"), "Trying to reach point: " << agent_->atrvjr_pose_);
    if(!agent_->go_to_waypoint(agent_->atrvjr_pose_.getX(), agent_->atrvjr_pose_.getY(), height, false))
    {
      if(isHaltRequested())
      {
        //RCLCPP_WARN(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] Halted");
        break;
      }
      RCLCPP_ERROR(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] Failed to call service go_to_waypoint");
      return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // ROS2: Usar el action client existente
  if (!agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("monitor_ugv"), "Action server not available after waiting");
  } else {
    goal.task.id = task_id;
    goal.task.type = 'F';
    goal.result = isHaltRequested() ? 0 : 1; 
    agent_->task_result_ac_->async_send_goal(goal);
  }
  
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'F');
  RCLCPP_INFO(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] MONITOR UGV TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();
  
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
}
void MonitorUGV::halt(){
  RCLCPP_INFO(rclcpp::get_logger("monitor_ugv"), "[MonitorUGV] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//GoNearWP
GoNearWP::GoNearWP(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
GoNearWP::~GoNearWP(){halt();}
void GoNearWP::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearWP::providedPorts() {return{};}
BT::NodeStatus GoNearWP::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("go_near_wp"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  mission_planner::msg::Waypoint nearest_wp;

  if(task->getType() != 'I' && task->getType() != 'A')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("go_near_wp"), "[GoNearWP] First task of the queue isn't type Inspect or InspectPVArray");
    return BT::NodeStatus::FAILURE;
  }

  if(task->getType() == 'I')
  {
    //Find the closest WP from the list
    float distance = -1;
    float tmp_distance;
    for(auto& waypoint : task->getInspectWaypoints())
    {
      tmp_distance = classes::distance(agent_->position_, waypoint);
      if(distance == -1 || tmp_distance < distance)
      {
        distance = tmp_distance;
        nearest_wp = waypoint;
      }
    }
  }

  if(task->getType() == 'A')
  {
    auto waypoint = task->getInspectWaypoints();
    nearest_wp = waypoint[0];
  }

  classes::Position near_waypoint = classes::closePose2D(agent_->position_, nearest_wp, 1.5);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Moving near WP...");
        if(agent_->go_to_waypoint(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_waypoint.getX(), near_waypoint.getY(),
                near_waypoint.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          RCLCPP_INFO(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
    //MAYBE PUT SOME SLEEP TIME HERE ********************************************************************************
  }
  return BT::NodeStatus::IDLE;
}
void GoNearWP::halt(){
  RCLCPP_INFO(rclcpp::get_logger("go_near_wp"), "[GoNearWP] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//TakeImage
TakeImage::TakeImage(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
TakeImage::~TakeImage(){halt();}
void TakeImage::init(AgentNode* agent){agent_ = agent;}
BT::PortsList TakeImage::providedPorts() {return{};}
BT::NodeStatus TakeImage::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("take_image"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("take_image"), "[TakeImage] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'I')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("take_image"), "[TakeImage] First task of the queue isn't type Inspect");
    return BT::NodeStatus::FAILURE;
  }

  // ROS2: Usar el action client existente en AgentNode
  auto goal = mission_planner::action::TaskResult::Goal();

  //TODO: Calling Inspection lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("take_image"), "[TakeImage] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // ROS2: Usar el action client existente
  if (!agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("take_image"), "Action server not available after waiting");
  } else {
    goal.task.id = task_id;
    goal.task.type = 'I';
    goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
    agent_->task_result_ac_->async_send_goal(goal);
  }
  
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'I');
  RCLCPP_INFO(rclcpp::get_logger("take_image"), "[TakeImage] INSPECT TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();

  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void TakeImage::halt(){
  RCLCPP_INFO(rclcpp::get_logger("take_image"), "[TakeImage] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}

//InspectPVArray
InspectPVArray::InspectPVArray(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
InspectPVArray::~InspectPVArray(){halt();}
void InspectPVArray::init(AgentNode* agent){agent_ = agent;}
BT::PortsList InspectPVArray::providedPorts() {return{};}
BT::NodeStatus InspectPVArray::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'A')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] First task of the queue isn't type Inspect PV Array");
    return BT::NodeStatus::FAILURE;
  }

  // ROS2: Usar el action client existente en AgentNode
  auto goal = mission_planner::action::TaskResult::Goal();

  //TODO: Calling Inspection lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  auto wp = task->getInspectWaypoints();

  // Función auxiliar para enviar resultado (evita duplicación de código)
  auto sendResult = [&](int result_value) {
    if (!agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "Action server not available after waiting");
    } else {
      goal.task.id = task_id;
      goal.task.type = 'A';
      goal.result = result_value;
      
      // Solar Panel 3_4 - Solo enviar coordenadas en caso de éxito
      if (result_value == 1) {
        geometry_msgs::msg::Point target_xyz;
        target_xyz.x = -36.6343;
        target_xyz.y =  62.293;
        geographic_msgs::msg::GeoPose target_gps;
        target_gps.position.latitude  = -7.96213167462045;
        target_gps.position.longitude = 38.54156780106911;
        goal.do_closer_inspection.xyz_coordinates.push_back(target_xyz);
        goal.do_closer_inspection.gps_coordinates.push_back(target_gps);
      }
      
      agent_->task_result_ac_->async_send_goal(goal);
    }
    
    if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'A');
    RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
    agent_->infoQueue();
  };

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
        {
          sendResult(isHaltRequested() ? 0 : 1);
          return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
        }
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
          {
            sendResult(isHaltRequested() ? 0 : 1);
            return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
          }
          RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Failed to call service take_off");
          sendResult(0);
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
            {
              sendResult(isHaltRequested() ? 0 : 1);
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
        {
          sendResult(isHaltRequested() ? 0 : 1);
          return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Moving to the beggining...");
        if(agent_->go_to_waypoint(wp[0].x, wp[0].y, wp[0].z, false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(wp[0].x, wp[0].y, wp[0].z))
          {
            if(isHaltRequested())
            {
              sendResult(isHaltRequested() ? 0 : 1);
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Moving to the end...");
          if(agent_->go_to_waypoint(wp[1].x, wp[1].y, wp[1].z, false))
          {
            while(!agent_->checkIfGoToServiceSucceeded(wp[1].x, wp[1].y, wp[1].z))
            {
              if(isHaltRequested())
              {
                sendResult(isHaltRequested() ? 0 : 1);
                return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Returning SUCCESS...");
            sendResult(1);
            return BT::NodeStatus::SUCCESS;
          }
          else
          {
            if(isHaltRequested())
            {
              sendResult(isHaltRequested() ? 0 : 1);
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Failed to call service go_to_waypoint");
            sendResult(0);
            return BT::NodeStatus::FAILURE;
          }
        }
        else
        {
          if(isHaltRequested())
          {
            sendResult(isHaltRequested() ? 0 : 1);
            return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
          }
          RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Failed to call service go_to_waypoint");
          sendResult(0);
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }

  sendResult(isHaltRequested() ? 0 : 1);
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void InspectPVArray::halt(){
  RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}

//GoNearStation
GoNearStation::GoNearStation(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
GoNearStation::~GoNearStation(){halt();}
void GoNearStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearStation::providedPorts() {return{};}
BT::NodeStatus GoNearStation::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("go_near_station"), "Failed to call stop");

  classes::Task* task;
  classes::Position tool_position;
  classes::Position near_tool_pose;

  if(agent_->tool_flag_ != "none")
    tool_position = agent_->tools_[agent_->tool_flag_].getPosition();
  else
  {
    if(agent_->task_queue_.empty())
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(rclcpp::get_logger("go_near_station"), "[GoNearStation] Task queue is empty");
      return BT::NodeStatus::FAILURE;
    }
    task = agent_->task_queue_.front();

    if(task->getType() != 'D')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      RCLCPP_WARN(rclcpp::get_logger("go_near_station"), "[GoNearStation] First task of the queue isn't type Deliver");
      return BT::NodeStatus::FAILURE;
    }

    tool_position = task->getToolPosition();
  }
  near_tool_pose = classes::closePose2D(agent_->position_, tool_position, 1.5);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_station"), "[GoNearStation] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_station"), "[GoNearStation] Moving near Tool...");
        if(agent_->go_to_waypoint(near_tool_pose.getX(), near_tool_pose.getY(), near_tool_pose.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_tool_pose.getX(), near_tool_pose.getY(),
                near_tool_pose.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          RCLCPP_INFO(rclcpp::get_logger("go_near_station"), "[GoNearStation] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_station"), "[GoNearStation] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }
  return BT::NodeStatus::IDLE;
}
void GoNearStation::halt(){
  RCLCPP_INFO(rclcpp::get_logger("go_near_station"), "[GoNearStation] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}


//PickTool
PickTool::PickTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
PickTool::~PickTool(){halt();}
void PickTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList PickTool::providedPorts() {return{};}
BT::NodeStatus PickTool::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("pick_tool"), "Failed to call stop");

  classes::Task* task;
  std::string tool_id;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("pick_tool"), "[PickTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("pick_tool"), "[PickTool] First task of the queue isn't type Deliver");
    return BT::NodeStatus::FAILURE;
  }
  tool_id = task->getToolID();

  //TODO: Calling Picking Tool lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("pick_tool"), "[PickTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = isHaltRequested() ? "none" : tool_id;
  RCLCPP_INFO(rclcpp::get_logger("pick_tool"), "[PickTool] PICK TOOL FINISHED");
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void PickTool::halt(){
  RCLCPP_INFO(rclcpp::get_logger("pick_tool"), "[PickTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}

//DropTool
DropTool::DropTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
DropTool::~DropTool(){halt();}
void DropTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList DropTool::providedPorts() {return{};}
BT::NodeStatus DropTool::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("drop_tool"), "Failed to call stop");
 
  //TODO: Calling Dropping Tool lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("drop_tool"), "[DropTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = "none";
  RCLCPP_INFO(rclcpp::get_logger("drop_tool"), "[DropTool] DROP TOOL FINISHED");
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void DropTool::halt(){
  RCLCPP_INFO(rclcpp::get_logger("drop_tool"), "[DropTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//DeliverTool
DeliverTool::DeliverTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
DeliverTool::~DeliverTool(){halt();}
void DeliverTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList DeliverTool::providedPorts() {return{};}
BT::NodeStatus DeliverTool::tick(){
  if(!agent_->stop(false))
    RCLCPP_ERROR(rclcpp::get_logger("deliver_tool"), "Failed to call stop");

  classes::Task* task;
  std::string tool_id;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("deliver_tool"), "[DeliverTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'D')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("deliver_tool"), "[DeliverTool] First task of the queue isn't type Deliver");
    return BT::NodeStatus::FAILURE;
  }
  tool_id = task->getToolID();

  // ROS2: Usar el action client existente en AgentNode
  auto goal = mission_planner::action::TaskResult::Goal();

  //TODO: Calling Tool Delivery lower level controllers (faked) 
  RCLCPP_INFO(rclcpp::get_logger("deliver_tool"), "[DeliverTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = isHaltRequested() ? tool_id : "none";

  // ROS2: Usar el action client existente
  if (!agent_->task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("deliver_tool"), "Action server not available after waiting");
  } else {
    goal.task.id = task_id;
    goal.task.type = 'D';
    goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
    agent_->task_result_ac_->async_send_goal(goal);
  }
  
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'D');
  RCLCPP_INFO(rclcpp::get_logger("deliver_tool"), "[DeliverTool] DELIVER TOOL TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();

  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void DeliverTool::halt(){
  RCLCPP_INFO(rclcpp::get_logger("deliver_tool"), "[DeliverTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//******************************* Conditions {
//MissionOver {
MissionOver::MissionOver(const std::string& name) : BT::ConditionNode(name, {}) {}
void MissionOver::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus MissionOver::tick(){
  if(agent_->mission_over_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//Idle {
Idle::Idle(const std::string& name) : BT::ConditionNode(name, {}) {}
void Idle::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus Idle::tick(){
  classes::Task* task;

  if(agent_->task_queue_.empty())
    return BT::NodeStatus::SUCCESS;

  task = agent_->task_queue_.front();
  switch(task->getType())
  {
    case 'W':
    case 'w':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsBatteryEnough {
IsBatteryEnough::IsBatteryEnough(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsBatteryEnough::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsBatteryEnough::tick(){
  if(agent_->battery_enough_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//IsBatteryFull {
IsBatteryFull::IsBatteryFull(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsBatteryFull::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsBatteryFull::tick(){
  if(agent_->battery_ > 0.95)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//IsTaskRecharge {
IsTaskRecharge::IsTaskRecharge(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskRecharge::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskRecharge::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_recharge"), "[IsTaskRecharge] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'R':
    case 'r':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskMonitor {
IsTaskMonitor::IsTaskMonitor(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskMonitor::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskMonitor::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_monitor"), "[IsTaskMonitor] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'M':
    case 'm':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskMonitorUGV {
IsTaskMonitorUGV::IsTaskMonitorUGV(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskMonitorUGV::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskMonitorUGV::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_monitor_ugv"), "[IsTaskMonitorUGV] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'F':
    case 'f':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskInspect {
IsTaskInspect::IsTaskInspect(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskInspect::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskInspect::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_inspect"), "[IsTaskInspect] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'I':
    case 'i':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskInspectPVArray {
IsTaskInspectPVArray::IsTaskInspectPVArray(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskInspectPVArray::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskInspectPVArray::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_inspect_pv_array"), "[IsTaskInspectPVArray] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'A':
    case 'a':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskDeliverTool {
IsTaskDeliverTool::IsTaskDeliverTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskDeliverTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskDeliverTool::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_task_deliver_tool"), "[IsTaskDeliverTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'D':
    case 'd':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsAgentNearChargingStation {
IsAgentNearChargingStation::IsAgentNearChargingStation(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearChargingStation::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearChargingStation::tick(){
  classes::Task* task;
  classes::Position assigned_charging_station;

  //Emergency Recharging
  if(agent_->task_queue_.empty())
  {
    for(auto& charging_station : agent_->known_positions_["charging_stations"])
      if(classes::distance2D(agent_->position_, charging_station.second) < agent_->distance_error_)
        return BT::NodeStatus::SUCCESS;

    if(classes::distance2D(agent_->position_, agent_->jackal_pose_) < agent_->distance_error_)
      return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
  }

  //Recharge Task
  task = agent_->task_queue_.front();
  if(task->getType() != 'R')
  {
    RCLCPP_WARN(rclcpp::get_logger("is_agent_near_charging_station"), "[IsAgentNearChargingStation] First task of the queue isn't type Recharge");
    return BT::NodeStatus::FAILURE;
  }

  if(classes::distance2D(agent_->position_, agent_->jackal_pose_) < agent_->distance_error_)
    return BT::NodeStatus::SUCCESS;

  assigned_charging_station = task->getChargingStation();

  if(assigned_charging_station.getID().empty())
  {
    for(auto& charging_station : agent_->known_positions_["charging_stations"])
    {
      if(classes::distance2D(agent_->position_, charging_station.second) < agent_->distance_error_)
      {
        //Assign and reserve this charging station for this Agent (to be improved)
        task->setChargingStation(&(charging_station.second));
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    if(classes::distance2D(agent_->position_, assigned_charging_station) < agent_->distance_error_)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}
// }

//IsAgentNearHumanTarget {
IsAgentNearHumanTarget::IsAgentNearHumanTarget(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearHumanTarget::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_agent_near_human_target"), "[IsAgentNearHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  classes::Position human_position;
  switch(task->getType())
  {
    case 'D':
    case 'd':
      human_position = task->getHumanPosition();
      if(classes::distance(human_position, agent_->position_) < 2.5)
        return BT::NodeStatus::SUCCESS;
      break;
    case 'M':
    case 'm':
      human_position = task->getHumanPosition();
      if(classes::distance(human_position, agent_->position_) < task->getDistance() + 1)
        return BT::NodeStatus::SUCCESS;
      break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("is_agent_near_human_target"), "[IsAgentNearHumanTarget] First task of the queue isn't type Monitor or Deliver");
      break;
  }
  return BT::NodeStatus::FAILURE;
}
// }

//IsAgentNearUGV {
IsAgentNearUGV::IsAgentNearUGV(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearUGV::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearUGV::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_agent_near_ugv"), "[IsAgentNearUGV] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'F':
    case 'f':
      if(classes::distance2D(agent_->atrvjr_pose_, agent_->position_) < agent_->distance_error_)
        return BT::NodeStatus::SUCCESS;
      break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("is_agent_near_ugv"), "[IsAgentNearUGV] First task of the queue isn't type MonitorUGV");
      break;
  }
  return BT::NodeStatus::FAILURE;
}
// }

//IsAgentNearWP {
IsAgentNearWP::IsAgentNearWP(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearWP::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearWP::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("is_agent_near_wp"), "[IsAgentNearWP] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'I' && task->getType() != 'A')
  {
    RCLCPP_WARN(rclcpp::get_logger("is_agent_near_wp"), "[IsAgentNearWP] First task of the queue isn't type Inspect or InspectPVArray");
    return BT::NodeStatus::FAILURE;
  }

  if(task->getType() == 'I')
  {
    for(auto& waypoint : task->getInspectWaypoints())
    {
      if(classes::distance(agent_->position_, waypoint) < agent_->distance_error_)
        return BT::NodeStatus::SUCCESS;
    }
  }
  if(task->getType() == 'A')
  {
    auto waypoint = task->getInspectWaypoints();
    if(classes::distance(agent_->position_, waypoint[0]) < agent_->distance_error_)
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
// }

//NeedToDropTheTool {
NeedToDropTheTool::NeedToDropTheTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void NeedToDropTheTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus NeedToDropTheTool::tick(){
  if(agent_->tool_flag_ == "none")
    return BT::NodeStatus::FAILURE;

  classes::Task* task;
  if(agent_->task_queue_.empty())
    return BT::NodeStatus::SUCCESS;
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
    return BT::NodeStatus::SUCCESS;
  std::string tool_id = task->getToolID();

  //If Agent first task is Deliver and has a tool, check if it is the correct one
  if(agent_->tool_flag_ == tool_id)
    return BT::NodeStatus::FAILURE;
  else
    return BT::NodeStatus::SUCCESS;
}
// }

//HasAgentTheTool {
HasAgentTheTool::HasAgentTheTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void HasAgentTheTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus HasAgentTheTool::tick(){
  if(agent_->tool_flag_ == "none")
    return BT::NodeStatus::FAILURE;

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("has_agent_the_tool"), "[HasAgentTheTool] Task queue is empty. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
  {
    RCLCPP_INFO(rclcpp::get_logger("has_agent_the_tool"), "[HasAgentTheTool] First task of the queue isn't type Deliver. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
  std::string tool_id = task->getToolID();

  if(agent_->tool_flag_ == tool_id)
    return BT::NodeStatus::SUCCESS;
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("has_agent_the_tool"), "[HasAgentTheTool] Agent has the incorrect tool for Deliver task. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
}
// }

//IsAgentNearStation {
IsAgentNearStation::IsAgentNearStation(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearStation::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearStation::tick(){
  classes::Task* task;
  classes::Position tool_position;

  if(agent_->tool_flag_ != "none")
    tool_position = agent_->tools_[agent_->tool_flag_].getPosition();
  else
  {
    if(agent_->task_queue_.empty())
    {
      RCLCPP_WARN(rclcpp::get_logger("is_agent_near_station"), "[IsAgentNearStation] Task queue is empty");
      return BT::NodeStatus::FAILURE;
    }
    task = agent_->task_queue_.front();

    if(task->getType() != 'D')
    {
      RCLCPP_WARN(rclcpp::get_logger("is_agent_near_station"), "[IsAgentNearStation] First task of the queue isn't type Deliver");
      return BT::NodeStatus::FAILURE;
    }

    tool_position = task->getToolPosition();
  }

  if(classes::distance(tool_position, agent_->position_) < agent_->distance_error_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }
// }

//******************************* Decorators {
//ForceRunningNode {
ForceRunningNode::ForceRunningNode(const std::string& name) : BT::DecoratorNode(name, {} ){
  setRegistrationID("ForceRunning");
}

BT::NodeStatus ForceRunningNode::tick(){
  setStatus(BT::NodeStatus::RUNNING);

  const BT::NodeStatus child_state = child_node_->executeTick();

  switch (child_state)
  {
    case BT::NodeStatus::FAILURE:
    case BT::NodeStatus::SUCCESS:
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    default:
      break;
  }
  return status();
}
// }
// }


//Behavior Tree Nodes registration function *************************************************************************
inline void RegisterNodes(BT::BehaviorTreeFactory& factory){
  //Actions
  factory.registerNodeType<GoNearChargingStation>("GoNearChargingStation");
  factory.registerNodeType<Recharge>("Recharge");
  factory.registerNodeType<BackToStation>("BackToStation");
  factory.registerNodeType<GoNearHumanTarget>("GoNearHumanTarget");
  factory.registerNodeType<GoNearUGV>("GoNearUGV");
  factory.registerNodeType<MonitorHumanTarget>("MonitorHumanTarget");
  factory.registerNodeType<MonitorUGV>("MonitorUGV");
  factory.registerNodeType<GoNearWP>("GoNearWP");
  factory.registerNodeType<TakeImage>("TakeImage");
  factory.registerNodeType<InspectPVArray>("InspectPVArray");
  factory.registerNodeType<GoNearStation>("GoNearStation");
  factory.registerNodeType<PickTool>("PickTool");
  factory.registerNodeType<DropTool>("DropTool");
  factory.registerNodeType<DeliverTool>("DeliverTool");

  //Conditions
  factory.registerNodeType<MissionOver>("MissionOver");
  factory.registerNodeType<Idle>("Idle");
  factory.registerNodeType<IsBatteryEnough>("IsBatteryEnough");
  factory.registerNodeType<IsBatteryFull>("IsBatteryFull");
  factory.registerNodeType<IsTaskRecharge>("IsTaskRecharge");
  factory.registerNodeType<IsTaskMonitor>("IsTaskMonitor");
  factory.registerNodeType<IsTaskMonitorUGV>("IsTaskMonitorUGV");
  factory.registerNodeType<IsTaskInspect>("IsTaskInspect");
  factory.registerNodeType<IsTaskInspectPVArray>("IsTaskInspectPVArray");
  factory.registerNodeType<IsTaskDeliverTool>("IsTaskDeliverTool");
  factory.registerNodeType<IsAgentNearChargingStation>("IsAgentNearChargingStation");
  factory.registerNodeType<IsAgentNearHumanTarget>("IsAgentNearHumanTarget");
  factory.registerNodeType<IsAgentNearUGV>("IsAgentNearUGV");
  factory.registerNodeType<IsAgentNearWP>("IsAgentNearWP");
  factory.registerNodeType<NeedToDropTheTool>("NeedToDropTheTool");
  factory.registerNodeType<HasAgentTheTool>("HasAgentTheTool");
  factory.registerNodeType<IsAgentNearStation>("IsAgentNearStation");

  //Decorators
  factory.registerNodeType<ForceRunningNode>("ForceRunning");
}


AgentNode::AgentNode(const mission_planner::msg::AgentBeacon& beacon, const rclcpp::NodeOptions& options) 
  : as2::Node("agent_behaviour_manager_" + beacon.id, options),
    battery_enough_(true), 
    loop_rate_(1),
    tool_flag_("none"), 
    timeout_(false), 
    beacon_(beacon), 
    mission_over_(false), 
    state_(0), 
    battery_(0)
{
  // Declarar parámetros
  this->declare_parameter<std::string>("id", "0");
  this->declare_parameter<std::string>("ns_prefix", "uav");
  this->declare_parameter<std::string>("pose_frame_id", "map");
  this->declare_parameter<std::string>("pose_topic", "/" + beacon_.id + "/self_localization/pose");
  this->declare_parameter<std::string>("state_topic", "/" + beacon_.id + "/platform/info");
  this->declare_parameter<std::string>("battery_topic", "/" + beacon_.id + "/sensor_measurements/battery");
  this->declare_parameter<float>("take_off_height", 10.0);
  this->declare_parameter<float>("distance_error", 2.0);
  this->declare_parameter<float>("goto_error", 1.0);

  // Obtener parámetros
  this->get_parameter("id", id_);
  this->get_parameter("ns_prefix", ns_prefix_);
  this->get_parameter("pose_frame_id", pose_frame_id_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("state_topic", state_topic_);
  this->get_parameter("battery_topic", battery_topic_);
  this->get_parameter("take_off_height", take_off_height_);
  this->get_parameter("distance_error", distance_error_);
  this->get_parameter("goto_error", goto_error_);

  // Configurar origin_geo_
  origin_geo_.latitude = 38.54130842044177;
  origin_geo_.longitude = -7.961568610186141;
  origin_geo_.altitude = 0;

  // Action server para recibir TaskList
  ntl_as_ = rclcpp_action::create_server<mission_planner::action::NewTaskList>(
    this,
    "/" + beacon_.id + "/task_list",
    std::bind(&AgentNode::handleNewTaskListGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AgentNode::handleNewTaskListCancel, this, std::placeholders::_1),
    std::bind(&AgentNode::handleNewTaskListAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "🟢 Agent Behaviour Manager LISTO para recibir tareas");

  // Action client para Battery Enough
  battery_ac_ = rclcpp_action::create_client<mission_planner::action::BatteryEnough>(
    this, 
    "/" + beacon_.id + "/battery_enough");

  // Aerostack2 action clients
  takeoff_ac_ = rclcpp_action::create_client<as2_msgs::action::Takeoff>(
    this,
    "take_off");

  land_ac_ = rclcpp_action::create_client<as2_msgs::action::Land>(
    this,
    "land");

  goto_ac_ = rclcpp_action::create_client<as2_msgs::action::GoToWaypoint>(
    this,
    "go_to_waypoint");

  // Publishers
  beacon_pub_ = this->create_publisher<mission_planner::msg::AgentBeacon>("/agent_beacon", 1);

  // Subscribers
  position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 1,
    std::bind(&AgentNode::positionCallback, this, std::placeholders::_1));
    
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_, 1,
    std::bind(&AgentNode::batteryCallback, this, std::placeholders::_1));
    
  platform_info_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    state_topic_, 1,
    std::bind(&AgentNode::platformInfoCallback, this, std::placeholders::_1));
    
  mission_over_sub_ = this->create_subscription<mission_planner::msg::MissionOver>(
    "/mission_over", 1,
    std::bind(&AgentNode::missionOverCallback, this, std::placeholders::_1));
    
  planner_beacon_sub_ = this->create_subscription<mission_planner::msg::PlannerBeacon>(
    "/planner_beacon", 1,
    std::bind(&AgentNode::beaconCallback, this, std::placeholders::_1));

  // UGV position subscribers
  atrvjr_geopose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
    "/atrvjr/geopose", 1,
    std::bind(&AgentNode::atrvjrPositionCallback, this, std::placeholders::_1));
    
  jackal_geopose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
    "/jackal0/geopose", 1,
    std::bind(&AgentNode::jackalPositionCallback, this, std::placeholders::_1));

  // Cargar archivo de configuración
  std::string path = ament_index_cpp::get_package_share_directory("mission_planner");
  this->declare_parameter<std::string>("config_file", path + "/config/conf.yaml");
  std::string config_file;
  this->get_parameter("config_file", config_file);
  
  RCLCPP_INFO(this->get_logger(), "🔧 Cargando configuración desde: %s", config_file.c_str());
  readConfigFile(config_file);
  // Inicializar Behavior Tree
  // [Implementar inicialización del Behavior Tree...]
}

AgentNode::~AgentNode() {}

// Callbacks
void AgentNode::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  position_.update(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
}

void AgentNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr battery) {
  battery_ = battery->percentage;
}

void AgentNode::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info) {
    // Mapear estados de Aerostack2 a los estados originales
    // info->status es de tipo as2_msgs::msg::PlatformStatus
    switch (info->status.state) {  // Acceder al campo 'state' dentro de PlatformStatus
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
            state_ = 5; // FLYING_MANUAL (como estado de emergencia)
            break;
        default:
            state_ = 0; // UNINITIALIZED
            break;
    }
    
    // Opcional: también puedes loguear el estado para debugging
    RCLCPP_DEBUG(this->get_logger(), "Platform state: %d -> Mapped state: %d", 
                 info->status.state, state_);
}

void AgentNode::missionOverCallback(const mission_planner::msg::MissionOver::SharedPtr value) {
  mission_over_ = value->value;
}

void AgentNode::beaconCallback(const mission_planner::msg::PlannerBeacon::SharedPtr beacon) {
  last_beacon_ = beacon->timestamp;
  timeout_ = false;
  beacon_.timeout = false;
}

bool AgentNode::checkBeaconTimeout(rclcpp::Time now) {
  auto timeout = rclcpp::Duration(5, 0);
  if(((now - last_beacon_) > timeout) && !timeout_) {
    RCLCPP_WARN(this->get_logger(), "[checkBeaconTimeout] Beacon timeout. Disconnection detected. Emptying the task queue.");
    emptyTheQueue();
    timeout_ = true;
    beacon_.timeout = true;
  }
  return timeout_;
}

void AgentNode::atrvjrPositionCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr geo_pose) {
  // Implementar conversión de coordenadas geográficas a cartesianas
  atrvjr_pose_ = classes::Position(geo_pose->pose.position.latitude, geo_pose->pose.position.longitude, 2.0);
}

void AgentNode::jackalPositionCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr geo_pose) {
  // Implementar conversión de coordenadas geográficas a cartesianas
  jackal_pose_ = classes::Position(geo_pose->pose.position.latitude, geo_pose->pose.position.longitude, 2.0);
}

// Aerostack2 Action calls
bool AgentNode::land(bool blocking) {
  auto goal_msg = as2_msgs::action::Land::Goal();
  auto goal_options = rclcpp_action::Client<as2_msgs::action::Land>::SendGoalOptions();
  
  auto future_goal_handle = land_ac_->async_send_goal(goal_msg, goal_options);
  
  if (blocking) {
    auto result_future = land_ac_->async_get_result(future_goal_handle.get());
    // Esperar y verificar resultado
  }
  
  return true; // Simplificado para el ejemplo
}

bool AgentNode::take_off(float height, bool blocking) {
  auto goal_msg = as2_msgs::action::Takeoff::Goal();
  goal_msg.takeoff_height = height;
  auto goal_options = rclcpp_action::Client<as2_msgs::action::Takeoff>::SendGoalOptions();
  
  auto future_goal_handle = takeoff_ac_->async_send_goal(goal_msg, goal_options);
  
  if (blocking) {
    auto result_future = takeoff_ac_->async_get_result(future_goal_handle.get());
    // Esperar y verificar resultado
  }
  
  return true; // Simplificado para el ejemplo
}

bool AgentNode::go_to_waypoint(float x, float y, float z, bool blocking) {
    if (!goto_ac_) {
        RCLCPP_ERROR(get_logger(), "GoToWaypoint action client not initialized");
        return false;
    }

    if (!goto_ac_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "GoToWaypoint action server not available");
        return false;
    }

    auto goal_msg = as2_msgs::action::GoToWaypoint::Goal();
    
    // Configurar target_pose como PointStamped
    goal_msg.target_pose.header.frame_id = pose_frame_id_;
    goal_msg.target_pose.header.stamp = this->now();
    goal_msg.target_pose.point.x = x;  // Usar 'point' en lugar de 'pose.position'
    goal_msg.target_pose.point.y = y;
    goal_msg.target_pose.point.z = z;
    
    // Configurar yaw mode (puedes ajustar según necesites)
    goal_msg.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW;  // O PATH_FACING, etc.
    
    // Configurar velocidad máxima (opcional)
    goal_msg.max_speed = 1.0;  // Ajusta según necesites
    
    auto goal_options = rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SendGoalOptions();
    
    auto future_goal_handle = goto_ac_->async_send_goal(goal_msg, goal_options);
    
    if (blocking) {
        auto result_future = goto_ac_->async_get_result(future_goal_handle.get());
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::seconds(30)) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "GoToWaypoint action failed to complete");
            return false;
        }
        
        auto result = result_future.get();
        return result.result->go_to_success;
    }
    
    return true;
}

bool AgentNode::stop(bool blocking) {
  if(state_ == 4) { // FLYING_AUTO
    return go_to_waypoint(position_.getX(), position_.getY(), position_.getZ(), blocking);
  }
  return true;
}

bool AgentNode::checkIfGoToServiceSucceeded(float x, float y, float z) {
  classes::Position position(x, y, z);
  if(classes::distance2D(position, position_) < goto_error_)
    return true;
  return false;
}

// Action server callbacks
rclcpp_action::GoalResponse AgentNode::handleNewTaskListGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const mission_planner::action::NewTaskList::Goal> goal) {
  
  RCLCPP_INFO(this->get_logger(), "🎯 [handleNewTaskListGoal] GOAL RECIBIDO!");
  RCLCPP_INFO(this->get_logger(), "📨 Agent ID del goal: %s", goal->agent_id.c_str());
  RCLCPP_INFO(this->get_logger(), "📊 Número de tareas: %zu", goal->task_list.size());

  RCLCPP_INFO(this->get_logger(), "✅ Goal ACEPTADO");
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AgentNode::handleNewTaskListCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTaskList>> goal_handle) {
  
  RCLCPP_INFO(this->get_logger(), "[handleNewTaskListCancel] Goal canceled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AgentNode::handleNewTaskListAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTaskList>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "🚀 [handleNewTaskListAccepted] PROCESANDO NUEVA LISTA DE TAREAS!");
  
  // Ejecutar en un thread separado
  std::thread([this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    
    RCLCPP_INFO(this->get_logger(), "📋 Procesando %zu tareas para el agente %s", 
                goal->task_list.size(), goal->agent_id.c_str());

    // Procesar cada tarea
    for (const auto& task_msg : goal->task_list) {
      RCLCPP_INFO(this->get_logger(), "🎯 Tarea: %s, Tipo: %c", 
                  task_msg.id.c_str(), task_msg.type);
    }

    // Vaciar la cola actual y añadir nuevas tareas
    emptyTheQueue();
    
for (const auto& task_msg : goal->task_list) {
  classes::Task* task = nullptr;
  
  // CREAR LA TAREA SEGÚN EL TIPO
  switch(task_msg.type) {
    case 'M': case 'm': {
      // Buscar el human target
      auto human_itr = human_targets_.find(task_msg.monitor.human_target_id);
      if (human_itr != human_targets_.end()) {
        task = new classes::Monitor(task_msg.id, 
                                  &human_itr->second, 
                                  task_msg.monitor.distance, 
                                  task_msg.monitor.number);
        RCLCPP_INFO(this->get_logger(), "👤 Monitor task creada para human: %s", 
                    task_msg.monitor.human_target_id.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Human target NO encontrado: %s", 
                     task_msg.monitor.human_target_id.c_str());
      }
      break;
    }
    
    case 'F': case 'f': {
      task = new classes::MonitorUGV(task_msg.id, 
                                   task_msg.monitor_ugv.ugv_id, 
                                   task_msg.monitor_ugv.height);
      RCLCPP_INFO(this->get_logger(), "🤖 MonitorUGV task creada para UGV: %s", 
                  task_msg.monitor_ugv.ugv_id.c_str());
      break;
    }
    
    case 'I': case 'i': {
      task = new classes::Inspect(task_msg.id, 
                                task_msg.inspect.waypoints);
      RCLCPP_INFO(this->get_logger(), "📸 Inspect task creada con %zu waypoints", 
                  task_msg.inspect.waypoints.size());
      break;
    }
    
    case 'A': case 'a': {
      task = new classes::InspectPVArray(task_msg.id, 
                                       task_msg.inspect.waypoints);
      RCLCPP_INFO(this->get_logger(), "🔋 InspectPVArray task creada con %zu waypoints", 
                  task_msg.inspect.waypoints.size());
      break;
    }
    
    case 'D': case 'd': {
      // Buscar tool y human target
      auto tool_itr = tools_.find(task_msg.deliver.tool_id);
      auto human_itr = human_targets_.find(task_msg.deliver.human_target_id);
      
      if (tool_itr != tools_.end() && human_itr != human_targets_.end()) {
        task = new classes::DeliverTool(task_msg.id, 
                                      &tool_itr->second, 
                                      &human_itr->second);
        RCLCPP_INFO(this->get_logger(), "📦 DeliverTool task creada - Tool: %s, Human: %s", 
                    task_msg.deliver.tool_id.c_str(), task_msg.deliver.human_target_id.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Tool o Human target NO encontrado");
      }
      break;
    }
    
    case 'R': case 'r': {
      // Para tareas de recarga, necesitarías crear classes::Recharge
      RCLCPP_WARN(this->get_logger(), "⚠️ Tipo de tarea Recharge no implementado aún");
      break;
    }
    
    case 'W': case 'w': {
      // Para tareas de espera
      RCLCPP_INFO(this->get_logger(), "⏳ Wait task creada");
      // task = new classes::Wait(task_msg.id); // Si tienes esta clase
      break;
    }
    
    default:
      RCLCPP_WARN(this->get_logger(), "⚠️ Tipo de tarea desconocido: %c", task_msg.type);
      break;
    }

      if(task) {
        addTaskToQueue(task);
        RCLCPP_INFO(this->get_logger(), "✅ Tarea %s añadida a la cola", task_msg.id.c_str());
      }
    }

    auto result = std::make_shared<mission_planner::action::NewTaskList::Result>();
    result->ack = true;
    goal_handle->succeed(result);
    
    RCLCPP_INFO(this->get_logger(), "🎉 Lista de tareas procesada EXITOSAMENTE");
    infoQueue(); // Mostrar estado de la cola
    
  }).detach();
}

// Task queue methods
void AgentNode::addTaskToQueue(classes::Task* task) {
    task_queue_.push(task);
}

void AgentNode::removeTaskFromQueue(const std::string& id, char type) {
    if(task_queue_.empty()) {
        RCLCPP_WARN(this->get_logger(), "[removeTaskFromQueue] Task queue already empty");
        return;
    }
    
    classes::Task* task = task_queue_.front();
    
    if((id == task->getID()) && (type == task->getType())) {
        delete task;
        task_queue_.pop();
        return;
    }
    
    RCLCPP_WARN(this->get_logger(), "[removeTaskFromQueue] Task isn't in the first place of the queue. Not deleting.");
}

void AgentNode::emptyTheQueue() {
    while(!task_queue_.empty()) {
        classes::Task* task = task_queue_.front();
        delete task;
        task_queue_.pop();
    }
}

int AgentNode::getQueueSize() {
    return task_queue_.size();
}

void AgentNode::infoQueue() {
    std::queue<classes::Task*> temp_queue = task_queue_;
    RCLCPP_INFO(this->get_logger(), "--- Task Queue Info ---");
    
    int position = 0;
    while(!temp_queue.empty()) {
        classes::Task* tmp = temp_queue.front();
        char task_type = tmp->getType();
        std::string task_description;
        
        switch(task_type) {
            case 'M': task_description = "Monitor"; break;
            case 'F': task_description = "MonitorUGV"; break;
            case 'I': task_description = "Inspect"; break;
            case 'A': task_description = "InspectPVArray"; break;
            case 'D': task_description = "DeliverTool"; break;
            case 'R': task_description = "Recharge"; break;
            case 'W': task_description = "Wait"; break;
            default: task_description = "Unknown Task"; break;
        }
        
        RCLCPP_INFO(this->get_logger(), "Position %d: %s - %s", 
                   position, tmp->getID().c_str(), task_description.c_str());
        temp_queue.pop();
        position++;
    }
    RCLCPP_INFO(this->get_logger(), "--- End Queue Info ---");
}

void AgentNode::taskQueueManager() {
    if(task_queue_.size() < 2)
        return;
        
    std::queue<classes::Task*> task_queue_aux = task_queue_;
    task_queue_aux.pop();
    
    classes::Task* current_task = task_queue_.front();
    classes::Task* next_task = task_queue_aux.front();
    char next_task_type = next_task->getType();
    
    // Check if the second task is a Recharge task
    if(next_task_type != 'R')
        return;
        
    float initial_percentage = next_task->getInitialPercentage();
    
    // Check if the Agent's battery is equal or lower than the recharge task initial percentage
    if(battery_ > initial_percentage)
        return;
        
    // Halt the first task, delete it from queue and start the Recharge task
    auto goal = mission_planner::action::TaskResult::Goal();
    goal.task.id = current_task->getID();
    goal.task.type = current_task->getType();
    
    RCLCPP_INFO_STREAM(this->get_logger(), "[taskQueueManager] " << goal.task.id << ": " << 
        (goal.task.type == 'M' ? "Monitor" : 
         goal.task.type == 'F' ? "MonitorUGV" : 
         goal.task.type == 'I' ? "Inspect" : 
         goal.task.type == 'A' ? "InspectPVArray" : 
         goal.task.type == 'D' ? "DeliverTool" :
         goal.task.type == 'R' ? "Recharge" :
         goal.task.type == 'W' ? "Wait" :
         "Halted to Recharge"));
         
    if(task_result_ac_->wait_for_action_server(std::chrono::seconds(1))) {
        goal.result = 2; // Halted to recharge
        task_result_ac_->async_send_goal(goal);
    }
}

void AgentNode::isBatteryEnough() {
    bool previous_state = battery_enough_;
    bool empty_queue = task_queue_.empty();
    
    // Planner node has decided that this Agent should stop recharging to go on
    if(!battery_enough_ && !empty_queue)
        battery_enough_ = true;

    // Check if this Agent has battery enough to fulfill its next task
    if(battery_ < 0.3)
        battery_enough_ = false;

    // Charged battery
    if(battery_ > 0.95)
        battery_enough_ = true;

    if(previous_state != battery_enough_) {
        // Planner node has decided that this Agent should stop recharging to go on
        if(battery_enough_ && !empty_queue) {
            RCLCPP_WARN(this->get_logger(), "[isBatteryEnough] Planner has set me up");
            return;
        }
        
        RCLCPP_WARN(this->get_logger(), "[isBatteryEnough] Advertising that battery_enough_ = %s", 
                   battery_enough_ ? "true" : "false");
        emptyTheQueue();
        
        if(battery_ac_->wait_for_action_server(std::chrono::seconds(1))) {
            auto goal = mission_planner::action::BatteryEnough::Goal();
            goal.value = battery_enough_;
            battery_ac_->async_send_goal(goal);
        }
    }
}

void AgentNode::readConfigFile(const std::string& config_file) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file);
        
        if(yaml_config["positions"]) {
            for(const auto& group : yaml_config["positions"]) {
                for(const auto& position : group.second) {
                    known_positions_[group.first.as<std::string>()][position.first.as<std::string>()] = 
                        classes::Position(position.first.as<std::string>(),
                                        position.second["x"].as<float>(),
                                        position.second["y"].as<float>(),
                                        position.second["z"].as<float>());
                }
            }
        }
        
        if(yaml_config["human_targets"]) {
            for(const auto& human_target : yaml_config["human_targets"]) {
                human_targets_[human_target.first.as<std::string>()] = 
                    classes::HumanTarget(human_target.first.as<std::string>(),
                                       human_target.second["x"].as<float>(),
                                       human_target.second["y"].as<float>(),
                                       human_target.second["z"].as<float>());
            }
        }
        
        if(yaml_config["tools"]) {
            for(const auto& tool : yaml_config["tools"]) {
                tools_[tool.first.as<std::string>()] = 
                    classes::Tool(tool.first.as<std::string>(),
                                tool.second["weight"].as<float>(),
                                tool.second["x"].as<float>(),
                                tool.second["y"].as<float>(),
                                tool.second["z"].as<float>());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Configuration file loaded successfully");
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Leer parámetros para crear el beacon
    auto temp_node = std::make_shared<rclcpp::Node>("temp_param_reader");
    
    std::string id, ns_prefix, type;
    temp_node->declare_parameter<std::string>("id", "0");
    temp_node->declare_parameter<std::string>("ns_prefix", "uav");
    temp_node->declare_parameter<std::string>("type", "ACW");
    
    temp_node->get_parameter("id", id);
    temp_node->get_parameter("ns_prefix", ns_prefix);
    temp_node->get_parameter("type", type);
    
    mission_planner::msg::AgentBeacon beacon;
    beacon.id = ns_prefix + id;
    beacon.type = type;
    
    // Crear y ejecutar el nodo principal
    auto node = std::make_shared<AgentNode>(beacon);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}