#include "mission_planner/high_level_planner.hpp"


using std::placeholders::_1;

// ---------------- Class Agent Constructors and Destructor ---------------- //

// CONSTRUCTOR POR DEFECTO
Agent::Agent() 
: id_(),
  type_(),
  battery_(1),
  battery_enough_(true),
  planner_(nullptr)

{
  nh_ = rclcpp::Node::make_shared("agent_node");  
  RCLCPP_INFO(nh_->get_logger(), "Agent default constructor called.");
}

// CONSTRUCTOR PRINCIPAL
Agent::Agent(Planner* planner, 
             std::string id, 
             std::string type, 
             rclcpp::Time first_beacon_time,
             mission_planner::msg::AgentBeacon first_beacon)
: planner_(planner),
  id_(id),
  type_(type),
  last_beacon_time_(first_beacon_time),
  last_beacon_(first_beacon),
  battery_(100.0),
  battery_enough_(true)
{
  // Nodo local del agente
  nh_ = rclcpp::Node::make_shared("agent_node_" + id_);  

  // Obtener nombres de topics (o usar defaults)
  nh_->declare_parameter<std::string>("pose_topic", "/ual/pose");
  nh_->declare_parameter<std::string>("battery_topic", "/battery_fake");
  nh_->get_parameter("pose_topic", pose_topic_);
  nh_->get_parameter("battery_topic", battery_topic_);

  // Crear subscriptores
  position_sub_ = nh_->create_subscription<as2_msgs::msg::PoseStampedWithID>("/" + id_ + pose_topic_, 10,
                      std::bind(&Agent::positionCallbackAS2, this, _1));

  battery_sub_ = nh_->create_subscription<sensor_msgs::msg::BatteryState>("/" + id_ + battery_topic_, 10,
                      std::bind(&Agent::batteryCallback, this, _1));

  // Crear clientes y servidores de acciones
  ntl_ac_ = rclcpp_action::create_client<mission_planner::action::NewTaskList>(nh_, "/" + id_ + "/task_list");

  battery_as_ = rclcpp_action::create_server<mission_planner::action::BatteryEnough>(
      nh_, 
      "/" + id_ + "/battery_enough",
      std::bind(&Agent::handleBatteryEnoughGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Agent::handleBatteryEnoughCancel, this, std::placeholders::_1),
      std::bind(&Agent::handleBatteryEnoughAccepted, this, std::placeholders::_1));

  task_result_as_ = rclcpp_action::create_server<mission_planner::action::TaskResult>(
      nh_, 
      "/" + id_ + "/task_result",
      [this](const rclcpp_action::GoalUUID & uuid, 
            std::shared_ptr<const mission_planner::action::TaskResult::Goal> goal) {
          return this->handleTaskResultGoal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle) {
          return this->handleTaskResultCancel(goal_handle);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle) {
          this->handleTaskResultAccepted(goal_handle);
      });

  RCLCPP_INFO(nh_->get_logger(), "Agente [%s] de tipo [%s] inicializado (ROS2).", id_.c_str(), type_.c_str());                    
}

// CONSTRUCTOR DE COPIA
Agent::Agent(const Agent& a) 
: id_(a.id_),
  type_(a.type_),
  planner_(a.planner_),
  last_beacon_time_(a.last_beacon_time_),
  last_beacon_(a.last_beacon_),
  position_(a.position_),
  battery_(a.battery_),
  battery_enough_(a.battery_enough_),
  pose_topic_(a.pose_topic_),
  battery_topic_(a.battery_topic_),
  old_first_task_id_(a.old_first_task_id_)
{
  nh_ = rclcpp::Node::make_shared("agent_copy_" + id_);

  // Recrear suscriptores
  position_sub_ = nh_->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      "/" + id_ + pose_topic_,
      10,
      std::bind(&Agent::positionCallbackAS2, this, std::placeholders::_1));

  battery_sub_ = nh_->create_subscription<sensor_msgs::msg::BatteryState>(
      "/" + id_ + battery_topic_,
      10,
      std::bind(&Agent::batteryCallback, this, std::placeholders::_1));

  // Recrear acciones - CLIENTES
  ntl_ac_ = rclcpp_action::create_client<mission_planner::action::NewTaskList>(
      nh_,
      "/" + id_ + "/task_list");

  // Recrear acciones - SERVIDORES con las nuevas funciones
  battery_as_ = rclcpp_action::create_server<mission_planner::action::BatteryEnough>(
      nh_,
      "/" + id_ + "/battery_enough",
      [this](const rclcpp_action::GoalUUID & uuid, 
             std::shared_ptr<const mission_planner::action::BatteryEnough::Goal> goal) {
          return this->handleBatteryEnoughGoal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::BatteryEnough>> goal_handle) {
          return this->handleBatteryEnoughCancel(goal_handle);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::BatteryEnough>> goal_handle) {
          this->handleBatteryEnoughAccepted(goal_handle);
      });

  task_result_as_ = rclcpp_action::create_server<mission_planner::action::TaskResult>(
      nh_,
      "/" + id_ + "/task_result",
      [this](const rclcpp_action::GoalUUID & uuid, 
             std::shared_ptr<const mission_planner::action::TaskResult::Goal> goal) {
          return this->handleTaskResultGoal(uuid, goal);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle) {
          return this->handleTaskResultCancel(goal_handle);
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle) {
          this->handleTaskResultAccepted(goal_handle);
      });

  RCLCPP_INFO(nh_->get_logger(), "Copia del agente [%s] creada.", id_.c_str());
}

// DESTRUCTOR

Agent::~Agent()
{
  RCLCPP_INFO(nh_->get_logger(), "Destruyendo agente [%s]", id_.c_str());
  // No es necesario "shutdown" manual en ROS2: shared_ptr libera los recursos automáticamente.
}


// ---------------- Class Agent Topic Methods ---------------- //

void Agent::updateSensorsInformation(){}

bool Agent::isBatteryForQueue()
{
  return true;
}

bool Agent::isBatteryEnough(classes::Task* task)
{
  if (battery_enough_)
  return true;

  if (task->getType() == 'I' && battery_ >= 0.5)
    battery_enough_ = true;
  return battery_enough_;
}

bool Agent::checkBeaconTimeout(rclcpp::Time now)
{
  auto timeout = rclcpp::Duration::from_seconds(5.0);
  if ((now - last_beacon_time_) > timeout)
    return true;
  else 
    return false;
}

// ---------------- Class Agent Task Queue Methods ---------------- //

bool Agent::isQueueEmpty()
{
  return task_queue_.empty();
}

void Agent::emptyTheQueue()
{
  while (!task_queue_.empty())
    task_queue_.pop();
}

void Agent::addTaskToQueue(classes::Task* task)
{
  task_queue_.push(task);
}

void Agent::replaceTaskFromQueue(std::string task_id)
{
  classes::Task* aux = new classes::Task(); 
  for (int i = 0; i < task_queue_.size(); i++)
  {
    if(task_queue_.front()->getID() == task_id)
      task_queue_.push(aux);
    else
      task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }
}

classes::Task* Agent::getFirstTask()
{
  return task_queue_.front();
}

classes::Task* Agent::getOldFirstTask()
{
  return old_task_queue_.front();
}

classes::Task* Agent::getLastTask()
{
  return task_queue_.back();
}

bool Agent::isTaskInQueue(std::string task_id)
{
  for(int i = 0; i < task_queue_.size(); i++)
  {
    if(task_queue_.front()->getID() == task_id)
      return true;
  }
  return false;
}

void Agent::setOldTaskQueue()
{
  old_task_queue_ = task_queue_;
}

void Agent::deleteOldTaskQueue()
{
  for(int i = 0; i < old_task_queue_.size(); i++)
  {
    if(old_task_queue_.front()->getType() == 'T')
      delete old_task_queue_.front();
    old_task_queue_.pop();
  }
}

int Agent::getQueueSize()
{
  return task_queue_.size();
}

void Agent::sendQueueToAgent()
{
  ntl_ac_ -> wait_for_action_server(std::chrono::seconds(5));
  mission_planner::action::NewTaskList::Goal goal;

  goal.agent_id = id_;

  auto queue_size = task_queue_.size();

  for (int i = 0; i < queue_size; i++)
  {
    mission_planner::msg::Task task_msg;
    classes::Task* task = task_queue_.front();

    task_msg.id = task->getID();
    task_msg.type = task->getType();

    switch(task_msg.type)
    {
      case 'M': case 'm':
        task_msg.monitor.human_target_id = task->getHumanID();
        task_msg.monitor.distance = task->getDistance();
        task_msg.monitor.number = task->getNumber();
        task_msg.monitor.agent_list = task->getAgentList();
        break;
      
      case 'F': case 'f':
        task_msg.monitor_ugv.ugv_id = task->getUGVID();
        task_msg.monitor_ugv.height = task->getHeight();
        break; 

      case 'I': case 'i':
        task_msg.inspect.waypoints = task->getInspectWaypoints();
        task_msg.inspect.agent_list = task->getAgentList();
        break;

      case 'A': case 'a':
        task_msg.inspect.waypoints = task->getInspectWaypoints();
        task_msg.inspect.agent_list = task->getAgentList();
        break;

      case 'D': case 'd':
        task_msg.deliver.tool_id = task->getToolID();
        task_msg.deliver.human_target_id = task->getHumanID ();
        break;

      case 'R': case 'r':
        task_msg.recharge.charging_station_id = task->getChargingStationID();
        task_msg.recharge.initial_percentage = task->getInitialPercentage();
        task_msg.recharge.final_percentage = task->getFinalPercentage();
        break;
      
      default:
        break;

    }

    goal.task_list.push_back(task_msg);
    
    task_queue_.push(task_queue_.front());
    task_queue_.pop();

  }

  ntl_ac_ -> async_send_goal(goal);

}

float Agent::computeTaskCost(classes::Task* task)
{
  float a, t, i;
  // float a, t, i, b;
  float agent_type_cost;
  float traveling_cost;
  // float battery_cost;
  float interruption_cost;

  // Cost weigths
  a = 50;
  t = 1;
  // b = 5;
  i = 3;

  // Agent type cost
  if(type_ == "PhysicalACW")
  {
    switch(task->getType())
    {
      case 'M' : case 'm':
      case 'F' : case 'f':
        agent_type_cost = 1;
        break;

      case 'I' : case 'i':
        agent_type_cost = 1;
        break;
      
      case 'A' : case 'a':
        agent_type_cost = 1;
        break;

      case 'D' : case 'd':
        agent_type_cost = 0;
        break;

      default:
        agent_type_cost = 0;
        break;
    }
  }

  else if(type_ == "InspectionACW")
  {
    switch(task->getType())
    {
      case 'M' : case 'm':
      case 'F' : case 'f':
        agent_type_cost = 0.5;
        break;

      case 'I' : case 'i':
        agent_type_cost = 0;
        break;
      
      case 'A' : case 'a':
        agent_type_cost = 0;
        break;

      case 'D' : case 'd':
        agent_type_cost = 1;
        break;

      default:
        agent_type_cost = 0;
        break;
    }
  }

  else if(type_ == "SafetyACW")
  {
    switch(task->getType())
    {
      case 'M' : case 'm':
      case 'F' : case 'f':
        agent_type_cost = 0;
        break;

      case 'I' : case 'i':
        agent_type_cost = 1;
        break;
      
      case 'A' : case 'a':
        agent_type_cost = 1;
        break;

      case 'D' : case 'd':
        agent_type_cost = 1;
        break;

      default:
        agent_type_cost = 0;
        break;
    }
  }

  // Traveling cost
  if(task_queue_.empty())
  {
    classes::Position human_position;
    classes::Position tool_position;
    classes::Position aux_position = classes::Position(0, 0, 0);

    mission_planner::msg::Waypoint central_position;
    std::vector<mission_planner::msg::Waypoint> inspect_waypoints;

    switch(task->getType())
    {
      case 'M' : case 'm':
        human_position = task->getHumanPosition();
        traveling_cost = classes::distance(position_, human_position);
        break;

      case 'F' : case 'f':
        traveling_cost = classes::distance(position_, aux_position);
        break; 

      case 'I' : case 'i':
        inspect_waypoints = task->getInspectWaypoints();
        central_position = classes::central_position(inspect_waypoints);
        traveling_cost = classes::distance(position_, central_position);
        break;

      case 'A' : case 'a':
        inspect_waypoints = task->getInspectWaypoints();
        central_position = classes::central_position(inspect_waypoints);
        traveling_cost = classes::distance(position_, central_position);
        break;

      case 'D' : case 'd':
        tool_position = task->getToolPosition();
        human_position = task->getHumanPosition();
        traveling_cost = classes::distance(position_, tool_position) + classes::distance(tool_position, human_position);
        break;

      default:
        traveling_cost = 0;
        break;
    }
  }

  else // If this is not the first task of this new asignment
  {
    classes::Position human_position;
    classes::Position tool_position;
    classes::Position previous_human_position;
    classes::Position previous_charging_position;
    classes::Position aux_position = classes::Position(0, 0, 0);

    mission_planner::msg::Waypoint central_position;
    mission_planner::msg::Waypoint previous_central_position;

    std::vector<mission_planner::msg::Waypoint> inspect_waypoints;
    std::vector<mission_planner::msg::Waypoint> previous_inspect_waypoints;

    classes::Task* previous_task = getLastTask();

    switch(previous_task->getType())
    {
      case 'M' : case 'm':
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_human_position = previous_task->getHumanPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, human_position);
            break;

          case 'F' : case 'f':
            traveling_cost = classes::distance(previous_human_position, aux_position);
            break;

          case 'I' : case 'i':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;

          case 'A' : case 'a':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;

          case 'D' : case 'd':
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            previous_human_position = previous_task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, tool_position) + classes::distance(tool_position, human_position);
            break;

          default:
            traveling_cost = 0;
            break;
        }
        break;
      
      case 'F' : case 'f':
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_human_position = previous_task->getHumanPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(aux_position, human_position);
            break;

          case 'F' : case 'f':
            traveling_cost = 0;
            break; 

          case 'I' : case 'i':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(aux_position, central_position);
            break;

          case 'A' : case 'a':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(aux_position, central_position);
            break;

          case 'D' : case 'd':
            previous_human_position = previous_task->getHumanPosition();
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(aux_position, tool_position) + classes::distance(tool_position, human_position);
            break;

          default:
            traveling_cost = 0;
            break;
        }
        break;

      case 'I' : case 'i':
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, human_position);
            break;

          case 'F' : case 'f':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, aux_position);
            break; 

          case 'I' : case 'i':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;

          case 'A' : case 'a':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;

          case 'D' : case 'd':
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, tool_position) + classes::distance(tool_position, human_position);
            break;

          default:
            traveling_cost = 0;
            break;
        }
        break;

      case 'A' : case 'a':
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, human_position);
            break;

          case 'F' : case 'f':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, aux_position);
            break;

          case 'I' : case 'i':  
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;

          case 'A' : case 'a':  
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;

          case 'D' : case 'd':
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, tool_position) + classes::distance(tool_position, human_position);
            break;

            default:
            traveling_cost = 0;
            break;
        }
        break;

      case 'D' : case 'd':  
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_human_position = previous_task->getHumanPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, human_position);
            break;

          case 'F' : case 'f':
            previous_human_position = previous_task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, aux_position);
            break; 

          case 'I' : case 'i':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;

          case 'A' : case 'a':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;

          case 'D' : case 'd':
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            previous_human_position = previous_task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, tool_position) + classes::distance(tool_position, human_position);
            break;

          default:
            traveling_cost = 0;
            break;
        }
        break;

      case 'R' : case 'r':
        switch(task -> getType())
        {
          case 'M' : case 'm':
            previous_charging_position = previous_task->getChargingStation();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_charging_position, human_position);
            break;

          case 'F' : case 'f':
            previous_charging_position = previous_task->getChargingStation();
            traveling_cost = classes::distance(previous_charging_position, aux_position);
            break; 

          case 'I' : case 'i':
            previous_charging_position = previous_task->getChargingStation();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_charging_position, central_position);
            break;

          case 'A' : case 'a':
            previous_charging_position = previous_task->getChargingStation();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_charging_position, central_position);
            break;

          case 'D' : case 'd':
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            previous_charging_position = previous_task->getChargingStation();
            traveling_cost = classes::distance(previous_charging_position, tool_position) + classes::distance(tool_position, human_position);
            break;

          default:
            traveling_cost = 0;
            break;
        }
        break;


      default:
        traveling_cost = 0;
        break;    

    }  
  }

  // Interruption cost
  if(old_task_queue_.empty())
    interruption_cost = 0;
  else if (task_queue_.empty())
  {
    classes::Task* old_first_task = getOldFirstTask();
    if(old_first_task == task)
      interruption_cost = -1;

    else
    {
      switch(old_first_task->getType())
      {
        case 'M' : case 'm':
        case 'F' : case 'f':
          switch(task -> getType())
          {
            case 'M' : case 'm':
            case 'F' : case 'f':
              interruption_cost = 1;
              break;

            case 'I' : case 'i':
              interruption_cost = 1;

            case 'A' : case 'a':
              interruption_cost = 0;
              break;
            
            case 'D' : case 'd':
              interruption_cost = 0;

            default:
              interruption_cost = 0;
              break;
          }
          break;

        case 'I' : case 'i':
          switch(task -> getType())
          {
            case 'M' : case 'm':
            case 'F' : case 'f':
              interruption_cost = 2;
              break;

            case 'I' : case 'i':
              interruption_cost = 1;

            case 'A' : case 'a':
              interruption_cost = 1;
              break;
            
            case 'D' : case 'd':
              interruption_cost = 0;

            default:
              interruption_cost = 0;
              break;
          }
          break;

        case 'A' : case 'a':
          switch(task -> getType())
          {
            case 'M' : case 'm':
            case 'F' : case 'f':
              interruption_cost = 2;
              break;
            
            case 'I' : case 'i':
              interruption_cost = 1;
              break;

            case 'A' : case 'a':
              interruption_cost = 1;
              break;

            case 'D' : case 'd':
              interruption_cost = 0;
              break;
            
            default:
              interruption_cost = 0;
              break;

            }
            break;

        case 'D' : case 'd':
          switch(task -> getType())
          {
            case 'M' : case 'm':
            case 'F' : case 'f':
              interruption_cost = 3;
              break;
            
            case 'I' : case 'i':
              interruption_cost = 2;
              break;

            case 'A' : case 'a':
              interruption_cost = 2;
              break;

            case 'D' : case 'd':
              interruption_cost = 1;
              break;
            
            default:
              interruption_cost = 0;
              break;
            }
            break;

          }
    }
  }

  else
    interruption_cost = 0;

return a * agent_type_cost + t * traveling_cost + i * interruption_cost;
// return a * agent_type_cost + t * traveling_cost + b * battery_cost + i * interruption_cost;
}


// ---------------- Class Agent Getters ---------------- //

std::string  Agent::getID()
{
  return id_;
}

std::string Agent::getType()
{
  return type_;
}

float Agent::getBattery()
{
  return battery_;
}

bool Agent::getLastBeaconTimeout()
{
  return last_beacon_.timeout;
}

// ---------------- Class Agent Setters ---------------- //

void Agent::setLastBeaconTime(rclcpp::Time last_beacon_time)
{
  last_beacon_time_ = last_beacon_time;
}

void Agent::setLastBeacon(mission_planner::msg::AgentBeacon last_beacon)
{
  last_beacon_ = last_beacon;
}

// ---------------- Class Agent Callbacks ---------------- //
void Agent::positionCallbackAS2(const geometry_msgs::msg::PoseStamped& pose)
{
  position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

void Agent::batteryCallback(const sensor_msgs::msg::BatteryState& battery)
{
  battery_ = battery.percentage;
}

rclcpp_action::GoalResponse Agent::handleBatteryEnoughGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_planner::action::BatteryEnough::Goal> goal)
{
  RCLCPP_INFO(nh_->get_logger(), "Received battery enough goal for agent %s", id_.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Agent::handleBatteryEnoughCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::BatteryEnough>> goal_handle)
{
  RCLCPP_INFO(nh_->get_logger(), "Received request to cancel battery enough goal for agent %s", id_.c_str());
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Agent::handleBatteryEnoughAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::BatteryEnough>> goal_handle)
{
  // Ejecutar en un hilo separado para no bloquear el executor
  std::thread{
    [this, goal_handle]() {
      const auto goal = goal_handle->get_goal();
      battery_enough_ = goal->value;
      
      RCLCPP_WARN(nh_->get_logger(), "Agent %s noticed that battery_enough_ = %s", 
                  id_.c_str(), (battery_enough_ ? "true" : "false"));
      
      // Publicar feedback (forma correcta en ROS2)
      auto feedback = std::make_shared<mission_planner::action::BatteryEnough::Feedback>();
      feedback->status = "battery_enough_ updated";
      goal_handle->publish_feedback(feedback);
      
      // Verificar si la acción fue cancelada
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<mission_planner::action::BatteryEnough::Result>();
        result->ack = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(nh_->get_logger(), "BatteryEnough goal canceled for agent %s", id_.c_str());
        return;
      }
      
      // Completar la acción exitosamente
      auto result = std::make_shared<mission_planner::action::BatteryEnough::Result>();
      result->ack = true;
      goal_handle->succeed(result);
      
      RCLCPP_INFO(nh_->get_logger(), "BatteryEnough goal succeeded for agent %s", id_.c_str());
      
      planner_->performTaskAllocation();
    }
  }.detach();
}

rclcpp_action::GoalResponse Agent::handleTaskResultGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_planner::action::TaskResult::Goal> goal)
{
    RCLCPP_INFO(nh_->get_logger(), "Received task result goal for agent %s, task ID: %s", 
                id_.c_str(), goal->task.id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Agent::handleTaskResultCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle)
{
    RCLCPP_INFO(nh_->get_logger(), "Received request to cancel task result goal for agent %s", id_.c_str());
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Agent::handleTaskResultAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::TaskResult>> goal_handle)
{
    std::thread{
        [this, goal_handle]() {
            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<mission_planner::action::TaskResult::Result>();
            result->ack = true;
            
            classes::Task* task = planner_->getPendingTask(goal->task.id);
            char task_type;
            
            if(planner_->getMissionOver())
            {
                if(task) {
                    task_type = task->getType();
                    RCLCPP_INFO(nh_->get_logger(), "Task %s halted because mission is over", goal->task.id.c_str());
                }
                goal_handle->succeed(result);
                return;
            }

            if(task == nullptr)
            {
                RCLCPP_INFO(nh_->get_logger(), "An already deleted Multi-UAV task ended");
                goal_handle->succeed(result);
                return;
            }

            // Check if task still exists
            task_type = task->getType();

            if(goal->task.type != task_type)
            {
                RCLCPP_INFO(nh_->get_logger(), "Task %s already deleted or modified. Ignoring result.", goal->task.id.c_str());
                goal_handle->succeed(result);
                return;
            }

            // Publicar feedback
            auto feedback = std::make_shared<mission_planner::action::TaskResult::Feedback>();
            feedback->status = "Processing task result";
            goal_handle->publish_feedback(feedback);

            // If task has been HALTED by the task queue manager as scheduled
            if(goal->result == 2)
            {
                if(task_queue_.front() == task)
                    task_queue_.pop();

                planner_->deletePendingTask(goal->task.id);
                planner_->performTaskAllocation();
            }
            // If task ended with SUCCESS
            else if(goal->result == 1)
            {
                if(task_queue_.front() == task)
                    task_queue_.pop();

                planner_->deletePendingTask(goal->task.id);
                planner_->performTaskAllocation();
            }
            // If task ended with FAILURE 
            else if(goal->result == 0)
            {
                // Check if task has been halted because of not having battery enough
                if(!battery_enough_)
                {
                    RCLCPP_WARN(nh_->get_logger(), "Battery not enough for agent: %s. Replanning...", id_.c_str());
                }
                else if(battery_ < 0.3)
                {
                    RCLCPP_WARN(nh_->get_logger(), "Task %s in %s FAILED due to low battery", goal->task.id.c_str(), id_.c_str());
                }
                else if(task == task_queue_.front())
                {
                    RCLCPP_INFO(nh_->get_logger(), "Task %s FAILED. Replanning...", goal->task.id.c_str());
                    task_queue_.pop();
                    planner_->deletePendingTask(goal->task.id);
                    planner_->performTaskAllocation();
                }
                else
                {
                    RCLCPP_INFO(nh_->get_logger(), "Task %s FAILED. But it is not the first in the queue, so it was probably postponed. Ignoring...", goal->task.id.c_str());
                }
            }

            // Request Closer Inspection if needed
            if(!goal->do_closer_inspection.xyz_coordinates.empty() || !goal->do_closer_inspection.gps_coordinates.empty())
            {
                auto do_closer_inspection_ac = rclcpp_action::create_client<mission_planner::action::DoCloserInspection>(
                    nh_, "/atrvjr/cooperation_use/do_closer_inspection");
                
                if(do_closer_inspection_ac->wait_for_action_server(std::chrono::seconds(1)))
                {
                    auto inspection_msg = mission_planner::action::DoCloserInspection::Goal();
                    
                    // CORRECCIÓN: Acceder directamente a los campos del goal y convertir tipos
                    inspection_msg.ids = goal->do_closer_inspection.ids;
                    inspection_msg.xyz_coordinates = goal->do_closer_inspection.xyz_coordinates;
                    
                    // CORRECCIÓN: Convertir GeoPose a GeoPoint
                    inspection_msg.gps_coordinates.clear();
                    for (const auto& geo_pose : goal->do_closer_inspection.gps_coordinates) {
                        inspection_msg.gps_coordinates.push_back(geo_pose.position);
                    }
                    
                    do_closer_inspection_ac->async_send_goal(inspection_msg);
                    
                    RCLCPP_INFO(nh_->get_logger(), "Sent closer inspection request with %zu coordinates", 
                              goal->do_closer_inspection.xyz_coordinates.size());
                }
            }

            // Finalizar la acción
            goal_handle->succeed(result);
        }
    }.detach();
}


// ---------------- Class Agent Visualization Method ---------------- //
void Agent::print(std::ostream& os)
{
  os << "Agent ID:" << id_;
  classes::Task* tmp;
  char task_type;
  auto queue_size = task_queue_.size();

  for(int i = 0; i < queue_size; i++)
  {
    tmp = task_queue_.front();
    task_type = tmp->getType();
    os << "\n\t" << tmp->getID() << ": " << (
        task_type == 'M' ? "Monitor" : 
        task_type == 'F' ? "MonitorUGV" : 
        task_type == 'I' ? "Inspect" : 
        task_type == 'A' ? "InspectPVArray" : 
        task_type == 'D' ? "DeliverTool" :
        task_type == 'R' ? "Recharge" :
        task_type == 'W' ? "Wait" : 
        "Task");
    task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }
}

// ---------------- Planner definitions ---------------- //
