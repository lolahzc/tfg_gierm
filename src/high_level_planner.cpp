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
  position_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>("/" + id_ + pose_topic_, 10,
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
  position_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
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
Planner::Planner(mission_planner::msg::PlannerBeacon beacon) : 
  rclcpp::Node("planner_node"),
  beacon_rate_(1),
  beacon_(beacon),
  mission_over_(false),
  recharge_task_(nullptr) 

{
    nt_as_ = rclcpp_action::create_server<mission_planner::action::NewTask>(
      this,
      "incoming_task_action",
      std::bind(&Planner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Planner::handle_cancel, this, std::placeholders::_1),
      std::bind(&Planner::handle_accepted, this, std::placeholders::_1)
    );

    // Cargar posiciones conocidas y objetos humanos
    std::string path = ament_index_cpp::get_package_share_directory("mission_planner");
    declare_parameter<std::string>("config_file", path + "/config/conf.yaml");
    get_parameter("config_file", config_file);

    readConfigFile(config_file);

    // Crear publishers y subscribers
    beacon_pub_ = create_publisher<mission_planner::msg::PlannerBeacon>(
      "/planner_beacon", 
      10);
    
      beacon_sub_ = create_subscription<mission_planner::msg::AgentBeacon>(
      "/agent_beacon", 
      100,
      std::bind(&Planner::beaconCallback, this, std::placeholders::_1));

    mission_over_sub_ = create_subscription<mission_planner::msg::MissionOver>(
      "/mission_over", 
      1,
      std::bind(&Planner::missionOverCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Planner node initialized.");

    recharge_task_ = new classes::Recharge("Recharge", 0.0, 1.0);

    RCLCPP_INFO(get_logger(), "Waiting Matlab's heuristic planning action server to be available...");

    hp_ac_ = rclcpp_action::create_client<mission_planner::action::HeuristicPlanning>(
      this,
      "/heuristic_planning");

    rclcpp::Rate waiting_server_rate(std::chrono::seconds(1));
    while(rclcpp::ok() && !isTopicAvailable("/heuristic_planning/status"))
    {
      waiting_server_rate.sleep();
    }

    RCLCPP_INFO(get_logger(), "[Planner] Entering main while loop...");

    rclcpp::Rate beacon_rate(beacon_rate_);
    while(rclcpp::ok() && !mission_over_) {
      checkBeaconsTimeout(now());
      beacon_.timestamp = now();
      beacon_pub_->publish(beacon_);
      rclcpp::spin_some(get_node_base_interface());
      beacon_rate.sleep(); 
    }

    RCLCPP_INFO(this->get_logger(), "[Planner] Mission Over. Waiting for the agents to finish");
    
    while(rclcpp::ok() && !agent_map_.empty()) {
      checkBeaconsTimeout(now());
      rclcpp::spin_some(get_node_base_interface());
      beacon_rate.sleep();
    }

    for(auto &task: pending_tasks_)
      delete task.second;
    pending_tasks_.clear();

}

Planner::~Planner(void){}


rclcpp_action::GoalResponse Planner::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_planner::action::NewTask::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received new task goal with ID: %s", goal->task.id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Planner::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTask>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel task");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Planner::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTask>> goal_handle)
{
    // Ejecutar en un hilo separado para no bloquear el executor
    std::thread{
        [this, goal_handle]() {
            this->execute_incoming_task(goal_handle);
        }
    }.detach();
}

void Planner::execute_incoming_task(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTask>> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<mission_planner::action::NewTask::Result>();
    auto feedback = std::make_shared<mission_planner::action::NewTask::Feedback>();

    try {
        // Publicar feedback inicial
        feedback->status = "Reading the New Task";
        goal_handle->publish_feedback(feedback);

        // Procesar la tarea
        std::string id = goal->task.id;
        char type = goal->task.type;
        char old_type;

        // Check if the task params are correct
        if(!checkTaskParams(goal))
        {
            RCLCPP_WARN(get_logger(), "[incomingTask] Incorrect task");
            
            auto task_itr = pending_tasks_.find(id);
            if(task_itr != pending_tasks_.end())
            {
                old_type = task_itr->second->getType();
                RCLCPP_INFO_STREAM(get_logger(), "[incomingTask] " << id << "(" << (
                    old_type == 'M' ? "Monitor" : 
                    old_type == 'I' ? "Inspect" :
                    old_type == 'A' ? "InspectPVArray" :
                    old_type == 'D' ? "DeliverTool" : 
                    old_type == 'R' ? "Recharge" : 
                    old_type == 'W' ? "Wait" : 
                    "Task") << ") is going to be deleted");
                
                deletePendingTask(id);
                performTaskAllocation();
            }
            
            result->ack = false;
            goal_handle->succeed(result);
            return;
        }

        feedback->status = "Checking if the New Task already exists";
        goal_handle->publish_feedback(feedback);

        // Check if the new task already exist
        auto task_itr = pending_tasks_.find(id);
        if(task_itr != pending_tasks_.end())
        {
            old_type = task_itr->second->getType();
            if(type == old_type)
            {
                if(!updateTaskParams(goal))
                {
                    result->ack = false;
                    goal_handle->abort(result);
                    return;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "[incomingTask] Task Params Updated. Allocating tasks...");
                    feedback->status = "Allocating pending tasks";
                    goal_handle->publish_feedback(feedback);
                    performTaskAllocation();
                    result->ack = true;
                    goal_handle->succeed(result);
                    return;
                }
            }
            else
            {
                RCLCPP_WARN_STREAM(get_logger(), "[incomingTask] Duplicated ID. An unfinished task is going to be deleted: " << id << "(" << (
                    old_type == 'M' ? "Monitor" : 
                    old_type == 'F' ? "MonitorUGV" : 
                    old_type == 'I' ? "Inspect" : 
                    old_type == 'A' ? "InspectPVArray" : 
                    old_type == 'D' ? "DeliverTool" : 
                    old_type == 'R' ? "Recharge" : 
                    old_type == 'W' ? "Wait" : 
                    "Task") << ")");
                deletePendingTask(id);
            }
        }

        feedback->status = "Adding the New Task to pending_tasks_ map";
        goal_handle->publish_feedback(feedback);

        // Crear la nueva tarea
        switch(goal->task.type)
        {
            case 'M':
            case 'm':
                pending_tasks_[id] = new classes::Monitor(id, &(human_targets_[goal->task.monitor.human_target_id]),
                    goal->task.monitor.distance, goal->task.monitor.number);
                monitor_tasks_.push_back(id);
                break;
            case 'F':
            case 'f':
                pending_tasks_[id] = new classes::MonitorUGV(id, goal->task.monitor_ugv.ugv_id, goal->task.monitor_ugv.height);
                monitor_tasks_.push_back(id);
                break;
            case 'I':
            case 'i':
                pending_tasks_[id] = new classes::Inspect(id, goal->task.inspect.waypoints);
                inspect_tasks_.push_back(id);
                break;
            case 'A':
            case 'a':
                pending_tasks_[id] = new classes::InspectPVArray(id, goal->task.inspect.waypoints);
                inspect_tasks_.push_back(id);
                break;
            case 'D':
            case 'd':
                pending_tasks_[id] = new classes::DeliverTool(id, &(tools_[goal->task.deliver.tool_id]),
                    &(human_targets_[goal->task.deliver.human_target_id]));
                deliver_tasks_.push_back(id);
                break;
            default:
                break;
        }
        
        RCLCPP_INFO_STREAM(get_logger(), "[incomingTask] Received a New Task:\n" << *pending_tasks_[id]);
        RCLCPP_INFO(get_logger(), "[incomingTask] Allocating tasks...");

        feedback->status = "Allocating pending tasks";
        goal_handle->publish_feedback(feedback);

        performTaskAllocation();

        // Verificar si se canceló la acción
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(), "[incomingTask] Incoming Task: Preempted");
            result->ack = false;
            goal_handle->canceled(result);
            return;
        }

        result->ack = true;
        goal_handle->succeed(result);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in execute_incoming_task: %s", e.what());
        result->ack = false;
        goal_handle->abort(result);
    }
}

void Planner::readConfigFile(std::string config_file)
{
  YAML::Node yaml_config = YAML::LoadFile(config_file);

  if(yaml_config["mission_id"])
    mission_id_ = yaml_config["mission_id"].as<std::string>();

  if(yaml_config["positions"])
  {
    for(auto const& group: yaml_config["positions"])
    {
      for(auto const& position: group.second)
      {
        known_positions_[group.first.as<std::string>()][position.first.as<std::string>()] = 
          classes::Position(
            position.first.as<std::string>(),
            position.second["x"].as<float>(),
            position.second["y"].as<float>(),
            position.second["z"].as<float>());
      }
    }
  }

  if(yaml_config["human_targets"])
  {
    for(auto const& human_target: yaml_config["human_targets"])
    {
      human_targets_[human_target.first.as<std::string>()] = 
        classes::HumanTarget(
          human_target.first.as<std::string>(),
          human_target.second["x"].as<float>(),
          human_target.second["y"].as<float>(),
          human_target.second["z"].as<float>());
    }
  }

  if(yaml_config["tools"])
  {
    for(auto const& tool: yaml_config["tools"])
    {
      tools_[tool.first.as<std::string>()] = 
        classes::Tool(
          tool.first.as<std::string>(),
          tool.second["weight"].as<float>(),
          tool.second["x"].as<float>(),
          tool.second["y"].as<float>(),
          tool.second["z"].as<float>());
    }
  }
}

bool Planner::checkTaskParams(const std::shared_ptr<const mission_planner::action::NewTask::Goal> goal)
{
  std::map <std::string, classes::HumanTarget>::iterator human_itr;
  std::map <std::string, classes::Position>::iterator position_itr;
  std::map <std::string, classes::Tool>::iterator tool_itr;

  switch(goal->task.type)
  {
    case 'M': case 'm':
      human_itr = human_targets_.find(goal->task.monitor.human_target_id);
      if(human_itr == human_targets_.end())
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: unknown human target ID"); 
        return false;
      }
      if(goal->task.monitor.distance == 0)
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: monitor distance cannot be zero"); 
        return false;
      }
      if(goal->task.monitor.number == 0)
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: number of monitors cannot be zero"); 
        return false;
      }
      break;

    case 'F': case 'f':
      if(goal->task.monitor_ugv.ugv_id == "")
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: unknown UGV ID"); 
        return false;
      }
      if(goal->task.monitor_ugv.height == 0)
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: monitor height cannot be zero"); 
        return false;
      }
      break;

    case 'I': case 'i':
      if(goal->task.inspect.waypoints.size() == 0)
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: no inspect waypoints provided"); 
        return false;
      }
      break;

    case 'A' : case 'a':
      if(goal->task.inspect.waypoints.size() != 2)
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: there has to be 2 waypoints");
        return false;
      }
      break;

    case 'D': case 'd':
      human_itr = human_targets_.find(goal->task.deliver.human_target_id);
      if(human_itr == human_targets_.end())
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: unknown human target ID"); 
        return false;
      }
      tool_itr = tools_.find(goal->task.deliver.tool_id);
      if(tool_itr == tools_.end())
      {
        RCLCPP_INFO(get_logger(), "[CheckTaskParams] Invalid task: unknown tool ID"); 
        return false;
      }
      break;

      default:
        return false;
        break;
  
    }
  
    return true;

}

void Planner::beaconCallback(const mission_planner::msg::AgentBeacon::SharedPtr beacon)
{
  auto agent_itr = agent_map_.find(beacon->id);
  if(agent_itr == agent_map_.end())
  {
    RCLCPP_INFO(get_logger(), "[beaconCallback] New Agent connected: %s", beacon->id.c_str());
    Agent agent(this, beacon->id, beacon->type, now(), *beacon);
    agent_map_.emplace(beacon->id, agent);

    if(beacon->type == "PhysicalACW")
      deliver_agents_.push_back(beacon->id);
    else if(beacon->type == "InspectionACW")
      inspect_agents_.push_back(beacon->id);
    else if(beacon->type == "SafetyACW")
      monitor_agents_.push_back(beacon->id);

    performTaskAllocation();

  }

  else
  {
    agent_map_[beacon->id].setLastBeaconTime(now());

    if(!beacon->timeout && agent_map_[beacon->id].getLastBeaconTimeout())
      RCLCPP_WARN_STREAM(get_logger(), "[beaconCallback] (" << beacon->id << ") Disconnected briefly, activated the emergency " 
          << "protocol by emptying its task queue and reconnected without Planner noticing");
      agent_map_[beacon->id].sendQueueToAgent();

  }

  agent_map_[beacon->id].setLastBeacon(*beacon);

}

void Planner::missionOverCallback(const mission_planner::msg::MissionOver::SharedPtr value)
{
  mission_over_ = value->value;
}

void Planner::performTaskAllocation()
{
  RCLCPP_INFO(get_logger(), "[performTaskAllocation] Waiting for Heuristic Planning action server to be available...");

  if (!hp_ac_->wait_for_action_server(std::chrono::seconds(1))) 
  {
    RCLCPP_WARN(get_logger(), "[performTaskAllocation] Heuristic planning action server not available");
    return;
  }

  auto goal = mission_planner::action::HeuristicPlanning::Goal();
  
  for(auto &agent : agent_map_)
  {
    if(agent.second.getBattery() > 0.3)
    {
      goal.available_agents.push_back(agent.first);
    }  
  }

  for(auto &task : pending_tasks_)
  {
    goal.remaining_tasks.push_back(task.first);
  }

  RCLCPP_INFO(get_logger(), "[performTaskAllocation] Requesting a task allocation...");

  auto future_goal_handle = hp_ac_->async_send_goal(goal);

  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto goal_handle = future_goal_handle.get();
    if(goal_handle)
    {
      auto future_result = hp_ac_->async_get_result(goal_handle);

      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS)
      {
        auto result = future_result.get();

        if(result.result->success)
        {
          for(auto &agent : agent_map_)
          {
            agent.second.setOldTaskQueue();
            agent.second.emptyTheQueue();
          }

          for(auto &queue : result.result->planning_result)
          {
            auto agent = queue.agent_id;
            for(auto &task : queue.queue)
            {
              if(task.id == "t_R")
              {
                agent_map_[agent].addTaskToQueue(recharge_task_);
              }
              else
              {
                agent_map_[agent].addTaskToQueue(pending_tasks_[task.id]);
              }
            }
          }

          RCLCPP_INFO(get_logger(), "[performTaskAllocation] Tasks Allocated:");
          for(auto &agent : agent_map_)
          {
            RCLCPP_INFO_STREAM(get_logger(), "[performTaskAllocation] " << agent.second);
          }

          for(auto &agent : agent_map_)
          {
            agent.second.sendQueueToAgent();
          }

          for(auto &agent : agent_map_)
          {
            agent.second.deleteOldTaskQueue();
          }

        }
        else
        {
          RCLCPP_WARN(get_logger(), "[performTaskAllocation] Task planning failed. %lu agents connected. %lu pending tasks", 
                      agent_map_.size(), pending_tasks_.size());
        }

      }
      else
      {
        RCLCPP_WARN(get_logger(), "[performTaskAllocation] Timeout waiting for result");
      }

    }

    else
    {
      RCLCPP_WARN(get_logger(), "[performTaskAllocation] Goal was rejected by server");
    }

  }

  else
  {
    RCLCPP_WARN(get_logger(), "[performTaskAllocation] Timeout reached");
  }
  return;
}

classes::Task* Planner::getPendingTask(std::string task_id)
{
    std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(task_id);
    if(task_itr != pending_tasks_.end())
        return task_itr->second;
    else
        return nullptr;
}

void Planner::deletePendingTask(std::string task_id)
{
  // Check if the task exists
  auto task_itr = pending_tasks_.find(task_id);
  if(task_itr != pending_tasks_.end())
  {
      char type = task_itr->second->getType();
      // Delete task from type_task_queue
      switch(type)
      {
          case 'M':
          case 'm':
          case 'F':
          case 'f':
              monitor_tasks_.erase(std::find(monitor_tasks_.begin(), monitor_tasks_.end(), task_itr->second->getID()));
              break;
          case 'I':
          case 'i':
              inspect_tasks_.erase(std::find(inspect_tasks_.begin(), inspect_tasks_.end(), task_itr->second->getID()));
              break;
          case 'A':
          case 'a':
              inspect_tasks_.erase(std::find(inspect_tasks_.begin(), inspect_tasks_.end(), task_itr->second->getID()));
              break;
          case 'D':
          case 'd':
              deliver_tasks_.erase(std::find(deliver_tasks_.begin(), deliver_tasks_.end(), task_itr->second->getID()));
              break;
          default:
              break;
      }
      // Delete task from agents_queue (because old_task_queue will use the pointer)
      if(!agent_map_.empty()) {
          for(auto &agent: agent_map_) {
              agent.second.replaceTaskFromQueue(task_id);
          }
      }
      // Delete task pointer
      delete task_itr->second;
      pending_tasks_.erase(task_itr);
  }
  return;
}


bool Planner::updateTaskParams(const std::shared_ptr<const mission_planner::action::NewTask::Goal> goal)
{
    classes::Task* aux = nullptr;
    std::string id = goal->task.id;
    char type = goal->task.type;

    // Mover las declaraciones fuera del switch
    std::string m_human_target_id;
    float distance;
    int number;
    std::string d_tool_id;
    std::string d_human_target_id;
    std::map<std::string, classes::HumanTarget>::iterator human_itr;
    std::map<std::string, classes::Tool>::iterator tool_itr;

    // Check if the task exists
    auto task_itr = pending_tasks_.find(id);
    if(task_itr != pending_tasks_.end())
    {
        switch(type)
        {
            case 'M':
            case 'm':
            {
                m_human_target_id = goal->task.monitor.human_target_id;
                distance = goal->task.monitor.distance;
                number = goal->task.monitor.number;

                human_itr = human_targets_.find(m_human_target_id);
                if(human_itr == human_targets_.end())
                {
                    RCLCPP_INFO(get_logger(), "[updateTaskParams] Invalid task: unknown human target ID");
                    return false;
                }
                aux = new classes::Monitor(id, &(human_targets_[m_human_target_id]), distance, number);
                task_itr->second->updateParams(aux);
                break;
            }
            case 'F':
            case 'f':
            {
                aux = new classes::MonitorUGV(id, goal->task.monitor_ugv.ugv_id, goal->task.monitor_ugv.height);
                task_itr->second->updateParams(aux);
                break;
            }
            case 'I':
            case 'i':
            {
                aux = new classes::Inspect(id, goal->task.inspect.waypoints);
                task_itr->second->updateParams(aux);
                break;
            }
            case 'A':
            case 'a':
            {
                aux = new classes::InspectPVArray(id, goal->task.inspect.waypoints);
                task_itr->second->updateParams(aux);
                break;
            }
            case 'D':
            case 'd':
            {
                d_tool_id = goal->task.deliver.tool_id;
                d_human_target_id = goal->task.deliver.human_target_id;

                auto human_itr_d = human_targets_.find(d_human_target_id);
                if(human_itr_d == human_targets_.end())
                {
                    RCLCPP_INFO(get_logger(), "[updateTaskParams] Invalid task: unknown human target ID");
                    return false;
                }
                auto tool_itr = tools_.find(d_tool_id);
                if(tool_itr == tools_.end())
                {
                    RCLCPP_INFO(get_logger(), "[updateTaskParams] Invalid task: unknown tool ID");
                    return false;
                }
                aux = new classes::DeliverTool(id, &(tools_[d_tool_id]), &(human_targets_[d_human_target_id]));
                task_itr->second->updateParams(aux);
                break;
            }
            default:
                break;
        }
    }

    if (aux != nullptr) {
        delete aux;
    }

    return true;
}

void Planner::checkBeaconsTimeout(rclcpp::Time now)
{
    // Delete disconnected Agent from agent_map_ and from the corresponding type_agents_ vector, and send them an empty
    // task just in case they still listen to this block
    std::queue <std::string> disconnected_agents;
    auto prev_size = agent_map_.size();
    std::string type;
    std::string id;

    if(!agent_map_.empty())
    {
        for(auto &agent: agent_map_) {
            if(agent.second.checkBeaconTimeout(now)) {
                disconnected_agents.push(agent.first);
            }
        }

        while(!disconnected_agents.empty())
        {
            id = disconnected_agents.front();
            RCLCPP_WARN_STREAM(get_logger(), "[checkBeaconsTimeout] Beacon Timeout. " << id << " disconnected.");

            // Erase disconnected UAV from agent type corresponding list
            type = agent_map_[id].getType();
            if(type == "PhysicalACW") {
                deliver_agents_.erase(std::find(deliver_agents_.begin(), deliver_agents_.end(), id));
            } else if(type == "InspectionACW") {
                inspect_agents_.erase(std::find(inspect_agents_.begin(), inspect_agents_.end(), id));
            } else if(type == "SafetyACW") {
                monitor_agents_.erase(std::find(monitor_agents_.begin(), monitor_agents_.end(), id));
            }

            // Send to the disconnected UAV an empty queue just in case
            RCLCPP_WARN_STREAM(get_logger(), "[checkBeaconsTimeout] Sending " << id << " an empty queue just in case");
            agent_map_[id].emptyTheQueue();
            agent_map_[id].sendQueueToAgent();

            // Erase it from memory
            agent_map_.erase(id);
            disconnected_agents.pop();
        }

        if(agent_map_.size() != prev_size)
        {
            RCLCPP_INFO_STREAM(get_logger(), "[checkBeaconsTimeout] Connected Agents: " << agent_map_.size() << ". Perform Task Allocation:");
            performTaskAllocation();
        }
    }

    return;
}


// Getters
bool Planner::getMissionOver() { return mission_over_; }

// Others
bool Planner::isTopicAvailable(const std::string &topic_name)
{
    auto topics = this->get_topic_names_and_types();
    
    for (const auto &topic : topics) {
        if (topic.first == topic_name) {
            return true;
        }
    }
    return false;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    mission_planner::msg::PlannerBeacon beacon;
    auto planner = std::make_shared<Planner>(beacon);
    
    RCLCPP_INFO(planner->get_logger(), "[main] Starting high_level_planner...");
    
    // ROS2 usa executors en lugar de spin en el main
    rclcpp::spin(planner);
    
    RCLCPP_INFO(planner->get_logger(), "[main] Ending...");
    rclcpp::shutdown();
    
    return 0;
}