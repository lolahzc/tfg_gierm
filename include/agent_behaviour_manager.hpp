#ifndef AGENT_H
#define AGENT_H

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include <string>
#include <queue>
#include <sstream>
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "mission_planner/classes.hpp"
#include "mission_planner/msg/agent_beacon.hpp"
#include "mission_planner/msg/planner_beacon.hpp"
#include "mission_planner/action/new_task_list.hpp"
#include "mission_planner/action/battery_enough.hpp"
#include "mission_planner/action/task_result.hpp"
#include "mission_planner/msg/mission_over.hpp"
#include "mission_planner/msg/waypoint.hpp"
#include "mission_planner/msg/task.hpp"
#include "mission_planner/action/request_mobile_charging_station.hpp"

// Nada segura de que esto vaya a funcionar de la misma manera que UAL
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/action/takeoff.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp"


// Forward declaration
class AgentNode;

// BT nodes declaration
class GoNearChargingStation : public BT::AsyncActionNode
{
  private :
        AgentNode* agent_;

  public : 
        GoNearChargingStation(const std::string& name, const BT::NodeConfiguration& config);
        ~GoNearChargingStation();
    void init(AgentNode* agent);
        static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
        
};

// Recharge
class Recharge : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		Recharge(const std::string& name, const BT::NodeConfiguration& config);
		~Recharge();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//BackToStation
class BackToStation : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		BackToStation(const std::string& name, const BT::NodeConfiguration& config);
		~BackToStation();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearHumanTarget
class GoNearHumanTarget : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearHumanTarget(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearHumanTarget();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//MonitorHumanTarget
class MonitorHumanTarget : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		MonitorHumanTarget(const std::string& name, const BT::NodeConfiguration& config);
		~MonitorHumanTarget();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearUGV
class GoNearUGV : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearUGV(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearUGV();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//MonitorUGV
class MonitorUGV : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		MonitorUGV(const std::string& name, const BT::NodeConfiguration& config);
		~MonitorUGV();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearWP
class GoNearWP : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearWP(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearWP();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//TakeImage
class TakeImage : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		TakeImage(const std::string& name, const BT::NodeConfiguration& config);
		~TakeImage();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//InspectPVArray
class InspectPVArray : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		InspectPVArray(const std::string& name, const BT::NodeConfiguration& config);
		~InspectPVArray();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearStation
class GoNearStation : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearStation(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearStation();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//PickTool
class PickTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		PickTool(const std::string& name, const BT::NodeConfiguration& config);
		~PickTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//DropTool
class DropTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		DropTool(const std::string& name, const BT::NodeConfiguration& config);
		~DropTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//DeliverTool
class DeliverTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		DeliverTool(const std::string& name, const BT::NodeConfiguration& config);
		~DeliverTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};


// BT Conditions

//MissionOver
class MissionOver : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		MissionOver(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//Idle
class Idle : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		Idle(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsBatteryEnough
class IsBatteryEnough : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsBatteryEnough(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsBatteryFull
class IsBatteryFull : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsBatteryFull(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskRecharge
class IsTaskRecharge : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskRecharge(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskMonitor
class IsTaskMonitor : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskMonitor(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskMonitorUGV
class IsTaskMonitorUGV : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskMonitorUGV(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskInspect
class IsTaskInspect : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskInspect(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskInspectPVArray
class IsTaskInspectPVArray : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskInspectPVArray(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskDeliverTool
class IsTaskDeliverTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskDeliverTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearChargingStation
class IsAgentNearChargingStation : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearChargingStation(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearHumanTarget
class IsAgentNearHumanTarget : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearHumanTarget(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearUGV
class IsAgentNearUGV : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearUGV(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearWP
class IsAgentNearWP : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearWP(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//NeedToDropTheTool
class NeedToDropTheTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		NeedToDropTheTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//HasAgentTheTool
class HasAgentTheTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		HasAgentTheTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearStation
class IsAgentNearStation : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearStation(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//******************************* Decorators
//ForceRunnigNode
class ForceRunningNode : public BT::DecoratorNode
{
  public:
    ForceRunningNode(const std::string& name);

  private:
    virtual BT::NodeStatus tick() override;
};


// Behavior Tree Nodes Registration
inline void RegisterNodes(BT::BehaviorTreeFactory& factory);


//Agent Node Class declaration **************************************************************************************
class AgentNode
{
  //Actions
	friend class GoNearChargingStation;
  friend class Recharge;
  friend class BackToStation;
  friend class GoNearHumanTarget;
  friend class MonitorHumanTarget;
  friend class GoNearUGV;
  friend class MonitorUGV;
  friend class GoNearWP;
  friend class TakeImage;
  friend class InspectPVArray;
  friend class GoNearStation;
  friend class PickTool;
  friend class DropTool;
  friend class DeliverTool;

  //Conditions
  friend class MissionOver;
  friend class Idle;
  friend class IsBatteryEnough;
  friend class IsBatteryFull;
  friend class IsTaskRecharge;
  friend class IsTaskMonitor;
  friend class IsTaskMonitorUGV;
  friend class IsTaskInspect;
  friend class IsTaskInspectPVArray;
  friend class IsTaskDeliverTool;
  friend class IsTaskRecharge;
  friend class IsAgentNearHumanTarget;
  friend class IsAgentNearUGV;
  friend class IsAgentNearWP;
  friend class NeedToDropTheTool;
  friend class HasAgentTheTool;
  friend class IsAgentNearStation;
  friend class IsAgentNearChargingStation;

  //Decorators
  friend class ForceRunningNode;

  private :

  rclcpp::Node::SharedPtr nh_;

  std::string id_;
  std::string ns_prefix_;

  geographic_msgs::msg::GeoPoint origin_geo_;

  rclcpp_action::Server<mission_planner::action::NewTaskList>::SharedPtr ntl_as_;
  rclcpp_action::Client<mission_planner::action::BatteryEnough>::SharedPtr battery_ac_;

  // Subscribers
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr state_sub_; // Aquí ya no tengo State como en UAL, revisar porque cambiar exactamente
  rclcpp::Subscription<mission_planner::msg::MissionOver>::SharedPtr mission_over_sub_;
  rclcpp::Subscription<mission_planner::msg::PlannerBeacon>::SharedPtr planner_beacon_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPose>::SharedPtr atrvjr_geopose_sub_; //No sé si es con ID/Stamped o qué, mirarlo bien
  rclcpp::Subscription<geographic_msgs::msg::GeoPose>::SharedPtr jackal_geopose_sub_; 

  classes::Position position_;
  float battery_;
  int state_;
  bool mission_over_;
  rclcpp::Time last_beacon_;
  bool timeout_;
  classes::Position atrvjr_pose_;
  classes::Position jackal_pose_;
  float take_off_height_;
  float distance_error_;
  float goto_error_;

  // Publishers
  rclcpp::Publisher<mission_planner::msg::PlannerBeacon>::SharedPtr beacon_pub_;
  mission_planner::msg::PlannerBeacon beacon_;
  rclcpp::Rate loop_rste_;

  std::queue<classes::Task*> task_queue_;
  bool battery_enough_;
  std::string tool_flag_;
  std::string pose_frame_id_;
  std::string pose_topic_;
  std::string state_topic_;
  std::string battery_topic_;

  std::string config_file_;
  std::string config_file_evora_;

  std::map <std::string, std::map <std::string, classes::Position>> known_positions_;
  std::map <std::string, classes::HumanTarget> human_targets_;
  std::map <std::string, classes::Tool> tools_;
  
protected:

  void ReadConfigFile(std::string config_file);
  void ReadConfigFileEvora(std::string congig_file);

public : 

  AgentNode(mission_planner::msg::AgentBeacon beacon);
  ~AgentNode(void);

  void isBatteryEnough();
  void addTaskFromQueue(classes::Task* task);
  void removeTaskFromQueue(std::string id, char type);
  void emptyTheQueue();
  int getQueueSize();
  void infoQueue();
  void taskQueueManager();

  void newTaskList(const std::shared_ptr<const mission_planner::action::NewTaskList::Goal> goal);
  void positionCallbackAS2(const geometry_msgs::msg::PoseStamped& pose);
  void batteryCallback(const sensor_msgs::msg::BatteryState& battery);
  void stateCallbackAS2(); // RELLENAR CON LO QUE SEA QUE USEMOS PARA SACAR EL STATE, ANTES SE SACABA DIRECTAMENTE DE UAL
  void beaconCallback(const mission_planner::msg::AgentBeacon::SharedPtr beacon);
  void missionOverCallback(const mission_planner::msg::MissionOver::SharedPtr value);
  void checkBeaconsTimeout(rclcpp::Time now);

  void atrvjrPositionCallback(const geographic_msgs::msg::GeoPoseStamped& geo_pose);
  void jackalPositionCallback(const geographic_msgs::msg::GeoPoseStamped& geo_pose);


  bool land(bool blocking);
  bool takeoff(float height, bool blocking);
  bool go_to_waypoint(float x, float y, float z, bool blocking);
  bool stop(bool blocking);
  bool checkIfGoToServiceSucceeded(float x, float y, float z);

}; 

#endif