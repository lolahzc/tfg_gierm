#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "thread"
#include "yaml-cpp/yaml.h"

#include <string>
#include <queue>
#include <map>
#include <set>
#include <sstream>
#include <functional>
#include <algorithm>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mission_planner/classes.hpp"

#include "mission_planner/msg/agent_beacon.hpp"
#include "mission_planner/msg/planner_beacon.hpp"
#include "mission_planner/msg/mission_over.hpp"
#include "mission_planner/msg/waypoint.hpp"
#include "mission_planner/msg/task.hpp"

#include "mission_planner/action/new_task.hpp"
#include "mission_planner/action/new_task_list.hpp"
#include "mission_planner/action/task_result.hpp"
#include "mission_planner/action/battery_enough.hpp"
#include "mission_planner/action/do_closer_inspection.hpp"
#include "mission_planner/action/heuristic_planning.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "as2_msgs/msg/pose_stamped_with_id.hpp" // No se si lo queremos con/sin ID

//Forward declarations
class Planner;

class Agent{
  private:
    std::string id_;
    std::string type_;
    std::queue<classes::Task*> task_queue_;
    std::queue<classes::Task*> old_task_queue_;
	std::string old_first_task_id_;

	Planner* planner_;
	rclcpp::Time last_beacon_time_;
	mission_planner::msg::AgentBeacon last_beacon_;

	//Node Handlers
	rclcpp::Node::SharedPtr nh_;

    //Subscribers
	rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr position_sub_;
	rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    //Topics
	std::string pose_topic_;
	std::string battery_topic_;
	classes::Position position_;
    float battery_; //percentage

    //Actions
    rclcpp_action::Client<mission_planner::action::NewTaskList>::SharedPtr ntl_ac_;
	rclcpp_action::Server<mission_planner::action::BatteryEnough>::SharedPtr battery_as_;
    rclcpp_action::Server<mission_planner::action::TaskResult>::SharedPtr task_result_as_;
	
    bool battery_enough_;
	
    mission_planner::action::BatteryEnough::Feedback battery_feedback_;
	mission_planner::action::BatteryEnough::Result battery_result_;

  public:
    //Constructors
    Agent();
		Agent(Planner* planner, std::string id, std::string type, rclcpp::Time first_beacon_time,
				mission_planner::msg::AgentBeacon first_beacon);
    Agent(const Agent& a);
    ~Agent();

    //Topic methods
	void updateSensorsInformation();
	bool isBatteryForQueue();
	bool isBatteryEnough(classes::Task* task);
	bool checkBeaconTimeout(rclcpp::Time now);

    //Task queue methods
    bool isQueueEmpty();
    void emptyTheQueue();
    void addTaskToQueue(classes::Task* task);
    void replaceTaskFromQueue(std::string task_id);
	classes::Task* getFirstTask();
	classes::Task* getOldFirstTask();
	classes::Task* getLastTask();
	bool isTaskInQueue(std::string task_id);
	void setOldTaskQueue();
	void deleteOldTaskQueue();
    int getQueueSize();
	void sendQueueToAgent();
	float computeTaskCost(classes::Task* task);

	//Getters
    std::string getID();
    std::string getType();
	float getBattery();
	bool getLastBeaconTimeout();

	//Setters
	void setLastBeaconTime(rclcpp::Time last_beacon_time);
	void setLastBeacon(mission_planner::msg::AgentBeacon last_beacon);

	//Callbacks
    void positionCallbackAS2(const geometry_msgs::msg::PoseStamped& pose);
    void batteryCallback(const sensor_msgs::msg::BatteryState& battery);
	void batteryEnoughCB(const mission_planner::action::BatteryEnough_Goal::ConstPtr& goal);
	void taskResultCB(const mission_planner::action::TaskResult_Goal::ConstPtr& goal);

    //Visualization method
    void print(std::ostream& os);
};

std::ostream& operator << (std::ostream& os, Agent& a){
  a.print(os);
  return os;
}

class Planner {
  private:
    //Node Handlers
    rclcpp::Node nh_;

	std::unique_ptr<rclcpp_action::Client<mission_planner::action::HeuristicPlanning>> hp_ac_;
	// actionlib::SimpleActionClient<mission_planner::HeuristicPlanningAction> hp_ac_;
	rclcpp_action::Server<mission_planner::action::NewTask> nt_as_;

    mission_planner::action::NewTask_Feedback nt_feedback_;
    mission_planner::action::NewTask_Result nt_result_;

    // Subscribers
    rclcpp::Subscription<mission_planner::msg::AgentBeacon>::SharedPtr beacon_sub_;
	rclcpp::Subscription<mission_planner::msg::MissionOver>::SharedPtr mission_over_sub_;
	bool mission_over_;

    // Publishers
	rclcpp::Publisher<mission_planner::msg::PlannerBeacon>::SharedPtr beacon_pub_;
    mission_planner::msg::PlannerBeacon beacon_;
    rclcpp::Rate beacon_rate_;

    std::string config_file;
	std::string mission_id_;

	std::map <std::string, std::map <std::string, classes::Position>> known_positions_;
    std::map <std::string, classes::HumanTarget> human_targets_;
    std::map <std::string, classes::Tool> tools_;

    std::map <std::string, Agent> agent_map_;
	std::vector <std::string> deliver_agents_;
	std::vector <std::string> inspect_agents_;
	std::vector <std::string> monitor_agents_;

    std::map <std::string, classes::Task*> pending_tasks_;
	classes::Task* recharge_task_;
	std::vector <std::string> deliver_tasks_;
	std::vector <std::string> inspect_tasks_;
	std::vector <std::string> monitor_tasks_;

  protected:
    void readConfigFile(std::string config_file);

  public:
	Planner(mission_planner::msg::PlannerBeacon beacon);
    ~Planner(void);

	bool checkTaskParams(const mission_planner::action::NewTask_Goal::ConstPtr& goal);

    //New Tasks callback
    void incomingTask(const mission_planner::action::NewTask_Goal::ConstPtr& goal);

    //Agent Beacon Handler
    void beaconCallback(const mission_planner::msg::AgentBeacon::ConstPtr& beacon);
	void missionOverCallback(const mission_planner::msg::MissionOver& value);

    //Method to reassign all not finished tasks
    void performTaskAllocation();

	//Pending Tasks methods
	classes::Task* getPendingTask(std::string task_id);
	void deletePendingTask(std::string task_id);
	bool updateTaskParams(const mission_planner::action::NewTask_Goal::ConstPtr& goal);
	void checkBeaconsTimeout(rclcpp::Time now);
	
	//Getters
	bool getMissionOver();

	// Others
	bool isTopicAvailable(const std::string &topic_name);
};

struct Cost{
	public:
		Cost(float cost, std::string id) : cost_(cost), id_(id) {}

		bool operator < (const Cost& cost) const {return (cost_ < cost.cost_);}
		bool operator > (const Cost& cost) const {return (cost_ > cost.cost_);}
		bool operator == (const Cost& cost) const {return (cost_ == cost.cost_);}

		float cost_;
		std::string id_;
};

#endif