#ifndef AGENT_BEHAVIOUR_MANAGER_H
#define AGENT_BEHAVIOUR_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yaml-cpp/yaml.h"

#include <string>
#include <sstream>
#include <queue>
#include <memory>
#include <mutex>
#include <atomic>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

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

// Aerostack2 official includes
#include "as2_msgs/msg/platform_info.hpp"
#include "as2_msgs/action/takeoff.hpp"
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"

#include <ament_index_cpp/get_package_share_directory.hpp> 

// Forward declaration
class AgentNode;

// Behavior Tree Nodes declaration ***********************************************************************************
// ******************************* Actions
class GoNearChargingStation : public BT::AsyncActionNode
{
private:
    AgentNode* agent_;

public:
    GoNearChargingStation(const std::string& name, const BT::NodeConfiguration& config);
    ~GoNearChargingStation();
    void init(AgentNode* agent);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

class Recharge : public BT::AsyncActionNode
{
private:
    AgentNode* agent_;
    // Per-instance log latches. These used to be function-local
    // `static bool first_run` flags that the success branch reset
    // to true on the way out, so the very next tick re-announced
    // the same thing - producing the endless 2 Hz repetition of
    // "INICIANDO SECUENCIA DE RETORNO"/"YA ESTAMOS EN CASA".
    bool announced_ = false;
    bool idle_logged_ = false;

public:
    Recharge(const std::string& name, const BT::NodeConfiguration& config);
    ~Recharge();
    void init(AgentNode* agent);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

class BackToStation : public BT::AsyncActionNode
{
private:
    AgentNode* agent_;
    // Per-instance log latches. These used to be function-local
    // `static bool first_run` flags that the success branch reset
    // to true on the way out, so the very next tick re-announced
    // the same thing - producing the endless 2 Hz repetition of
    // "INICIANDO SECUENCIA DE RETORNO"/"YA ESTAMOS EN CASA".
    bool announced_ = false;
    bool idle_logged_ = false;

public:
    BackToStation(const std::string& name, const BT::NodeConfiguration& config);
    ~BackToStation();
    void init(AgentNode* agent);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

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

// ******************************* Conditions
class MissionOver : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    MissionOver(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class Idle : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    Idle(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsBatteryEnough : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsBatteryEnough(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsBatteryFull : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsBatteryFull(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskRecharge : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskRecharge(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskMonitor : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskMonitor(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskMonitorUGV : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskMonitorUGV(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskInspect : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskInspect(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskInspectPVArray : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskInspectPVArray(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsTaskDeliverTool : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsTaskDeliverTool(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsAgentNearChargingStation : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsAgentNearChargingStation(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsAgentNearHumanTarget : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsAgentNearHumanTarget(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsAgentNearUGV : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsAgentNearUGV(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsAgentNearWP : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsAgentNearWP(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class NeedToDropTheTool : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    NeedToDropTheTool(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class HasAgentTheTool : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    HasAgentTheTool(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

class IsAgentNearStation : public BT::ConditionNode
{
private:
    AgentNode* agent_;

public:
    IsAgentNearStation(const std::string& name);
    void init(AgentNode* agent);
    BT::NodeStatus tick() override;
};

// ******************************* Decorators
class ForceRunningNode : public BT::DecoratorNode
{
public:
    ForceRunningNode(const std::string& name);

private:
    virtual BT::NodeStatus tick() override;
};

// Behavior Tree Nodes registration function *************************************************************************
inline void RegisterNodes(BT::BehaviorTreeFactory& factory);

// Thread-safe wrapper around std::queue<classes::Task*>. AgentNode's task
// queue is read continuously by the BT thread (Idle, BackToStation, and
// every task-type leaf's empty()/front() checks) while
// AgentNode::handleNewTaskListAccepted() rewrites it wholesale
// (emptyTheQueue() + re-push) from a fresh detached std::thread per
// incoming NewTaskList goal - previously with zero synchronization. That
// data race let a freshly-assigned task go invisible to the BT thread
// indefinitely (observed in testing: an agent sat idle for 100+ seconds
// after accepting a task it never noticed). Every operation here is
// guarded by the same mutex so a write is always fully visible to the
// next read, matching the plain std::queue interface used at every call
// site so no other code needs to change.
class ThreadSafeTaskQueue
{
public:
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    classes::Task* front() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.front();
    }
    void push(classes::Task* task)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(task);
    }
    void pop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.pop();
    }
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    // Snapshot for callers that iterate a copy (e.g. AgentNode::infoQueue(),
    // AgentNode::taskQueueManager()) without holding the lock while doing so.
    operator std::queue<classes::Task*>() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_;
    }
    // Deletes and pops every element as a single critical section. Plain
    // empty()/front()/pop() in a loop (the previous implementation) let a
    // concurrent reader observe a torn intermediate state - e.g. see the
    // queue as empty mid-clear right before it's about to be refilled -
    // which is what previously stranded a freshly re-assigned task.
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            delete queue_.front();
            queue_.pop();
        }
    }
    // Pops the front element only if it matches id/type, as a single
    // critical section, so a concurrent clear() can't run between the
    // match check and the pop (which would otherwise pop a different task,
    // or pop on an already-emptied queue). Returns the popped task (caller
    // owns it and must delete it) or nullptr if the queue was empty or the
    // front didn't match.
    classes::Task* popFrontIfMatches(const std::string& id, char type)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return nullptr;
        classes::Task* task = queue_.front();
        if (task->getID() != id || task->getType() != type) return nullptr;
        queue_.pop();
        return task;
    }

private:
    mutable std::mutex mutex_;
    std::queue<classes::Task*> queue_;
};

// Agent Node Class declaration **************************************************************************************
class AgentNode : public as2::Node
{
    // Actions
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

    // Conditions
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

    // Decorators
    friend class ForceRunningNode;

private:
    std::string id_;
    std::string ns_prefix_;

    geographic_msgs::msg::GeoPoint origin_geo_;

    // Action Server to receive TaskList
    rclcpp_action::Server<mission_planner::action::NewTaskList>::SharedPtr ntl_as_;
    
    // Action Client to Battery Enough
    rclcpp_action::Client<mission_planner::action::BatteryEnough>::SharedPtr battery_ac_;

    rclcpp_action::Client<mission_planner::action::TaskResult>::SharedPtr task_result_ac_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr platform_info_sub_;
    rclcpp::Subscription<mission_planner::msg::MissionOver>::SharedPtr mission_over_sub_;
    rclcpp::Subscription<mission_planner::msg::PlannerBeacon>::SharedPtr planner_beacon_sub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr atrvjr_geopose_sub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr jackal_geopose_sub_;
    
    classes::Position position_;
    float battery_; // percentage
    int state_;

    // --- Telemetry trustworthiness ---------------------------------------
    // state_ is initialised to 0 and, until this was fixed, the switch in
    // platformInfoCallback() also mapped *every unrecognised* platform
    // status to 0 via its `default:` branch. Because 0 was accepted as a
    // "landed" state by every landing wait, three completely different
    // situations were indistinguishable:
    //   (a) the platform genuinely reports DISARMED (really on the ground),
    //   (b) no PlatformInfo has ever arrived (node just started),
    //   (c) PlatformInfo stopped arriving / carried an unknown value.
    // In cases (b) and (c) waitForState(state==0||1||2) returns REACHED on
    // its very first predicate evaluation - before it ever sleeps - so the
    // code logged "Aterrizaje completado con exito" for a drone that was
    // still at take_off_height_. That is why the logs claimed landings that
    // never happened. These flags let us tell the three cases apart, so a
    // landing is only ever reported when telemetry actually backs it up.
    std::atomic_bool platform_info_received_{false};
    std::atomic<double> last_platform_info_s_{0.0};
    std::atomic_bool platform_armed_{false};
    std::atomic_bool pose_received_{false};
    std::atomic<double> last_pose_s_{0.0};

    bool mission_over_;
    rclcpp::Time last_beacon_;
    bool timeout_;
    classes::Position atrvjr_pose_;
    classes::Position jackal_pose_;
    float take_off_height_;
    float distance_error_;
    float goto_error_;

    // Publishers
    rclcpp::Publisher<mission_planner::msg::AgentBeacon>::SharedPtr beacon_pub_;
    // Tarea que el agente ejecuta ahora mismo, como "t_1|I" (cadena vacia si
    // no tiene ninguna). Lo consume scripts/mission_viz.py para pintar cada
    // dron del color de su tarea: desde fuera no habia forma de saber que
    // estaba haciendo cada dron salvo leyendo los logs.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
    mission_planner::msg::AgentBeacon beacon_;
    rclcpp::Rate loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_beacon_;

    ThreadSafeTaskQueue task_queue_;
    // Set while BackToStation/Recharge have called land() and are waiting to
    // confirm the platform actually reached a landed state. A new task
    // arriving mid-confirmation used to overwrite task_queue_ immediately,
    // which flips the Idle condition node to FAILURE and makes BT.CPP halt
    // the in-progress landing wait to switch to the new task - sending a
    // GoTo goal to the platform while its Land goal was still active, which
    // left the platform stuck reporting LANDING forever in testing.
    // handleNewTaskListAccepted() defers applying a new task list while
    // this is true, so the landing confirmation gets to finish first.
    std::atomic_bool landing_in_progress_{false};
    bool battery_enough_;
    std::string tool_flag_;
    std::string pose_frame_id_;
    std::string pose_topic_;
    std::string state_topic_;
    std::string battery_topic_;

    std::string config_file_;
    std::string config_file_evora_;

    std::map<std::string, std::map<std::string, classes::Position>> known_positions_;
    std::map<std::string, classes::HumanTarget> human_targets_;
    std::map<std::string, classes::Tool> tools_;

    // Aerostack2 action clients
    rclcpp_action::Client<as2_msgs::action::Takeoff>::SharedPtr takeoff_ac_;
    rclcpp_action::Client<as2_msgs::action::Land>::SharedPtr land_ac_;
    rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SharedPtr goto_ac_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_cli_;

    // Behavior Tree Members
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    std::unique_ptr<std::thread> bt_thread_;
    std::atomic_bool bt_running_;

    void initializeBehaviorTree();
    void initializeBTNodes();
    void executeBehaviorTree();

protected:
    void readConfigFile(const std::string& config_file);
    void readEvoraConfigFile(const std::string& config_file);

public:
    AgentNode(const mission_planner::msg::AgentBeacon& beacon, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AgentNode();
        
    // Task queue methods
    void addTaskToQueue(classes::Task* task);
    void removeTaskFromQueue(const std::string& id, char type);
    void emptyTheQueue();
    int getQueueSize();
    void infoQueue();
    void taskQueueManager();
    void isBatteryEnough();

    // New Task List Action callback
    rclcpp_action::GoalResponse handleNewTaskListGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const mission_planner::action::NewTaskList::Goal> goal);
    rclcpp_action::CancelResponse handleNewTaskListCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTaskList>> goal_handle);
    void handleNewTaskListAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_planner::action::NewTaskList>> goal_handle);
    
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr battery);
    void platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info);
    void missionOverCallback(const mission_planner::msg::MissionOver::SharedPtr value);
    void beaconCallback(const mission_planner::msg::PlannerBeacon::SharedPtr beacon);
    bool checkBeaconTimeout(rclcpp::Time now);
    void atrvjrPositionCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr geo_pose);
    void jackalPositionCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr geo_pose);
    
    // Aerostack2 Service calls
    bool arm();
    // Cancels any GoToWaypoint goal still running server-side. The GoTo goal
    // handle is never retained, so this cancels all of them for this drone.
    // Must be called before land(): waitForArrival() returns as soon as we are
    // within OUR distance tolerance, which is looser than the GoTo behavior's
    // own convergence threshold, so GoTo is usually still active when we start
    // landing. Two active behaviors then assert conflicting control modes at
    // ~150 ms intervals and the controller never latches a reference
    // ("State changed, but ref not recived yet"), so the drone drifts upward
    // instead of descending - the "flies away and never comes back" bug.
    bool cancelGoTo();

    // Telemetry-trust helpers. See the comment on platform_info_received_.
    bool platformInfoFresh(double max_age_s = 3.0) const;
    bool poseFresh(double max_age_s = 3.0) const;
    // True only when recent telemetry positively confirms the drone is on
    // the ground: fresh PlatformInfo, a genuinely landed/disarmed platform
    // state, and a fresh pose whose altitude is at ground level. Never
    // returns true merely because telemetry is missing.
    bool isConfirmedLanded(float ground_z = 0.0f);
    // Bateria minima para poder volver a la base desde la posicion actual.
    // Crece con la distancia: ver el comentario en la implementacion.
    float batteryNeededToGetHome();
    // Human-readable name for our internal remapped state_ numbering.
    static const char* stateName(int s);
    bool land(bool blocking);
    bool take_off(float height, bool blocking);
    bool go_to_waypoint(float x, float y, float z, bool blocking);
    bool stop(bool blocking);
    bool checkIfGoToServiceSucceeded(float x, float y, float z);
    
    // Helper methods for Aerostack2 actions
    void executeTakeoffAction(float height);
    void executeLandAction();
    void executeGoToWaypointAction(float x, float y, float z);
};

#endif