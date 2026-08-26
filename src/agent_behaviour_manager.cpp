#include <cstdio>
#include "mission_planner/agent_behaviour_manager.hpp"
#include "as2_msgs/msg/yaw_mode.hpp"

#include <chrono>
#include <thread>
#include <functional>

// Behavior Tree structure xml definition
static const char* behaviour_tree_xml = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
    </BehaviorTree>
</root>
 )";

// Every "wait until we reach this waypoint" loop below used to poll
// checkIfGoToServiceSucceeded() with no time limit. If the platform never
// converges within goto_error_ (PID overshoot/oscillation, a stuck
// controller, etc.), that loop - and with it the single BT tick calling it -
// never returns. Since executeBehaviorTree() only logs/checks mission state
// between ticks, the agent then goes silent forever: no further log lines,
// no way to notice or recover. waitForArrival() bounds that wait so a stuck
// movement fails loudly instead of hanging the whole agent indefinitely.
enum class ArrivalWaitResult { REACHED, HALTED, TIMED_OUT };

static ArrivalWaitResult waitForArrival(
    AgentNode* agent, float x, float y, float z,
    const std::function<bool()>& halt_requested,
    std::chrono::milliseconds poll_period = std::chrono::milliseconds(100),
    double timeout_s = 30.0)
{
  auto start = std::chrono::steady_clock::now();
  while (!agent->checkIfGoToServiceSucceeded(x, y, z)) {
    if (halt_requested()) return ArrivalWaitResult::HALTED;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout_s) {
      return ArrivalWaitResult::TIMED_OUT;
    }
    std::this_thread::sleep_for(poll_period);
  }
  return ArrivalWaitResult::REACHED;
}

// Same problem, different symptom: several leaves wait for agent_->state_ to
// reach a target platform state (e.g. FLYING after take_off(), or
// DISARMED/LANDED after land()) with an unbounded while loop. If the
// platform never reports that transition, the agent hangs identically to
// the position-wait case above - this is what left a drone stuck in
// LANDING forever after a successful land() call that never got confirmed.
static ArrivalWaitResult waitForState(
    const std::function<bool()>& state_reached,
    const std::function<bool()>& halt_requested,
    std::chrono::milliseconds poll_period = std::chrono::milliseconds(100),
    double timeout_s = 60.0)
{
  auto start = std::chrono::steady_clock::now();
  while (!state_reached()) {
    if (halt_requested()) return ArrivalWaitResult::HALTED;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout_s) {
      return ArrivalWaitResult::TIMED_OUT;
    }
    std::this_thread::sleep_for(poll_period);
  }
  return ArrivalWaitResult::REACHED;
}

// Recharge::tick() and BackToStation::tick() run their own state machine in a
// `while(!isHaltRequested())` loop whose `default:` branch (any platform
// state other than the few it explicitly handles) just sleeps and retries
// with no bound. That default case is reachable even after the per-step
// waitForState()/waitForArrival() calls above return HALTED: BT.CPP's own
// scheduler can halt() and later re-tick one of these nodes while the
// platform is still sitting in a leftover state (e.g. LANDING) from an
// action the previous invocation abandoned mid-flight. Without an overall
// deadline on the loop itself, that leftover state spins the `default:`
// branch forever - this is what left a drone parked in LANDING permanently
// even after the individual state-wait timeouts were added.
constexpr double kOverallTickDeadlineSeconds = 180.0;

// A drone counts as physically on the ground only when its altitude is within
// this margin of the ground level where it is parked. take_off_height_
// defaults to 10 m, so 1.5 m cleanly separates "landed" from "hovering"
// without being so tight that pose noise or uneven terrain trips it.
constexpr float kOnGroundAltitudeMargin = 1.5f;

// Parametros para decidir cuando un dron debe dejar la tarea y volver a base.
// kBatteryDrainPerSecond debe coincidir con battery_faker (battery_decrease
// por tick de 500 ms => x2 por segundo).
constexpr float kCruiseSpeedMps = 1.2f;          // velocidad de crucero observada
constexpr float kBatteryDrainPerSecond = 0.008f; // 0.004 por tick a 2 Hz
constexpr float kMinBatteryReserve = 0.22f;      // colchon para aterrizar

// Monotonic seconds, used to age-check incoming telemetry.
static double nowSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

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
      case 0: // UNINITIALIZED - also what the platform reports right after a
              // completed land()+disarm, not just before the first arm
      case 1: // LANDED_DISARMED
      case 2: // LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(agent_->get_logger(), "[GoNearChargingStation] Calling take_off");
        // Aerostack2 take_off action call
        agent_->arm();
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
          switch (waitForState(
              [this]() { return agent_->state_ == 4; },
              [this]() { return isHaltRequested(); }))
          {
            case ArrivalWaitResult::HALTED:
              return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED:
              break;
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
          switch (waitForArrival(agent_, assigned_charging_station.getX(), assigned_charging_station.getY(),
                assigned_charging_station.getZ() + 1, [this]{ return isHaltRequested(); },
                std::chrono::milliseconds(1)))
          {
            case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(agent_->get_logger(), "[GoNearChargingStation] Timeout waiting to reach charging station");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED: break;
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
      case 3: // TAKING_OFF
      case 5: // FLYING_MANUAL
      case 6: // LANDING
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
      if(isHaltRequested()) return BT::NodeStatus::IDLE;
      return BT::NodeStatus::FAILURE;
    }
    final_percentage = task->getFinalPercentage();
    recharge_task = true;
    goal.task.id = task->getID();
    goal.task.type = task->getType();
  }

  if(!announced_) {
    RCLCPP_INFO(agent_->get_logger(), "[recarga] Iniciando recarga (objetivo %.0f%%).",
      final_percentage * 100.0f);
    announced_ = true;
  }

  auto tick_start = std::chrono::steady_clock::now();
  while(!isHaltRequested())
  {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - tick_start).count() >
        kOverallTickDeadlineSeconds) {
      RCLCPP_ERROR(agent_->get_logger(),
        "[recarga] Timeout global (%.0f s) sin completar la recarga. Estado=%s(%d), bateria=%.0f%%.",
        kOverallTickDeadlineSeconds, AgentNode::stateName(agent_->state_), agent_->state_,
        agent_->battery_ * 100.0f);
      announced_ = false;
      return BT::NodeStatus::FAILURE;
    }

    // Same reasoning as BackToStation: state_ defaults to 0 (DESARMADO), so
    // without fresh telemetry this loop would treat a flying drone as parked
    // and "recharge" it in mid-air.
    if (!agent_->platformInfoFresh()) {
      RCLCPP_WARN_THROTTLE(agent_->get_logger(), *agent_->get_clock(), 5000,
        "[recarga] Sin telemetria fresca de la plataforma; esperando datos antes de actuar.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    switch(agent_->state_)
    {
      case 0: // DESARMADO - also reported right after a completed land()
      case 1: // EN_SUELO_DESARMADO
      case 2: // EN_SUELO_ARMADO
        if(agent_->battery_ >= final_percentage)
        {
          if(recharge_task && agent_->task_result_ac_)
          {
            goal.result = 1;
            agent_->task_result_ac_->async_send_goal(goal);
            agent_->removeTaskFromQueue(goal.task.id, 'R');
            RCLCPP_INFO(agent_->get_logger(),
              "[recarga] Completada (bateria %.0f%%). Resultado enviado al planificador.",
              agent_->battery_ * 100.0f);
          } else {
            RCLCPP_INFO(agent_->get_logger(),
              "[recarga] Completada (bateria %.0f%%).", agent_->battery_ * 100.0f);
          }
          announced_ = false;
          return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO_THROTTLE(agent_->get_logger(), *agent_->get_clock(), 10000,
          "[recarga] En curso: bateria %.0f%% de %.0f%% objetivo.",
          agent_->battery_ * 100.0f, final_percentage * 100.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        break;
      case 4: // VOLANDO - hay que bajar antes de poder recargar
        RCLCPP_INFO(agent_->get_logger(), "[recarga] En vuelo; aterrizando para recargar.");
        agent_->cancelGoTo();
        if(!agent_->land(false)) return BT::NodeStatus::FAILURE;
        agent_->landing_in_progress_ = true;
        // isConfirmedLanded() instead of a bare state check - see the note in
        // BackToStation. Recharging only makes sense once we are really down.
        switch (waitForState(
            [this]() { return agent_->isConfirmedLanded(); },
            [this]() { return isHaltRequested(); }))
        {
          case ArrivalWaitResult::HALTED:
            agent_->landing_in_progress_ = false;
            return BT::NodeStatus::IDLE;
          case ArrivalWaitResult::TIMED_OUT:
            agent_->landing_in_progress_ = false;
            RCLCPP_ERROR(agent_->get_logger(),
              "[recarga] Aterrizaje NO confirmado tras 60 s: estado=%s(%d), z=%.2f m, telemetria=%s.",
              AgentNode::stateName(agent_->state_), agent_->state_,
              agent_->poseFresh() ? agent_->position_.getZ() : -1.0f,
              agent_->platformInfoFresh() ? "fresca" : "AUSENTE");
            return BT::NodeStatus::FAILURE;
          case ArrivalWaitResult::REACHED:
            agent_->landing_in_progress_ = false;
            RCLCPP_INFO(agent_->get_logger(),
              "[recarga] Aterrizaje CONFIRMADO (z=%.2f m).", agent_->position_.getZ());
            break;
        }
        break;
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        break;
    }
  }

  // Salida limpia por Halt sin enviar mensajes erróneos
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
  // A new task means we are no longer returning to base; reset the latches so
  // the next genuine return announces itself once more.
  if (!agent_->task_queue_.empty()) {
      announced_ = false;
      idle_logged_ = false;
      return BT::NodeStatus::FAILURE;
  }

  if(!announced_) {
      RCLCPP_INFO(agent_->get_logger(), "[volver-a-base] Iniciando secuencia de retorno.");
      announced_ = true;
  }

  // --- EL ARREGLO ESTÁ AQUÍ ---
  // Mapeamos el ID del dron a los nombres exactos de tu conf.yaml
  std::string nearest_station = "";
  if (agent_->id_ == "drone0") nearest_station = "charging_station_drone0";
  else if (agent_->id_ == "drone1") nearest_station = "charging_station_drone1";
  else if (agent_->id_ == "drone2") nearest_station = "charging_station_drone2";
  else nearest_station = "charging_station_4"; // Por defecto

  float target_x = 0.0;
  float target_y = 0.0;
  float target_z = 2.0;
  // Ground level at the station, used to tell "landed here" from "hovering
  // over here". target_z is deliberately 2 m above it (approach altitude).
  float ground_z = 0.0;

  // Buscar en las posiciones cargadas desde conf.yaml
  if (agent_->known_positions_["charging_stations"].find(nearest_station) != agent_->known_positions_["charging_stations"].end()) {
      auto station_pos = agent_->known_positions_["charging_stations"][nearest_station];
      target_x = station_pos.getX();
      target_y = station_pos.getY();
      ground_z = station_pos.getZ();
      // Le damos un poco de altura de seguridad para el viaje
      target_z = station_pos.getZ() + 2.0; 
  } else {
      RCLCPP_WARN(agent_->get_logger(), "⚠️ Aún no encuentro %s. Usando 0,0 por emergencia.", nearest_station.c_str());
  }

  auto tick_start = std::chrono::steady_clock::now();
  while(!isHaltRequested())
  {
    if (!agent_->task_queue_.empty()) {
        announced_ = false;
        idle_logged_ = false;
        return BT::NodeStatus::FAILURE;
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - tick_start).count() >
        kOverallTickDeadlineSeconds) {
      RCLCPP_ERROR(agent_->get_logger(),
        "[volver-a-base] Timeout global (%.0f s) sin completar el retorno. Estado=%s(%d), z=%.2f m.",
        kOverallTickDeadlineSeconds, AgentNode::stateName(agent_->state_), agent_->state_,
        agent_->poseFresh() ? agent_->position_.getZ() : -1.0f);
      announced_ = false;
      idle_logged_ = false;
      return BT::NodeStatus::FAILURE;
    }

    // Never act on a state we cannot currently verify. state_ starts at 0
    // (DESARMADO), so without this guard a node that has not yet received any
    // PlatformInfo would read "on the ground" and start commanding a takeoff.
    if (!agent_->platformInfoFresh()) {
      RCLCPP_WARN_THROTTLE(agent_->get_logger(), *agent_->get_clock(), 5000,
        "[volver-a-base] Sin telemetria fresca de la plataforma; esperando datos antes de actuar.");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    switch(agent_->state_)
    {
      case 0: // Also reported as "disarmed" right after a completed land(), not just pre-arm
      case 1: // LANDED_DISARMED
      case 2: // LANDED_ARMED
        // Comprobamos si ya estamos en nuestra base
        // "Home" requires all three of: the right X/Y, telemetry that
        // positively confirms we are on the ground, and a fresh pose. The old
        // check tested X/Y only yet still printed "Y EN EL SUELO", so a drone
        // hovering at take_off_height_ directly above its own charging station
        // was declared landed-at-home and the tree stopped trying to bring it
        // down. That is the "takes off and never comes back" symptom.
        if (std::abs(agent_->position_.getX() - target_x) < agent_->distance_error_ &&
            std::abs(agent_->position_.getY() - target_y) < agent_->distance_error_ &&
            agent_->isConfirmedLanded(ground_z)) {

            if (!idle_logged_) {
              RCLCPP_INFO(agent_->get_logger(),
                "[volver-a-base] En base y en el suelo confirmado (z=%.2f m, estado=%s). Fin.",
                agent_->position_.getZ(), AgentNode::stateName(agent_->state_));
              idle_logged_ = true;
            }
            // ReactiveSequence(Idle, BackToStation) restarts BackToStation
            // fresh from this exact branch on every root tick while idle -
            // each restart spawns a new AsyncActionNode thread just to
            // immediately report "still home". Measured in testing: ~5
            // restarts/sec per idle drone, enough concurrent thread churn
            // across 3 drones to visibly slow down AS2/Gazebo message
            // processing elsewhere (a drone's landing confirmation took
            // 119s instead of ~10s in one run). There's no reason an
            // already-parked drone needs to re-confirm that faster than 2Hz.
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(agent_->get_logger(), "🚀 Despegando para volver a casa...");
            agent_->arm();
            if(!agent_->take_off(agent_->take_off_height_, false)) return BT::NodeStatus::FAILURE;

            switch (waitForState(
                [this]() { return agent_->state_ == 4; },
                [this]() { return isHaltRequested(); }))
            {
              case ArrivalWaitResult::HALTED:
                return BT::NodeStatus::IDLE;
              case ArrivalWaitResult::TIMED_OUT:
                RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar hacia casa.");
                return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::REACHED:
                break;
            }
        }
        break;

      case 4: // FLYING_AUTO
        RCLCPP_INFO(agent_->get_logger(), "✈️ Viajando a %s [%.2f, %.2f] a %.2fm...", nearest_station.c_str(), target_x, target_y, target_z);
        
        if(agent_->go_to_waypoint(target_x, target_y, target_z, false)) {
            
            switch (waitForArrival(agent_, target_x, target_y, target_z,
                  [this]{ return isHaltRequested() || !agent_->task_queue_.empty(); },
                  std::chrono::milliseconds(200)))
            {
              case ArrivalWaitResult::HALTED: return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::TIMED_OUT:
                RCLCPP_ERROR(agent_->get_logger(), "[BackToStation] Timeout waiting to reach %s", nearest_station.c_str());
                return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::REACHED: break;
            }
            
            RCLCPP_INFO(agent_->get_logger(), "[volver-a-base] Posicion alcanzada. Aterrizando.");

            // Stop the GoTo before landing, otherwise both behaviors stay
            // active and fight over the platform's control mode.
            agent_->cancelGoTo();

            if(!agent_->land(false)) {
                RCLCPP_ERROR(agent_->get_logger(), "❌ Fallo al intentar aterrizar.");
                return BT::NodeStatus::FAILURE;
            }

            // Success now requires isConfirmedLanded(): fresh telemetry, a
            // genuinely landed platform state AND a ground-level altitude.
            // The previous predicate accepted state_==0, which is also the
            // value used when no telemetry had arrived - so it returned
            // REACHED on its first evaluation, before ever sleeping, and
            // logged a completed landing for a drone still in the air.
            agent_->landing_in_progress_ = true;
            switch (waitForState(
                [this, ground_z]() { return agent_->isConfirmedLanded(ground_z); },
                [this]() { return isHaltRequested(); }))
            {
              case ArrivalWaitResult::HALTED:
                agent_->landing_in_progress_ = false;
                return BT::NodeStatus::IDLE;
              case ArrivalWaitResult::TIMED_OUT:
                agent_->landing_in_progress_ = false;
                RCLCPP_ERROR(agent_->get_logger(),
                  "[volver-a-base] Aterrizaje NO confirmado tras 60 s: estado=%s(%d), z=%.2f m, telemetria=%s. El dron puede seguir en el aire.",
                  AgentNode::stateName(agent_->state_), agent_->state_,
                  agent_->poseFresh() ? agent_->position_.getZ() : -1.0f,
                  agent_->platformInfoFresh() ? "fresca" : "AUSENTE");
                return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::REACHED:
                agent_->landing_in_progress_ = false;
                break;
            }

            RCLCPP_INFO(agent_->get_logger(),
              "[volver-a-base] Aterrizaje CONFIRMADO (z=%.2f m, estado=%s).",
              agent_->position_.getZ(), AgentNode::stateName(agent_->state_));
            idle_logged_ = true;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;

      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        break;
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
  if(agent_->task_queue_.empty()) return BT::NodeStatus::FAILURE;
  classes::Task* task = agent_->task_queue_.front();

  float distance = (task->getType() == 'M') ? task->getDistance() : 1.5;
  classes::Position human_position = task->getHumanPosition();
  classes::Position near_human_pose = classes::closePose2D(agent_->position_, human_position, distance);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 0: // UNINITIALIZED - also what the platform reports right after a
              // completed land()+disarm, not just before the first arm
      case 1: // LANDED_DISARMED
      case 2: // LANDED_ARMED
        if(isHaltRequested()) return BT::NodeStatus::IDLE;
        agent_->arm();
        if(!agent_->take_off(agent_->take_off_height_, false)) return BT::NodeStatus::FAILURE;
        switch (waitForState(
            [this]() { return agent_->state_ == 4; },
            [this]() { return isHaltRequested(); }))
        {
          case ArrivalWaitResult::HALTED:
            return BT::NodeStatus::IDLE;
          case ArrivalWaitResult::TIMED_OUT:
            RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
            return BT::NodeStatus::FAILURE;
          case ArrivalWaitResult::REACHED:
            break;
        }
        break;
      case 4:
        if(isHaltRequested()) return BT::NodeStatus::IDLE;
        if(agent_->go_to_waypoint(near_human_pose.getX(), near_human_pose.getY(), near_human_pose.getZ(), false)) {
            switch (waitForArrival(agent_, near_human_pose.getX(), near_human_pose.getY(), near_human_pose.getZ(),
                  [this]{ return isHaltRequested(); }, std::chrono::milliseconds(100)))
            {
              case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
              case ArrivalWaitResult::TIMED_OUT:
                RCLCPP_ERROR(rclcpp::get_logger("go_near_human_target"),
                  "[GoNearHumanTarget] Timeout waiting to reach human target");
                return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::REACHED: break;
            }
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
    }
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
  if(agent_->task_queue_.empty()) return BT::NodeStatus::FAILURE;
  classes::Task* task = agent_->task_queue_.front();
  std::string task_id = task->getID();

  if(task->getType() != 'M') return BT::NodeStatus::FAILURE;

  RCLCPP_INFO(agent_->get_logger(), "📷 [MonitorHumanTarget] Iniciando monitorización (Esperando 5 segundos)...");
  
  for(int i = 0; i < 50; i++) {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(agent_->get_logger(), "✅ [MonitorHumanTarget] Monitorización completada.");

  // Preparar mensaje de éxito
  auto goal = mission_planner::action::TaskResult::Goal();
  goal.task.id = task_id;
  goal.task.type = 'M';
  goal.result = 1; 

  // Disparo asíncrono sin esperas usando el cliente de la clase
  if (agent_->task_result_ac_) {
      agent_->task_result_ac_->async_send_goal(goal);
      RCLCPP_INFO(agent_->get_logger(), "📨 Resultado 'SUCCESS' disparado hacia el planificador.");
  } else {
      RCLCPP_ERROR(agent_->get_logger(), "❌ ERROR: task_result_ac_ no está inicializado.");
  }

  agent_->removeTaskFromQueue(task_id, 'M');
  return BT::NodeStatus::SUCCESS;
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
      case 0: //UNINITIALIZED - also what the platform reports right after a
              // completed land()+disarm, not just before the first arm
      case 1: //LANDED_DISARMED
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        agent_->arm();
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          switch (waitForState(
              [this]() { return agent_->state_ == 4; },
              [this]() { return isHaltRequested(); }))
          {
            case ArrivalWaitResult::HALTED:
              return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED:
              break;
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Moving near UGV...");
        if(agent_->go_to_waypoint(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(), false))
        {
          switch (waitForArrival(agent_, near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(),
                [this]{ return isHaltRequested(); }, std::chrono::milliseconds(1)))
          {
            case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(rclcpp::get_logger("go_near_ugv"), "[GoNearUGV] Timeout waiting to reach UGV");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED: break;
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
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  
  classes::Task* task = agent_->task_queue_.front();
  std::string task_id = task->getID();
  float height = task->getHeight();

  RCLCPP_INFO(rclcpp::get_logger("monitor_ugv"), "🤖 [MonitorUGV] Moviendo al UGV..."); 
  
  while(!isHaltRequested())
  {
    // Solo intenta ir una vez cada segundo para no saturar
    if(!agent_->go_to_waypoint(agent_->atrvjr_pose_.getX(), agent_->atrvjr_pose_.getY(), height, false))
    {
      if(isHaltRequested()) return BT::NodeStatus::IDLE;
      return BT::NodeStatus::FAILURE;
    }
    
    // Aquí podrías añadir tu condición de éxito (ej. si has estado 5 segundos encima). 
    // Por ahora rompemos para simular que acabó con éxito.
    break; 
  }

  if (isHaltRequested()) return BT::NodeStatus::IDLE;

  if (agent_->task_result_ac_) {
    auto goal = mission_planner::action::TaskResult::Goal();
    goal.task.id = task_id;
    goal.task.type = 'F';
    goal.result = 1; 
    agent_->task_result_ac_->async_send_goal(goal);
  }
  
  agent_->removeTaskFromQueue(task_id, 'F');
  return BT::NodeStatus::SUCCESS;
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
      case 0: //UNINITIALIZED - also what the platform reports right after a
              // completed land()+disarm, not just before the first arm
      case 1: //LANDED_DISARMED
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        agent_->arm();
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          switch (waitForState(
              [this]() { return agent_->state_ == 4; },
              [this]() { return isHaltRequested(); }))
          {
            case ArrivalWaitResult::HALTED:
              return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED:
              break;
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Moving near WP...");
        if(agent_->go_to_waypoint(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(), false))
        {
          switch (waitForArrival(agent_, near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(),
                [this]{ return isHaltRequested(); }, std::chrono::milliseconds(1)))
          {
            case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(rclcpp::get_logger("go_near_wp"), "[GoNearWP] Timeout waiting to reach waypoint");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED: break;
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
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        // These are transient states we're just passing through (or an
        // unrecognized one) - sleep so this doesn't busy-spin at 100% CPU
        // checking isHaltRequested() as fast as the CPU allows.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        break;
    }
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

  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  classes::Task* task = agent_->task_queue_.front();
  std::string task_id = task->getID();

  RCLCPP_INFO(rclcpp::get_logger("take_image"), "📸 [TakeImage] Simulando toma de imagen (1 segundo)...");
  
  for(int i = 0; i < 100; i++) // 1 segundo (100 * 10ms)
  {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Enviar el resultado directamente
  if (agent_->task_result_ac_) {
    auto goal = mission_planner::action::TaskResult::Goal();
    goal.task.id = task_id;
    goal.task.type = 'I';
    goal.result = 1; 
    agent_->task_result_ac_->async_send_goal(goal);
    RCLCPP_INFO(rclcpp::get_logger("take_image"), "📨 Resultado de Imagen 'SUCCESS' disparado.");
  }
  
  agent_->removeTaskFromQueue(task_id, 'I');
  return BT::NodeStatus::SUCCESS;
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

  if(agent_->task_queue_.empty()) {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    return BT::NodeStatus::FAILURE;
  }
  
  classes::Task* task = agent_->task_queue_.front();
  // id y tipo copiados POR VALOR: emptyTheQueue() (que dispara
  // isBatteryEnough desde el hilo del executor) borra las tareas, dejando
  // este puntero colgando mientras el nodo sigue esperando en waitForArrival.
  std::string task_id = task->getID();
  const char task_type = task->getType();

  if(task_type != 'I' && task_type != 'A') {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  auto goal = mission_planner::action::TaskResult::Goal();
  auto wp = task->getInspectWaypoints();

  auto sendResult = [&](int result_value) {
    if (agent_->task_result_ac_) {
      goal.task.id = task_id;
      goal.task.type = task_type;
      goal.result = result_value;
      
      if (result_value == 1) { 
        geometry_msgs::msg::Point target_xyz;
        target_xyz.x = -36.6343; target_xyz.y = 62.293;
        geographic_msgs::msg::GeoPose target_gps;
        target_gps.position.latitude = -7.96213167462045;
        target_gps.position.longitude = 38.54156780106911;
        goal.do_closer_inspection.xyz_coordinates.push_back(target_xyz);
        goal.do_closer_inspection.gps_coordinates.push_back(target_gps);
      }
      
      agent_->task_result_ac_->async_send_goal(goal);
      RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "📨 Resultado de inspección disparado hacia el planificador.");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "❌ ERROR: task_result_ac_ no está inicializado.");
    }
    // Se quita de la cola tanto si sale bien como si falla. Antes solo se
    // borraba en caso de exito, asi que una tarea fallida se quedaba en la
    // cola del agente para siempre: el arbol volvia a tickear este mismo nodo,
    // reintentaba, volvia a agotar los 30 s de waitForArrival... y el dron se
    // quedaba dando vueltas indefinidamente en el punto lejano sin volver a
    // casa. El planner, por su parte, SI borra la tarea al recibir result=0 y
    // la reasigna, con lo que los dos quedaban ademas descoordinados.
    agent_->removeTaskFromQueue(task_id, task_type);
  };

  if (wp.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "❌ Error: Faltan waypoints en la tarea (Mínimo 2).");
      sendResult(0);
      return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"),
    "[inspeccion] Iniciando %s (tipo %c, %zu waypoints).", task_id.c_str(), task_type, wp.size());

  auto tick_start = std::chrono::steady_clock::now();
  while(!isHaltRequested())
  {
    // Red de seguridad: sin esto el nodo podia reintentar sin fin.
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - tick_start).count() >
        kOverallTickDeadlineSeconds) {
      RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"),
        "[inspeccion] %s abandonada tras %.0f s sin completarse (estado=%s). Se informa al planificador para que la reasigne.",
        task_id.c_str(), kOverallTickDeadlineSeconds, AgentNode::stateName(agent_->state_));
      sendResult(0);
      return BT::NodeStatus::FAILURE;
    }

    if (agent_->state_ != 4) {
        RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "🚁 Solicitando despegue a Aerostack2...");
        agent_->arm();
        if(!agent_->take_off(agent_->take_off_height_, false)) {
            sendResult(0);
            return BT::NodeStatus::FAILURE;
        }
        switch (waitForState(
            [this]() { return agent_->state_ == 4; },
            [this]() { return isHaltRequested(); }))
        {
          case ArrivalWaitResult::HALTED:
            return BT::NodeStatus::IDLE;
          case ArrivalWaitResult::TIMED_OUT:
            RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
            return BT::NodeStatus::FAILURE;
          case ArrivalWaitResult::REACHED:
            break;
        }
    }

    if (agent_->state_ == 4) {
        RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "📍 Moviendo al PRIMER punto [%.2f, %.2f, %.2f]...", wp[0].x, wp[0].y, wp[0].z);
        if(agent_->go_to_waypoint(wp[0].x, wp[0].y, wp[0].z, false)) {
            switch (waitForArrival(agent_, wp[0].x, wp[0].y, wp[0].z,
                  [this]{ return isHaltRequested(); }, std::chrono::milliseconds(100)))
            {
              case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
              case ArrivalWaitResult::TIMED_OUT:
                RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Timeout waiting to reach first point");
                return BT::NodeStatus::FAILURE;
              case ArrivalWaitResult::REACHED: break;
            }

            RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "📍 Moviendo al SEGUNDO punto [%.2f, %.2f, %.2f]...", wp[1].x, wp[1].y, wp[1].z);
            if(agent_->go_to_waypoint(wp[1].x, wp[1].y, wp[1].z, false)) {
                switch (waitForArrival(agent_, wp[1].x, wp[1].y, wp[1].z,
                      [this]{ return isHaltRequested(); }, std::chrono::milliseconds(100)))
                {
                  case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
                  case ArrivalWaitResult::TIMED_OUT:
                    RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "[InspectPVArray] Timeout waiting to reach second point");
                    return BT::NodeStatus::FAILURE;
                  case ArrivalWaitResult::REACHED: break;
                }

                RCLCPP_INFO(rclcpp::get_logger("inspect_pv_array"), "✅ Inspección completada con éxito.");
                sendResult(1);
                return BT::NodeStatus::SUCCESS;
            }
        }
        RCLCPP_ERROR(rclcpp::get_logger("inspect_pv_array"), "❌ Fallo en la navegación.");
        sendResult(0);
        return BT::NodeStatus::FAILURE;
    }
  }
  
  return BT::NodeStatus::IDLE;
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
      case 0: //UNINITIALIZED - also what the platform reports right after a
              // completed land()+disarm, not just before the first arm
      case 1: //LANDED_DISARMED
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        agent_->arm();
        if(!agent_->take_off(agent_->take_off_height_, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          RCLCPP_ERROR(rclcpp::get_logger("go_near_station"), "[GoNearStation] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          switch (waitForState(
              [this]() { return agent_->state_ == 4; },
              [this]() { return isHaltRequested(); }))
          {
            case ArrivalWaitResult::HALTED:
              return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(agent_->get_logger(), "⏱️ Timeout esperando a alcanzar el estado FLYING tras despegar.");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED:
              break;
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        RCLCPP_INFO(rclcpp::get_logger("go_near_station"), "[GoNearStation] Moving near Tool...");
        if(agent_->go_to_waypoint(near_tool_pose.getX(), near_tool_pose.getY(), near_tool_pose.getZ(), false))
        {
          switch (waitForArrival(agent_, near_tool_pose.getX(), near_tool_pose.getY(), near_tool_pose.getZ(),
                [this]{ return isHaltRequested(); }, std::chrono::milliseconds(1)))
          {
            case ArrivalWaitResult::HALTED: return BT::NodeStatus::IDLE;
            case ArrivalWaitResult::TIMED_OUT:
              RCLCPP_ERROR(rclcpp::get_logger("go_near_station"), "[GoNearStation] Timeout waiting to reach tool");
              return BT::NodeStatus::FAILURE;
            case ArrivalWaitResult::REACHED: break;
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
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    RCLCPP_WARN(rclcpp::get_logger("deliver_tool"), "[DeliverTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  
  classes::Task* task = agent_->task_queue_.front();
  std::string task_id = task->getID();

  if(task->getType() != 'D') return BT::NodeStatus::FAILURE;

  RCLCPP_INFO(rclcpp::get_logger("deliver_tool"), "📦 [DeliverTool] Entregando herramienta...");
  
  for(int i = 0; i < 100; i++) // 1 segundo simulación
  {
    if(isHaltRequested()) return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  agent_->tool_flag_ = "none";

  if (agent_->task_result_ac_) {
    auto goal = mission_planner::action::TaskResult::Goal();
    goal.task.id = task_id;
    goal.task.type = 'D';
    goal.result = 1; 
    agent_->task_result_ac_->async_send_goal(goal);
    RCLCPP_INFO(rclcpp::get_logger("deliver_tool"), "📨 Resultado 'SUCCESS' disparado hacia el planificador.");
  }
  
  agent_->removeTaskFromQueue(task_id, 'D');
  return BT::NodeStatus::SUCCESS;
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
    battery_(0),
    bt_running_(false)
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

  task_result_ac_ = rclcpp_action::create_client<mission_planner::action::TaskResult>(
      this, 
      "/" + beacon_.id + "/task_result"
  );

  // Action client para Battery Enough
  battery_ac_ = rclcpp_action::create_client<mission_planner::action::BatteryEnough>(
    this, 
    "/" + beacon_.id + "/battery_enough");

  // Aerostack2 action clients
  takeoff_ac_ = rclcpp_action::create_client<as2_msgs::action::Takeoff>(
    this,
    "/" + beacon_.id + "/TakeoffBehavior");

  land_ac_ = rclcpp_action::create_client<as2_msgs::action::Land>(
    this,
    "/" + beacon_.id + "/LandBehavior");

  goto_ac_ = rclcpp_action::create_client<as2_msgs::action::GoToWaypoint>(
    this,
    "/" + beacon_.id + "/GoToBehavior");

  arm_cli_ = this->create_client<std_srvs::srv::SetBool>(
    "/" + beacon_.id + "/set_arming_state");

  // Publishers
  beacon_pub_ = this->create_publisher<mission_planner::msg::AgentBeacon>("/agent_beacon", 1);
  task_pub_ = this->create_publisher<std_msgs::msg::String>("/" + beacon_.id + "/current_task", 1);

  timer_beacon_ = this->create_wall_timer(
    std::chrono::seconds(1), 
    [this]() {
        // Publicamos la estructura beacon_ que ya se inicializa en el constructor
        this->beacon_pub_->publish(this->beacon_);

        // Tarea actual, para el HUD de Gazebo (mission_viz.py).
        std_msgs::msg::String task_msg;
        if (!this->task_queue_.empty()) {
          classes::Task* t = this->task_queue_.front();
          if (t) task_msg.data = t->getID() + "|" + std::string(1, t->getType());
        }
        this->task_pub_->publish(task_msg);

        // Single, bounded heartbeat: one line every 10 s per drone stating
        // what telemetry ACTUALLY says. Deliberately sourced straight from
        // the subscriptions rather than from what the behaviour tree believes,
        // so it can be used to check the tree's own claims - the previous logs
        // asserted completed landings that telemetry never backed up.
        char z_buf[32];
        if (this->poseFresh()) {
          snprintf(z_buf, sizeof(z_buf), "%.2f m", this->position_.getZ());
        } else {
          snprintf(z_buf, sizeof(z_buf), "sin datos");
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "[latido] estado=%s(%d) | z=%s | bateria=%.0f%% | tareas=%zu | telemetria=%s",
          stateName(this->state_), this->state_, z_buf,
          this->battery_ * 100.0f,
          this->task_queue_.size(),
          this->platformInfoFresh() ? "ok" : "AUSENTE");
    });

  // Subscribers
  position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SensorDataQoS(),
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

  initializeBehaviorTree();

}

void AgentNode::initializeBehaviorTree() {
  RCLCPP_INFO(this->get_logger(), "🌳 Inicializando Behavior Tree...");
  
  try {
      // Registrar todos los nodos BT
      RegisterNodes(factory_);

      // VERIFICAR ÁRBOLES REGISTRADOS
      auto trees = factory_.registeredBehaviorTrees();
      RCLCPP_INFO(this->get_logger(), "📋 Árboles BT registrados:");
      for (const auto& tree : trees) {
          RCLCPP_INFO(this->get_logger(), "  - %s", tree.c_str());
      }
      
      // Cargar el BT desde el directorio share del paquete (SIN /src/)
      std::string package_path = ament_index_cpp::get_package_share_directory("mission_planner");
      std::string bt_xml_file = package_path + "/behaviour_tree.xml";
      
      RCLCPP_INFO(this->get_logger(), "📁 Cargando BT desde: %s", bt_xml_file.c_str());
      
      // Verificar que el archivo existe
      std::ifstream file(bt_xml_file);
      if (!file.good()) {
          RCLCPP_ERROR(this->get_logger(), "❌ Archivo BT no encontrado: %s", bt_xml_file.c_str());
          
          // Buscar alternativas
          RCLCPP_INFO(this->get_logger(), "🔍 Buscando archivo BT alternativo...");
          
          std::vector<std::string> possible_paths = {
              package_path + "/behaviour_tree.xml",  // Ruta correcta
              "src/tfg_gierm/src/behaviour_tree.xml", // Ruta desde workspace
              "../src/tfg_gierm/src/behaviour_tree.xml",
              "behaviour_tree.xml"
          };
          
          bool found = false;
          for (const auto& path : possible_paths) {
              std::ifstream alt_file(path);
              if (alt_file.good()) {
                  bt_xml_file = path;
                  RCLCPP_INFO(this->get_logger(), "✅ Encontrado en: %s", bt_xml_file.c_str());
                  found = true;
                  break;
              }
          }
          
          if (!found) {
              RCLCPP_FATAL(this->get_logger(), "❌ No se pudo encontrar behaviour_tree.xml");
              
              // Debug: listar contenido del directorio
              RCLCPP_INFO(this->get_logger(), "📂 Contenido de %s:", package_path.c_str());
              std::string ls_cmd = "ls -la " + package_path;
              std::system(ls_cmd.c_str());
              
              return;
          }
      }
      
      // Crear el árbol de comportamiento desde archivo
      tree_ = factory_.createTreeFromFile(bt_xml_file);
      RCLCPP_INFO(this->get_logger(), "✅ Behavior Tree creado exitosamente");
      
      // Inicializar todos los nodos BT con este AgentNode
      initializeBTNodes();
      
      // Iniciar el hilo de ejecución del BT
      bt_running_ = true;
      bt_thread_ = std::make_unique<std::thread>([this]() {
          this->executeBehaviorTree();
      });
      
      RCLCPP_INFO(this->get_logger(), "🚀 Behavior Tree ejecutándose en segundo plano");
      
  } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Error creando Behavior Tree: %s", e.what());
  }
}

void AgentNode::initializeBTNodes() {
    RCLCPP_INFO(this->get_logger(), "🔧 Inicializando nodos del Behavior Tree...");
    
    // Inicializar todos los nodos de acción (igual que en ROS1)
    auto nodes = tree_.nodes;
    for (auto& node : nodes) {
        // Actions
        if (auto action = dynamic_cast<GoNearChargingStation*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<Recharge*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<BackToStation*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<GoNearHumanTarget*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<MonitorHumanTarget*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<GoNearUGV*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<MonitorUGV*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<GoNearWP*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<TakeImage*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<InspectPVArray*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<GoNearStation*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<PickTool*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<DropTool*>(node.get())) {
            action->init(this);
        } else if (auto action = dynamic_cast<DeliverTool*>(node.get())) {
            action->init(this);
        }
        // Conditions
        else if (auto condition = dynamic_cast<MissionOver*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<Idle*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsBatteryEnough*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsBatteryFull*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskRecharge*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskMonitor*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskMonitorUGV*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskInspect*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskInspectPVArray*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsTaskDeliverTool*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsAgentNearChargingStation*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsAgentNearHumanTarget*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsAgentNearUGV*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsAgentNearWP*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<NeedToDropTheTool*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<HasAgentTheTool*>(node.get())) {
            condition->init(this);
        } else if (auto condition = dynamic_cast<IsAgentNearStation*>(node.get())) {
            condition->init(this);
        }
    }
    RCLCPP_INFO(this->get_logger(), "✅ Todos los nodos BT inicializados");
}

void AgentNode::executeBehaviorTree() {
    RCLCPP_INFO(this->get_logger(), "🔄 Iniciando ejecución del Behavior Tree...");
    
    while (rclcpp::ok() && bt_running_) {
        try {
            BT::NodeStatus status = tree_.tickRoot();
            
            if (mission_over_ && status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "🏁 Misión terminada y dron en casa. Deteniendo BT.");
                break;
            }

            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
                RCLCPP_DEBUG(this->get_logger(), "🔄 BT Status: %d", static_cast<int>(status));
                last_log = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error en ejecución del BT: %s", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "🛑 Behavior Tree detenido");
}

AgentNode::~AgentNode() 
{
  bt_running_ = false;
  if (bt_thread_ && bt_thread_->joinable()) {
      bt_thread_->join();
  }
}

void AgentNode::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  position_.update(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
  pose_received_ = true;
  last_pose_s_ = nowSeconds();
}

void AgentNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr battery) {
  battery_ = battery->percentage;

  // isBatteryEnough() decide si hay que abortar la tarea e ir a recargar, pero
  // NO se llamaba desde ningun sitio: era codigo muerto, asi que
  // battery_enough_ se quedaba con su valor inicial (true) para siempre y la
  // condicion IsBatteryEnough del arbol devolvia SUCCESS aunque la bateria
  // estuviera al 16%. Reevaluarlo aqui es lo natural: se ejecuta justo cuando
  // llega una lectura nueva. Es barato (dos comparaciones) y solo actua de
  // verdad cuando el estado cambia.
  isBatteryEnough();
}

void AgentNode::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info) {
    int old_state = state_;

    // Every branch here maps an *explicitly recognised* AS2 status onto our
    // internal numbering. DISARMED now has its own case: it used to share
    // `default:` with every unknown/garbage value, which is precisely what
    // made "no telemetry yet" indistinguishable from "safely on the ground".
    bool recognised = true;
    switch (info->status.state) {
        case as2_msgs::msg::PlatformStatus::DISARMED:
            state_ = 0;
            break;
        case as2_msgs::msg::PlatformStatus::LANDED:
            // Armed -> ready to take off; disarmed -> powered down.
            state_ = info->armed ? 2 : 1;
            break;
        case as2_msgs::msg::PlatformStatus::TAKING_OFF:
            state_ = 3;
            break;
        case as2_msgs::msg::PlatformStatus::FLYING:
            state_ = 4;
            break;
        case as2_msgs::msg::PlatformStatus::LANDING:
            state_ = 6;
            break;
        case as2_msgs::msg::PlatformStatus::EMERGENCY:
            state_ = 5;
            break;
        default:
            // Hold the previous state instead of silently reporting 0.
            recognised = false;
            break;
    }

    platform_armed_ = info->armed;
    if (recognised) {
        platform_info_received_ = true;
        last_platform_info_s_ = nowSeconds();
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[telemetria] Estado de plataforma no reconocido (%d); conservando %s(%d).",
            static_cast<int>(info->status.state), stateName(state_), state_);
    }

    if (state_ != old_state) {
        if (poseFresh()) {
            RCLCPP_INFO(this->get_logger(),
                "[estado] %s(%d) -> %s(%d) | armado=%s | z=%.2f m",
                stateName(old_state), old_state, stateName(state_), state_,
                info->armed ? "si" : "no", position_.getZ());
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[estado] %s(%d) -> %s(%d) | armado=%s | z=sin datos",
                stateName(old_state), old_state, stateName(state_), state_,
                info->armed ? "si" : "no");
        }
    }

    static bool first_info = true;
    if (first_info) {
        RCLCPP_INFO(this->get_logger(),
            "[telemetria] Primer PlatformInfo recibido. Estado inicial: %s(%d).",
            stateName(state_), state_);
        first_info = false;
    }

    // landing_in_progress_ mirrors the observed platform state rather than
    // being managed by whichever BT leaf happened to call land(). If that
    // leaf's own wait gives up early while the Land action is still running
    // server-side, a stale `false` here would let a new task fire a GoTo that
    // fights the still-active Land for control mode - the "flies away instead
    // of landing" bug.
    landing_in_progress_ = (state_ == 6);
}

// --- Telemetry-trust helpers ---------------------------------------------
bool AgentNode::platformInfoFresh(double max_age_s) const {
  if (!platform_info_received_) return false;
  return (nowSeconds() - last_platform_info_s_.load()) <= max_age_s;
}

bool AgentNode::poseFresh(double max_age_s) const {
  if (!pose_received_) return false;
  return (nowSeconds() - last_pose_s_.load()) <= max_age_s;
}

const char* AgentNode::stateName(int s) {
  switch (s) {
    case 0: return "DESARMADO";
    case 1: return "EN_SUELO_DESARMADO";
    case 2: return "EN_SUELO_ARMADO";
    case 3: return "DESPEGANDO";
    case 4: return "VOLANDO";
    case 5: return "EMERGENCIA";
    case 6: return "ATERRIZANDO";
    default: return "DESCONOCIDO";
  }
}

bool AgentNode::isConfirmedLanded(float ground_z) {
  // Deliberately conservative: missing telemetry is never treated as landed.
  if (!platformInfoFresh()) return false;
  if (!(state_ == 0 || state_ == 1 || state_ == 2)) return false;
  if (!poseFresh()) return false;
  return position_.getZ() <= (ground_z + kOnGroundAltitudeMargin);
}

void AgentNode::missionOverCallback(const mission_planner::msg::MissionOver::SharedPtr value) {
  mission_over_ = value->value;
  if(mission_over_) {
      RCLCPP_WARN(this->get_logger(), "🚨 ¡ALERTA! SE HA RECIBIDO LA ORDEN: MISSION OVER");
      RCLCPP_WARN(this->get_logger(), "🛑 Cancelando tareas y volviendo a casa...");
  }
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

// The platform's own state machine rejects TAKE_OFF from DISARMED with
// "Invalid transition: DISARMED -> TAKE_OFF" - mission_sequencer.py only
// arms each drone once at mission start, but the platform auto-disarms
// after every completed landing. Without an explicit re-arm before each
// subsequent take_off(), a drone's second (and every later) flight would
// have its Takeoff goal silently rejected and just wait out our own
// state-wait timeout every time. Called unconditionally before every
// take_off() - re-arming an already-armed platform is a harmless no-op
// server-side (it just logs "UAV is already armed" and returns
// success=false, which is why this treats a false response as fine to
// proceed on rather than a hard failure; only a genuinely unreachable
// service - the server not responding at all - is treated as fatal).
bool AgentNode::arm() {
  if (!arm_cli_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "❌ Arming service no responde en /%s/set_arming_state", beacon_.id.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future = arm_cli_->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "❌ Timeout esperando respuesta de /%s/set_arming_state", beacon_.id.c_str());
    return false;
  }
  auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ /%s/set_arming_state devolvió false (normal si ya estaba armado)", beacon_.id.c_str());
  }
  return true;
}

bool AgentNode::cancelGoTo() {
  if (!goto_ac_) return true;
  if (!goto_ac_->action_server_is_ready()) return true;
  auto future = goto_ac_->async_cancel_all_goals();
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(),
      "[movimiento] Timeout cancelando el GoTo pendiente antes de aterrizar.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[movimiento] GoTo pendiente cancelado.");
  return true;
}

bool AgentNode::land(bool blocking) {
  // 1. Seguridad: Esperar conexión
  if (!land_ac_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "❌ Land Server no responde en /%s/LandBehavior", beacon_.id.c_str());
    return false;
  }

  auto goal_msg = as2_msgs::action::Land::Goal();
  goal_msg.land_speed = 0.5; // Velocidad de descenso suave

  auto goal_options = rclcpp_action::Client<as2_msgs::action::Land>::SendGoalOptions();
  // Previously fire-and-forget: with no callbacks, a rejected or aborted
  // Land goal was indistinguishable from one still quietly in progress -
  // we only ever inferred landing from the separate platform_info state
  // topic, so a rejection left the agent waiting on a state transition
  // that was never coming until our own timeout eventually gave up.
  goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::Land>::SharedPtr& handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "❌ Land goal RECHAZADO por /%s/LandBehavior", beacon_.id.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "✅ Land goal aceptado por /%s/LandBehavior", beacon_.id.c_str());
      }
    };
  goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::Land>::WrappedResult& result) {
      RCLCPP_INFO(this->get_logger(), "🛬 [Land] Resultado de la acción: code=%d", static_cast<int>(result.code));
    };

  RCLCPP_INFO(this->get_logger(), "🛬 ENVIANDO ATERRIZAJE...");
  auto future_goal_handle = land_ac_->async_send_goal(goal_msg, goal_options);

  if (blocking) {
    auto result_future = land_ac_->async_get_result(future_goal_handle.get());
  }
  return true;
}

bool AgentNode::take_off(float height, bool blocking) {
  // 1. Seguridad: Esperar conexión
  if (!takeoff_ac_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "❌ Takeoff Server no responde en /%s/TakeoffBehavior", beacon_.id.c_str());
    return false;
  }

  auto goal_msg = as2_msgs::action::Takeoff::Goal();
  goal_msg.takeoff_height = height;
  goal_msg.takeoff_speed = 1.0; // Velocidad explícita

  auto goal_options = rclcpp_action::Client<as2_msgs::action::Takeoff>::SendGoalOptions();
  goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::Takeoff>::SharedPtr& handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "❌ Takeoff goal RECHAZADO por /%s/TakeoffBehavior", beacon_.id.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "✅ Takeoff goal aceptado por /%s/TakeoffBehavior", beacon_.id.c_str());
      }
    };
  goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::Takeoff>::WrappedResult& result) {
      RCLCPP_INFO(this->get_logger(), "🚀 [Takeoff] Resultado de la acción: code=%d", static_cast<int>(result.code));
    };

  RCLCPP_INFO(this->get_logger(), "🚀 ENVIANDO DESPEGUE (h=%.2f)...", height);
  auto future_goal_handle = takeoff_ac_->async_send_goal(goal_msg, goal_options);

  if (blocking) {
    auto result_future = takeoff_ac_->async_get_result(future_goal_handle.get());
  }
  return true;
}

bool AgentNode::go_to_waypoint(float x, float y, float z, bool blocking) {
  // 1. Seguridad: Esperar conexión
  if (!goto_ac_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "❌ GoTo Server no responde en /%s/GoToBehavior", beacon_.id.c_str());
    return false;
  }

  auto goal_msg = as2_msgs::action::GoToWaypoint::Goal();
  
  goal_msg.target_pose.header.stamp = this->now();
  goal_msg.target_pose.header.frame_id = "earth";  

  goal_msg.target_pose.point.x = x;
  goal_msg.target_pose.point.y = y;
  goal_msg.target_pose.point.z = z;
  
  goal_msg.max_speed = 1.0;
  goal_msg.yaw.mode = as2_msgs::msg::YawMode::KEEP_YAW; 

  auto goal_options = rclcpp_action::Client<as2_msgs::action::GoToWaypoint>::SendGoalOptions();
  goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::SharedPtr& handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "❌ GoTo goal RECHAZADO por /%s/GoToBehavior", beacon_.id.c_str());
      }
    };
  goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<as2_msgs::action::GoToWaypoint>::WrappedResult& result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "📍 [GoTo] Resultado de la acción no exitoso: code=%d", static_cast<int>(result.code));
      }
    };

  RCLCPP_INFO(this->get_logger(), "📍 MOVIENDO a [%.2f, %.2f, %.2f] en frame %s",
              x, y, z, goal_msg.target_pose.header.frame_id.c_str());
  
  auto future_goal_handle = goto_ac_->async_send_goal(goal_msg, goal_options);
  
  if (blocking) {
    auto result_future = goto_ac_->async_get_result(future_goal_handle.get());
    if(result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(this->get_logger(), "⚠️ El movimiento no terminó con éxito total");
        return false;
    }
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

rclcpp_action::GoalResponse AgentNode::handleNewTaskListGoal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const mission_planner::action::NewTaskList::Goal> goal) 
{
  // 1. Escudo contra misiones cuando ya terminó todo
  if(mission_over_) {
      RCLCPP_WARN(this->get_logger(), "⛔ GOAL RECHAZADO: La misión ha terminado.");
      return rclcpp_action::GoalResponse::REJECT;
  }

  // 2. Escudo contra re-asignaciones redundantes
  // if (!task_queue_.empty()) {
  //     RCLCPP_WARN(this->get_logger(), "⚠️ GOAL RECHAZADO: El dron ya está ejecutando tareas. Ignorando nueva misión del planificador para evitar interrupciones (Halt).");
  //     return rclcpp_action::GoalResponse::REJECT;
  // }

  RCLCPP_INFO(this->get_logger(), "🎯 [handleNewTaskListGoal] GOAL RECIBIDO y ACEPTADO!");
  RCLCPP_INFO(this->get_logger(), "📨 Agent ID del goal: %s | Tareas: %zu", goal->agent_id.c_str(), goal->task_list.size());
  
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
  if(mission_over_) {
     auto result = std::make_shared<mission_planner::action::NewTaskList::Result>();
     result->ack = false;
     goal_handle->abort(result);
     return;
  }
  RCLCPP_INFO(this->get_logger(), "🚀 [handleNewTaskListAccepted] PROCESANDO NUEVA LISTA DE TAREAS!");
  
  RCLCPP_INFO(this->get_logger(), "🚀 [handleNewTaskListAccepted] PROCESANDO NUEVA LISTA DE TAREAS!");
  
  // Ejecutar en un thread separado
  std::thread([this, goal_handle]() {

    if(mission_over_) return;

    // Don't overwrite task_queue_ while a landing confirmation is in
    // flight - see landing_in_progress_'s comment in the header. Applying a
    // new task here means a leaf like GoNearWP can call go_to_waypoint()
    // while land() is still active server-side: the platform then gets a
    // GoTo (POSITION) goal and a Land (SPEED) goal fighting over control
    // mode simultaneously, and the drone flies off instead of landing.
    // Bound is 190s - just past the 180s overall-tick-deadline safety net,
    // which is the SLOWEST path that's still guaranteed to clear this flag.
    // The previous 20s bound was shorter than the 60s per-wait timeout on
    // the same flag, so it could (and did) expire while our own code was
    // still legitimately waiting for a landing that was going to succeed
    // within that 60s - this is what caused the exact GoTo/Land fight above.
    auto wait_start = std::chrono::steady_clock::now();
    while (landing_in_progress_ && rclcpp::ok()) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() > 190.0) {
        RCLCPP_ERROR(this->get_logger(),
          "[handleNewTaskListAccepted] Landing confirmation still in progress after 190s (safety nets should have cleared this), applying new task list anyway");
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

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
    classes::Task* task = task_queue_.popFrontIfMatches(id, type);
    if (task) {
        delete task;
        return;
    }

    if (task_queue_.empty()) {
        RCLCPP_WARN(this->get_logger(), "[removeTaskFromQueue] Task queue already empty");
    } else {
        RCLCPP_WARN(this->get_logger(), "[removeTaskFromQueue] Task isn't in the first place of the queue. Not deleting.");
    }
}

void AgentNode::emptyTheQueue() {
    task_queue_.clear();
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
         
    if(task_result_ac_) {
        goal.result = 2; // Halted to recharge
        task_result_ac_->async_send_goal(goal);
    }
}

float AgentNode::batteryNeededToGetHome() {
    // Autonomia necesaria para volver a la base desde donde estamos, con
    // reserva. Un umbral fijo del 30% servia estando cerca, pero un dron a 60 m
    // gasta mas del 30% solo en volver: se quedaba sin bateria a medio camino,
    // aterrizaba donde podia y ya no habia forma de recuperarlo. Ahora el
    // umbral SUBE con la distancia, asi que la vuelta se inicia mientras aun
    // queda autonomia suficiente.
    if (!poseFresh())
        return kMinBatteryReserve;

    std::string station = "charging_station_" + id_;
    auto &stations = known_positions_["charging_stations"];
    auto it = stations.find(station);
    if (it == stations.end())
        return kMinBatteryReserve;

    const float dist = classes::distance2D(position_, it->second);
    const float seconds_home = dist / kCruiseSpeedMps;
    const float needed = seconds_home * kBatteryDrainPerSecond + kMinBatteryReserve;
    // Nunca pedir mas del 90%: si no, un dron lejano no despegaria jamas.
    return std::min(needed, 0.90f);
}

void AgentNode::isBatteryEnough() {
    bool previous_state = battery_enough_;
    bool empty_queue = task_queue_.empty();

    const float threshold = batteryNeededToGetHome();
    const bool below_threshold = battery_ < threshold;

    // "El planner me ha reactivado": si estando en modo recarga me llega una
    // tarea, es que el planner ha decidido que siga. Pero SOLO se acepta si la
    // bateria da de verdad para ello. Sin esa condicion se producia un
    // flip-flop a 2 Hz: la cola no vacia lo ponia a true, el umbral lo volvia a
    // poner a false, y el dron nunca llegaba a emprender el regreso mientras se
    // quedaba sin bateria en el sitio.
    if(!battery_enough_ && !empty_queue && !below_threshold)
        battery_enough_ = true;

    // Check if this Agent has battery enough to fulfill its next task
    if(below_threshold)
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
        
        if(!battery_enough_) {
            RCLCPP_WARN(this->get_logger(),
                "[bateria] %.0f%% por debajo del minimo para volver a casa (%.0f%%). Abortando tarea y regresando.",
                battery_ * 100.0f, threshold * 100.0f);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "[bateria] %.0f%%: autonomia suficiente de nuevo.", battery_ * 100.0f);
        }
        emptyTheQueue();
        
        if(battery_ac_) {
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