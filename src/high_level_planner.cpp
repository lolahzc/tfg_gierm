#include "mission_planner/high_level_planner.hpp"

// class Agent Constructors
Agent::Agent() 
  : id_(), type_(), position_(), battery_(), task_queue_(), old_task_queue_(), old_first_task_id_(), planner_(nullptr),
    last_beacon_time_(), last_beacon_(), nh_(nullptr), position_sub_(nullptr), battery_sub_(nullptr),
    pose_topic_(), battery_topic_(), battery_enough_(true), ntl_ac_(nullptr),
    battery_feedback_(), battery_result_()
{
  // Constructor por defecto: no inicializa nh_ ni crea recursos (como actions o suscriptores), ya que id_ no está definido.
  // battery_as_ y task_result_as_ se inicializan automáticamente a nullptr por defecto.
}