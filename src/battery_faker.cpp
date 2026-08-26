#include "mission_planner/battery_faker.hpp"


#include <chrono>
#include <thread>

BatteryFaker::BatteryFaker() 
  : Node("battery_faker"),
    loop_rate_(0.2),
    // battery_increase_(0.001f),
    // battery_decrease_(0.001f)
    battery_increase_(0.05f), // Carga muy rápida (5% cada medio segundo)
    battery_decrease_(0.02f)  // Descarga rápida (2% cada medio segundo)
{
    // Declarar todos los parámetros
    this->declare_parameter<std::string>("battery_mode", "static");
    // Ritmos por tick de 500 ms. Con los valores por defecto un dron pasa de
    // 100% a 30% (umbral de "bateria insuficiente" en isBatteryEnough) tras
    // ~87 s de vuelo, y se recarga de 30% a 100% en ~18 s posado en su base.
    this->declare_parameter<double>("battery_decrease", 0.004);
    this->declare_parameter<double>("battery_increase", 0.020);
    // Carga de rescate por tick para un dron varado fuera de una base.
    // A 0 se desactiva y un dron que se quede sin bateria lejos queda perdido.
    this->declare_parameter<double>("rescue_trickle", 0.0015);
    this->declare_parameter<std::string>("id", "drone0");  // Valor por defecto específico
    this->declare_parameter<std::string>("pose_topic", "");
    this->declare_parameter<std::string>("state_topic", "");
    this->declare_parameter<std::string>("config_file", "");

    // Obtener parámetros PRIMERO
    this->get_parameter("battery_mode", battery_mode_);
    {
      double dec = 0.004, inc = 0.020;
      double resc = 0.0015;
      this->get_parameter("battery_decrease", dec);
      this->get_parameter("battery_increase", inc);
      this->get_parameter("rescue_trickle", resc);
      battery_decrease_ = static_cast<float>(dec);
      battery_increase_ = static_cast<float>(inc);
      rescue_trickle_ = static_cast<float>(resc);
    }
    this->get_parameter("id", id_);
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("state_topic", state_topic_);
    this->get_parameter("config_file", config_file_);

    // Si los topics están vacíos, configurar valores por defecto
    if (pose_topic_.empty()) {
        pose_topic_ = "/" + id_ + "/self_localization/pose";
    }
    if (state_topic_.empty()) {
        state_topic_ = "/" + id_ + "/platform/info";
    }

    // Determinar modo de batería
    if (battery_mode_ == "recharge_anywhere")
        mode_ = 1;
    else if (battery_mode_ == "recharge_in_base")
        mode_ = 2;
    else if (battery_mode_ == "only_discharge")
        mode_ = 3;
    else // static
        mode_ = 0;

    // Cargar archivo de configuración
    if (config_file_.empty()) {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("mission_planner");
            config_file_ = package_share_dir + "/config/conf.yaml";
            RCLCPP_INFO(this->get_logger(), "Found package share directory: %s", package_share_dir.c_str());
        } catch (const std::exception& e)  {
            RCLCPP_ERROR(this->get_logger(), "Package 'mission_planner' not found: %s", e.what());
        }
    }

    RCLCPP_INFO(this->get_logger(), "Reading config file: %s", config_file_.c_str());
    readConfigFile(config_file_);

    // Publishers
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/" + id_ + "/sensor_measurements/battery", 1);

    // Subscribers
    control_sub_ = this->create_subscription<mission_planner::msg::BatteryControl>(
        "/" + id_ + "/battery_fake/control", 1,
        std::bind(&BatteryFaker::controlCallback, this, std::placeholders::_1));

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(), 
        std::bind(&BatteryFaker::positionCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
        state_topic_, 1,
        std::bind(&BatteryFaker::platformInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Battery Faker READY for UAV: %s", id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "State topic: %s", state_topic_.c_str());
    
    battery_.percentage = 1.0f;

    // Timer para actualizar la batería. Guardado en un miembro: como local se
    // destruía al terminar el constructor y update_battery() no se llamaba nunca.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&BatteryFaker::update_battery, this));

    RCLCPP_INFO(this->get_logger(),
        "[bateria] Faker activo (modo=%s): descarga %.4f/tick, carga %.4f/tick (tick=500 ms).",
        battery_mode_.c_str(), battery_decrease_, battery_increase_);
}

// Tope de la carga de rescate: lo justo para volver a una base, no mas.
static constexpr float kRescueThreshold = 0.55f;

void BatteryFaker::update_battery() {
    int flag;
    switch(mode_) {
        case 1: // Recharge anywhere when landed
            switch(state_) {
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                    battery_.percentage = battery_.percentage + battery_increase_;
                    if(battery_.percentage > 1.0f)
                        battery_.percentage = 1.0f;
                    break;
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                default:
                    break;
            }
            break;
        case 2: // Recharge only in recharging base
            switch(state_) {
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                    flag = 0;
                    for(auto& charging_station : known_positions_["charging_stations"]) {
                        if(classes::distance(position_, charging_station.second) < 0.5f)
                            flag = 1;
                    }
                    if(flag) {
                        battery_.percentage = battery_.percentage + battery_increase_;
                        if(battery_.percentage > 1.0f)
                            battery_.percentage = 1.0f;
                    } else if(rescue_trickle_ > 0.0f && battery_.percentage < kRescueThreshold) {
                        // Rescate: un dron que se queda sin bateria lejos aterriza
                        // donde puede y, sin esto, se queda INUTILIZADO para siempre
                        // (no esta en una base, luego nunca recarga, luego nunca
                        // puede volver a una base). Con una carga lenta acaba
                        // teniendo autonomia para regresar el solo. Es mucho mas
                        // lento que la carga en base, asi que no quita sentido a
                        // volver a casa; solo evita perder el dron.
                        battery_.percentage += rescue_trickle_;
                        if(battery_.percentage > kRescueThreshold)
                            battery_.percentage = kRescueThreshold;
                        if(!rescue_announced_) {
                            RCLCPP_WARN(this->get_logger(),
                                "[bateria] %s quedo en tierra fuera de una base; carga de rescate lenta hasta el %.0f%% para que pueda volver.",
                                id_.c_str(), kRescueThreshold * 100.0f);
                            rescue_announced_ = true;
                        }
                    }
                    break;
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                default:
                    break;
            }
            break;
        case 3: // Recharge disabled
            switch(state_) {
                case 3: // TAKING_OFF
                case 4: // FLYING_AUTO
                case 5: // FLYING_MANUAL
                case 6: // LANDING
                    battery_.percentage = battery_.percentage - battery_decrease_;
                    if(battery_.percentage < 0.0f)
                        battery_.percentage = 0.0f;
                    break;
                case 0: // UNINITIALIZED
                case 1: // LANDED_DISARMED
                case 2: // LANDED_ARMED
                default:
                    break;
            }
            break;
        case 0: // Battery Static
        default:
            break;
    }
    
    battery_.header.stamp = this->now();
    battery_pub_->publish(battery_);
    
    // RCLCPP_DEBUG(this->get_logger(), "Mode: %d\tUAV State: %d\tPercentage: %.3f", 
    //              mode_, state_, battery_.percentage);
}

void BatteryFaker::readConfigFile(const std::string& config_file) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_file);
        if(yaml_config["positions"]) {
            for(auto const& group : yaml_config["positions"]) {
                for(auto const& position : group.second) {
                    known_positions_[group.first.as<std::string>()][position.first.as<std::string>()] = 
                        classes::Position(position.first.as<std::string>(),
                                        position.second['x'].as<float>(),
                                        position.second['y'].as<float>(),
                                        position.second['z'].as<float>());
                }
            }
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading config file: %s", e.what());
    }
}

void BatteryFaker::controlCallback(const mission_planner::msg::BatteryControl::SharedPtr control) {
    if(control->percentage != -1.0f)
        battery_.percentage = control->percentage;
    mode_ = control->mode;
    if(control->battery_increase >= 0.0f)
        battery_increase_ = control->battery_increase;
    if(control->battery_decrease >= 0.0f)
        battery_decrease_ = control->battery_decrease;
}

void BatteryFaker::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    position_.update(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
}

void BatteryFaker::platformInfoCallback(const as2_msgs::msg::PlatformInfo::SharedPtr info) {
    // Mapear estados de Aerostack2 a los estados originales.
    //
    // OJO con DISARMED: antes caia en el `default:` y se mapeaba a 0
    // (UNINITIALIZED), que en update_battery() no recarga en ningun modo. Pero
    // los drones SE DESARMAN al terminar de aterrizar, asi que ese era su
    // estado normal estando posados en la base: el faker nunca veia los
    // estados 1/2, los unicos que recargan, y la bateria se quedaba clavada
    // (observado: drone2 al 0% "recargando" indefinidamente sobre su propia
    // estacion). DISARMED significa "en el suelo", asi que va con los estados
    // de aterrizado.
    switch (info->status.state) {
        case as2_msgs::msg::PlatformStatus::DISARMED:
            state_ = 1; // en el suelo, desarmado
            break;
        case as2_msgs::msg::PlatformStatus::LANDED:
            state_ = info->armed ? 2 : 1; // en el suelo, armado o no
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
            state_ = 5; // FLYING_MANUAL
            break;
        default:
            state_ = 0; // UNINITIALIZED
            break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryFaker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}