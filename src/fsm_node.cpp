#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <chrono>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;

class BumpGoNode : public rclcpp::Node {
public:
    BumpGoNode() : Node("bump_go"), state_(State::FORWARD), scan_received_(false),
                   obstacle_count_(0), clear_count_(0) {

        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort().durability_volatile();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
                last_scan_      = std::move(msg);
                last_scan_time_ = this->now();
                if (!scan_received_) {
                    scan_received_ = true;
                    state_ts_      = this->now();
                    RCLCPP_INFO(get_logger(), "First scan! front=%.2fm",
                        minZone(0, 30));
                }
            });

        vel_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/bt_markers", 10);

        timer_ = create_wall_timer(200ms, [this]() { control_cycle(); });

        RCLCPP_INFO(get_logger(), "BumpGo FSM improved started");
    }

private:
    enum class State { FORWARD, BACK, TURN, STOP };
    State        state_;
    rclcpp::Time state_ts_;
    rclcpp::Time last_scan_time_;
    bool         scan_received_;

    // ① Hysteresis counters
    int obstacle_count_;
    int clear_count_;
    static constexpr int HYSTERESIS_ON  = 3;  // scans pour confirmer obstacle
    static constexpr int HYSTERESIS_OFF = 3;  // scans pour confirmer dégagement

    // ③ Vitesse progressive — current speed
    float current_speed_ = 0.0f;
    static constexpr float ALPHA = 0.2f;  // facteur de lissage

    const rclcpp::Duration BACKING_TIME {3s};
    const rclcpp::Duration TURNING_TIME {3s};
    const rclcpp::Duration SCAN_TIMEOUT {5s};
    static constexpr float SPEED_LINEAR  = 0.2f;
    static constexpr float SPEED_ANGULAR = 0.5f;
    static constexpr float OBSTACLE_DIST = 0.5f;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

    // ── HELPER distance minimale dans une zone ──────────
    float minZone(int center_deg, int half_deg) {
        if (!last_scan_) return std::numeric_limits<float>::infinity();
        const auto& r = last_scan_->ranges;
        int   n   = static_cast<int>(r.size());
        float min = std::numeric_limits<float>::infinity();
        for (int d = center_deg - half_deg; d <= center_deg + half_deg; ++d) {
            int idx = ((d % n) + n) % n;
            if (std::isfinite(r[idx]) &&
                r[idx] > last_scan_->range_min &&
                r[idx] < last_scan_->range_max)
                min = std::min(min, r[idx]);
        }
        return min;
    }

    // ── HELPER publish TwistStamped ─────────────────────
    void publishVel(float lin, float ang) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x  = lin;
        cmd.twist.angular.z = ang;
        vel_pub_->publish(cmd);
    }

    // ③ Vitesse progressive — filtre passe-bas
    float smoothSpeed(float target) {
        current_speed_ = current_speed_ + ALPHA * (target - current_speed_);
        return current_speed_;
    }

    // ③ Vitesse adaptée selon distance obstacle
    float adaptiveSpeed() {
        float dist  = minZone(0, 30);
        float ratio = (dist - OBSTACLE_DIST) / (1.5f - OBSTACLE_DIST);
        ratio = std::max(0.0f, std::min(1.0f, ratio));
        return smoothSpeed(SPEED_LINEAR * ratio);
    }

    // ① Hysteresis — check obstacle avec confirmation
    bool check_forward_2_back() {
        // 5 zones Lidar — avant + avant-gauche + avant-droit
        float front = minZone(0,   30);
        float fl    = minZone(45,  20);
        float fr    = minZone(315, 20);
        float nearest = std::min({front, fl, fr});

        if (nearest < OBSTACLE_DIST) {
            obstacle_count_ = std::min(obstacle_count_ + 1, HYSTERESIS_ON);
            clear_count_    = 0;
        } else {
            clear_count_    = std::min(clear_count_ + 1, HYSTERESIS_OFF);
            obstacle_count_ = std::max(0, obstacle_count_ - 1);
        }
        return obstacle_count_ >= HYSTERESIS_ON;
    }

    bool check_forward_2_stop() {
        if (!scan_received_) return false;
        return (this->now() - last_scan_time_) > SCAN_TIMEOUT;
    }

    bool check_stop_2_forward() {
        return scan_received_ &&
               (this->now() - last_scan_time_) < SCAN_TIMEOUT;
    }

    bool check_back_2_turn() {
        return (this->now() - state_ts_) > BACKING_TIME;
    }

    bool check_turn_2_forward() {
        return (this->now() - state_ts_) > TURNING_TIME;
    }

    // ② Choix de direction — côté le plus dégagé
    float bestTurnDirection() {
        float left  = minZone(90,  30);
        float right = minZone(270, 30);
        RCLCPP_INFO(get_logger(), "Turn dir: L=%.2f R=%.2f → %s",
            left, right, left > right ? "gauche" : "droite");
        return (left > right) ? 1.0f : -1.0f;
    }

    // ③ RViz markers
    void publishMarkers(const std::string& state_name) {
        visualization_msgs::msg::MarkerArray markers;

        // Marker texte — état courant
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "base_link";
        text.header.stamp    = this->now();
        text.ns              = "fsm_state";
        text.id              = 0;
        text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action          = visualization_msgs::msg::Marker::ADD;
        text.pose.position.z = 0.5;
        text.scale.z         = 0.3;
        text.color.r = 1.0f; text.color.g = 1.0f;
        text.color.b = 1.0f; text.color.a = 1.0f;
        text.text            = "FSM: " + state_name;
        text.lifetime        = rclcpp::Duration(1s);
        markers.markers.push_back(text);

        // Marker zone frontale — rouge si obstacle, vert si libre
        float front = minZone(0, 30);
        visualization_msgs::msg::Marker zone;
        zone.header.frame_id = "base_link";
        zone.header.stamp    = this->now();
        zone.ns              = "front_zone";
        zone.id              = 1;
        zone.type            = visualization_msgs::msg::Marker::CYLINDER;
        zone.action          = visualization_msgs::msg::Marker::ADD;
        zone.pose.position.x = 0.3;
        zone.scale.x = zone.scale.y = 0.1;
        zone.scale.z = 0.05;
        zone.color.a = 0.8f;
        if (front < OBSTACLE_DIST) {
            zone.color.r = 1.0f; zone.color.g = 0.0f; zone.color.b = 0.0f;
        } else if (front < 1.0f) {
            zone.color.r = 1.0f; zone.color.g = 1.0f; zone.color.b = 0.0f;
        } else {
            zone.color.r = 0.0f; zone.color.g = 1.0f; zone.color.b = 0.0f;
        }
        zone.lifetime = rclcpp::Duration(1s);
        markers.markers.push_back(zone);

        marker_pub_->publish(markers);
    }

    void control_cycle() {
        if (!scan_received_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for scan...");
            return;
        }

        std::string state_name;
        float turn_dir = 1.0f;

        switch (state_) {
        case State::FORWARD:
            state_name = "FORWARD";
            // ③ vitesse progressive
            publishVel(adaptiveSpeed(), 0.0f);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "FORWARD speed=%.2f front=%.2fm",
                current_speed_, minZone(0, 30));
            if (check_forward_2_stop()) {
                state_    = State::STOP;
                state_ts_ = this->now();
                RCLCPP_WARN(get_logger(), "→ STOP");
            } else if (check_forward_2_back()) {
                turn_dir_ = bestTurnDirection();
                state_    = State::BACK;
                state_ts_ = this->now();
                RCLCPP_WARN(get_logger(), "→ BACK (obstacle confirmé x%d)",
                    HYSTERESIS_ON);
            }
            break;

        case State::BACK:
            state_name = "BACK";
            publishVel(-SPEED_LINEAR, 0.0f);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "BACK");
            if (check_back_2_turn()) {
                state_    = State::TURN;
                state_ts_ = this->now();
                RCLCPP_INFO(get_logger(), "→ TURN");
            }
            break;

        case State::TURN:
            state_name = "TURN";
            publishVel(0.0f, turn_dir_ * SPEED_ANGULAR);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "TURN");
            if (check_turn_2_forward()) {
                obstacle_count_ = 0;
                clear_count_    = 0;
                current_speed_  = 0.0f;
                state_    = State::FORWARD;
                state_ts_ = this->now();
                RCLCPP_INFO(get_logger(), "→ FORWARD");
            }
            break;

        case State::STOP:
            state_name = "STOP";
            publishVel(0.0f, 0.0f);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "STOP");
            if (check_stop_2_forward()) {
                state_    = State::FORWARD;
                state_ts_ = this->now();
                RCLCPP_INFO(get_logger(), "→ FORWARD");
            }
            break;
        }

        publishMarkers(state_name);
    }

    float turn_dir_ = 1.0f;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpGoNode>());
    rclcpp::shutdown();
    return 0;
}
