#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include <limits>
#include <algorithm>
#include <memory>

using namespace std::chrono_literals;

const std::string KEY_PUB       = "vel_pub";
const std::string KEY_SCAN      = "last_scan";
const std::string KEY_SCAN_TIME = "last_scan_time";
const std::string KEY_NODE      = "ros_node";
const std::string KEY_MARKER    = "marker_pub";
// ← shared counter so A_Tourner can reset it
const std::string KEY_OBS_COUNT = "obs_count";

using TwistPub  = rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr;
using MarkerPub = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
using LaserPtr  = sensor_msgs::msg::LaserScan::SharedPtr;
using NodePtr   = rclcpp::Node*;

static constexpr float OBSTACLE_DIST  = 0.5f;
static constexpr float SPEED_LINEAR   = 0.2f;
static constexpr float SPEED_ANGULAR  = 0.5f;
static constexpr int   HYSTERESIS_ON  = 2;   // 2 scans @ 1.3Hz ≈ 1.5s
static constexpr float ALPHA          = 0.2f;

// ── HELPERS ─────────────────────────────────────────────
float minZone(const LaserPtr& scan, int center_deg, int half_deg) {
    if (!scan) return std::numeric_limits<float>::infinity();
    const auto& r = scan->ranges;
    int   n   = static_cast<int>(r.size());
    float min = std::numeric_limits<float>::infinity();
    for (int d = center_deg - half_deg; d <= center_deg + half_deg; ++d) {
        int idx = ((d % n) + n) % n;
        if (std::isfinite(r[idx]) &&
            r[idx] > scan->range_min &&
            r[idx] < scan->range_max)
            min = std::min(min, r[idx]);
    }
    return min;
}

void publishVel(const TwistPub& pub, NodePtr node, float lin, float ang) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = node->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x  = lin;
    cmd.twist.angular.z = ang;
    pub->publish(cmd);
}

void publishMarker(const MarkerPub& pub, NodePtr node,
                   const std::string& label, float front) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker text;
    text.header.frame_id = "base_link";
    text.header.stamp    = node->now();
    text.ns   = "bt_state"; text.id = 0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.z = 0.5;
    text.scale.z = 0.3;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.text    = "BT: " + label;
    text.lifetime = rclcpp::Duration(1s);
    markers.markers.push_back(text);

    visualization_msgs::msg::Marker zone;
    zone.header.frame_id = "base_link";
    zone.header.stamp    = node->now();
    zone.ns   = "front_zone"; zone.id = 1;
    zone.type = visualization_msgs::msg::Marker::CYLINDER;
    zone.action = visualization_msgs::msg::Marker::ADD;
    zone.pose.position.x = 0.3;
    zone.scale.x = zone.scale.y = 0.1; zone.scale.z = 0.05;
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

    pub->publish(markers);
}

// ══ CONDITIONS ══════════════════════════════════════════
class C_ObstacleDetecte : public BT::ConditionNode {
public:
    C_ObstacleDetecte(const std::string& name, const BT::NodeConfig& cfg)
        : BT::ConditionNode(name, cfg) {}
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        if (!scan) return BT::NodeStatus::FAILURE;

        float front = minZone(scan, 0,   30);
        float fl    = minZone(scan, 45,  20);
        float fr    = minZone(scan, 315, 20);
        float nearest = std::min({front, fl, fr});

        // Read shared counter from blackboard
        int count = 0;
        config().blackboard->get<int>(KEY_OBS_COUNT, count);

        if (nearest < OBSTACLE_DIST) {
            count = std::min(count + 1, HYSTERESIS_ON);
        } else {
            count = std::max(0, count - 1);
        }

        // Write back
        config().blackboard->set<int>(KEY_OBS_COUNT, count);

        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
            "C_Obstacle: nearest=%.2f count=%d/%d", nearest, count, HYSTERESIS_ON);

        return count >= HYSTERESIS_ON
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }
};

class C_LaserInactif : public BT::ConditionNode {
public:
    C_LaserInactif(const std::string& name, const BT::NodeConfig& cfg)
        : BT::ConditionNode(name, cfg) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        rclcpp::Time last;
        [[maybe_unused]] auto ok =
            config().blackboard->get<rclcpp::Time>(KEY_SCAN_TIME, last);
        return (node->now() - last) > rclcpp::Duration(5s)
            ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// ══ ACTIONS ═════════════════════════════════════════════
class A_Avancer : public BT::SyncActionNode {
public:
    A_Avancer(const std::string& name, const BT::NodeConfig& cfg)
        : BT::SyncActionNode(name, cfg), current_speed_(0.0f) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        auto pub  = config().blackboard->get<TwistPub>(KEY_PUB);
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        auto mpub = config().blackboard->get<MarkerPub>(KEY_MARKER);

        float front = minZone(scan, 0, 30);
        float ratio = (front - OBSTACLE_DIST) / (1.5f - OBSTACLE_DIST);
        ratio = std::max(0.0f, std::min(1.0f, ratio));
        float target = SPEED_LINEAR * ratio;
        current_speed_ = current_speed_ + ALPHA * (target - current_speed_);
        float speed = std::max(0.05f, current_speed_);

        publishVel(pub, node, speed, 0.0f);
        publishMarker(mpub, node, "AVANCER", front);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "[BT] AVANCER speed=%.2f front=%.2fm", speed, front);
        return BT::NodeStatus::SUCCESS;
    }
private:
    float current_speed_;
};

class A_Reculer : public BT::StatefulActionNode {
public:
    A_Reculer(const std::string& name, const BT::NodeConfig& cfg)
        : BT::StatefulActionNode(name, cfg) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        start_ = std::chrono::steady_clock::now();
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        RCLCPP_WARN(node->get_logger(), "[BT] RECULER start");
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        auto pub  = config().blackboard->get<TwistPub>(KEY_PUB);
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        auto mpub = config().blackboard->get<MarkerPub>(KEY_MARKER);
        publishVel(pub, node, -SPEED_LINEAR, 0.0f);
        publishMarker(mpub, node, "RECULER", minZone(scan, 0, 30));
        return (std::chrono::steady_clock::now() - start_) >= 3s
            ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    void onHalted() override {}
private:
    std::chrono::steady_clock::time_point start_;
};

class A_Tourner : public BT::StatefulActionNode {
public:
    A_Tourner(const std::string& name, const BT::NodeConfig& cfg)
        : BT::StatefulActionNode(name, cfg), direction_(1.0f) {}
    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override {
        start_    = std::chrono::steady_clock::now();
        min_time_ = 3s;   // minimum turn duration
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        float left  = minZone(scan, 90,  30);
        float right = minZone(scan, 270, 30);
        direction_  = (left > right) ? 1.0f : -1.0f;
        RCLCPP_INFO(node->get_logger(),
            "[BT] TOURNER %s (L=%.2f R=%.2f)",
            direction_ > 0 ? "gauche" : "droite", left, right);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto pub  = config().blackboard->get<TwistPub>(KEY_PUB);
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        auto mpub = config().blackboard->get<MarkerPub>(KEY_MARKER);

        publishVel(pub, node, 0.0f, direction_ * SPEED_ANGULAR);
        publishMarker(mpub, node, "TOURNER", minZone(scan, 0, 30));

        auto elapsed = std::chrono::steady_clock::now() - start_;

        // Must turn at least min_time_ AND front must be clear
        float front = minZone(scan, 0, 30);
        bool time_ok  = elapsed >= min_time_;
        bool path_ok  = front > OBSTACLE_DIST * 1.5f;  // 0.75m clear ahead

        if (time_ok && path_ok) {
            // ← Reset hysteresis counter so C_ObstacleDetecte starts fresh
            config().blackboard->set<int>(KEY_OBS_COUNT, 0);
            RCLCPP_INFO(node->get_logger(),
                "[BT] TOURNER done — front=%.2f CLEAR", front);
            return BT::NodeStatus::SUCCESS;
        }

        // Safety: max 8s to avoid infinite spin
        if (elapsed >= 8s) {
            config().blackboard->set<int>(KEY_OBS_COUNT, 0);
            RCLCPP_WARN(node->get_logger(), "[BT] TOURNER timeout");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        config().blackboard->set<int>(KEY_OBS_COUNT, 0);
    }

private:
    std::chrono::steady_clock::time_point start_;
    std::chrono::seconds min_time_;
    float direction_;
};

class A_Stop : public BT::SyncActionNode {
public:
    A_Stop(const std::string& name, const BT::NodeConfig& cfg)
        : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        auto pub  = config().blackboard->get<TwistPub>(KEY_PUB);
        auto node = config().blackboard->get<NodePtr>(KEY_NODE);
        auto scan = config().blackboard->get<LaserPtr>(KEY_SCAN);
        auto mpub = config().blackboard->get<MarkerPub>(KEY_MARKER);
        publishVel(pub, node, 0.0f, 0.0f);
        publishMarker(mpub, node, "STOP", minZone(scan, 0, 30));
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
            "[BT] STOP");
        return BT::NodeStatus::SUCCESS;
    }
};

// ══ NŒUD ROS2 ═══════════════════════════════════════════
class BTNavigator : public rclcpp::Node {
public:
    BTNavigator() : Node("bt_navigator") {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort().durability_volatile();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                last_scan_      = msg;
                last_scan_time_ = this->now();
                if (!scan_received_) {
                    scan_received_ = true;
                    RCLCPP_INFO(get_logger(), "First scan! front=%.2fm",
                        minZone(last_scan_, 0, 30));
                }
            });

        vel_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/bt_markers", 10);
        last_scan_time_ = this->now();

        init_timer_ = create_wall_timer(0ms, [this]() {
            init_timer_->cancel();
            init_bt();
            timer_ = create_wall_timer(200ms, [this]() {
                if (!scan_received_) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Waiting for scan...");
                    return;
                }
                tick_bt();
            });
        });

        RCLCPP_INFO(get_logger(), "BTNavigator created");
    }

private:
    void init_bt() {
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<C_ObstacleDetecte>("C_ObstacleDetecte");
        factory.registerNodeType<C_LaserInactif>   ("C_LaserInactif");
        factory.registerNodeType<A_Avancer>         ("A_Avancer");
        factory.registerNodeType<A_Reculer>         ("A_Reculer");
        factory.registerNodeType<A_Tourner>         ("A_Tourner");
        factory.registerNodeType<A_Stop>            ("A_Stop");

        this->declare_parameter<std::string>("tree_file", "");
        std::string tree_file;
        this->get_parameter("tree_file", tree_file);

        auto bb = BT::Blackboard::create();
        bb->set<TwistPub>    (KEY_PUB,       vel_pub_);
        bb->set<MarkerPub>   (KEY_MARKER,    marker_pub_);
        bb->set<NodePtr>     (KEY_NODE,      this);
        bb->set<LaserPtr>    (KEY_SCAN,      nullptr);
        bb->set<rclcpp::Time>(KEY_SCAN_TIME, this->now());
        bb->set<int>         (KEY_OBS_COUNT, 0);  // ← shared hysteresis counter

        try {
            tree_ = factory.createTreeFromFile(tree_file, bb);
            groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_, 1666);
            RCLCPP_INFO(get_logger(), "BT loaded + Groot2 on port 1666");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "BT error: %s", e.what());
        }
    }

    void tick_bt() {
        tree_.rootBlackboard()->set<LaserPtr>    (KEY_SCAN,      last_scan_);
        tree_.rootBlackboard()->set<rclcpp::Time>(KEY_SCAN_TIME, last_scan_time_);
        tree_.tickOnce();
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr     vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    BT::Tree tree_;
    std::unique_ptr<BT::Groot2Publisher> groot2_pub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Time last_scan_time_;
    bool scan_received_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BTNavigator>());
    rclcpp::shutdown();
    return 0;
}