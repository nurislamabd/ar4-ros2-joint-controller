#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

// Trapezoidal velocity motion profile for a single joint.
//
// The joint accelerates at a constant rate up to max_vel, cruises, then
// decelerates so that it arrives at the target with zero velocity. The switch
// to the deceleration phase is decided by the kinematic stopping distance
// v^2 / (2a): once the remaining distance is no greater than the stopping
// distance, the joint begins to brake.
struct TrapProfile {
    double current_pos;
    double target_pos;
    double current_vel;
    double max_vel;
    double accel;
    bool done;

    TrapProfile()
        : current_pos(0.0), target_pos(0.0), current_vel(0.0),
          max_vel(1.5), accel(2.0), done(false) {}

    // Advance the profile by dt seconds.
    void update(double dt) {
        if (done) return;

        double error = target_pos - current_pos;
        double dist = std::abs(error);
        int direction = (error > 0.0) ? 1 : -1;

        // Snap to the target once we are within a small tolerance.
        if (dist < 0.001) {
            current_pos = target_pos;
            current_vel = 0.0;
            done = true;
            return;
        }

        // Kinematic stopping distance from the current velocity.
        double stopping_dist = (current_vel * current_vel) / (2.0 * accel);

        if (stopping_dist >= dist) {
            // Deceleration phase.
            current_vel -= accel * dt;
            if (current_vel < 0.0) current_vel = 0.0;
        } else if (current_vel < max_vel) {
            // Acceleration phase.
            current_vel += accel * dt;
            if (current_vel > max_vel) current_vel = max_vel;
        }
        // Otherwise the joint is cruising at max_vel.

        // Integrate position, clamping to the target so we never overshoot.
        double step = direction * current_vel * dt;
        if (std::abs(step) >= dist) {
            current_pos = target_pos;
            current_vel = 0.0;
            done = true;
            return;
        }
        current_pos += step;
    }

    void set_target(double target) {
        target_pos = target;
        current_vel = 0.0;
        done = false;
    }
};

// Publishes /joint_states for a 6-DOF arm plus a single actuated gripper joint,
// driving each joint with a trapezoidal velocity profile. The arm continuously
// oscillates between a home pose and a configurable target pose.
class ArmController : public rclcpp::Node {
public:
    ArmController() : Node("arm_controller") {
        declare_parameter("joint_targets", std::vector<double>{0, 0, 0, 0, 0, 0});
        declare_parameter("max_velocity", 1.5);
        declare_parameter("acceleration", 2.0);
        declare_parameter("publish_rate", 50.0);
        // Gripper opening in meters (URDF limit: 0 = closed, 0.02 = open).
        declare_parameter("gripper_target", 0.0);
        // Seconds to pause at each end of the oscillation before reversing.
        declare_parameter("dwell_time", 1.0);

        auto targets = get_parameter("joint_targets").as_double_array();
        double max_vel = get_parameter("max_velocity").as_double();
        double accel = get_parameter("acceleration").as_double();
        double rate = get_parameter("publish_rate").as_double();
        double gripper_target = get_parameter("gripper_target").as_double();
        dwell_time_ = get_parameter("dwell_time").as_double();

        // Joint names must match the URDF. Only the actuated gripper joint is
        // published; robot_state_publisher derives the mimicking
        // gripper_finger_right_joint from the URDF <mimic> tag.
        joint_names_ = {"joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6",
                        "gripper_finger_left_joint"};

        // Initialize the profiles: 6 arm joints + 1 gripper.
        profiles_.resize(7);
        for (int i = 0; i < 6; i++) {
            profiles_[i].max_vel = max_vel;
            profiles_[i].accel = accel;
            profiles_[i].set_target(targets[i]);
        }
        // The gripper travels a short stroke, so it moves slowly.
        profiles_[6].max_vel = 0.05;
        profiles_[6].accel = 0.2;
        profiles_[6].set_target(gripper_target);

        // The two poses the arm oscillates between.
        pose_a_ = std::vector<double>(7, 0.0);  // home
        pose_b_ = {targets[0], targets[1], targets[2],
                   targets[3], targets[4], targets[5], gripper_target};
        at_b_ = true;  // the profiles above were initialized toward pose_b_

        publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        int period_ms = static_cast<int>(1000.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ArmController::tick, this));

        nominal_dt_ = 1.0 / rate;
        last_time_ = now();
        last_stamp_ = now();
        RCLCPP_INFO(get_logger(), "AR4 joint controller started at %.0f Hz", rate);
    }

private:
    void tick() {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Guard against a non-monotonic system clock: a negative or implausibly
        // large dt would make the profile lurch, so fall back to the nominal
        // timer period in that case.
        if (dt <= 0.0 || dt > 5.0 * nominal_dt_) {
            dt = nominal_dt_;
        }

        // Keep the published timestamp strictly monotonic so downstream
        // consumers (e.g. robot_state_publisher / TF) never receive data
        // stamped in the past if the system clock steps backwards.
        if (current_time <= last_stamp_) {
            last_stamp_ = last_stamp_ + rclcpp::Duration::from_seconds(nominal_dt_);
        } else {
            last_stamp_ = current_time;
        }

        for (auto& p : profiles_) {
            p.update(dt);
        }

        // Once every joint has reached its target, pause, then reverse so the
        // arm oscillates between the two poses continuously.
        bool all_done = true;
        for (auto& p : profiles_) {
            if (!p.done) { all_done = false; break; }
        }
        if (all_done) {
            dwell_elapsed_ += dt;
            if (dwell_elapsed_ >= dwell_time_) {
                const auto& next = at_b_ ? pose_a_ : pose_b_;
                for (size_t i = 0; i < profiles_.size(); i++) {
                    profiles_[i].set_target(next[i]);
                }
                at_b_ = !at_b_;
                dwell_elapsed_ = 0.0;
            }
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = last_stamp_;
        msg.name = joint_names_;
        for (auto& p : profiles_) {
            msg.position.push_back(p.current_pos);
            msg.velocity.push_back(p.current_vel);
        }
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<TrapProfile> profiles_;
    std::vector<std::string> joint_names_;
    std::vector<double> pose_a_;   // home pose
    std::vector<double> pose_b_;   // configured target pose
    bool at_b_ = false;            // pose currently being driven toward
    double dwell_time_ = 1.0;      // seconds to pause at each endpoint
    double dwell_elapsed_ = 0.0;   // time spent paused at the current endpoint
    double nominal_dt_ = 0.02;     // expected timer period (1 / publish_rate)
    rclcpp::Time last_time_;
    rclcpp::Time last_stamp_;      // monotonic timestamp for published messages
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmController>());
    rclcpp::shutdown();
    return 0;
}
