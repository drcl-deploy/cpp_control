// Onboard base-state estimation for the G1, for deployments with no motion
// capture.
//
// WHAT THIS IS
//
// The difftrack policies observe a WORLD-frame base pose and twist
// (`imitation.global_obs = true`). In simulation that comes from the plant's
// ground truth; in the lab it came from OptiTrack. Neither exists on a robot
// standing in a room with no cameras, and an IMU alone cannot supply it — it
// has no position and no world linear velocity.
//
// So this node estimates them, from what the robot does have: the pelvis IMU,
// the joint encoders, and the leg kinematics. It is a PASSIVE LISTENER. It
// subscribes to `/lowstate` and publishes `nav_msgs/Odometry` on `/odom`. It
// never writes a command, never claims a ros2_control interface, and never
// touches the robot's SDK. cpp_control keeps sole ownership of `/lowcmd`,
// which is the whole reason this is a standalone node rather than a controller
// inside legged_control2's controller_manager.
//
// THE ESTIMATOR ITSELF IS NOT OURS
//
// The filter is legged_control2's (Qiayuan Liao, Hybrid Robotics) — the same
// `legged::LinearKalmanFilter` + `legged::GmObserver` pair that BeyondMimic's
// motion_tracking_controller runs on this robot, with that project's own G1
// parameters. legged_control2 ships as closed-source binaries, so this file is
// glue: unitree_hg LowState in, `legged::LeggedModel` filled, filter stepped,
// Odometry out. Everything numerical happens inside their .so.
//
//   contact-aided EKF   state = [base position, base linear velocity,
//                       one contact-point position per foot]
//   process             IMU specific force, rotated by the IMU attitude
//   measurement         leg kinematics to each foot, weighted by a contact
//                       probability from the generalized-momentum observer's
//                       estimated contact wrench
//
// WHAT IT CANNOT DO
//
// There is no absolute position reference in the loop, so POSITION AND YAW
// DRIFT. Height and the two tilt axes are observable (feet on the ground, IMU
// gravity vector); x, y and heading are dead reckoning and drift without
// bound. That is a property of the sensor set, not of this filter.
//
// It is survivable here only because a difftrack clip is short and is anchored
// onto the robot at entry, so what matters is drift ACCUMULATED DURING ONE
// CLIP, not absolute accuracy. Check that on your own clip against the sim's
// ground truth before believing it on hardware:
//
//     bash scripts/run_difftrack_sim2sim.sh -E compare -d 15 g1_walk
//
// FRAME CONVENTIONS — the part that silently ruins a deployment
//
// The published Odometry follows REP-105 exactly, because the consumer has no
// way to tell a wrong convention from a bad estimate:
//
//   header.frame_id   "odom"    pose is in the world/odom frame
//   child_frame_id    "pelvis"  TWIST IS IN THE BODY FRAME
//
// So `twist.linear` and `twist.angular` are BODY-frame, and a consumer wanting
// world-frame velocity must rotate them by `pose.orientation`. cpp_control's
// `odom_twist_frame` parameter defaults to `child` and does exactly that.
// Publishing a world-frame twist under a body-frame contract is the same class
// of bug as MuJoCo's free-joint qvel[3:6], which cost a 4x tracking-duration
// regression in this project's own sim2mujoco path and was invisible until the
// robot rotated.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <legged_estimation/GmObserver.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_model/LeggedModel.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unitree_hg/msg/low_state.hpp>

namespace
{

/// Read a whole file, or throw with the path in the message.
std::string read_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("cannot open URDF: " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

class LeggedOdomNode : public rclcpp::Node
{
public:
    LeggedOdomNode() : rclcpp::Node("legged_odom")
    {
        // ── Model ────────────────────────────────────────────────────────────
        const auto urdf_path = declare_parameter<std::string>(
            "urdf_path", "/opt/ros/jazzy/share/unitree_description/urdf/g1/main.urdf");
        base_frame_ = declare_parameter<std::string>("base_name", "pelvis");
        const auto six_dof = declare_parameter<std::vector<std::string>>(
            "six_dof_contact_names", {"LL_FOOT", "LR_FOOT"});
        // The empty default is spelled out rather than written `{}`: a bare
        // brace-init binds to declare_parameter's ParameterDescriptor overload
        // instead of the default-value one, which leaves the parameter with NO
        // default and throws the moment it is not set in the yaml.
        const auto three_dof = declare_parameter<std::vector<std::string>>(
            "three_dof_contact_names", std::vector<std::string>{});

        model_ = std::make_shared<legged::LeggedModel>(read_file(urdf_path));
        model_->setBaseNames(base_frame_);
        model_->setEndEffectorNames(three_dof, six_dof);

        // ── Joints: LowState motor index -> pinocchio joint index ────────────
        //
        // Resolved BY NAME against the URDF, never by assuming the two orders
        // agree. `joint_names[i]` is the joint that LowState's motor_state[i]
        // reports, so the list is in unitree_hg motor order and its length is
        // how many motors this robot has that the model knows about.
        joint_names_ =
            declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
        if (joint_names_.empty())
            throw std::runtime_error("legged_odom: joint_names is required (unitree_hg motor order)");

        const auto model_joints = model_->getJointNames();
        joint_to_model_.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto it = std::find(model_joints.begin(), model_joints.end(), joint_names_[i]);
            if (it == model_joints.end())
                throw std::runtime_error("legged_odom: joint '" + joint_names_[i] +
                                         "' is not in the URDF");
            joint_to_model_[i] = static_cast<size_t>(std::distance(model_joints.begin(), it));
        }
        num_joints_ = model_->getNumJoints();
        if (joint_names_.size() != num_joints_)
        {
            // Not fatal: a model with more joints than motors (hands, a head)
            // is fine as long as every MOTOR is placed. The unplaced ones stay
            // at zero, which is what a frozen joint is.
            RCLCPP_WARN(get_logger(),
                        "legged_odom: %zu motors mapped into a %zu-joint model; the %zu unmapped "
                        "model joints are held at zero",
                        joint_names_.size(), num_joints_, num_joints_ - joint_names_.size());
        }

        // ── Filter ───────────────────────────────────────────────────────────
        //
        // Defaults are legged_control2's own, overridden with the values
        // BeyondMimic's motion_tracking_controller ships for THIS robot.
        legged::LinearKalmanFilter::Configurations cfg;
        cfg.imuAccelerationNoiseDensity =
            declare_parameter<double>("estimation.imu.acceleration_noise_density",
                                      cfg.imuAccelerationNoiseDensity);
        cfg.imuAccelerationBiasNoiseDensity =
            declare_parameter<double>("estimation.imu.acceleration_bias_noise_density",
                                      cfg.imuAccelerationBiasNoiseDensity);
        cfg.contactProcessNoisePosition = declare_parameter<double>(
            "estimation.contact.process_noise_position", cfg.contactProcessNoisePosition);
        cfg.contactSensorNoisePosition = declare_parameter<double>(
            "estimation.contact.sensor_noise_position", cfg.contactSensorNoisePosition);
        cfg.contactHeightSensorNoise = declare_parameter<double>(
            "estimation.contact.height_sensor_noise", cfg.contactHeightSensorNoise);
        cfg.contactRadius =
            declare_parameter<double>("estimation.contact.radius", cfg.contactRadius);
        cfg.contactForceThreshold =
            declare_parameter<double>("estimation.contact.force_threshold", cfg.contactForceThreshold);
        cfg.contactForceScale =
            declare_parameter<double>("estimation.contact.force_scale", cfg.contactForceScale);
        cfg.contactZmpLengthX =
            declare_parameter<double>("estimation.contact.zmp_length_x", cfg.contactZmpLengthX);
        cfg.contactZmpLengthY =
            declare_parameter<double>("estimation.contact.zmp_length_y", cfg.contactZmpLengthY);
        cfg.positionNoiseDensity =
            declare_parameter<double>("estimation.position.noise", cfg.positionNoiseDensity);

        kf_ = std::make_shared<legged::LinearKalmanFilter>(model_);
        kf_->setConfigurations(cfg);
        kf_->reset();

        gm_ = std::make_shared<legged::GmObserver>(
            model_, declare_parameter<double>("estimation.gm_cutoff_frequency", 10.0));

        // ── Sizes ────────────────────────────────────────────────────────────
        //
        // Pinocchio floating base:
        //   q   = [ x y z | qx qy qz qw | joints ]      nq = 7 + nj  (36)
        //   v   = [ vx vy vz | wx wy wz | dq ]          nv = 6 + nj  (35)
        //   tau = [ joints ]                                    nj  (29)
        //
        // and for a FREEFLYER both halves of v are expressed in the LOCAL
        // (body) frame. That is why the gyro goes in raw and the KF's velocity
        // comes back through getVelocityLocal() rather than getVelocityGlobal().
        //
        // NOTE THE THIRD LINE. `tau` is nj, NOT nv: LeggedModel carries joint
        // torques only and the closed binary appends the six base rows itself.
        // Sizing it to nv here (the obvious symmetry, and what this file did
        // first) does not raise — it corrupts the heap on the first update,
        // because GmObserver builds an nv-long vector from a member it believes
        // is nj and writes six elements past the end. It dies as
        //   Fatal glibc error: malloc.c:2599 (sysmalloc): assertion failed
        // several stack frames away from the cause.
        //
        // So: CHECK the sizes the constructor chose, never impose them. The
        // vectors come back by reference and .setZero(n) on an Eigen vector
        // RESIZES it, which is how a read-only-looking line reshapes a model
        // that another library is holding a view of.
        const auto& pin = model_->getPinModel();
        const auto expect = [](const char* what, Eigen::Index got, Eigen::Index want) {
            if (got != want)
                throw std::runtime_error(std::string("legged_odom: legged_model sized ") + what +
                                         " to " + std::to_string(got) + ", expected " +
                                         std::to_string(want) +
                                         " — legged_control2's layout changed, and writing to it "
                                         "on that assumption would corrupt memory rather than fail");
        };
        expect("q", model_->getGeneralizedPosition().size(), pin.nq);
        expect("v", model_->getGeneralizedVelocity().size(), pin.nv);
        expect("tau", model_->getTorque().size(), static_cast<Eigen::Index>(num_joints_));

        model_->getGeneralizedPosition().setZero();
        model_->getGeneralizedVelocity().setZero();
        model_->getTorque().setZero();
        model_->getGeneralizedPosition()(6) = 1.0;  // qw

        // ── I/O ──────────────────────────────────────────────────────────────
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        publish_tf_ = declare_parameter<bool>("publish_tf", false);
        const auto lowstate_topic = declare_parameter<std::string>("lowstate_topic", "/lowstate");
        const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
        publish_decimation_ = declare_parameter<int>("publish_decimation", 1);
        max_dt_ = declare_parameter<double>("max_dt", 0.05);
        nominal_dt_ = declare_parameter<double>("nominal_dt", 0.002);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS());
        if (publish_tf_)
            tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
            lowstate_topic, rclcpp::SensorDataQoS(),
            [this](unitree_hg::msg::LowState::SharedPtr msg) { on_lowstate(msg); });

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res)
            {
                kf_->reset();
                have_prev_ = false;
                res->success = true;
                res->message = "state estimator reset to the origin";
                RCLCPP_WARN(get_logger(), "legged_odom: RESET — the world origin is now here");
            });

        RCLCPP_INFO(get_logger(),
                    "legged_odom: %s -> %s | base=%s contacts=%zu joints=%zu/%zu | twist is in "
                    "the CHILD (body) frame, per REP-105",
                    lowstate_topic.c_str(), odom_topic.c_str(), base_frame_.c_str(),
                    six_dof.size() + three_dof.size(), joint_names_.size(), num_joints_);
    }

private:
    void on_lowstate(const unitree_hg::msg::LowState::SharedPtr& msg)
    {
        const rclcpp::Time now = this->get_clock()->now();

        // Step off the MEASURED interval, never a nominal one: a dropped or
        // late state message otherwise integrates the IMU over a period the
        // robot did not spend there. First message, or an implausible gap
        // (a pause, a reconnect), falls back to the nominal period rather than
        // integrating a second of acceleration in one step.
        double dt = nominal_dt_;
        if (have_prev_)
        {
            const double measured = (now - prev_stamp_).seconds();
            if (measured > 1e-5 && measured < max_dt_)
                dt = measured;
            else if (measured >= max_dt_)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "legged_odom: %.1f ms since the last LowState — clamping the "
                                     "integration step to %.1f ms",
                                     measured * 1e3, nominal_dt_ * 1e3);
        }
        prev_stamp_ = now;
        have_prev_ = true;

        auto& q = model_->getGeneralizedPosition();
        auto& v = model_->getGeneralizedVelocity();
        auto& tau = model_->getTorque();

        // Attitude and angular rate come straight from the IMU and are NOT
        // estimated: the pelvis quaternion is Unitree's own onboard fusion and
        // the gyro is body-local, which is exactly pinocchio's freeflyer
        // convention. Only position and linear velocity are filtered.
        const auto& imu = msg->imu_state;
        const Eigen::Quaterniond quat(imu.quaternion[0], imu.quaternion[1], imu.quaternion[2],
                                      imu.quaternion[3]);  // msg is w,x,y,z
        q(3) = quat.x();
        q(4) = quat.y();
        q(5) = quat.z();
        q(6) = quat.w();  // pinocchio stores x,y,z,w

        v(3) = imu.gyroscope[0];
        v(4) = imu.gyroscope[1];
        v(5) = imu.gyroscope[2];

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const size_t j = joint_to_model_[i];
            q(7 + j) = msg->motor_state[i].q;
            v(6 + j) = msg->motor_state[i].dq;
            tau(j) = msg->motor_state[i].tau_est;  // tau is nj, with no base rows
        }

        // Close the loop: the filter's own last estimate is the base state the
        // kinematics are evaluated at. Without it the foot positions the
        // measurement update compares against would be computed at the origin.
        q.head<3>() = kf_->getPosition();
        v.head<3>() = kf_->getVelocityLocal();

        model_->update();

        // Contact wrenches from the generalized-momentum observer; the filter
        // turns their magnitude into a per-foot contact probability, which is
        // what weights each leg's kinematic measurement.
        gm_->update(dt);
        kf_->setContactWrenches(gm_->getContactWrenches());
        kf_->setAccelerationLocal(Eigen::Vector3d(
            imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]));

        kf_->updateImuProcess(dt);
        kf_->updateContactsMeasurement(dt);

        if (++tick_ % std::max(1, publish_decimation_) != 0)
            return;
        publish(now, quat, Eigen::Vector3d(imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]));
    }

    void publish(const rclcpp::Time& stamp, const Eigen::Quaterniond& quat,
                 const Eigen::Vector3d& gyro)
    {
        const Eigen::Vector3d p = kf_->getPosition();
        const Eigen::Vector3d v_body = kf_->getVelocityLocal();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = p.x();
        odom.pose.pose.position.y = p.y();
        odom.pose.pose.position.z = p.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();

        // REP-105: the twist is in child_frame_id, i.e. the BODY frame. Both
        // halves — the gyro is already body-local and the linear velocity is
        // rotated into the body here rather than left in the world.
        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();
        odom.twist.twist.angular.x = gyro.x();
        odom.twist.twist.angular.y = gyro.y();
        odom.twist.twist.angular.z = gyro.z();

        odom_pub_->publish(odom);

        if (tf_pub_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = odom.header;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = p.x();
            tf.transform.translation.y = p.y();
            tf.transform.translation.z = p.z();
            tf.transform.rotation = odom.pose.pose.orientation;
            tf2_msgs::msg::TFMessage m;
            m.transforms.push_back(tf);
            tf_pub_->publish(m);
        }
    }

    std::shared_ptr<legged::LeggedModel> model_;
    std::shared_ptr<legged::LinearKalmanFilter> kf_;
    std::shared_ptr<legged::GmObserver> gm_;

    std::vector<std::string> joint_names_;
    std::vector<size_t> joint_to_model_;
    size_t num_joints_ = 0;

    std::string base_frame_, odom_frame_;
    bool publish_tf_ = false;
    int publish_decimation_ = 1;
    double max_dt_ = 0.05;
    double nominal_dt_ = 0.002;

    rclcpp::Time prev_stamp_;
    bool have_prev_ = false;
    uint64_t tick_ = 0;

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<LeggedOdomNode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("legged_odom"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
