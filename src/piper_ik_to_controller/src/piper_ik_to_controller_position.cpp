#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <algorithm>

#include "piper_eval_msgs/msg/piper_teleop_metric.hpp"
#include <osqp/osqp.h>
#include <osqp/cs.h>

class PositionIKControllerNode : public rclcpp::Node
{
public:
    PositionIKControllerNode(const rclcpp::NodeOptions &options) : Node("piper_ik_to_controller_position", options)
    {
        // --- Parameters ---
        this->declare_parameter<double>("control_rate_hz", 30.0);
        // Note: Gains are less critical in Iterative IK, but used for convergence speed
        this->declare_parameter<double>("linear_error_gain", 2.0);
        this->declare_parameter<double>("angular_error_gain", 2.0);
        this->declare_parameter<double>("singularity_damping", 0.05);
        this->declare_parameter<double>("max_joint_velocity", 3.0);
        this->declare_parameter<int>("ik_iterations", 50);      // NEW: Max iterations per step
        this->declare_parameter<double>("ik_tolerance", 0.005); // NEW: Stop if error < 5mm

        control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
        singularity_damping_ = this->get_parameter("singularity_damping").as_double();
        max_joint_velocity_ = this->get_parameter("max_joint_velocity").as_double();
        ik_max_iter_ = this->get_parameter("ik_iterations").as_int();
        ik_tol_ = this->get_parameter("ik_tolerance").as_double();

        if (!initializeKDLSolver())
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize KDL. Shutting down.");
            throw std::runtime_error("KDL initialization failed");
        }

        if (!initializeOSQPSolver())
        {
            RCLCPP_FATAL(get_logger(), "Failed to initialize OSQP. Shutting down.");
            throw std::runtime_error("OSQP initialization failed");
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", qos, std::bind(&PositionIKControllerNode::onPose, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states_feedback", qos, std::bind(&PositionIKControllerNode::onJointState, this, std::placeholders::_1));

        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        metrics_pub_ = this->create_publisher<piper_eval_msgs::msg::PiperTeleopMetric>("/piper/teleop_metrics_raw", 10);
        last_target_header_ = std::make_shared<std_msgs::msg::Header>();

        auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
        control_timer_ = this->create_wall_timer(control_period, std::bind(&PositionIKControllerNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Iterative Position IK Controller (Newton-Raphson + QP Safety) Ready.");
    }

    ~PositionIKControllerNode()
    {
        if (osqp_initialized_)
        {
            osqp_cleanup(workspace_);
            if (data_.P)
                c_free(data_.P);
            if (data_.A)
                c_free(data_.A);
        }
    }

private:
    bool initializeKDLSolver()
    {
        std::string urdf_string;
        this->get_parameter_or("robot_description", urdf_string, std::string(""));
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(urdf_string, tree))
            return false;
        if (!tree.getChain("base_link", "gripper_base", chain_))
            return false;

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
        dof_ = chain_.getNrOfJoints();

        urdf::Model model;
        if (!model.initString(urdf_string))
            return false;

        joint_names_.resize(dof_);
        q_min_.resize(dof_);
        q_max_.resize(dof_);
        unsigned int j = 0;
        for (const auto &segment : chain_.segments)
        {
            auto joint = segment.getJoint();
            if (joint.getType() != KDL::Joint::None)
            {
                joint_names_[j] = joint.getName();
                auto urdf_joint = model.getJoint(joint.getName());
                if (urdf_joint && urdf_joint->limits)
                {
                    q_min_(j) = urdf_joint->limits->lower;
                    q_max_(j) = urdf_joint->limits->upper;
                }
                j++;
            }
        }
        current_joint_positions_.resize(dof_);
        current_joint_positions_.data.setZero();
        return true;
    }

    bool initializeOSQPSolver()
    {
        P_x_.resize(dof_, 1.0);
        P_i_.resize(dof_);
        P_p_.resize(dof_ + 1);
        A_x_.resize(dof_, 1.0);
        A_i_.resize(dof_);
        A_p_.resize(dof_ + 1);

        for (unsigned int i = 0; i < dof_; ++i)
        {
            P_i_[i] = i;
            P_p_[i] = i;
            A_i_[i] = i;
            A_p_[i] = i;
        }
        P_p_[dof_] = dof_;
        A_p_[dof_] = dof_;

        q_vec_.resize(dof_);
        l_vec_.resize(dof_);
        u_vec_.resize(dof_);

        osqp_set_default_settings(&settings_);
        settings_.verbose = false;
        settings_.warm_start = true;
        settings_.polish = true;

        data_.n = dof_;
        data_.m = dof_;

        csc *P_csc = (csc *)c_malloc(sizeof(csc));
        P_csc->m = dof_;
        P_csc->n = dof_;
        P_csc->nzmax = dof_;
        P_csc->x = P_x_.data();
        P_csc->i = P_i_.data();
        P_csc->p = P_p_.data();
        P_csc->nz = -1;
        data_.P = P_csc;

        csc *A_csc = (csc *)c_malloc(sizeof(csc));
        A_csc->m = dof_;
        A_csc->n = dof_;
        A_csc->nzmax = dof_;
        A_csc->x = A_x_.data();
        A_csc->i = A_i_.data();
        A_csc->p = A_p_.data();
        A_csc->nz = -1;
        data_.A = A_csc;

        data_.q = q_vec_.data();
        data_.l = l_vec_.data();
        data_.u = u_vec_.data();

        if (osqp_setup(&workspace_, &data_, &settings_) != 0)
            return false;
        osqp_initialized_ = true;
        return true;
    }

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < dof_; ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it != msg->name.end())
            {
                current_joint_positions_(i) = msg->position[std::distance(msg->name.begin(), it)];
            }
        }
        if (!has_received_joint_state_)
        {
            has_received_joint_state_ = true;
            KDL::Frame initial_frame;
            fk_solver_->JntToCart(current_joint_positions_, initial_frame);
            target_frame_ = initial_frame;
        }
    }

    void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_frame_.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        target_frame_.M = KDL::Rotation::Quaternion(
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        *last_target_header_ = msg->header;
        has_target_ = true;
    }

    void controlLoop()
    {
        if (!has_received_joint_state_ || !has_target_ || !osqp_initialized_)
            return;

        KDL::JntArray q_curr_real(dof_);
        KDL::Frame frame_target;
        std_msgs::msg::Header target_header;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            q_curr_real = current_joint_positions_;
            frame_target = target_frame_;
            target_header = *last_target_header_;
        }

        auto t_pre_ik = this->get_clock()->now();

        // --- STRATEGY CHANGE: Newton-Raphson Iterative IK ---
        // 1. Start with the current robot state as the "guess"
        KDL::JntArray q_sol = q_curr_real;

        // 2. Iterate to find the joint angles that create ZERO error
        for (int iter = 0; iter < ik_max_iter_; ++iter)
        {
            KDL::Frame frame_sol;
            fk_solver_->JntToCart(q_sol, frame_sol);

            KDL::Twist error = KDL::diff(frame_sol, frame_target);

            // Check convergence (using translation magnitude)
            if (error.vel.Norm() < ik_tol_ && error.rot.Norm() < ik_tol_ * 2.0)
            {
                break;
            }

            Eigen::Matrix<double, 6, 1> err_eigen;
            err_eigen << error.vel.x(), error.vel.y(), error.vel.z(),
                error.rot.x(), error.rot.y(), error.rot.z();

            KDL::Jacobian J_kdl(dof_);
            jac_solver_->JntToJac(q_sol, J_kdl);
            Eigen::MatrixXd J = J_kdl.data;

            // Damped Least Squares for this step
            Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
            double manipulability = sqrt(std::abs(JJt.determinant()));
            double lambda = (manipulability < 0.01) ? singularity_damping_ : 0.0;

            Eigen::Matrix<double, 6, 6> A = JJt + lambda * Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::VectorXd delta_q = J.transpose() * A.inverse() * err_eigen;

            // Apply the update to our virtual solution
            for (unsigned int i = 0; i < dof_; ++i)
            {
                q_sol(i) += delta_q(i);
                // IMPORTANT: Keep virtual solution within joint limits during iteration
                q_sol(i) = std::clamp(q_sol(i), q_min_(i), q_max_(i));
            }
        }

        // --- 3. Safety: Enforce Velocity Limits via QP ---
        // We now have 'q_sol' (Position Goal) and 'q_curr_real' (Current State).
        // We need to calculate the velocity required to get there in ONE timestep.
        double dt = 1.0 / control_rate_hz_;
        Eigen::VectorXd q_dot_needed(dof_);

        for (unsigned int i = 0; i < dof_; ++i)
        {
            q_dot_needed(i) = (q_sol(i) - q_curr_real(i)) / dt;
        }

        // Solve QP to find the closest safe velocity to 'q_dot_needed'
        Eigen::VectorXd q_dot_safe(dof_);

        // Cost: Minimize difference from needed velocity
        for (unsigned int i = 0; i < dof_; ++i)
            q_vec_[i] = (c_float)-q_dot_needed(i);
        osqp_update_lin_cost(workspace_, q_vec_.data());

        // Constraints: Absolute Velocity limits & Joint Position Limits
        for (unsigned int i = 0; i < dof_; ++i)
        {
            double v_lim = max_joint_velocity_;
            double p_min_v = (q_min_(i) - q_curr_real(i)) / dt;
            double p_max_v = (q_max_(i) - q_curr_real(i)) / dt;
            l_vec_[i] = (c_float)std::max(-v_lim, p_min_v);
            u_vec_[i] = (c_float)std::min(v_lim, p_max_v);
        }
        osqp_update_bounds(workspace_, l_vec_.data(), u_vec_.data());

        if (osqp_solve(workspace_) == 0 && workspace_->info->status_val == OSQP_SOLVED)
        {
            for (unsigned int i = 0; i < dof_; ++i)
                q_dot_safe(i) = workspace_->solution->x[i];
        }
        else
        {
            q_dot_safe = q_dot_needed; // Fallback (with simple clamp below)
        }

        // --- 4. Final Position Command Integration ---
        KDL::JntArray q_next(dof_);
        for (unsigned int i = 0; i < dof_; ++i)
        {
            // Simply clamp velocity if QP failed
            if (workspace_->info->status_val != OSQP_SOLVED)
            {
                q_dot_safe(i) = std::clamp(q_dot_safe(i), -max_joint_velocity_, max_joint_velocity_);
            }

            q_next(i) = q_curr_real(i) + q_dot_safe(i) * dt;
            q_next(i) = std::clamp(q_next(i), q_min_(i), q_max_(i));
        }

        auto t_post_ik = this->get_clock()->now();
        sendJointCommands(q_next);

        // --- Metrics ---
        auto t_cmd = this->get_clock()->now();
        piper_eval_msgs::msg::PiperTeleopMetric metric;
        metric.header = target_header;
        metric.target_pose.position.x = frame_target.p.x();
        metric.target_pose.position.y = frame_target.p.y();
        metric.target_pose.position.z = frame_target.p.z();
        frame_target.M.GetQuaternion(metric.target_pose.orientation.x, metric.target_pose.orientation.y, metric.target_pose.orientation.z, metric.target_pose.orientation.w);
        metric.ik_solver_delay_ms = (t_post_ik - t_pre_ik).seconds() * 1000.0;
        metric.e2e_latency_ms_at_control = (t_cmd - target_header.stamp).seconds() * 1000.0;
        metrics_pub_->publish(metric);
    }

    void sendJointCommands(const KDL::JntArray &q)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        msg.name = joint_names_;
        msg.position.assign(q.data.data(), q.data.data() + dof_);
        joint_command_pub_->publish(msg);
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::mutex state_mutex_;
    bool has_received_joint_state_ = false, has_target_ = false;

    rclcpp::Publisher<piper_eval_msgs::msg::PiperTeleopMetric>::SharedPtr metrics_pub_;
    std::shared_ptr<std_msgs::msg::Header> last_target_header_;

    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    unsigned int dof_;
    std::vector<std::string> joint_names_;
    KDL::JntArray q_min_, q_max_, current_joint_positions_;
    KDL::Frame target_frame_;

    double control_rate_hz_, singularity_damping_, max_joint_velocity_, ik_tol_;
    int ik_max_iter_;

    OSQPWorkspace *workspace_ = nullptr;
    OSQPSettings settings_;
    OSQPData data_;
    bool osqp_initialized_ = false;
    std::vector<c_float> q_vec_, l_vec_, u_vec_, P_x_, A_x_;
    std::vector<c_int> P_i_, A_i_, P_p_, A_p_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<PositionIKControllerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
