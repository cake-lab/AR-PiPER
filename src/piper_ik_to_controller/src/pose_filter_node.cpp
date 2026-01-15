#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <Eigen/Dense>
#include <deque>
#include <string>
#include <memory>
#include <vector>

// Forward declaration of the filter base class
class PoseFilterBase;

class PoseFilterNode : public rclcpp::Node
{
public:
    PoseFilterNode();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::unique_ptr<PoseFilterBase> filter_;

    // NEW: Prediction time to compensate for system latency
    double prediction_dt_;
};

// --- Filter Implementations ---

class PoseFilterBase
{
public:
    virtual ~PoseFilterBase() = default;
    // Modified signature to accept prediction time
    virtual geometry_msgs::msg::PoseStamped filter(const geometry_msgs::msg::PoseStamped &pose_msg, rclcpp::Time current_time, double predict_dt) = 0;
};

class MovingAverageFilter : public PoseFilterBase
{
public:
    MovingAverageFilter(size_t window_size = 10) : window_size_(window_size) {}

    geometry_msgs::msg::PoseStamped filter(const geometry_msgs::msg::PoseStamped &pose_msg, rclcpp::Time, double) override
    {
        // Moving average doesn't support prediction easily, keeping as is
        position_window_.push_back(Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
        orientation_window_.push_back(Eigen::Quaterniond(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z));

        if (position_window_.size() > window_size_)
        {
            position_window_.pop_front();
            orientation_window_.pop_front();
        }

        Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
        for (const auto &pos : position_window_)
            avg_position += pos;
        avg_position /= position_window_.size();

        Eigen::Vector4d avg_orientation_vec = Eigen::Vector4d::Zero();
        for (const auto &q : orientation_window_)
        {
            if (orientation_window_.front().dot(q) < 0)
                avg_orientation_vec += Eigen::Vector4d(-q.w(), -q.x(), -q.y(), -q.z());
            else
                avg_orientation_vec += Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        }
        avg_orientation_vec.normalize();

        geometry_msgs::msg::PoseStamped filtered_pose;
        filtered_pose.header = pose_msg.header;
        filtered_pose.pose.position.x = avg_position.x();
        filtered_pose.pose.position.y = avg_position.y();
        filtered_pose.pose.position.z = avg_position.z();
        filtered_pose.pose.orientation.w = avg_orientation_vec[0];
        filtered_pose.pose.orientation.x = avg_orientation_vec[1];
        filtered_pose.pose.orientation.y = avg_orientation_vec[2];
        filtered_pose.pose.orientation.z = avg_orientation_vec[3];

        return filtered_pose;
    }

private:
    size_t window_size_;
    std::deque<Eigen::Vector3d> position_window_;
    std::deque<Eigen::Quaterniond> orientation_window_;
};

class PoseKalmanFilter : public PoseFilterBase
{
public:
    PoseKalmanFilter() : is_initialized_(false), last_time_(0.0)
    {
        // State: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        int dim_x = 10;
        int dim_z = 7;

        x_ = Eigen::VectorXd::Zero(dim_x);
        F_ = Eigen::MatrixXd::Identity(dim_x, dim_x);
        H_ = Eigen::MatrixXd::Zero(dim_z, dim_x);
        P_ = Eigen::MatrixXd::Identity(dim_x, dim_x) * 1.0;
        Q_ = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.01;
        R_ = Eigen::MatrixXd::Identity(dim_z, dim_z) * 0.1;

        // Measurement matrix H
        H_(0, 0) = 1;
        H_(1, 1) = 1;
        H_(2, 2) = 1; // Pos
        H_(3, 7) = 1;
        H_(4, 8) = 1;
        H_(5, 9) = 1;
        H_(6, 6) = 1; // Orient
    }

    geometry_msgs::msg::PoseStamped filter(const geometry_msgs::msg::PoseStamped &pose_msg, rclcpp::Time current_ros_time, double predict_dt) override
    {
        double current_time_sec = current_ros_time.seconds();

        if (!is_initialized_)
        {
            x_ << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                0, 0, 0,
                pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z;
            last_time_ = current_time_sec;
            is_initialized_ = true;
            return pose_msg;
        }

        double dt = current_time_sec - last_time_;
        if (dt <= 0)
            dt = 0.01;
        last_time_ = current_time_sec;

        // --- PREDICT STEP (Standard KF) ---
        F_(0, 3) = dt;
        F_(1, 4) = dt;
        F_(2, 5) = dt;
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // --- UPDATE STEP ---
        Eigen::VectorXd z(7);
        z << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w;

        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;

        // --- NEW: FUTURE PREDICTION (Latency Compensation) ---
        // We use the current estimated velocity to project the position forward
        // x_pred = x_curr + v_curr * predict_dt
        double pred_x = x_(0) + x_(3) * predict_dt;
        double pred_y = x_(1) + x_(4) * predict_dt;
        double pred_z = x_(2) + x_(5) * predict_dt;

        geometry_msgs::msg::PoseStamped filtered_pose;
        filtered_pose.header = pose_msg.header; // Keep original timestamp for metrics!
        filtered_pose.pose.position.x = pred_x;
        filtered_pose.pose.position.y = pred_y;
        filtered_pose.pose.position.z = pred_z;

        // Note: We do not predict orientation change here to avoid quaternion math instability
        // and because linear lag is the primary issue (20cm error).
        Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
        q.normalize();
        filtered_pose.pose.orientation.w = q.w();
        filtered_pose.pose.orientation.x = q.x();
        filtered_pose.pose.orientation.y = q.y();
        filtered_pose.pose.orientation.z = q.z();

        return filtered_pose;
    }

private:
    bool is_initialized_;
    double last_time_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd F_, H_, P_, Q_, R_;
};

// --- Node Implementation ---

PoseFilterNode::PoseFilterNode() : Node("pose_filter_node")
{
    this->declare_parameter<std::string>("input_topic", "/target_pose_raw");
    this->declare_parameter<std::string>("output_topic", "/target_pose");
    this->declare_parameter<std::string>("filter_type", "kalman");
    this->declare_parameter<int>("moving_avg.window_size", 10);

    // NEW: Latency Compensation Parameter
    // Default to 0.06s (60ms) to act as a conservative prediction
    this->declare_parameter<double>("prediction_time_s", 0.03);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string filter_type = this->get_parameter("filter_type").as_string();
    prediction_dt_ = this->get_parameter("prediction_time_s").as_double();

    RCLCPP_INFO(this->get_logger(), "Filter: %s | Prediction: %.3fs", filter_type.c_str(), prediction_dt_);

    if (filter_type == "moving_avg")
    {
        int window_size = this->get_parameter("moving_avg.window_size").as_int();
        filter_ = std::make_unique<MovingAverageFilter>(window_size);
    }
    else
    {
        filter_ = std::make_unique<PoseKalmanFilter>();
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        input_topic, qos, std::bind(&PoseFilterNode::pose_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);
}

void PoseFilterNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    auto filtered_pose = filter_->filter(*msg, this->get_clock()->now(), prediction_dt_);
    publisher_->publish(filtered_pose);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFilterNode>());
    rclcpp::shutdown();
    return 0;
}
