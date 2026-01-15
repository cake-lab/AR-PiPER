#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <memory>
#include <string>

class PiperEePosePublisher : public rclcpp::Node
{
public:
    PiperEePosePublisher() : Node("piper_ee_pose_publisher")
    {
        // Declare parameters with default values from the plan
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("ee_link", "gripper_base");
        this->declare_parameter<double>("rate_hz", 100.0);

        base_frame_ = this->get_parameter("base_frame").as_string();
        ee_link_ = this->get_parameter("ee_link").as_string();
        double rate_hz = this->get_parameter("rate_hz").as_double();

        RCLCPP_INFO(this->get_logger(), "Starting EE Pose Publisher:");
        RCLCPP_INFO(this->get_logger(), "  Base Frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  EE Link: %s", ee_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Rate (Hz): %.1f", rate_hz);

        // Initialize TF listener and buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Create the publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/piper/ee_pose", 10);

        // Create a timer to publish at the specified rate
        auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(timer_period, std::bind(&PiperEePosePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            // Lookup the transform from base_frame to ee_link
            // We use tf2::TimePointZero to get the latest available transform
            transform = tf_buffer_->lookupTransform(base_frame_, ee_link_, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            // Use throttle to avoid spamming warnings
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000, // Throttle duration in ms (1 second)
                "Could not lookup transform from '%s' to '%s': %s",
                base_frame_.c_str(), ee_link_.c_str(), ex.what());
            return;
        }

        // Create and populate the PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg;

        // Per the plan: header.stamp = TF stamp, header.frame_id = base_frame
        pose_msg.header.stamp = transform.header.stamp;
        pose_msg.header.frame_id = base_frame_; // The pose is expressed in the base_frame

        // Copy the transform data directly to the pose
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;
        pose_msg.pose.orientation = transform.transform.rotation;

        publisher_->publish(pose_msg);
    }

    // Parameters
    std::string base_frame_;
    std::string ee_link_;

    // ROS 2 Infrastructure
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PiperEePosePublisher>());
    rclcpp::shutdown();
    return 0;
}
