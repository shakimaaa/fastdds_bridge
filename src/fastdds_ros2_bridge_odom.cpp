// Odometry 话题桥接（ROS 父进程 + DDS worker 子进程）
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>

namespace
{
struct OdomPacket
{
    uint8_t tag;
    uint64_t timestamp;
    double position_x;
    double position_y;
    double position_z;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
    double pose_covariance[36];
    double twist_covariance[36];
    char frame_id[64];
    char child_frame_id[64];
};

bool write_all(int fd, const void* buf, size_t n)
{
    const char* p = static_cast<const char*>(buf);
    while (n > 0)
    {
        const ssize_t w = ::write(fd, p, n);
        if (w <= 0)
        {
            return false;
        }
        p += static_cast<size_t>(w);
        n -= static_cast<size_t>(w);
    }
    return true;
}
} // namespace

class OdomForwarderNode : public rclcpp::Node
{
public:
    OdomForwarderNode(const std::string& robot_type, int tx_fd)
        : Node("odom_forwarder_node")
        , tx_fd_(tx_fd)
        , robot_type_(robot_type)
    {
        declare_parameter("debug_output", false);
        declare_parameter("ros_topic", "/Odometry");
        debug_output_ = get_parameter("debug_output").as_bool();
        ros_topic_ = get_parameter("ros_topic").as_string();

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ros_topic_, 5, std::bind(&OdomForwarderNode::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        OdomPacket p{};
        p.tag = 'O';
        p.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
        p.position_x = msg->pose.pose.position.x;
        p.position_y = msg->pose.pose.position.y;
        p.position_z = msg->pose.pose.position.z;
        p.orientation_x = msg->pose.pose.orientation.x;
        p.orientation_y = msg->pose.pose.orientation.y;
        p.orientation_z = msg->pose.pose.orientation.z;
        p.orientation_w = msg->pose.pose.orientation.w;
        p.linear_x = msg->twist.twist.linear.x;
        p.linear_y = msg->twist.twist.linear.y;
        p.linear_z = msg->twist.twist.linear.z;
        p.angular_x = msg->twist.twist.angular.x;
        p.angular_y = msg->twist.twist.angular.y;
        p.angular_z = msg->twist.twist.angular.z;
        for (size_t i = 0; i < 36; ++i)
        {
            p.pose_covariance[i] = msg->pose.covariance[i];
            p.twist_covariance[i] = msg->twist.covariance[i];
        }
        std::snprintf(p.frame_id, sizeof(p.frame_id), "%s", msg->header.frame_id.c_str());
        std::snprintf(p.child_frame_id, sizeof(p.child_frame_id), "%s", msg->child_frame_id.c_str());
        (void)write_all(tx_fd_, &p, sizeof(p));
        if (debug_output_)
        {
            RCLCPP_INFO(get_logger(), "Odometry relayed to DDS worker");
        }
    }

    int tx_fd_;
    std::string robot_type_;
    std::string ros_topic_;
    bool debug_output_{false};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char* argv[])
{
    std::string robot_type = "lr_pro";
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "robot")
        {
            robot_type = argv[i + 1];
            break;
        }
    }

    int pipefd[2];
    if (::pipe(pipefd) != 0)
    {
        return 1;
    }

    const std::string worker = ament_index_cpp::get_package_prefix("fastdds_ros2_bridge") +
                               "/lib/fastdds_ros2_bridge/fastdds_ros2_bridge_odom_worker";
    const pid_t pid = ::fork();
    if (pid == 0)
    {
        ::close(pipefd[1]);
        char fd_s[16];
        std::snprintf(fd_s, sizeof(fd_s), "%d", pipefd[0]);
        ::execl(worker.c_str(), "fastdds_ros2_bridge_odom_worker",
                "--robot", robot_type.c_str(), "--fd", fd_s, reinterpret_cast<char*>(0));
        _exit(127);
    }

    ::close(pipefd[0]);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomForwarderNode>(robot_type, pipefd[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
