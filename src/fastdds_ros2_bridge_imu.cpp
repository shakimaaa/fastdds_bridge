#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <unistd.h>

namespace
{
struct ImuPacket
{
    uint8_t tag;
    float ox, oy, oz, ow;
    float avx, avy, avz;
    float lax, lay, laz;
};

bool write_all(int fd, const void* buf, size_t n)
{
    const char* p = static_cast<const char*>(buf);
    while (n > 0)
    {
        const ssize_t w = ::write(fd, p, n);
        if (w <= 0) return false;
        p += static_cast<size_t>(w);
        n -= static_cast<size_t>(w);
    }
    return true;
}
} // namespace

class IMUForwarderNode : public rclcpp::Node
{
public:
    IMUForwarderNode(int tx_fd) : Node("imu_forwarder_node"), tx_fd_(tx_fd)
    {
        declare_parameter("debug_output", false);
        debug_output_ = get_parameter("debug_output").as_bool();
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_tf/imu", 10, std::bind(&IMUForwarderNode::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        ImuPacket p{};
        p.tag = 'U';
        p.ox = msg->orientation.x; p.oy = msg->orientation.y; p.oz = msg->orientation.z; p.ow = msg->orientation.w;
        p.avx = msg->angular_velocity.x; p.avy = msg->angular_velocity.y; p.avz = msg->angular_velocity.z;
        p.lax = msg->linear_acceleration.x; p.lay = msg->linear_acceleration.y; p.laz = msg->linear_acceleration.z;
        (void)write_all(tx_fd_, &p, sizeof(p));
        if (debug_output_) RCLCPP_INFO(get_logger(), "IMU relayed to DDS worker");
    }

    int tx_fd_;
    bool debug_output_{false};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char* argv[])
{
    std::string robot_type = "lr_pro";
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "robot") { robot_type = argv[i + 1]; break; }
    }

    int pipefd[2];
    if (::pipe(pipefd) != 0) return 1;
    const std::string worker = ament_index_cpp::get_package_prefix("fastdds_ros2_bridge") +
                               "/lib/fastdds_ros2_bridge/fastdds_ros2_bridge_imu_worker";
    const pid_t pid = ::fork();
    if (pid == 0)
    {
        ::close(pipefd[1]);
        char fd_s[16];
        std::snprintf(fd_s, sizeof(fd_s), "%d", pipefd[0]);
        ::execl(worker.c_str(), "fastdds_ros2_bridge_imu_worker",
                "--robot", robot_type.c_str(), "--fd", fd_s, reinterpret_cast<char*>(0));
        _exit(127);
    }

    ::close(pipefd[0]);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUForwarderNode>(pipefd[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
