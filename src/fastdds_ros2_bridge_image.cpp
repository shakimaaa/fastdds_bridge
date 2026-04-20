//
// Created by Taseny on 25-7-15.
//

#include <ament_index_cpp/get_package_prefix.hpp>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unistd.h>

namespace
{
struct Header
{
    uint8_t tag;
    uint32_t width, height, step, data_size;
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

class ImageBridgeNode : public rclcpp::Node
{
public:
    explicit ImageBridgeNode(int tx_fd) : Node("image_bridge_node"), tx_fd_(tx_fd)
    {
        declare_parameter("debug_output", false);
        debug_output_ = get_parameter("debug_output").as_bool();
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d435i", 5, std::bind(&ImageBridgeNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        Header h{};
        h.tag = 'G';
        h.width = msg->width;
        h.height = msg->height;
        h.step = msg->step;
        h.data_size = static_cast<uint32_t>(msg->data.size());
        (void)write_all(tx_fd_, &h, sizeof(h));
        if (!msg->data.empty()) (void)write_all(tx_fd_, msg->data.data(), msg->data.size());
        if (debug_output_) RCLCPP_INFO(get_logger(), "Image relayed to DDS worker.");
    }

    int tx_fd_;
    bool debug_output_{false};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
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
                               "/lib/fastdds_ros2_bridge/fastdds_ros2_bridge_image_worker";
    const pid_t pid = ::fork();
    if (pid == 0)
    {
        ::close(pipefd[1]);
        char fd_s[16];
        std::snprintf(fd_s, sizeof(fd_s), "%d", pipefd[0]);
        ::execl(worker.c_str(), "fastdds_ros2_bridge_image_worker",
                "--robot", robot_type.c_str(), "--fd", fd_s, reinterpret_cast<char*>(0));
        _exit(127);
    }

    ::close(pipefd[0]);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageBridgeNode>(pipefd[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
