#include <ament_index_cpp/get_package_prefix.hpp>
#include <algorithm>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <string>
#include <unistd.h>

namespace
{
struct Header
{
    uint8_t tag;
    uint64_t timestamp;
    float resolution, length_x, length_y;
    uint32_t rows, cols;
    uint32_t frame_len;
    uint32_t basic_layers_count;
    uint32_t layers_count;
};

struct LayerHead
{
    uint32_t name_len;
    uint32_t data_count;
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

class ElevMapForwarderNode : public rclcpp::Node
{
public:
    ElevMapForwarderNode(int tx_fd) : Node("elevmap_forwarder_node"), tx_fd_(tx_fd)
    {
        declare_parameter("debug_output", false);
        declare_parameter("ros_topic", "/elevation_mapping/filtered_elevation_map");
        debug_output_ = get_parameter("debug_output").as_bool();
        ros_topic_ = get_parameter("ros_topic").as_string();
        sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
            ros_topic_, 5, std::bind(&ElevMapForwarderNode::cb, this, std::placeholders::_1));
    }

private:
    void cb(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        Header h{};
        h.tag = 'E';
        h.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL + msg->header.stamp.nanosec;
        h.resolution = msg->info.resolution;
        h.length_x = msg->info.length_x;
        h.length_y = msg->info.length_y;
        h.rows = static_cast<uint32_t>(msg->info.length_x / msg->info.resolution);
        h.cols = static_cast<uint32_t>(msg->info.length_y / msg->info.resolution);
        h.frame_len = static_cast<uint32_t>(msg->header.frame_id.size());
        h.basic_layers_count = static_cast<uint32_t>(msg->layers.size());
        h.layers_count = static_cast<uint32_t>(std::min(msg->data.size(), msg->layers.size()));

        (void)write_all(tx_fd_, &h, sizeof(h));
        (void)write_all(tx_fd_, msg->header.frame_id.data(), msg->header.frame_id.size());
        for (const auto& s : msg->layers)
        {
            uint32_t n = static_cast<uint32_t>(s.size());
            (void)write_all(tx_fd_, &n, sizeof(n));
            if (n) (void)write_all(tx_fd_, s.data(), n);
        }
        for (size_t i = 0; i < h.layers_count; ++i)
        {
            LayerHead lh{};
            lh.name_len = static_cast<uint32_t>(msg->layers[i].size());
            lh.data_count = static_cast<uint32_t>(msg->data[i].data.size());
            (void)write_all(tx_fd_, &lh, sizeof(lh));
            if (lh.name_len) (void)write_all(tx_fd_, msg->layers[i].data(), lh.name_len);
            if (lh.data_count) (void)write_all(tx_fd_, msg->data[i].data.data(), lh.data_count * sizeof(float));
        }
        if (debug_output_) RCLCPP_INFO(get_logger(), "ElevMap relayed to DDS worker");
    }

    int tx_fd_;
    bool debug_output_{false};
    std::string ros_topic_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
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
                               "/lib/fastdds_ros2_bridge/fastdds_ros2_bridge_elevmap_worker";
    const pid_t pid = ::fork();
    if (pid == 0)
    {
        ::close(pipefd[1]);
        char fd_s[16];
        std::snprintf(fd_s, sizeof(fd_s), "%d", pipefd[0]);
        ::execl(worker.c_str(), "fastdds_ros2_bridge_elevmap_worker",
                "--robot", robot_type.c_str(), "--fd", fd_s, reinterpret_cast<char*>(0));
        _exit(127);
    }

    ::close(pipefd[0]);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElevMapForwarderNode>(pipefd[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
