// elevation_mapping 车体点云桥接（ROS 父进程 + DDS worker 子进程）
#include <ament_index_cpp/get_package_prefix.hpp>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
struct Header
{
    uint8_t tag;
    uint64_t timestamp_ns;
    uint8_t is_dense;
    uint32_t frame_len;
    uint32_t layer_len;
    uint32_t n_points;
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

class ElevMapBodyCloudForwarderNode : public rclcpp::Node
{
public:
    ElevMapBodyCloudForwarderNode(const std::string& robot_type, int tx_fd)
        : Node("elevmap_body_cloud_forwarder_node")
        , robot_type_(robot_type)
        , tx_fd_(tx_fd)
    {
        declare_parameter("debug_output", false);
        declare_parameter("ros_topic", "/elevation_mapping/body_elevation_cloud");
        declare_parameter("layer_name", "inpaint");
        debug_output_ = get_parameter("debug_output").as_bool();
        ros_topic_ = get_parameter("ros_topic").as_string();
        layer_name_ = get_parameter("layer_name").as_string();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            ros_topic_, 5,
            std::bind(&ElevMapBodyCloudForwarderNode::pointcloud_callback, this, std::placeholders::_1));
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        std::vector<float> xyz;
        xyz.reserve(n * 3u);
        for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            xyz.push_back(*iter_x);
            xyz.push_back(*iter_y);
            xyz.push_back(*iter_z);
        }

        Header h{};
        h.tag = 'B';
        h.timestamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
                         static_cast<uint64_t>(msg->header.stamp.nanosec);
        h.is_dense = msg->is_dense ? 1u : 0u;
        h.frame_len = static_cast<uint32_t>(msg->header.frame_id.size());
        h.layer_len = static_cast<uint32_t>(layer_name_.size());
        h.n_points = static_cast<uint32_t>(n);

        (void)write_all(tx_fd_, &h, sizeof(h));
        (void)write_all(tx_fd_, msg->header.frame_id.data(), msg->header.frame_id.size());
        (void)write_all(tx_fd_, layer_name_.data(), layer_name_.size());
        if (!xyz.empty())
        {
            (void)write_all(tx_fd_, xyz.data(), xyz.size() * sizeof(float));
        }
        if (debug_output_)
        {
            RCLCPP_INFO(get_logger(), "Body cloud relayed to DDS worker (%u points)", h.n_points);
        }
    }

    std::string robot_type_;
    std::string ros_topic_;
    std::string layer_name_;
    bool debug_output_{false};
    int tx_fd_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
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
                               "/lib/fastdds_ros2_bridge/fastdds_ros2_bridge_elevmap_body_cloud_worker";
    const pid_t pid = ::fork();
    if (pid == 0)
    {
        ::close(pipefd[1]);
        char fd_s[16];
        std::snprintf(fd_s, sizeof(fd_s), "%d", pipefd[0]);
        ::execl(worker.c_str(), "fastdds_ros2_bridge_elevmap_body_cloud_worker",
                "--robot", robot_type.c_str(), "--fd", fd_s, reinterpret_cast<char*>(0));
        _exit(127);
    }
    ::close(pipefd[0]);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElevMapBodyCloudForwarderNode>(robot_type, pipefd[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
