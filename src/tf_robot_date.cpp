#include <ament_index_cpp/get_package_prefix.hpp>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "tf_robot_date_ipc.hpp"

namespace
{

bool read_exact(int fd, void* buf, size_t n)
{
    char* p = static_cast<char*>(buf);
    while (n > 0)
    {
        const ssize_t r = ::read(fd, p, n);
        if (r <= 0)
        {
            return false;
        }
        p += static_cast<size_t>(r);
        n -= static_cast<size_t>(r);
    }
    return true;
}

std::string dds_worker_executable_path()
{
    const std::string prefix = ament_index_cpp::get_package_prefix("fastdds_ros2_bridge");
    return prefix + "/lib/fastdds_ros2_bridge/tf_robot_date_dds_worker";
}

} // namespace

class TfRobotDateRosBridge final : public rclcpp::Node
{
public:
    explicit TfRobotDateRosBridge(int dds_rx_fd)
        : Node("fastdds_ros2_joint_bridge")
        , dds_rx_fd_(dds_rx_fd)
    {
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        can_id_to_motor_name_map_ = {
            {0x4, "FL_hip_joint"},
            {0x5, "FL_thigh_joint"},
            {0x6, "FL_calf_joint"},
            {0x1, "FR_hip_joint"},
            {0x2, "FR_thigh_joint"},
            {0x3, "FR_calf_joint"},
            {0xa, "RL_hip_joint"},
            {0xb, "RL_thigh_joint"},
            {0xc, "RL_calf_joint"},
            {0x7, "RR_hip_joint"},
            {0x8, "RR_thigh_joint"},
            {0x9, "RR_calf_joint"},
        };

        io_thread_ = std::thread([this] {
            io_loop();
        });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TfRobotDateRosBridge::publish_odom_transform, this));

        RCLCPP_INFO(this->get_logger(),
            "ROS 桥已启动；机器人侧 Fast-DDS 在子进程 tf_robot_date_dds_worker（与 rmw_fastrtps 不同进程，避免符号冲突）");
    }

    ~TfRobotDateRosBridge() override
    {
        running_ = false;
        if (dds_rx_fd_ >= 0)
        {
            ::close(dds_rx_fd_);
        }
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
    }

private:
    int dds_rx_fd_;
    std::thread io_thread_;
    std::atomic<bool> running_{true};

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    std::map<int, std::string> can_id_to_motor_name_map_;

    sensor_msgs::msg::Imu latest_imu_msg_;
    std::mutex imu_mu_;
    bool has_imu_data_ = false;

    void io_loop()
    {
        while (running_)
        {
            uint8_t tag = 0;
            if (!read_exact(dds_rx_fd_, &tag, sizeof tag))
            {
                break;
            }
            if (tag == tf_robot_date_ipc::kTagMotor)
            {
                tf_robot_date_ipc::MotorPacket p{};
                p.tag = tag;
                if (!read_exact(dds_rx_fd_, reinterpret_cast<char*>(&p) + 1, sizeof p - 1))
                {
                    break;
                }
                publish_joint_states(p);
            }
            else if (tag == tf_robot_date_ipc::kTagImu)
            {
                tf_robot_date_ipc::ImuPacket p{};
                p.tag = tag;
                if (!read_exact(dds_rx_fd_, reinterpret_cast<char*>(&p) + 1, sizeof p - 1))
                {
                    break;
                }
                std::lock_guard<std::mutex> lock(imu_mu_);
                latest_imu_msg_.orientation.set__w(p.qw);
                latest_imu_msg_.orientation.set__x(p.qx);
                latest_imu_msg_.orientation.set__y(p.qy);
                latest_imu_msg_.orientation.set__z(p.qz);
                has_imu_data_ = true;
            }
        }
        if (rclcpp::ok())
        {
            RCLCPP_WARN(this->get_logger(), "与 DDS 子进程的管道已关闭或出错");
        }
    }

    void publish_joint_states(const tf_robot_date_ipc::MotorPacket& p)
    {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();

        for (int i = 0; i < 12; ++i)
        {
            const auto it = can_id_to_motor_name_map_.find(static_cast<int>(p.motor_id[i]));
            if (it != can_id_to_motor_name_map_.end())
            {
                joint_state_msg.name.push_back(it->second);
            }
            else
            {
                joint_state_msg.name.push_back("unknown_joint");
            }
            joint_state_msg.position.push_back(p.position[i]);
            joint_state_msg.velocity.push_back(p.velocity[i]);
            joint_state_msg.effort.push_back(p.effort[i]);
        }
        joint_state_publisher_->publish(joint_state_msg);
    }

    void publish_odom_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        {
            std::lock_guard<std::mutex> lock(imu_mu_);
            if (has_imu_data_)
            {
                transform_stamped.transform.rotation = latest_imu_msg_.orientation;
            }
            else
            {
                transform_stamped.transform.rotation.w = 1.0;
            }
        }

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        tf_broadcaster_->sendTransform(transform_stamped);
    }
};

int main(int argc, char** argv)
{
    std::signal(SIGCHLD, SIG_IGN);

    int pipefd[2];
    if (::pipe(pipefd) != 0)
    {
        std::perror("pipe");
        return 1;
    }

    const std::string worker_exe = dds_worker_executable_path();
    const pid_t pid = ::fork();
    if (pid < 0)
    {
        std::perror("fork");
        return 1;
    }

    if (pid == 0)
    {
        ::close(pipefd[0]);
        char fd_arg[32];
        std::snprintf(fd_arg, sizeof fd_arg, "%d", pipefd[1]);
        ::execl(worker_exe.c_str(), "tf_robot_date_dds_worker", fd_arg, reinterpret_cast<char*>(0));
        std::perror("execl tf_robot_date_dds_worker");
        _exit(127);
    }

    ::close(pipefd[1]);
    const int dds_rx = pipefd[0];

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfRobotDateRosBridge>(dds_rx);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
