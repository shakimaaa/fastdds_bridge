// elevation_mapping 话题桥接：订阅 grid_map_msgs/GridMap，转发到 DDS

#ifndef FASTDDS_ROS2_BRIDGE_ELEVMAP_HPP
#define FASTDDS_ROS2_BRIDGE_ELEVMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include "elevmap_topic_basePubSubTypes.hpp"
#include "PublisherApp.hpp"
#include <string>

namespace eprosima::fastdds {

class ElevMapForwarderNode : public rclcpp::Node, PublisherApp<elevmap_topic_base::ElevMapDataPubSubType>
{
public:
    ElevMapForwarderNode(const std::string& robot_type);
    ~ElevMapForwarderNode();

private:
    static constexpr size_t QUEUE_SIZE = 5;

    void grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    elevmap_topic_base::ElevMapData convert_ros2_to_dds(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    std::string robot_type_;
    std::string dds_topic_;
    std::string ros_topic_;
    bool debug_output_;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_subscription_;
    elevmap_topic_base::ElevMapData dds_elevmap_data_;
    CLIParser dds_qos_config_;
};

} // namespace eprosima::fastdds

#endif // FASTDDS_ROS2_BRIDGE_ELEVMAP_HPP
