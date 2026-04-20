#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include "motor_command_12PubSubTypes.hpp"
#include "motor_state_12PubSubTypes.hpp"
#include "motor_command_16PubSubTypes.hpp"
#include "motor_state_16PubSubTypes.hpp"
#include "SubscriberApp.hpp"
#include "PublisherApp.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"

using namespace eprosima::fastdds;
using namespace eprosima::fastdds::dds;

namespace eprosima::fastdds
{
    // 基类，包含共同的成员变量和方法
    class JointBridgeNodeBase : public rclcpp::Node
    {
    public:
        JointBridgeNodeBase() : Node("joint_bridge_node")
        {
            // 声明并获取debug输出参数
            declare_parameter("debug_output", false);
            debug_output_ = get_parameter("debug_output").as_bool();
            declare_parameter("robot_controllers", "");

            ros2_control_yaml = get_parameter("robot_controllers").as_string();
            YAML::Node joint_name_config = YAML::LoadFile(ros2_control_yaml);

            // 检查并获取 `joints` 节点
            if (joint_name_config["leg_controller"] && joint_name_config["leg_controller"]["ros__parameters"] &&
                joint_name_config["leg_controller"]["ros__parameters"]["joints"]) {
                YAML::Node joints = joint_name_config["leg_controller"]["ros__parameters"]["joints"];
                for (const auto& joint : joints) {
                    joint_names_.push_back(joint.as<std::string>());
                }
            } else {
                std::cerr << "Error: 'joints' not found in YAML configuration." << std::endl;
            }

            initialize_joint_name_mapping();

            // 创建ROS2关节状态订阅者,订阅/joint_states话题
            joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&JointBridgeNodeBase::ros2_joint_state_callback, this, std::placeholders::_1));

            joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/leg_controller/commands", 10);

            YAML::Node config = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");

            if (!config["global_topic_domain"])
            {
                std::cout << "Warning: Missing required node: 'global_topic_domain' in YAML file, use default domain:0"
                    << std::endl;
                dds_qos_config_sub.cfg.domain = 0;
                dds_qos_config_pub.cfg.domain = 0;
            }
            else
            {
                dds_qos_config_sub.cfg.domain = config["global_topic_domain"].as<int>();
                dds_qos_config_pub.cfg.domain = config["global_topic_domain"].as<int>();
                std::cout << "INFO: set joint topic sub to domain: " << dds_qos_config_sub.cfg.domain << std::endl;
                std::cout << "INFO: set joint topic pub to domain: " << dds_qos_config_pub.cfg.domain << std::endl;
            }

            // 创建定时器,每1ms检查一次FastDDS的关节指令
            update_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1), std::bind(&JointBridgeNodeBase::process_dds_commands, this));
            dds_qos_config_sub.cfg.samples = 10;
            dds_qos_config_sub.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
            dds_qos_config_pub.cfg.samples = 10;
            dds_qos_config_pub.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
        }

        virtual ~JointBridgeNodeBase() = default;

        void initialize_joint_name_mapping()
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                joint_name_to_index_[joint_names_[i]] = i;
                RCLCPP_INFO(this->get_logger(), "Joint %s mapped to index %zu", joint_names_[i].c_str(), i);
            }
        }

        // 虚函数，子类需要实现
        virtual void init_fastdds() = 0;
        virtual void cleanup_fastdds() = 0;
        virtual void ros2_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) = 0;
        virtual void process_dds_commands() = 0;

    protected:
        std::vector<std::string> joint_names_;
        std::unordered_map<std::string, size_t> joint_name_to_index_;
        bool debug_output_;
        std::string ros2_control_yaml;
        CLIParser dds_qos_config_sub;
        CLIParser dds_qos_config_pub;

        // ROS2相关成员变量
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_publisher_;
        rclcpp::TimerBase::SharedPtr update_timer_;
    };

    // 12个电机的关节桥接节点类
    class JointBridgeNode12 : public JointBridgeNodeBase, 
                             public SubscriberApp<Motor_Command_12::motor_cmdPubSubType>,
                             public PublisherApp<Motor_State_12::motor_statePubSubType>
    {
    public:
        JointBridgeNode12(const std::string& robot_type) 
            : JointBridgeNodeBase(), robot_type_(robot_type)
        {
            cmd_topic_ = robot_type_ + "_motor_cmd_sim";
            state_topic_ = robot_type_ + "_motor_state_sim";
            init_fastdds();
        }

        ~JointBridgeNode12()
        {
            cleanup_fastdds();
        }

        void init_fastdds() override
        {
            this->SubscriberApp<Motor_Command_12::motor_cmdPubSubType>::init_fastdds_sub(dds_qos_config_sub.cfg, cmd_topic_);
            this->PublisherApp<Motor_State_12::motor_statePubSubType>::init_fastdds_pub(dds_qos_config_pub.cfg, state_topic_);
        }

        void cleanup_fastdds() override
        {
            this->PublisherApp<Motor_State_12::motor_statePubSubType>::stop_pub();
            this->SubscriberApp<Motor_Command_12::motor_cmdPubSubType>::stop_sub();
        }

        // 虚函数，子类可以重写订阅数据处理
        void on_data_available(dds::DataReader* reader) override
        {
            SampleInfo info;
            Motor_Command_12::motor_cmd data;
            if (reader->take_next_sample(&data, &info) == 0)
            {
                if (info.valid_data)
                {
                    received_joint_commands_ = data;
                }
            }
        }

        void ros2_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) override
        {
            for (int i = 0; i < msg->name.size(); i++)
            {
                dds_joint_states_.position()[joint_name_to_index_[msg->name[i]]] = msg->position[i];
                dds_joint_states_.speed()[joint_name_to_index_[msg->name[i]]] = msg->velocity[i];
                dds_joint_states_.isonline()[i] = true;
            }
            // 通过DDS发布关节状态
            this->PublisherApp<Motor_State_12::motor_statePubSubType>::writer_->write(&dds_joint_states_);
            if (debug_output_)
            {
                RCLCPP_INFO(this->get_logger(), "Joint states forwarded to FastDDS.");
            }
        }

        void process_dds_commands() override
        {
            // 创建Float64MultiArray消息
            auto command_msg = std_msgs::msg::Float64MultiArray();
            command_msg.data.resize(joint_names_.size(), 0.0);
            for (int i = 0; i < joint_names_.size(); i++)
            {
                command_msg.data[i] = received_joint_commands_.torque()[i];
            }

            // 发布ROS2话题
            joint_command_publisher_->publish(command_msg);

            if (debug_output_)
            {
                RCLCPP_INFO(this->get_logger(), "Published joint commands with indices mapped correctly.");
            }
        }

    private:
        Motor_Command_12::motor_cmd received_joint_commands_;
        Motor_State_12::motor_state dds_joint_states_;
        std::string cmd_topic_;
        std::string state_topic_;
        std::string robot_type_;
    };

    // 16个电机的关节桥接节点类
    class JointBridgeNode16 : public JointBridgeNodeBase, 
                             public SubscriberApp<Motor_Command_16::motor_cmdPubSubType>,
                             public PublisherApp<Motor_State_16::motor_statePubSubType>
    {
    public:
        JointBridgeNode16(const std::string& robot_type) 
            : JointBridgeNodeBase(), robot_type_(robot_type)
        {
            cmd_topic_ = robot_type_ + "_motor_cmd_sim";
            state_topic_ = robot_type_ + "_motor_state_sim";
            init_fastdds();
        }

        ~JointBridgeNode16()
        {
            cleanup_fastdds();
        }

        void init_fastdds() override
        {
            this->SubscriberApp<Motor_Command_16::motor_cmdPubSubType>::init_fastdds_sub(dds_qos_config_sub.cfg, cmd_topic_);
            this->PublisherApp<Motor_State_16::motor_statePubSubType>::init_fastdds_pub(dds_qos_config_pub.cfg, state_topic_);
        }

        void cleanup_fastdds() override
        {
            this->PublisherApp<Motor_State_16::motor_statePubSubType>::stop_pub();
            this->SubscriberApp<Motor_Command_16::motor_cmdPubSubType>::stop_sub();
        }

        // 虚函数，子类可以重写订阅数据处理
        void on_data_available(dds::DataReader* reader) override
        {
            SampleInfo info;
            Motor_Command_16::motor_cmd data;
            if (reader->take_next_sample(&data, &info) == 0)
            {
                if (info.valid_data)
                {
                    received_joint_commands_ = data;
                }
            }
        }

        void ros2_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) override
        {
            for (int i = 0; i < msg->name.size(); i++)
            {
                dds_joint_states_.position()[joint_name_to_index_[msg->name[i]]] = msg->position[i];
                dds_joint_states_.speed()[joint_name_to_index_[msg->name[i]]] = msg->velocity[i];
                dds_joint_states_.isonline()[i] = true;
            }
            // 通过DDS发布关节状态
            this->PublisherApp<Motor_State_16::motor_statePubSubType>::writer_->write(&dds_joint_states_);
            if (debug_output_)
            {
                RCLCPP_INFO(this->get_logger(), "Joint states forwarded to FastDDS.");
            }
        }

        void process_dds_commands() override
        {
            // 创建Float64MultiArray消息
            auto command_msg = std_msgs::msg::Float64MultiArray();
            command_msg.data.resize(joint_names_.size(), 0.0);
            for (int i = 0; i < joint_names_.size(); i++)
            {
                command_msg.data[i] = received_joint_commands_.torque()[i];
            }

            // 发布ROS2话题
            joint_command_publisher_->publish(command_msg);

            if (debug_output_)
            {
                RCLCPP_INFO(this->get_logger(), "Published joint commands with indices mapped correctly.");
            }
        }

    private:
        Motor_Command_16::motor_cmd received_joint_commands_;
        Motor_State_16::motor_state dds_joint_states_;
        std::string cmd_topic_;
        std::string state_topic_;
        std::string robot_type_;
    };

    // 工厂函数，根据机器人类型创建相应的节点
    std::shared_ptr<rclcpp::Node> create_joint_bridge_node(const std::string& robot_type)
    {
        if (robot_type == "w1") {
            std::cout<<"w1"<<"<Motor_Command_16>"<<"<Motor_State_16>"<<std::endl;
            return std::make_shared<JointBridgeNode16>(robot_type);
        }
        else if (robot_type == "w1_mini") {
            std::cout<<"w1_mini"<<"<Motor_Command_16>"<<"<Motor_State_16>"<<std::endl;
            return std::make_shared<JointBridgeNode16>(robot_type);
        }
        else if (robot_type == "lr_pro") {
            std::cout<<"lr_pro"<<"<Motor_Command_12>"<<"<Motor_State_12>"<<std::endl;
            return std::make_shared<JointBridgeNode12>(robot_type);
        }
        else if (robot_type == "lr") {
            std::cout<<"lr"<<"<Motor_Command_12>"<<"<Motor_State_12>"<<std::endl;
            return std::make_shared<JointBridgeNode12>(robot_type);
        }
        else {
            std::cout<<"[ERROR] robot_type: "<<robot_type<<" is not supported."<<std::endl;
            return std::make_shared<JointBridgeNode12>(robot_type);
        }
    }
}

// 主函数
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // 从命令行参数或环境变量获取机器人类型
    std::string robot_type = "lr_pro"; // 默认值
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "robot") {
            robot_type = argv[i + 1];
            break;
        }
    }
    
    // 创建相应的节点
    auto node = create_joint_bridge_node(robot_type);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
