/**
 * 仅链接 /workspace/src/env/fast_dds/local 中的 Fast-DDS 3 + fastcdr2，不与 ROS 的 libfastrtps 同进程。
 * 由 tf_robot_date_real fork+exec 启动，通过管道向父进程发送关节与 IMU 数据。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include "SubscriberApp.hpp"
#include "motor_state_12PubSubTypes.hpp"
#include "imu_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"
#include "tf_robot_date_ipc.hpp"

using SubscriberAppBase_motor = eprosima::fastdds::SubscriberApp<Motor_State_12::motor_statePubSubType>;
using SubscriberAppBase_imu = eprosima::fastdds::SubscriberApp<imu_topic_base::IMUDataPubSubType>;

namespace
{

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

class TfRobotDateDdsWorker final
    : public SubscriberAppBase_motor
    , public SubscriberAppBase_imu
{
public:
    explicit TfRobotDateDdsWorker(int out_fd, std::string robot_type)
        : out_fd_(out_fd)
        , robot_type_(std::move(robot_type))
    {
    }

    void init_dds()
    {
        eprosima::fastdds::CLIParser dds_qos_config;
        dds_qos_config.cfg.samples = 10;
        dds_qos_config.cfg.delivery_mechanism =
                eprosima::fastdds::CLIParser::DeliveryMechanismKind::UDPv4;

        YAML::Node config = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");

        if (!config["global_topic_domain"])
        {
            std::fprintf(stderr,
                "Warning: Missing 'global_topic_domain' in YAML, use domain 0\n");
            dds_qos_config.cfg.domain = 0;
        }
        else
        {
            dds_qos_config.cfg.domain = config["global_topic_domain"].as<int>();
            std::fprintf(stderr, "INFO: DDS domain %d\n", dds_qos_config.cfg.domain);
        }

        SubscriberAppBase_motor::init_fastdds_sub(dds_qos_config.cfg, robot_type_ + "_motor_state");
        SubscriberAppBase_imu::init_fastdds_sub(dds_qos_config.cfg, robot_type_ + "_imu");
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override
    {
        if (reader == SubscriberAppBase_motor::reader_)
        {
            process_motor(reader);
        }
        else if (reader == SubscriberAppBase_imu::reader_)
        {
            process_imu(reader);
        }
    }

private:
    int out_fd_;
    std::string robot_type_;

    void process_motor(eprosima::fastdds::dds::DataReader* reader)
    {
        FASTDDS_CONST_SEQUENCE(DataSeq, Motor_State_12::motor_state);

        DataSeq delivery_mechanisms_sequence;
        eprosima::fastdds::dds::SampleInfoSeq info_sequence;

        Motor_State_12::motor_state last_valid_data;
        bool has_valid_data = false;

        while ((!SubscriberAppBase_motor::is_stopped()) &&
            (eprosima::fastdds::dds::RETCODE_OK ==
                reader->take(delivery_mechanisms_sequence, info_sequence)))
        {
            for (eprosima::fastdds::dds::LoanableCollection::size_type i = info_sequence.length(); i > 0; --i)
            {
                const size_t index = i - 1;
                if ((info_sequence[index].instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) &&
                    info_sequence[index].valid_data)
                {
                    last_valid_data = delivery_mechanisms_sequence[index];
                    has_valid_data = true;
                    break;
                }
            }
            reader->return_loan(delivery_mechanisms_sequence, info_sequence);
        }

        if (!has_valid_data)
        {
            return;
        }

        tf_robot_date_ipc::MotorPacket p{};
        p.tag = tf_robot_date_ipc::kTagMotor;
        for (int i = 0; i < 12; ++i)
        {
            p.motor_id[i] = last_valid_data.motorId()[i];
            p.position[i] = last_valid_data.position()[i];
            p.velocity[i] = last_valid_data.speed()[i];
            p.effort[i] = last_valid_data.current()[i];
        }
        (void)write_all(out_fd_, &p, sizeof p);
    }

    void process_imu(eprosima::fastdds::dds::DataReader* reader)
    {
        FASTDDS_CONST_SEQUENCE(DataSeq, imu_topic_base::IMUData);

        DataSeq delivery_mechanisms_sequence;
        eprosima::fastdds::dds::SampleInfoSeq info_sequence;

        imu_topic_base::IMUData last_valid_data;
        bool has_valid_data = false;

        while ((!SubscriberAppBase_motor::is_stopped()) &&
            (eprosima::fastdds::dds::RETCODE_OK ==
                reader->take(delivery_mechanisms_sequence, info_sequence)))
        {
            for (eprosima::fastdds::dds::LoanableCollection::size_type i = info_sequence.length(); i > 0; --i)
            {
                const size_t index = i - 1;
                if ((info_sequence[index].instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) &&
                    info_sequence[index].valid_data)
                {
                    last_valid_data = delivery_mechanisms_sequence[index];
                    has_valid_data = true;
                    break;
                }
            }
            reader->return_loan(delivery_mechanisms_sequence, info_sequence);
        }

        if (!has_valid_data)
        {
            return;
        }

        tf_robot_date_ipc::ImuPacket p{};
        p.tag = tf_robot_date_ipc::kTagImu;
        p.qw = last_valid_data.orientation_w();
        p.qx = last_valid_data.orientation_x();
        p.qy = last_valid_data.orientation_y();
        p.qz = last_valid_data.orientation_z();
        p.avx = last_valid_data.angular_velocity_x();
        p.avy = last_valid_data.angular_velocity_y();
        p.avz = last_valid_data.angular_velocity_z();
        p.lax = last_valid_data.linear_acceleration_x();
        p.lay = last_valid_data.linear_acceleration_y();
        p.laz = last_valid_data.linear_acceleration_z();
        (void)write_all(out_fd_, &p, sizeof p);
    }
};

int main(int argc, char** argv)
{
    std::string robot_type = "lr_pro";
    int out_fd = -1;
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "--robot")
        {
            robot_type = argv[i + 1];
        }
        else if (std::string(argv[i]) == "--fd")
        {
            out_fd = std::atoi(argv[i + 1]);
        }
    }
    if (out_fd < 0)
    {
        std::fprintf(stderr, "usage: %s --robot <name> --fd <write_fd>\n", argv[0]);
        return 1;
    }

    TfRobotDateDdsWorker worker(out_fd, robot_type);
    worker.init_dds();
    std::fprintf(stderr, "tf_robot_date_dds_worker: DDS ok, blocking\n");
    for (;;)
    {
        pause();
    }
}
