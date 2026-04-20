#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <unistd.h>

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "PublisherApp.hpp"
#include "odom_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"

using namespace eprosima::fastdds;

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
} // namespace

namespace eprosima::fastdds
{
class OdomDdsWorker : public PublisherApp<odom_topic_base::OdomDataPubSubType>
{
public:
    OdomDdsWorker(int in_fd, const std::string& topic)
        : in_fd_(in_fd)
    {
        CLIParser dds_qos_config;
        YAML::Node config = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");
        dds_qos_config.cfg.domain =
                config["global_topic_domain"] ? config["global_topic_domain"].as<int>() : 0;
        dds_qos_config.cfg.samples = 5;
        dds_qos_config.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
        init_fastdds_pub(dds_qos_config.cfg, topic);
        std::fprintf(stderr, "odom worker started, topic=%s domain=%u\n", topic.c_str(), dds_qos_config.cfg.domain);
    }

    void run()
    {
        OdomPacket p{};
        while (read_exact(in_fd_, &p, sizeof(p)))
        {
            if (p.tag != 'O')
            {
                continue;
            }
            odom_topic_base::OdomData out;
            out.frame_id(std::string(p.frame_id));
            out.child_frame_id(std::string(p.child_frame_id));
            out.timestamp(p.timestamp);
            out.position_x(p.position_x);
            out.position_y(p.position_y);
            out.position_z(p.position_z);
            out.orientation_x(p.orientation_x);
            out.orientation_y(p.orientation_y);
            out.orientation_z(p.orientation_z);
            out.orientation_w(p.orientation_w);
            out.linear_x(p.linear_x);
            out.linear_y(p.linear_y);
            out.linear_z(p.linear_z);
            out.angular_x(p.angular_x);
            out.angular_y(p.angular_y);
            out.angular_z(p.angular_z);
            for (size_t i = 0; i < 36; ++i)
            {
                out.pose_covariance().push_back(p.pose_covariance[i]);
                out.twist_covariance().push_back(p.twist_covariance[i]);
            }
            writer_->write(&out);
        }
    }

private:
    int in_fd_;
};
} // namespace eprosima::fastdds

int main(int argc, char** argv)
{
    std::string robot_type = "lr_pro";
    int in_fd = -1;
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "--robot")
        {
            robot_type = argv[i + 1];
        }
        else if (std::string(argv[i]) == "--fd")
        {
            in_fd = std::atoi(argv[i + 1]);
        }
    }
    if (in_fd < 0)
    {
        return 1;
    }

    eprosima::fastdds::OdomDdsWorker worker(in_fd, robot_type + "_odom");
    worker.run();
    return 0;
}

