#include <cstdlib>
#include <cstdint>
#include <string>
#include <unistd.h>

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "PublisherApp.hpp"
#include "imu_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"

using namespace eprosima::fastdds;

namespace
{
struct ImuPacket
{
    uint8_t tag;
    float ox, oy, oz, ow;
    float avx, avy, avz;
    float lax, lay, laz;
};

bool read_exact(int fd, void* buf, size_t n)
{
    char* p = static_cast<char*>(buf);
    while (n > 0)
    {
        const ssize_t r = ::read(fd, p, n);
        if (r <= 0) return false;
        p += static_cast<size_t>(r);
        n -= static_cast<size_t>(r);
    }
    return true;
}
} // namespace

namespace eprosima::fastdds
{
class ImuWorker : public PublisherApp<imu_topic_base::IMUDataPubSubType>
{
public:
    ImuWorker(int in_fd, const std::string& topic) : in_fd_(in_fd)
    {
        CLIParser qos;
        YAML::Node cfg = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");
        qos.cfg.domain = cfg["global_topic_domain"] ? cfg["global_topic_domain"].as<int>() : 0;
        qos.cfg.samples = 10;
        qos.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
        init_fastdds_pub(qos.cfg, topic);
    }

    void run()
    {
        ImuPacket p{};
        while (read_exact(in_fd_, &p, sizeof(p)))
        {
            if (p.tag != 'U') continue;
            imu_topic_base::IMUData out;
            out.orientation_x() = p.ox;
            out.orientation_y() = p.oy;
            out.orientation_z() = p.oz;
            out.orientation_w() = p.ow;
            out.angular_velocity_x() = p.avx;
            out.angular_velocity_y() = p.avy;
            out.angular_velocity_z() = p.avz;
            out.linear_acceleration_x() = p.lax;
            out.linear_acceleration_y() = p.lay;
            out.linear_acceleration_z() = p.laz;
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
        if (std::string(argv[i]) == "--robot") robot_type = argv[i + 1];
        else if (std::string(argv[i]) == "--fd") in_fd = std::atoi(argv[i + 1]);
    }
    if (in_fd < 0) return 1;

    eprosima::fastdds::ImuWorker w(in_fd, robot_type + "_imu_sim");
    w.run();
    return 0;
}

