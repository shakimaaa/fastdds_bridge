#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <unistd.h>

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "PublisherApp.hpp"
#include "config.h"
#include "elevmap_point_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"

using namespace eprosima::fastdds;

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
class ElevBodyCloudWorker : public PublisherApp<elevmap_point_topic_base::ElevMapBodyPointCloudPubSubType>
{
public:
    ElevBodyCloudWorker(int in_fd, const std::string& topic)
        : in_fd_(in_fd)
    {
        CLIParser dds_qos_config;
        YAML::Node config = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");
        dds_qos_config.cfg.domain =
                config["global_topic_domain"] ? config["global_topic_domain"].as<int>() : 0;
        dds_qos_config.cfg.samples = 5;
        dds_qos_config.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
        init_fastdds_pub(dds_qos_config.cfg, topic);
    }

    void run()
    {
        Header h{};
        while (read_exact(in_fd_, &h, sizeof(h)))
        {
            if (h.tag != 'B')
            {
                continue;
            }

            std::string frame(h.frame_len, '\0');
            std::string layer(h.layer_len, '\0');
            if ((!frame.empty() && !read_exact(in_fd_, &frame[0], frame.size())) ||
                (!layer.empty() && !read_exact(in_fd_, &layer[0], layer.size())))
            {
                break;
            }

            std::vector<float> xyz(static_cast<size_t>(h.n_points) * 3u);
            if (!xyz.empty() && !read_exact(in_fd_, xyz.data(), xyz.size() * sizeof(float)))
            {
                break;
            }

            elevmap_point_topic_base::ElevMapBodyPointCloud out;
            out.frame_id(frame);
            out.timestamp_ns(h.timestamp_ns);
            out.is_dense(h.is_dense != 0);
            out.layer_name(layer);
            for (uint32_t i = 0; i < h.n_points; ++i)
            {
                elevmap_point_topic_base::PointXYZ p;
                p.x(xyz[3u * i + 0]);
                p.y(xyz[3u * i + 1]);
                p.z(xyz[3u * i + 2]);
                out.points().push_back(p);
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

    eprosima::fastdds::ElevBodyCloudWorker worker(in_fd, robot_type + "_elevmap_body_cloud");
    worker.run();
    return 0;
}

