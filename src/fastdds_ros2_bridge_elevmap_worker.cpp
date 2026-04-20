#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <unistd.h>

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "PublisherApp.hpp"
#include "elevmap_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"

using namespace eprosima::fastdds;

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
class ElevWorker : public PublisherApp<elevmap_topic_base::ElevMapDataPubSubType>
{
public:
    ElevWorker(int in_fd, const std::string& topic) : in_fd_(in_fd)
    {
        CLIParser qos;
        YAML::Node cfg = YAML::LoadFile(std::string(THIS_COM_CONFIG) + "domain_config.yaml");
        qos.cfg.domain = cfg["global_topic_domain"] ? cfg["global_topic_domain"].as<int>() : 0;
        qos.cfg.samples = 5;
        qos.cfg.delivery_mechanism = CLIParser::DeliveryMechanismKind::UDPv4;
        init_fastdds_pub(qos.cfg, topic);
    }

    void run()
    {
        Header h{};
        while (read_exact(in_fd_, &h, sizeof(h)))
        {
            if (h.tag != 'E') continue;
            elevmap_topic_base::ElevMapData out;
            std::string frame(h.frame_len, '\0');
            if (!frame.empty() && !read_exact(in_fd_, &frame[0], frame.size())) break;
            out.frame_id(frame);
            out.timestamp(h.timestamp);
            out.resolution(h.resolution);
            out.length_x(h.length_x);
            out.length_y(h.length_y);
            out.rows(h.rows);
            out.cols(h.cols);

            for (uint32_t i = 0; i < h.basic_layers_count; ++i)
            {
                uint32_t n = 0;
                if (!read_exact(in_fd_, &n, sizeof(n))) return;
                std::string s(n, '\0');
                if (n && !read_exact(in_fd_, &s[0], n)) return;
                if (out.basic_layers().size() < elevmap_topic_base::MAX_LAYERS) out.basic_layers().push_back(s);
            }

            for (uint32_t i = 0; i < h.layers_count; ++i)
            {
                LayerHead lh{};
                if (!read_exact(in_fd_, &lh, sizeof(lh))) return;
                std::string name(lh.name_len, '\0');
                if (lh.name_len && !read_exact(in_fd_, &name[0], name.size())) return;
                std::vector<float> vals(lh.data_count);
                if (lh.data_count && !read_exact(in_fd_, vals.data(), lh.data_count * sizeof(float))) return;
                elevmap_topic_base::LayerData layer;
                layer.name(name);
                layer.data() = vals;
                if (out.layers().size() < elevmap_topic_base::MAX_LAYERS) out.layers().push_back(layer);
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
        if (std::string(argv[i]) == "--robot") robot_type = argv[i + 1];
        else if (std::string(argv[i]) == "--fd") in_fd = std::atoi(argv[i + 1]);
    }
    if (in_fd < 0) return 1;

    eprosima::fastdds::ElevWorker w(in_fd, robot_type + "_elevmap_sim");
    w.run();
    return 0;
}

