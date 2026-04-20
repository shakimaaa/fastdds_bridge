#include <cstdlib>
#include <cstdint>
#include <string>
#include <unistd.h>

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "PublisherApp.hpp"
#include "image_topic_basePubSubTypes.hpp"
#include "yaml-cpp/yaml.h"
#include "config.h"

using namespace eprosima::fastdds;

namespace
{
struct Header
{
    uint8_t tag;
    uint32_t width, height, step, data_size;
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
class ImageWorker : public PublisherApp<image_topic_base::ImageDataPubSubType>
{
public:
    ImageWorker(int in_fd, const std::string& topic) : in_fd_(in_fd)
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
            if (h.tag != 'G') continue;
            image_topic_base::ImageData out;
            out.width() = h.width;
            out.height() = h.height;
            out.step() = h.step;
            if (h.data_size > image_topic_base::MAX_IMAGE_BYTES) h.data_size = image_topic_base::MAX_IMAGE_BYTES;
            if (!read_exact(in_fd_, out.data().data(), h.data_size)) break;
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

    eprosima::fastdds::ImageWorker w(in_fd, "image_" + robot_type + "_sim");
    w.run();
    return 0;
}

