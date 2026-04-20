// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file SubscriberApp.cpp
 *
 */

#include "SubscriberApp.hpp"

#include <condition_variable>
#include <stdexcept>

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/TCPv6TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv6TransportDescriptor.hpp>

#include "CLIParser.hpp"
#include "motor_command_12PubSubTypes.hpp"
#include "motor_state_12PubSubTypes.hpp"
#include "motor_command_16PubSubTypes.hpp"
#include "motor_state_16PubSubTypes.hpp"
#include "imu_topic_basePubSubTypes.hpp"
#include "image_topic_basePubSubTypes.hpp"

using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastdds::dds;

namespace eprosima::fastdds {
    template<typename T>
    SubscriberApp<T>::SubscriberApp()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new T())
        , received_samples_(0)
        , samples_(0)  // 默认值
        , stop_(false)
    {
    }

    template<typename T>
void SubscriberApp<T>::init_fastdds_sub(
        const CLIParser::dds_qos_config& config,
        const std::string& topic_name)
{
     samples_= config.samples;
    // Require bounded type. Plain is only required for SHM/DATA_SHARING.
    if (!type_->is_bounded())
    {
        throw std::runtime_error(
                  "Generated type is not bounded; bounded type is required for this subscriber.");
    }
    if ((config.delivery_mechanism == CLIParser::DeliveryMechanismKind::SHM ||
         config.delivery_mechanism == CLIParser::DeliveryMechanismKind::DATA_SHARING) &&
        !type_->is_plain(eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION))
    {
        throw std::runtime_error(
                  "Generated type is not plain; plain type is required for SHM/DATA_SHARING delivery.");
    }

    // Create the participant
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("DeliveryMechanisms_sub_participant");
    pqos.transport().use_builtin_transports = false;

    uint32_t max_samples = samples_;
    if (max_samples == 0)
    {
        const int32_t def_depth =
                DATAREADER_QOS_DEFAULT.resource_limits().max_samples_per_instance;
        if (def_depth == LENGTH_UNLIMITED || def_depth < 0)
        {
            max_samples = 10;
        }
        else
        {
            max_samples = static_cast<uint32_t>(def_depth);
        }
    }

    // Transport default definitions
    pqos.transport().use_builtin_transports = false;
    LibrarySettings library_settings;
    library_settings.intraprocess_delivery = IntraprocessDeliveryType::INTRAPROCESS_OFF;

    switch (config.delivery_mechanism)
    {
        case CLIParser::DeliveryMechanismKind::INTRA_PROCESS:   // (It should never reach this section
        {
            // No transport needed, but at least a transport needs to be declared to avoid participant creation failure
            pqos.transport().use_builtin_transports = true;
            library_settings.intraprocess_delivery = IntraprocessDeliveryType::INTRAPROCESS_FULL;
            break;
        }
        case CLIParser::DeliveryMechanismKind::SHM:
        case CLIParser::DeliveryMechanismKind::DATA_SHARING:
        {
            std::shared_ptr<SharedMemTransportDescriptor> shm_transport_ =
                    std::make_shared<SharedMemTransportDescriptor>();
            shm_transport_->segment_size(shm_transport_->max_message_size() * max_samples);
            pqos.transport().user_transports.push_back(shm_transport_);
            break;
        }
        case CLIParser::DeliveryMechanismKind::LARGE_DATA:
        {
            // Large Data is a builtin transport
            pqos.transport().use_builtin_transports = true;
            pqos.setup_transports(BuiltinTransports::LARGE_DATA);
            break;
        }
        case CLIParser::DeliveryMechanismKind::TCPv4:
        {
            Locator tcp_v4_initial_peers_locator_;
            tcp_v4_initial_peers_locator_.kind = LOCATOR_KIND_TCPv4;
            tcp_v4_initial_peers_locator_.port = 5100;
            std::string tcp_ip_address = "127.0.0.1";
            if (!config.tcp_ip_address.empty())
            {
                tcp_ip_address = config.tcp_ip_address;
            }
            IPLocator::setIPv4(tcp_v4_initial_peers_locator_, tcp_ip_address);
            pqos.wire_protocol().builtin.initialPeersList.push_back(tcp_v4_initial_peers_locator_);
            pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastdds::dds::c_TimeInfinite;
            pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = dds::Duration_t(5, 0);
            pqos.transport().user_transports.push_back(std::make_shared<TCPv4TransportDescriptor>());
            break;
        }
        case CLIParser::DeliveryMechanismKind::TCPv6:
        {
            Locator tcp_v6_initial_peers_locator_;
            tcp_v6_initial_peers_locator_.kind = LOCATOR_KIND_TCPv6;
            tcp_v6_initial_peers_locator_.port = 5100;
            std::string tcp_ip_address = "::1";
            if (!config.tcp_ip_address.empty())
            {
                tcp_ip_address = config.tcp_ip_address;
            }
            IPLocator::setIPv6(tcp_v6_initial_peers_locator_, tcp_ip_address);
            pqos.wire_protocol().builtin.initialPeersList.push_back(tcp_v6_initial_peers_locator_);
            pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastdds::dds::c_TimeInfinite;
            pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = dds::Duration_t(5, 0);
            pqos.transport().user_transports.push_back(std::make_shared<TCPv6TransportDescriptor>());
            break;
        }
        case CLIParser::DeliveryMechanismKind::UDPv4:
        {
            pqos.transport().user_transports.push_back(std::make_shared<UDPv4TransportDescriptor>());
            break;
        }
        case CLIParser::DeliveryMechanismKind::UDPv6:
        {
            pqos.transport().user_transports.push_back(std::make_shared<UDPv6TransportDescriptor>());
            break;
        }
        default:
        {
            pqos.transport().use_builtin_transports = true;
            break;
        }
    }

    auto factory = DomainParticipantFactory::get_instance();
    factory->set_library_settings(library_settings);
    participant_ = factory->create_participant(config.domain, pqos, nullptr, StatusMask::none());
    if (participant_ == nullptr)
    {
        throw std::runtime_error("Participant initialization failed");
    }

    // Register the type
    type_.register_type(participant_);

    // Create the subscriber
    SubscriberQos sub_qos = SUBSCRIBER_QOS_DEFAULT;
    participant_->get_default_subscriber_qos(sub_qos);
    subscriber_ = participant_->create_subscriber(sub_qos, nullptr, StatusMask::none());
    if (subscriber_ == nullptr)
    {
        throw std::runtime_error("Subscriber initialization failed");
    }

    // Create the topic
    TopicQos topic_qos = TOPIC_QOS_DEFAULT;
    participant_->get_default_topic_qos(topic_qos);
    topic_ = participant_->create_topic(topic_name, type_.get_type_name(), topic_qos);
    if (topic_ == nullptr)
    {
        throw std::runtime_error("Topic initialization failed");
    }

    // Create the data reader
    DataReaderQos reader_qos = DATAREADER_QOS_DEFAULT;
    subscriber_->get_default_datareader_qos(reader_qos);
    reader_qos.reliability().kind = ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    // reader_qos.durability().kind = DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    // reader_qos.history().kind = HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        if (config.Durability_Qos_open)
        {
            reader_qos.durability().kind = DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        }
        else
        {
            reader_qos.durability().kind = DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        }
        if (config.save_all_history)
        {
            reader_qos.history().kind = HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
        }
        else
        {
            reader_qos.history().kind = HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        }
    // 与 DataWriterHistory 不同，DataReader 的 History 不会把 allocated_samples 限制到 max_samples 以下。
    // 默认 allocated_samples=100、max_samples=10 时，PoolConfig 初始池大于上限，易在内部触发异常分配。
    const int32_t depth = static_cast<int32_t>(max_samples);
    reader_qos.history().depth = depth;
    reader_qos.resource_limits().max_instances = 1;
    reader_qos.resource_limits().max_samples_per_instance = depth;
    reader_qos.resource_limits().max_samples = depth;
    reader_qos.resource_limits().allocated_samples = depth;
    if (CLIParser::DeliveryMechanismKind::DATA_SHARING == config.delivery_mechanism)
    {
        reader_qos.data_sharing().automatic();
    }
    else
    {
        reader_qos.data_sharing().off();
    }
    reader_ = subscriber_->create_datareader(topic_, reader_qos, this, StatusMask::all());
    if (reader_ == nullptr)
    {
        throw std::runtime_error("DataReader initialization failed");
    }
}
    template<typename T>
SubscriberApp<T>::~SubscriberApp()
{
    if (nullptr != participant_)
    {
        // Delete DDS entities contained within the DomainParticipant
        participant_->delete_contained_entities();

        // Delete DomainParticipant
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }
}
    template<typename T>
void SubscriberApp<T>::on_subscription_matched(
        DataReader* /*reader*/,
        const SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}
    template<typename T>
void SubscriberApp<T>::on_data_available(
        DataReader* reader)
{
    // FASTDDS_CONST_SEQUENCE(DataSeq, DeliveryMechanisms);
    //
    // DataSeq delivery_mechanisms_sequence;
    // SampleInfoSeq info_sequence;
    // while ((!is_stopped()) && (RETCODE_OK == reader->take(delivery_mechanisms_sequence, info_sequence)))
    // {
    //     for (LoanableCollection::size_type i = 0; i < info_sequence.length(); ++i)
    //     {
    //         if ((info_sequence[i].instance_state == ALIVE_INSTANCE_STATE) && info_sequence[i].valid_data)
    //         {
    //             const DeliveryMechanisms& delivery_mechanisms_ = delivery_mechanisms_sequence[i];
    //
    //             received_samples_++;
    //             std::cout << "Sample: '" << delivery_mechanisms_.message().data() << "' with index: '" <<
    //                 delivery_mechanisms_.index() << "' RECEIVED" << std::endl;
    //             if ((samples_ > 0) && (received_samples_ >= samples_))
    //             {
    //                 stop();
    //             }
    //         }
    //     }
    //     reader->return_loan(delivery_mechanisms_sequence, info_sequence);
    // }
}
    template<typename T>
bool SubscriberApp<T>::is_stopped()
{
    return stop_.load();
}
    template<typename T>
    void SubscriberApp<T>::stop_sub()
    {
        stop_.store(true);
        cv_.notify_all();
    }

    // void SubscriberApp::run()
    // {
    //     std::unique_lock<std::mutex> lock_(mutex_);
    //     cv_.wait(lock_, [&]
    //             {
    //                 return is_stopped();
    //             });
    // }
    template class SubscriberApp<Motor_Command_12::motor_cmdPubSubType>;
    template class SubscriberApp<Motor_State_12::motor_statePubSubType>;
    template class SubscriberApp<Motor_Command_16::motor_cmdPubSubType>;
    template class SubscriberApp<Motor_State_16::motor_statePubSubType>;
    template class SubscriberApp<imu_topic_base::IMUDataPubSubType>;
    template class SubscriberApp<image_topic_base::ImageDataPubSubType>;

} // namespace eprosima::fastdds
