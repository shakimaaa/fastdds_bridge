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
 * @file SubscriberApp.h
 *
 */

#ifndef FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__SUBSCRIBERAPP_HPP
#define FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__SUBSCRIBERAPP_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "CLIParser.hpp"


namespace eprosima::fastdds {
    template<typename T>
class SubscriberApp : public dds::DataReaderListener
{
public:

    // SubscriberApp(
    //         const CLIParser::delivery_mechanisms_config& config,
    //         const std::string& topic_name);
        // 构造函数：不再需要参数，后续通过 init_fastdds 统一初始化
    SubscriberApp();
        // 独立的初始化函数
    void init_fastdds_sub(
         const CLIParser::dds_qos_config& config,
         const std::string& topic_name);

    ~SubscriberApp();

        // 虚函数，子类可以重写订阅数据处理
        virtual void on_data_available(
                dds::DataReader* reader) override;

        //! Subscriber matched method
        virtual void on_subscription_matched(
                dds::DataReader* reader,
                const dds::SubscriptionMatchedStatus& info) override;
        void stop_sub();
        //! Return the current state of execution

        dds::DomainParticipant* participant_;

        dds::Subscriber* subscriber_;

        dds::Topic* topic_;

        dds::DataReader* reader_;

        dds::TypeSupport type_;

        std::condition_variable cv_;

        std::mutex mutex_;

        uint32_t received_samples_;

        uint32_t samples_;

        std::atomic<bool> stop_;
// private:
        bool is_stopped();


};

} // namespace eprosima::fastdds


#endif // FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__SUBSCRIBERAPP_HPP
