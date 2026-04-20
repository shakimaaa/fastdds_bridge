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
 * @file PublisherApp.hpp
 *
 */

#ifndef FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__PUBLISHERAPP_HPP
#define FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__PUBLISHERAPP_HPP


#include <condition_variable>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "CLIParser.hpp"


namespace eprosima::fastdds {
    template<typename T>
    class PublisherApp : public dds::DataWriterListener
    {
    public:
        PublisherApp();  // 默认构造函数

        // PublisherApp(
        //         const CLIParser::delivery_mechanisms_config& config,
        //         const std::string& topic_name);
        // 显式的 FastDDS 初始化函数
        void init_fastdds_pub(
                const CLIParser::dds_qos_config& config,
                const std::string& topic_name);

        ~PublisherApp();

        //! Publisher matched method
        void on_publication_matched(
                dds::DataWriter* writer,
                const dds::PublicationMatchedStatus& info) override;

        void stop_pub();
        dds::DataWriter* writer_;
        std::condition_variable cv_;

        std::mutex mutex_;
        int32_t matched_;
        //! Return the current state of execution
        bool is_stopped();
    private:

        dds::DomainParticipant* participant_;

        dds::Publisher* publisher_;

        dds::Topic* topic_;

        dds::TypeSupport type_;

        const uint32_t period_ms_ = 100; // in ms

        uint32_t index_of_last_sample_sent_;

        uint16_t samples_;

        std::atomic<bool> stop_;
    };

} // namespace eprosima::fastdds


#endif // FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__PUBLISHERAPP_HPP
