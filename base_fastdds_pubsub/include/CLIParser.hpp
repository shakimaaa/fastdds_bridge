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

#include <csignal>
#include <cstdlib>
#include <iostream>

#include <fastdds/dds/log/Log.hpp>
#include <fastdds/rtps/attributes/BuiltinTransports.hpp>
#include <fastdds/dds/core/policy/QosPolicies.hpp>

#ifndef FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__CLIPARSER_HPP
#define FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__CLIPARSER_HPP


namespace eprosima::fastdds {

using dds::Log;

class CLIParser
{
public:
    CLIParser()
    = default;
    //! Entity kind enumeration
    enum class EntityKind : uint8_t
    {
        PUBLISHER,
        SUBSCRIBER,
        PUBSUB,
        UNDEFINED
    };
    //! Delivery mechanism enumeration
    enum class DeliveryMechanismKind : uint8_t
    {
        DATA_SHARING,
        INTRA_PROCESS,
        LARGE_DATA,
        SHM,
        TCPv4,
        TCPv6,
        UDPv4,
        UDPv6,
        DEFAULT
    };
    //! DeliveryMechanisms structure for the application
    struct dds_qos_config
    {
        CLIParser::EntityKind entity = CLIParser::EntityKind::UNDEFINED;
        bool ignore_local_endpoints = false;
        uint16_t samples = 0;
        uint32_t domain = 0;
        DeliveryMechanismKind delivery_mechanism = DeliveryMechanismKind::DEFAULT;
        std::string tcp_ip_address;
        bool save_all_history = false;
        bool Durability_Qos_open = false;
    };


    dds_qos_config cfg;
};
} // namespace eprosima::fastdds


#endif // FASTDDS_EXAMPLES_CPP_DELIVERY_MECHANISMS__CLIPARSER_HPP
