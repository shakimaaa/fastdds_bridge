#pragma once

#include <cstdint>

namespace tf_robot_date_ipc
{

#pragma pack(push, 1)
struct MotorPacket
{
    uint8_t tag;
    uint32_t motor_id[12];
    float position[12];
    float velocity[12];
    float effort[12];
};

struct ImuPacket
{
    uint8_t tag;
    float qw;
    float qx;
    float qy;
    float qz;
};
#pragma pack(pop)

constexpr uint8_t kTagMotor = static_cast<uint8_t>('M');
constexpr uint8_t kTagImu = static_cast<uint8_t>('I');

} // namespace tf_robot_date_ipc

