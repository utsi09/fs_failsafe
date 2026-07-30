#pragma once
#include <cstdint>
#include <cstring>
#include "rclcpp/rclcpp.hpp"

namespace cdr_parser {

inline int64_t parse_stamp_ns(const std::shared_ptr<rclcpp::SerializedMessage>& msg)
{
    const auto& rcl_msg = msg->get_rcl_serialized_message();
    if (rcl_msg.buffer_length < 12) return -1;

    const uint8_t* buf = rcl_msg.buffer;
    bool little_endian = (buf[1] == 0x01);

    int32_t sec;
    uint32_t nanosec;

    if (little_endian) {
        std::memcpy(&sec, &buf[4], 4);
        std::memcpy(&nanosec, &buf[8], 4);
    } else {
        sec = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];
        nanosec = (buf[8] << 24) | (buf[9] << 16) | (buf[10] << 8) | buf[11];
    }

    if (sec == 0 && nanosec == 0) return -1;
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nanosec);
}

}
