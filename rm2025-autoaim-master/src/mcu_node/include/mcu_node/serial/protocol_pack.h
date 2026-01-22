//
// Created by XianY on 2023/3/16.
//

#ifndef ULTRA_VISION_FRAMEWORK_PROTOCOL_PACK_H
#define ULTRA_VISION_FRAMEWORK_PROTOCOL_PACK_H

#include "array"
#include "bit"
#include "packer.h"
#include "communication.h"
#include "crc_16.hpp"
#include "protocol_data.h"


struct ProtocolDefinition {
    static constexpr Byte Head{0x3e};
    static constexpr Byte Tail{0xff};
};

template <std::size_t DATA_LEN>
PROTOCOL ProtocolPack {
    const Byte head{ProtocolDefinition::Head};
    const Byte length{DATA_LEN};
    std::array<Byte, DATA_LEN> data{};
    std::array<Byte, 2> check{};

    static bool Verify(const ProtocolPack<DATA_LEN>& protocol_pack) {
        return protocol_pack.length == DATA_LEN && protocol_pack.head == ProtocolDefinition::Head &&
               Crc16::Verify(protocol_pack.data, protocol_pack.check);
    }
};

#endif  // ULTRA_VISION_FRAMEWORK_PROTOCOL_PACK_H
