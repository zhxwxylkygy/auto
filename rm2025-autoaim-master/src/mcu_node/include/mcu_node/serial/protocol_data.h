//
// Created by XianY on 2023/3/19.
//

#ifndef ULTRA_VISION_FRAMEWORK_PROTOCOL_DATA_H
#define ULTRA_VISION_FRAMEWORK_PROTOCOL_DATA_H

#include "concepts"

struct IProtocolData {
    ~IProtocolData() = default;
};

template<typename T>
concept ProtocolData = std::derived_from<T, IProtocolData>;

#endif //ULTRA_VISION_FRAMEWORK_PROTOCOL_DATA_H
