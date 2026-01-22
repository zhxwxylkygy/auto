//
// Created by XianY on 2023/3/16.
//

#ifndef ULTRA_VISION_FRAMEWORK_COMMUNICATION_H
#define ULTRA_VISION_FRAMEWORK_COMMUNICATION_H

#include "array"

using Byte = unsigned char;

template<std::size_t BUFFER_SIZE>
class ICommunication {
public:
    static constexpr const std::size_t kBufferSize = BUFFER_SIZE;

    virtual int Recv(std::array<Byte, kBufferSize> &) const = 0;

    virtual int Send(const std::array<Byte, kBufferSize> &) const = 0;

    virtual ~ICommunication() = default;
};

#endif //ULTRA_VISION_FRAMEWORK_COMMUNICATION_H
