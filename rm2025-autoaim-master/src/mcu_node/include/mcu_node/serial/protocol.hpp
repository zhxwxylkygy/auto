//
// Created by XianY on 2023/3/16.
//

#ifndef ULTRA_VISION_FRAMEWORK_PROTOCOL_HPP
#define ULTRA_VISION_FRAMEWORK_PROTOCOL_HPP

#include <cstddef>
#include <cstring>
#include "memory"
#include "optional"
#include "protocol_pack.h"
#include "communication.h"
#include "crc_16.hpp"
#include "protocol_data.h"


class Protocol {
   private:
    template <typename T>
    static ProtocolPack<sizeof(T)> Encode(T data)
        requires ProtocolData<T>;

    template <typename T>
    static std::optional<T> Decode(const ProtocolPack<sizeof(T)>& protocol_pack)
        requires ProtocolData<T>;

   public:
    template <std::size_t LEN>
    static void Send(const ICommunication<LEN>& communication, const ProtocolData auto& data);

    template <typename T, size_t LEN>
    static auto Recv(const ICommunication<LEN>& communication)
        requires ProtocolData<T>;
};

template <typename To, typename From>
constexpr To BitCast(const From& from) noexcept {
    std::unique_ptr<To> to{static_cast<To*>(::calloc(1, sizeof(To)))};
    ::memcpy(to.get(), &from, std::min(sizeof(To), sizeof(From)));
    return *to;
}

template <typename T>
ProtocolPack<sizeof(T)> Protocol::Encode(T data)
    requires ProtocolData<T>
{
    ProtocolPack<sizeof(T)> protocol_pack{.data = BitCast<std::array<Byte, sizeof(T)>>(data),
                                          .check = Crc16::Encode(protocol_pack.data)};
    return protocol_pack;
}

template <std::size_t LEN>
void Protocol::Send(const ICommunication<LEN>& communication, const ProtocolData auto& data) {
    communication.Send(BitCast<std::array<Byte, LEN>>(Protocol::Encode(data)));
}

template <typename T>
std::optional<T> Protocol::Decode(const ProtocolPack<sizeof(T)>& protocol_pack)
    requires ProtocolData<T>
{
    return ProtocolPack<sizeof(T)>::Verify(protocol_pack) ? std::optional<T>{std::bit_cast<T>(protocol_pack.data)}
                                                          : std::nullopt;
}

template <typename T, size_t LEN>
auto Protocol::Recv(const ICommunication<LEN>& communication)
    requires ProtocolData<T>
{
    std::array<Byte, LEN> recv{};
    communication.Recv(recv);

    return Protocol::Decode<T>(BitCast<ProtocolPack<sizeof(T)>>(recv));
}

#endif  // ULTRA_VISION_FRAMEWORK_PROTOCOL_HPP
