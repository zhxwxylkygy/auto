#include <vofa_bridge/vofa_bridge.h>
#include <iostream>

namespace vpie {
void VofaBridge::Open(std::string_view ip, unsigned short port) {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    memset(&addr_serv_, 0, sizeof(addr_serv_));
    addr_serv_.sin_family = AF_INET;
    addr_serv_.sin_addr.s_addr = inet_addr(ip.data());
    addr_serv_.sin_port = htons(port);
}
void VofaBridge::Close() const {
    close(sock_fd_);
}

void VofaBridge::SendData() {
    if (data_buffer_.empty())
        return;
    std::unique_ptr<char> ptr = std::make_unique<char>(sizeof(float) * data_buffer_.size() + data_tail_.size());
    std::memcpy(ptr.get(), data_buffer_.data(), sizeof(float) * data_buffer_.size());
    std::memcpy(ptr.get() + sizeof(float) * data_buffer_.size(), data_tail_.data(), data_tail_.size());
    sendto(sock_fd_, ptr.get(), sizeof(float) * data_buffer_.size() + data_tail_.size(), 0,
           reinterpret_cast<sockaddr*>(&addr_serv_), sizeof(addr_serv_));
    ClearData();
}

void VofaBridge::ClearData() {
    data_buffer_.clear();
    data_buffer_.resize(0);
}

void VofaBridge::Reset(std::string_view ip, unsigned short port) {
    Close();
    Open(ip, port);
}

VofaBridge& VofaBridge::Get() {
    static VofaBridge instance{};
    return instance;
}

VofaBridge::~VofaBridge() {
    Close();
}

VofaBridge::VofaBridge() {
    Reset("127.0.0.1", 1347);
};

}  // namespace vpie