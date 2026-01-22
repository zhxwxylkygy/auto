//
// Created by wpie on 23-10-5.
//

#ifndef INC_0601_SERIAL_K1_H
#define INC_0601_SERIAL_K1_H
#include "array"
#include "chrono"
#include "libserial/SerialPort.h"
#include "memory"
#include "string"
#include "communication.h"
#include "protocol_data.h"

class Serial : public ICommunication<64> {
   public:
    using SerialID = const char*;

    Serial() = delete;

    explicit Serial(Serial::SerialID serial_id);
    // 阻塞式的通讯器

    ~Serial() override;

    void Close();

    int Recv(std::array<Byte, kBufferSize>& send_buf) const final;

    [[nodiscard]] int Send(const std::array<Byte, kBufferSize>& send_buf) const final;

    [[nodiscard]] bool IsOpen() const;

   private:
    std::shared_ptr<LibSerial::SerialPort> handle_ = std::make_shared<LibSerial::SerialPort>();

    std::chrono::milliseconds block_time_;

    bool is_open_{false};

    void Open(SerialID serial_id);

    [[nodiscard]] bool VerifyOpen() const;
};
#endif  // INC_0601_SERIAL_K1_H
