//
// Created by wpie on 23-10-5.
//

#include "mcu_node/serial/serial.h"
#include <exception>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <utility>
#include "fmt/core.h"

Serial::Serial(Serial::SerialID serial_id) {
    Open(std::move(serial_id));
    is_open_ = VerifyOpen();
    if (is_open_) {
        handle_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        handle_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_DEFAULT);
        handle_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_DEFAULT);
        handle_->SetParity(LibSerial::Parity::PARITY_DEFAULT);
        handle_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    }
}

void Serial::Open(Serial::SerialID serial_id) {
    handle_->Open(serial_id);
}

bool Serial::VerifyOpen() const {
    return handle_->IsOpen();
}

bool Serial::IsOpen() const {
    return is_open_;
}

void Serial::Close() {
    handle_->Close();
}

Serial::~Serial() {
    Close();
}

int Serial::Recv(std::array<Byte, kBufferSize>& send_buf) const {
    LibSerial::DataBuffer dataBuffer;
    dataBuffer.resize(kBufferSize);
    try{
        handle_->Read(dataBuffer, kBufferSize, 100);
    }catch(const std::exception e){
        rclcpp::Logger logger = rclcpp::get_logger("serial");
        RCLCPP_INFO_STREAM(logger, "serial bombbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
        std::terminate();
    }
    
    memcpy(send_buf.data(), dataBuffer.data(), sizeof(send_buf));
    return 0;
}

int Serial::Send(const std::array<Byte, kBufferSize>& send_buf) const {
    LibSerial::DataBuffer dataBuffer;
    dataBuffer.resize(kBufferSize);
    memcpy(dataBuffer.data(), send_buf.data(), sizeof(send_buf));
    handle_->Write(dataBuffer);
    return 0;
}
