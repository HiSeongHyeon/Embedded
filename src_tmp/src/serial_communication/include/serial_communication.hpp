#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <string>    // std::string
#include <termios.h> // speed_t

namespace SerialCom {

extern int serial_port_fd;

// 시리얼 포트 초기화 함수
bool initialize(const std::string &port_name, speed_t baud_rate = B115200);

// 1바이트 데이터 전송 함수
bool sendByte(unsigned char data_byte);

// 시리얼 포트 닫기 함수
void closePort();

} // namespace SerialCom

#endif // SERIAL_COMMUNICATION_HPP