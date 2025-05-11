#include "serial_communication.hpp"
#include <cstring> // strerror
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

namespace SerialCom {

int serial_port_fd = -1; // 파일 범위 정적 변수로 선언하여 외부 직접 접근 방지
                         // 권장 또는 클래스 정적 멤버로 변경

bool initialize(const std::string &port_name, speed_t baud_rate) {
  serial_port_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_port_fd < 0) {
    std::cerr << "SerialCom Error " << errno << " opening " << port_name << ": "
              << strerror(errno) << std::endl;
    return false;
  }
  fcntl(serial_port_fd, F_SETFL, 0); // 블로킹 모드로 다시 설정

  struct termios tty;
  if (tcgetattr(serial_port_fd, &tty) != 0) {
    std::cerr << "SerialCom Error " << errno
              << " from tcgetattr: " << strerror(errno) << std::endl;
    close(serial_port_fd);
    serial_port_fd = -1;
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  cfsetispeed(&tty, baud_rate);
  cfsetospeed(&tty, baud_rate);
  tcflush(serial_port_fd, TCIFLUSH);
  if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
    std::cerr << "SerialCom Error " << errno
              << " from tcsetattr: " << strerror(errno) << std::endl;
    close(serial_port_fd);
    serial_port_fd = -1;
    return false;
  }
  std::cout << "[SerialCom] Port " << port_name
            << " opened at specified baud rate." << std::endl;
  return true;
}

bool sendByte(unsigned char data_byte) {
  if (serial_port_fd < 0) {
    std::cerr << "SerialCom Error: Serial port not open for sending."
              << std::endl;
    return false;
  }
  int n = write(serial_port_fd, &data_byte, 1);
  if (n < 0) {
    std::cerr << "SerialCom Error writing to serial port: " << strerror(errno)
              << std::endl;
    return false;
  }
  return (n == 1);
}

void closePort() {
  if (serial_port_fd >= 0) {
    close(serial_port_fd);
    serial_port_fd = -1;
    std::cout << "[SerialCom] Port closed." << std::endl;
  }
}

} // namespace SerialCom