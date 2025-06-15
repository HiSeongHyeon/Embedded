#include "serial_communication.hpp"
#include "get_grid_size.hpp"
#include <cstring> // strerror
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

namespace SerialCom {

    int serial_port_fd = -1; // 파일 범위 정적 변수로 선언하여 외부 직접 접근 방지

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
      //tty.c_cflag &= ~CRTSCTS;
      tty.c_cflag &= CRTSCTS;
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

    bool sendCommandToArduino(bool glare_detected, const std::pair<int, int>& grid_coords) {
        unsigned char command_byte = 0; // 기본값: 접기 (Glare 없음)

        if (glare_detected) {
            // grid_coords는 (col, row)
            if (grid_coords.first != -1 && grid_coords.second != -1) {
                command_byte |= (1 << 7); // 최상위 비트: Glare 감지됨 (1)

                // get_grid_size()는 coordinate_mapping 모듈에 있으므로,
                // CMakeLists.txt에서 serial_communication_lib가
                // coordinate_mapping_lib의 헤더에 접근 가능하도록 설정해야 함.
                // 또는, 그리드 크기를 인자로 받거나 여기서 직접 정의할 수도 있음.
                std::pair<int, int> grid_dims = get_grid_size(); // (4, 3) 또는 (3,3) 등
                int num_cols = grid_dims.first;

                int col = grid_coords.first;
                int row = grid_coords.second;

                // 가정: 하위 4비트를 [C1, C0, R1, R0] (Col MSB, Col LSB, Row MSB, Row LSB) 순서
                // col (0, 1, 2 for 3xN grid), row (0, 1, 2 for Nx3 grid) -> 각 2비트
                if (col >= 0 && col < num_cols && row >= 0 && row < grid_dims.second) {
                     unsigned char col_bits = static_cast<unsigned char>(col) & 0x03; // 하위 2비트
                     unsigned char row_bits = static_cast<unsigned char>(row) & 0x03; // 하위 2비트
                     command_byte |= (col_bits << 2); // Col을 비트 3, 2로
                     command_byte |= row_bits;        // Row를 비트 1, 0으로
                } else {
                    std::cerr << "SerialCom Warning: Grid coords (" << col << "," << row
                              << ") out of range for " << num_cols << "x" << grid_dims.second
                              << " grid. Sending retract command." << std::endl;
                    command_byte = 0; // 오류 시 안전하게 접기
                }
            } else {
                std::cerr << "SerialCom Warning: Glare detected, but grid_coords are invalid. Sending retract command." << std::endl;
                command_byte = 0; // 좌표 변환 실패 시 접기
            }
        }
        // Glare 미감지 시 command_byte는 0 (접기) 유지

        std::cout << "[SerialCom] Sending Command Byte (binary): ";
        for (int i = 7; i >= 0; --i) {
            std::cout << ((command_byte >> i) & 1);
            if (i == 4) std::cout << "_";
        }
        std::cout << " (Decimal: " << static_cast<int>(command_byte) << ")" << std::endl;

        return sendByte(command_byte); // 내부 sendByte 함수 호출
    }

} // namespace SerialCom
