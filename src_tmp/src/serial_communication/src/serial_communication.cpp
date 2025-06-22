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
      
      int flags = fcntl(serial_port_fd, F_GETFL, 0);
      if (flags == -1) {
          std::cerr << "SerialCom Error getting flags: " << strerror(errno) << std::endl;
          close(serial_port_fd); serial_port_fd = -1; return false;
      }
      flags |= O_NONBLOCK; // 기존 플래그에 논블로킹 플래그를 추가(OR 연산)
      if (fcntl(serial_port_fd, F_SETFL, flags) == -1) {
          std::cerr << "SerialCom Error setting non-blocking: " << strerror(errno) << std::endl;
          close(serial_port_fd); serial_port_fd = -1; return false;
      }

      struct termios tty;
      if (tcgetattr(serial_port_fd, &tty) != 0) {
        std::cerr << "SerialCom Error " << errno
                  << " from tcgetattr: " << strerror(errno) << std::endl;
        close(serial_port_fd);
        serial_port_fd = -1;
        return false;
      }
      cfmakeraw(&tty);
      // 입력 플래그 (c_iflag)
      tty.c_iflag |= IGNBRK;  // BREAK 신호 무시
      tty.c_iflag &= ~(BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF); // 나머지 비활성화

      // 출력 플래그 (c_oflag)
      tty.c_oflag &= ~OPOST; // 출력 후처리 비활성화
      tty.c_oflag &= ~ONLCR; // NL -> CR-NL 변환 방지

      // 제어 플래그 (c_cflag)
      tty.c_cflag |= CRTSCTS; // 하드웨어 흐름 제어 활성화
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_cflag &= ~CSIZE;
      tty.c_cflag |= CS8;
      tty.c_cflag &= ~PARENB;
      tty.c_cflag &= ~CSTOPB;

      // 로컬 플래그 (c_lflag)
      tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN); // 에코 및 특수 처리 비활성화

      // 제어 문자 (c_cc)
      // minicom 설정과 동일하게 맞춰보기
      tty.c_cc[VMIN] = 1;  // 최소 1바이트를 읽을 때까지 대기
      tty.c_cc[VTIME] = 5; // 문자 간 타임아웃 0.5초

      // Baud Rate 설정
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
      // std::cout << "[SerialCom] Port " << port_name
      //           << " opened at specified baud rate." << std::endl;
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
        // 쓰기에 실패한 경우
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 이 경우는 오류가 아니라, "지금 당장은 쓸 수 없음 (버퍼가 꽉 찼거나 장치가 바쁨)"을 의미.
            // 데이터는 전송되지 않았지만, 프로그램이 멈추지는 않음.
            std::cout << "[SerialCom Info] write would block. Data not sent." << std::endl;
            return false; // 전송 실패로 간주
        } else {
            // 그 외의 심각한 쓰기 오류
            std::cerr << "SerialCom Error writing to serial port: " << strerror(errno) << std::endl;
            return false;
        }
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

        // std::cout << "[SerialCom] Sending Command Byte (binary): ";
        // for (int i = 7; i >= 0; --i) {
        //     std::cout << ((command_byte >> i) & 1);
        //     if (i == 4) std::cout << "_";
        // }
        // std::cout << " (Decimal: " << static_cast<int>(command_byte) << ")" << std::endl;

        return sendByte(command_byte); // 내부 sendByte 함수 호출
    }

} // namespace SerialCom
