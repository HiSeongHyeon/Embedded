#include "config.hpp"
#include "coordinates.hpp" // camera_to_driver_coords, to_bit_list 선언
#include "get_grid_size.hpp"
#include <chrono> // std::chrono
#include <iostream>
#include <thread>  // std::this_thread::sleep_for
#include <utility> // std::pair
#include <vector>

// --- 시리얼 통신 관련 헤더 ---
#include <cstring>   // strerror
#include <errno.h>   // Error number definitions
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

// --- 시리얼 포트 관련 전역 변수 및 함수 ---
int serial_port_fd = -1; // 시리얼 포트 파일 디스크립터

// 시리얼 포트 초기화 함수 (Baud Rate 기본값 115200)
bool init_serial(const char *port_name, speed_t baud_rate = B115200) {
  serial_port_fd =
      open(port_name, O_RDWR | O_NOCTTY | O_NDELAY); // Non-blocking 모드 추가
  if (serial_port_fd < 0) {
    std::cerr << "Error " << errno << " opening " << port_name << ": "
              << strerror(errno) << std::endl;
    return false;
  }

  // Non-blocking 모드로 설정 (fcntl)
  fcntl(serial_port_fd, F_SETFL,
        0); // 블로킹 모드로 다시 설정 후, termios 설정 (선택적)
  // fcntl(serial_port_fd, F_SETFL, FNDELAY); // Non-blocking

  struct termios tty;
  if (tcgetattr(serial_port_fd, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno)
              << std::endl;
    close(serial_port_fd);
    serial_port_fd = -1;
    return false;
  }

  // 기본 설정 (8N1, Raw 모드와 유사하게)
  cfmakeraw(&tty); // Raw 모드 설정 (대부분의 특수 문자 처리 비활성화)
  tty.c_cflag |= (CLOCAL | CREAD); // 로컬 라인, 수신 가능
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8 데이터 비트
  tty.c_cflag &= ~PARENB;  // 패리티 없음
  tty.c_cflag &= ~CSTOPB;  // 1 스톱 비트
  tty.c_cflag &= ~CRTSCTS; // 하드웨어 흐름 제어 없음

  // 입력 모드: Raw input
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // 출력 모드: Raw output
  tty.c_oflag &= ~OPOST;

  // 최소 수신 문자 수 및 타임아웃 (Non-canonical mode)
  tty.c_cc[VMIN] = 0;  // 읽을 최소 문자 수 (0: non-blocking)
  tty.c_cc[VTIME] = 1; // 문자 간 타임아웃 (0.1초)

  // Baud Rate 설정
  cfsetispeed(&tty, baud_rate);
  cfsetospeed(&tty, baud_rate);

  // 설정 적용
  tcflush(serial_port_fd, TCIFLUSH); // 버퍼 비우기
  if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno)
              << std::endl;
    close(serial_port_fd);
    serial_port_fd = -1;
    return false;
  }
  return true;
}

// 1바이트 데이터 전송 함수
bool send_byte_serial(unsigned char data_byte) {
  if (serial_port_fd < 0) {
    std::cerr << "Error: Serial port not open." << std::endl;
    return false;
  }
  int n = write(serial_port_fd, &data_byte, 1);
  if (n < 0) {
    std::cerr << "Error writing to serial port: " << strerror(errno)
              << std::endl;
    return false;
  }
  return (n == 1);
}

// 시리얼 포트 닫기 함수
void close_serial() {
  if (serial_port_fd >= 0) {
    close(serial_port_fd);
    serial_port_fd = -1;
    std::cout << "[Serial] Port closed." << std::endl;
  }
}

int main() {
  // --- 시리얼 포트 설정 ---
  const char *arduino_port =
      "/dev/ttyUSB0"; // <<--- 실제 Arduino 포트로 수정
                      // 예: "/dev/ttyUSB0" 또는 Raspberry Pi의 경우
                      // "/dev/serial0" 등
  if (!init_serial(arduino_port, B115200)) { // Baud rate 115200으로 설정
    return -1; // 시리얼 포트 열기 실패 시 종료
  }
  // 프로그램 종료 시 시리얼 포트 자동 닫기 등록
  atexit(close_serial);
  std::cout << "[Serial] Port " << arduino_port
            << " opened successfully at 115200 bps." << std::endl;

  // 임의의 태양 중심점
  std::pair<double, double> sun_center_test_case = {320.0,
                                                    120.0}; // 테스트용 좌표

  std::cout << "[Test] Using sun_center: (" << sun_center_test_case.first
            << ", " << sun_center_test_case.second << ")" << std::endl;

  // 1. 좌표 변환 함수 호출
  std::pair<int, int> grid = camera_to_driver_coords(sun_center_test_case);

  // 결과 출력
  std::cout << "Grid Position: (" << grid.first << " [col], " << grid.second
            << " [row])" << std::endl;

  // 2. 비트 리스트 변환 함수 호출 (디버깅/확인용)
  std::vector<int> bit_list_vec = to_bit_list(grid);

  std::cout << "Grid Position as bit list (for verification): [";
  for (size_t i = 0; i < bit_list_vec.size(); ++i) {
    std::cout << bit_list_vec[i] << (i == bit_list_vec.size() - 1 ? "" : ", ");
  }
  std::cout << "]" << std::endl;

  // --- Arduino로 전송할 1바이트 패킷 생성 ---
  unsigned char command_byte = 0;
  bool glare_detected_flag = true; // **임의의 값: 항상 Glare 감지됨으로 가정**
                                   // 실제 Glare 감지 모듈과 연동 시 이 값 사용

  if (glare_detected_flag) {
    // 최상위 비트 설정 (Glare 감지됨)
    command_byte |= (1 << 7); // 1000 0000

    // 그리드 좌표 (col, row)를 0-11 사이의 단일 인덱스로 변환
    std::pair<int, int> grid_dims = get_grid_size();
    int num_cols = grid_dims.first;
    int grid_index =
        grid.second * num_cols +
        grid.first; // (row * num_cols + col) - Python to_bit_list와 일치시킴

    if (grid_index >= 0 && grid_index < 12) { // 유효한 그리드 인덱스 (0~11)
      // 최하위 4비트에 그리드 인덱스 설정
      command_byte |= (static_cast<unsigned char>(grid_index) & 0x0F);
    } else {
      std::cerr << "Warning: Calculated grid index " << grid_index
                << " is out of range (0-11). Sending retract command."
                << std::endl;
      command_byte = 0; // 오류 시 접기 명령 (Glare 없음)
    }
  } else {
    // Glare 감지되지 않음 (command_byte는 이미 0)
  }

  // 생성된 명령 바이트 출력 (이진수 형태)
  std::cout << "Command Byte to send (binary): ";
  for (int i = 7; i >= 0; --i) {
    std::cout << ((command_byte >> i) & 1);
    if (i == 4)
      std::cout << " "; // 가독성을 위해 공백 추가
  }
  std::cout << " (Decimal: " << static_cast<int>(command_byte) << ")"
            << std::endl;

  // 3. Arduino로 명령 전송
  if (send_byte_serial(command_byte)) {
    std::cout << "[Serial] Command byte sent successfully to Arduino."
              << std::endl;
  } else {
    std::cout << "[Serial] Failed to send command byte to Arduino."
              << std::endl;
  }

  // 간단한 테스트를 위해 잠시 대기 (Arduino 반응 확인용)
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // close_serial(); // atexit 핸들러가 프로그램 종료 시 자동으로 호출함

  return 0;
}