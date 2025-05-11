#include "return_position.h" // Glare 감지 모듈 (solar_position)
#include <chrono> // high_resolution_clock, duration_cast, milliseconds
#include <cstdio> // popen, pclose, fgetc, feof
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility> // std::pair
#include <vector>

// --- 좌표 변환 및 시리얼 통신 모듈 헤더 ---
#include "config.hpp"
#include "coordinates.hpp" // camera_to_driver_coords, to_bit_list
#include "get_grid_size.hpp"
#include "serial_communication.hpp"

#include <cstdlib>

using namespace std;
using namespace cv;
using namespace std::chrono;

// (추가) 프로그램 종료 시 시리얼 포트 자동 닫기를 위한 핸들러
void cleanup_serial_main_handler() { SerialCom::closePort(); }
// 여기까지

int main() {
  // (추가) 시리얼 포트 초기화
  const char *arduino_port = "/dev/ttyUSB0"; // <<--- 실제 Arduino 포트로 수정
  if (!SerialCom::initialize(arduino_port, B115200)) { // Baud rate 115200
    cerr << "Error: Failed to initialize serial port " << arduino_port << endl;
    return -1; // 시리얼 포트 열기 실패 시 종료
  }
  atexit(cleanup_serial_main_handler); // 프로그램 종료 시 포트 자동 닫기 등록
  cout << "[Serial] Port " << arduino_port << " opened successfully." << endl;
  // 여기까지

  solar_position sp;

  const char *cmd =
      "libcamera-vid -t 0 -n --width 640 --height 480 --codec mjpeg -o - | "
      "ffmpeg -f mjpeg -i - -f image2pipe -vcodec copy -";

  FILE *pipe = popen(cmd, "r");
  if (!pipe) {
    cerr << "ffmpeg 실행 실패\n";
    return -1;
  }

  vector<uchar> buffer;
  int c;
  bool start_found = false;
  bool first_frame = true;

  while (true) {
    buffer.clear();
    start_found = false;

    while ((c = fgetc(pipe)) != EOF) {
      buffer.push_back((uchar)c);

      if (!start_found && buffer.size() >= 2 &&
          buffer[buffer.size() - 2] == 0xFF &&
          buffer[buffer.size() - 1] == 0xD8) {
        start_found = true;
      }

      if (start_found && buffer.size() >= 2 &&
          buffer[buffer.size() - 2] == 0xFF &&
          buffer[buffer.size() - 1] == 0xD9) {
        break;
      }
    }

    if (feof(pipe)) {
      cerr << "파이프 종료됨\n";
      break;
    }

    if (buffer.size() < 100)
      continue;

    Mat jpegData(buffer);
    Mat frame = imdecode(jpegData, IMREAD_COLOR);
    if (frame.empty())
      continue;

    if (first_frame) {
      sp.sd.startVideo(frame);
      first_frame = false;
    }

    auto start = high_resolution_clock::now();

    auto sunPos = sp.getSunCoordinates(frame);
    double area = sp.getSunArea();

    bool glare_is_detected_flag = (sunPos.first != -1 && sunPos.second != -1);
    std::pair<int, int> grid_coords = {-1, -1};
    std::vector<int> bit_list_for_grid;

    if (glare_is_detected_flag) {
      std::pair<double, double> sun_center_for_transform = {
          static_cast<double>(sunPos.first),
          static_cast<double>(sunPos.second)};
      grid_coords = camera_to_driver_coords(sun_center_for_transform);

      std::vector<int> bit_list = to_bit_list(grid_coords);
    }

    // (추가) Arduino 명령 바이트 생성 및 전송
    unsigned char command_to_arduino = 0; // 기본값: 접기 (Glare 없음)

    if (glare_is_detected_flag) {
      if (grid_coords.first != -1 &&
          grid_coords.second != -1) {   // 유효한 그리드 좌표인 경우
        command_to_arduino |= (1 << 7); // 최상위 비트: Glare 감지됨 (1)

        std::pair<int, int> grid_dims = get_grid_size();
        int num_cols = grid_dims.first;
        // camera_to_driver_coords는 (col, row) 순서로 반환
        int grid_index = grid_coords.second * num_cols + grid_coords.first;

        if (grid_index >= 0 &&
            grid_index < 9) { // 3x3 그리드의 유효 인덱스 (0~8)
          command_to_arduino |= (static_cast<unsigned char>(grid_index) &
                                 0x0F); // 최하위 4비트에 인덱스
        } else {
          cerr << "Warning: Calculated grid index " << grid_index
               << " is out of range (0-11). Defaulting to retract command."
               << endl;
          command_to_arduino = 0; // 오류 시 안전하게 접기 명령
        }
      } else {
        // Glare는 감지되었으나 좌표 변환 실패 또는 유효하지 않은 그리드
        cerr << "Warning: Glare detected, but coordinate transformation failed "
                "or grid is invalid. Sending retract command."
             << endl;
        command_to_arduino = 0; // 접기 명령
      }
    }
    // Glare가 감지되지 않은 경우 command_to_arduino는 0 (접기) 유지

    // Arduino로 명령 전송
    if (!SerialCom::sendByte(command_to_arduino)) {
      cerr << "Error: Failed to send command to Arduino." << endl;
      // 여기에 시리얼 통신 재시도 로직 또는 에러 처리 추가 가능
    }
    // 여기까지

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start).count();

    cout << "Detected sun position: (" << sunPos.first << ", " << sunPos.second
         << ")\n";

    if (glare_is_detected_flag && grid_coords.first != -1) {
      cout << " | Grid Coords: (" << grid_coords.first << ", "
           << grid_coords.second << ")\n";
      cout << " | BitList: [";
      for (size_t i = 0; i < bit_list_for_grid.size(); ++i) {
        cout << bit_list_for_grid[i]
             << (i == bit_list_for_grid.size() - 1 ? "" : ", ");
      }
      cout << "]\n";
    } else if (glare_is_detected_flag) {
      cout << " | Grid Coords: Transform Failed\n";
    }

    cout << "Processing time: " << duration << " ms\n";

    sp.sd.drawSun(frame);
    imshow("Sun Detection", frame);

    if (waitKey(10) == 27)
      break;
  }

  pclose(pipe);
  sp.sd.endVideo();
  destroyAllWindows();

  cout << ">>>>> Video Ended <<<<<\n";
  return 0;
}
