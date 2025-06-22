#define GRID_TEST // 그리드 좌표 테스트 모듈 활성화

#include <return_position.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <chrono>

// 추가
#include <utility> // std::pair
#include <string>

// --- 좌표 변환 및 시리얼 통신 모듈 헤더 ---
#include "config.hpp"
#include "coordinates.hpp" // camera_to_driver_coords, to_bit_list
#include "get_grid_size.hpp"
#include "serial_communication.hpp"

#include <cstdlib>

using namespace std;
using namespace cv;
using namespace std::chrono;

// 프로그램 종료 시 시리얼 포트 자동 닫기를 위한 핸들러
void cleanup_serial_main_handler() { SerialCom::closePort(); }

int main() {
    glare_position gp;
    glare_detector gd;
    position_queue pq;

    cv::Point2f glarePos;
    cv::Point2f avg_glarePos = {0, 0};

    // 시리얼 포트 초기화
    const char *arduino_port = "/dev/ttyACM0"; // <<--- 실제 Arduino 포트
    if (!SerialCom::initialize(arduino_port, B115200)) { // Baud rate 115200
        cerr << "Error: Failed to initialize serial port " << arduino_port << endl;
    return -1; // 시리얼 포트 열기 실패 시 종료
    }
    atexit(cleanup_serial_main_handler); // 프로그램 종료 시 포트 자동 닫기 등록
    cout << "[Serial] Port " << arduino_port << " opened successfully." << endl;

    bool debug_mode = true;
    // 0.2 -> 0.8
    const double brightness_threshold = 0.2;
    const double stddev_threshold = 0.5;

    // // 기존 카메라 스트림 코드
    // const char* cmd =
    //     "libcamera-vid -t 0 -n --width 1280 --height 480 --codec mjpeg -o - 2>/dev/null | "
    //     // "ffmpeg -f mjpeg -i - -f image2pipe -vcodec copy -";
    //     "ffmpeg -f mjpeg -analyzeduration 10000000 -probesize 10000000 -i - -f image2pipe -vcodec copy -";

    // 노출 수동 조절 코드
    const char* cmd =
        "libcamera-vid -t 0 -n --width 1280 --height 480 "
        "--shutter 6000 --gain 1.0 --awbgains 1.2,1.2 "
        "--codec mjpeg -o - 2>/dev/null | "
        "stdbuf -o0 ffmpeg -f mjpeg -analyzeduration 10000000 -probesize 10000000 -i - -f image2pipe -vcodec copy -";


    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        cerr << "ffmpeg 실행 실패\n";
        return -1;
    }

    vector<uchar> buffer;
    int c;
    bool start_found = false;
    bool first_frame = true;

    while (true) {
        ////////////////////      카메라 파이프 연결      ////////////////////
        buffer.clear();
        start_found = false;

        while ((c = fgetc(pipe)) != EOF) {
            buffer.push_back((uchar)c);

            if (!start_found && buffer.size() >= 2 &&
                buffer[buffer.size() - 2] == 0xFF && buffer[buffer.size() - 1] == 0xD8) {
                start_found = true;
            }

            if (start_found && buffer.size() >= 2 &&
                buffer[buffer.size() - 2] == 0xFF && buffer[buffer.size() - 1] == 0xD9) {
                break;
            }
        }

        if (feof(pipe)) {
            cerr << "파이프 종료됨\n";
            break;
        }

        if (buffer.size() < 100) continue;

        Mat jpegData(buffer);
        Mat frame = imdecode(jpegData, IMREAD_COLOR);
        if (frame.empty()) continue;

        if (first_frame) {
            gp.gd.startVideo(frame);
            first_frame = false;
        }
        //////////////////////////////////////////////////////////////////////
        
        // glare detect 시간, 실행시간 측정 시작
        auto start = high_resolution_clock::now();

        auto brightness = gd.isBrightArea(frame);
        auto stddev = gd.isStandardArea(frame);

        // cout << "brightness: "<< brightness << ", stddev: "<< stddev << "\n";

        // check foward brightness state, 차량 전방 밝기 상태와 표준 편차를 동시에 고려
        // 밝은 곳 or 표준편차가 큰 곳(태양에 의한 과도한 노출 조정 or 그림자 고려)
        if (brightness > brightness_threshold || stddev > stddev_threshold) {
            // intensity, saturation, contrast에 따라 gphoto map 생성
            cv::Mat gphoto = gd.computePhotometricMap(frame);

            // gphoto map 상에서 원형의 glare를 찾아 ggeo map 생성
            cv::Mat ggeo = gd.computeGeometricMap(gphoto);

            // gphoto, ggeo map에 따라 priority 부여
            cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);

            // priority에 따라 찾은 glare의 좌표 반환
            glarePos = gp.getPriorityBasedGlareCenter(priority, gphoto, ggeo, gd);

            // glare detect 시간 측정 종료
            auto detect_end = high_resolution_clock::now();
            auto detect_duration = duration_cast<milliseconds>(detect_end - start).count();
            cout << "Detect Processing time: " << detect_duration << " ms\n"; 
            
            // glare의 좌표가 유효한 지 여부에 따라 queue에 저장 후 glare의 평균 좌표 반환
            pq.push(glarePos);
            if (pq.shouldReturnAverage() == 1) {        // glare가 존재하는 경우. glare의 평균 좌표 최신화
                avg_glarePos = pq.getAvgCoord();
            }
            else if (pq.shouldReturnAverage() == 0){    // glare가 잠깐 사라진 경우. 바로 선바이저가 접히는 것을 방지하기 위해 glare 평균 좌표 유지
                continue;
            }
            else{                                       // glare가 존재하지 않는 경우. 유효하지 않은 좌표 반환
                avg_glarePos = {-1, -1};
            }

            // cout << pq.shouldReturnAverage() << "\n";

            // debug_mode가 True, glare의 좌표가 양수(카메라 프레임 내에 존재할 경우)일 때 원으로 표시
            if (debug_mode && glarePos.x > 0) {
                cv::circle(frame, glarePos, 10, cv::Scalar(255, 0, debug_color), 2);
                cout << "Detected glare position: (" << glarePos.x << ", " << glarePos.y << ")\n";
                cout << "Detected avg glare position: (" << avg_glarePos.x << ", " << avg_glarePos.y << ")\n";

            } 

        } 
        // 어두운 곳이면서 표준 편차가 작은 곳(카메라가 빛을 정면으로 촬영하지 않을 때)
        else{
            avg_glarePos = {-1, -1};
        }

        // 카메라 기준 coord -> 운전자 기준 coord로 변환
        bool glare_is_detected_flag = (avg_glarePos.x != -1 && avg_glarePos.y != -1);
        std::pair<int, int> grid_coords = {-1, -1};
        std::vector<int> bit_list_for_grid;

        if (glare_is_detected_flag) {
            std::pair<double, double> sun_center_for_transform = {
                static_cast<double>(avg_glarePos.x),
                static_cast<double>(avg_glarePos.y)};
            grid_coords = camera_to_driver_coords(sun_center_for_transform);

            std::vector<int> bit_list = to_bit_list(grid_coords);
            bit_list_for_grid = bit_list;
        }

        // Arduino 명령 바이트 생성 및 전송
        if (!SerialCom::sendCommandToArduino(glare_is_detected_flag, grid_coords)) {
                cerr << "[Main] Error: Failed to send command to Arduino via SerialCom module." << endl;
        }

        // 실행시간 측정 종료
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();    

        cout << "Total Processing time: " << duration << " ms\n";

        // 좌표 변환 시각화 함수 (테스트용)
        #ifdef GRID_TEST
            std::pair<int, int> grid_dims_to_visualize = 
            get_grid_size(); // 전체 그리드 크기 가져오기

            cv::Mat frame_with_grid; // 결과를 받을 Mat

            visualize_grid_on_frame(frame,
                                    frame_with_grid, 
                                    grid_dims_to_visualize
                                    );

            imshow("visualize grid index", frame_with_grid);
        #endif

        imshow("glare Detection", frame);

        if (waitKey(1) == 27) {
            glare_is_detected_flag = 0;
            SerialCom::sendCommandToArduino(glare_is_detected_flag, grid_coords);
            break;
        }
    }

    pclose(pipe);
    gp.gd.endVideo();
    destroyAllWindows();

    cout << ">>>>> Video Ended <<<<<\n";
    return 0;
}