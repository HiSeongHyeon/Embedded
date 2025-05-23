#define CAMERA
// #define VIDEO

#ifdef CAMERA

#include <return_position.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <chrono>

// 추가가
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


// [추후에 활성화] 아두이노 연결시
// 프로그램 종료 시 시리얼 포트 자동 닫기를 위한 핸들러
void cleanup_serial_main_handler() { SerialCom::closePort(); }
// 

int main() {
    glare_position gp;
    glare_detector gd;
    position_queue pq;

    cv::Point2f glarePos;
    cv::Point2f avg_glarePos;

    // // [추후에 활성화] 아두이노 연결시
    // // (추가) 시리얼 포트 초기화
    // const char *arduino_port = "/dev/ttyACM0"; // <<--- 실제 Arduino 포트로 수정
    // if (!SerialCom::initialize(arduino_port, B115200)) { // Baud rate 115200
    //     cerr << "Error: Failed to initialize serial port " << arduino_port << endl;
    // return -1; // 시리얼 포트 열기 실패 시 종료
    // }
    // atexit(cleanup_serial_main_handler); // 프로그램 종료 시 포트 자동 닫기 등록
    // cout << "[Serial] Port " << arduino_port << " opened successfully." << endl;
    // //여기까지

    bool debug_mode = false;
    const double brightness_threshold = -1; 

    // 기존 카메라 스트림 코드
    // const char* cmd =
    //     "libcamera-vid -t 0 -n --width 640 --height 480 --codec mjpeg -o - 2>/dev/null | "
    //     // "ffmpeg -f mjpeg -i - -f image2pipe -vcodec copy -";
    //     "ffmpeg -f mjpeg -analyzeduration 10000000 -probesize 10000000 -i - -f image2pipe -vcodec copy -";

    // 노출 수동 조절 코드
    const char* cmd =
        "libcamera-vid -t 0 -n --width 640 --height 480 "
        "--shutter 100000 --gain 1.0 --awbgains 1.2,1.2 "
        "--codec mjpeg -o - 2>/dev/null | "
        "ffmpeg -f mjpeg -analyzeduration 10000000 -probesize 10000000 -i - -f image2pipe -vcodec copy -";
    

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

        auto start = high_resolution_clock::now();

        auto brightness = gd.isBrightArea(frame);

        
        
        // check foward brightness state
        if (brightness > brightness_threshold) {
            // Compute the photometric glare map from intensity, saturation, and contrast
            cv::Mat gphoto = gd.computePhotometricMap(frame);
            
            // Compute the geometric glare map from circle detection on gphoto
            cv::Mat ggeo = gd.computeGeometricMap(gphoto);

            cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);

            glarePos = gp.getPriorityBasedGlareCenter(priority, gphoto, ggeo, gd);

            auto detect_end = high_resolution_clock::now();
            auto detect_duration = duration_cast<milliseconds>(detect_end - start).count();
            cout << "Detect Processing time: " << detect_duration << " ms\n"; 

            pq.push(glarePos);
            if (pq.shouldReturnAverage() == 1) {
                avg_glarePos = pq.getAvgCoord();
            }
            else if (pq.shouldReturnAverage() == 0){
                continue;
            }
            else{
                avg_glarePos = {-1, -1};
            }
            
            cout << pq.shouldReturnAverage() << "\n";

            // // 시각화를 위한 threshold 기반 glare mask 생성
            // cv::Mat glare_map = gd.combineMaps(gphoto, ggeo);
            // cv::Mat glare_mask;
            // cv::threshold(glare_map, glare_mask, 0.8, 1.0, cv::THRESH_BINARY);

            // gd.drawGlareContours(glare_mask, frame);  // Draw enclosing contours from mask

            if (debug_mode && glarePos.x > 0) {
                // ✅ glarePos를 기반으로 imshow에 원 표시
                cv::circle(frame, glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
                cout << "Detected glare position: (" << glarePos.x << ", " << glarePos.y << ")\n";

            } else {
                
                cv::circle(frame, avg_glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
                cout << "Detected glare position: (" << avg_glarePos.x << ", " << avg_glarePos.y << ")\n";
            }
            
        }  
        else{
            avg_glarePos.x = -1;
            avg_glarePos.y = -1;
        }

        // 종현이 형 코드 추가
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

        // // [추후에 활성화] 아두이노 연결시
        // // (추가) Arduino 명령 바이트 생성 및 전송
        // if (!SerialCom::sendCommandToArduino(glare_is_detected_flag, grid_coords)) {
        //         cerr << "[Main] Error: Failed to send command to Arduino via SerialCom module." << endl;
        // }
              

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();    
        
        
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
        
        cout << "Total Processing time: " << duration << " ms\n";



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

#elif VIDEO

#include <return_position.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <chrono>

// 추가가
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

// [추후에 활성화] 아두이노 연결시
// 프로그램 종료 시 시리얼 포트 자동 닫기를 위한 핸들러
// void cleanup_serial_main_handler() { SerialCom::closePort(); }
// 

int main() {
    glare_position gp;
    glare_detector gd;
    position_queue pq;

    // [추후에 활성화] 아두이노 연결시
    // (추가) 시리얼 포트 초기화
    // const char *arduino_port = "/dev/ttyACM0"; // <<--- 실제 Arduino 포트로 수정
    // if (!SerialCom::initialize(arduino_port, B115200)) { // Baud rate 115200
    //     cerr << "Error: Failed to initialize serial port " << arduino_port << endl;
    // return -1; // 시리얼 포트 열기 실패 시 종료
    // }
    // atexit(cleanup_serial_main_handler); // 프로그램 종료 시 포트 자동 닫기 등록
    // cout << "[Serial] Port " << arduino_port << " opened successfully." << endl;
    // 여기까지

    // bool debug_mode = true;
    bool debug_mode = false;
    const double brightness_threshold = 0.3; 

    // Replace this with your video file path
    string videoPath = "/home/tiger2/Embedded/src_tmp/src/glare_detection/resources/sun_clouds_1.mp4";

    // Open video file
    VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        cerr << "Failed to open video file: " << videoPath << endl;
        return -1;
    }

    bool first_frame = true;
    Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        if (first_frame) {
            gp.gd.startVideo(frame);
            first_frame = false;
        }

        auto start = high_resolution_clock::now();

        auto brightness = gd.isBrightArea(frame);

        cv::Point2f glarePos;
        cv::Point2f avg_glarePos;
        
        // check foward brightness state
        if (brightness > brightness_threshold) {
            // Compute the photometric glare map from intensity, saturation, and contrast
            cv::Mat gphoto = gd.computePhotometricMap(frame);
            
            // Compute the geometric glare map from circle detection on gphoto
            cv::Mat ggeo = gd.computeGeometricMap(gphoto);

            cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);

            cv::Point2f glarePos = gp.getPriorityBasedGlareCenter(priority, gphoto, ggeo, gd);

            auto detect_end = high_resolution_clock::now();
            auto detect_duration = duration_cast<milliseconds>(detect_end - start).count();
            cout << "Detect Processing time: " << detect_duration << " ms\n"; 

            pq.push(glarePos);
            if (pq.shouldReturnAverage()) {
                avg_glarePos = pq.getAvgCoord();
            }

            // 시각화를 위한 threshold 기반 glare mask 생성
            cv::Mat glare_map = gd.combineMaps(gphoto, ggeo);
            cv::Mat glare_mask;
            cv::threshold(glare_map, glare_mask, 0.5, 1.0, cv::THRESH_BINARY);

            // gd.drawGlareContours(glare_mask, frame);  // Draw enclosing contours from mask

            if (debug_mode && glarePos.x >= 0) {
                // ✅ glarePos를 기반으로 imshow에 원 표시
                cv::circle(frame, glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
            } else {
                cv::circle(frame, avg_glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
            }

            
        }  
        else{
            avg_glarePos.x = -1;
            avg_glarePos.y = -1;
        }

        // 종현이 형 코드 추가
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

        // [추후에 활성화] 아두이노 연결시
        // // (추가) Arduino 명령 바이트 생성 및 전송
        // if (!SerialCom::sendCommandToArduino(glare_is_detected_flag, grid_coords)) {
        //         cerr << "[Main] Error: Failed to send command to Arduino via SerialCom module." << endl;
        // }
        //       

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();    
        
        cout << "Detected glare position: (" << avg_glarePos.x << ", " << avg_glarePos.y << ")\n";

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
        
        cout << "Total Processing time: " << duration << " ms\n";



        imshow("glare Detection", frame);
        
        if (waitKey(1) == 27) break;
    }
    
    cap.release();
    gp.gd.endVideo();
    destroyAllWindows();

    cout << ">>>>> Video Ended <<<<<\n";
    return 0;
}

#else
// Test accuracy for video

#include <return_position.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <chrono>
#include <filesystem>
#include <thread>

// 추가가
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

// [추후에 활성화] 아두이노 연결시
// 프로그램 종료 시 시리얼 포트 자동 닫기를 위한 핸들러
// void cleanup_serial_main_handler() { SerialCom::closePort(); }
// 

#define IMAGE 1 

namespace fs = std::filesystem;

bool ends_with(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string> getVideoFilesInFolder(const std::string& folderPath) {
    std::vector<std::string> videoFiles;
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {
            std::string path = entry.path().string();
            // 확장자 확인
            // if (ends_with(path, ".mp4") || ends_with(path, ".avi") || ends_with(path, ".mov")) {
            //     videoFiles.push_back(path);
            // }
            // image일 경우
            if (ends_with(path, ".jpg") || ends_with(path, ".png")) {
                videoFiles.push_back(path);
            }
        }
    }
    return videoFiles;
}

int main() {
    glare_position gp;
    glare_detector gd;
    position_queue pq;

    // [추후에 활성화] 아두이노 연결시
    // (추가) 시리얼 포트 초기화
    // const char *arduino_port = "/dev/ttyACM0"; // <<--- 실제 Arduino 포트로 수정
    // if (!SerialCom::initialize(arduino_port, B115200)) { // Baud rate 115200
    //     cerr << "Error: Failed to initialize serial port " << arduino_port << endl;
    // return -1; // 시리얼 포트 열기 실패 시 종료
    // }
    // atexit(cleanup_serial_main_handler); // 프로그램 종료 시 포트 자동 닫기 등록
    // cout << "[Serial] Port " << arduino_port << " opened successfully." << endl;
    // 여기까지

    // bool debug_mode = true;
    bool debug_mode = true;
    const double brightness_threshold = -1; 

    // variable for calculate test run time
    int priority2_valid = 0;

    int time_count = 0;

    double detect_time = 0.0;
    double total_time = 0.0;
    double avg_detect_time = 0.0;
    double avg_total_time = 0.0;


    std::string folderPath = "/home/tiger2/Embedded/src_tmp/src/glare_detection/resources/image";  // 비디오가 들어있는 폴더 경로
    std::vector<std::string> videoPaths = getVideoFilesInFolder(folderPath);

    for (const auto& videoPath : videoPaths) {
        std::cout << "\n=== 처리 중: " << videoPath << " ===\n";

        cv::VideoCapture cap(videoPath);
        if (!cap.isOpened()) {
            std::cerr << "비디오 열기 실패: " << videoPath << std::endl;
            continue;
        }
        bool first_frame = true;
        Mat frame;

        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            if (first_frame) {
                gp.gd.startVideo(frame);
                first_frame = false;
            }

            auto start = high_resolution_clock::now();

            auto brightness = gd.isBrightArea(frame);

            cv::Point2f glarePos;
            cv::Point2f avg_glarePos;
            
            // check foward brightness state
            if (brightness > brightness_threshold) {
                // Compute the photometric glare map from intensity, saturation, and contrast
                cv::Mat gphoto = gd.computePhotometricMap(frame);
                
                // Compute the geometric glare map from circle detection on gphoto
                cv::Mat ggeo = gd.computeGeometricMap(gphoto);

                cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);
                
                // test용 코드(priority 2 여부 검사)
                cv::Mat mask_priority2 = (priority ==2);
                priority2_valid = cv::countNonZero(mask_priority2);

                cv::Point2f glarePos = gp.getPriorityBasedGlareCenter(priority, gphoto, ggeo, gd);

                if (priority2_valid) {
                    auto detect_end = high_resolution_clock::now();
                    auto detect_duration = duration_cast<milliseconds>(detect_end - start).count();
                    
                    time_count ++;
                    detect_time += detect_duration;
                }
                else{
                    cerr << "No exist priority 2 in image \n";
                }
                
                pq.push(glarePos);
                if (pq.shouldReturnAverage()) {
                    avg_glarePos = pq.getAvgCoord();
                }

                // 시각화를 위한 threshold 기반 glare mask 생성
                // cv::Mat glare_map = gd.combineMaps(gphoto, ggeo);
                // cv::Mat glare_mask;
                // cv::threshold(glare_map, glare_mask, 0.5, 1.0, cv::THRESH_BINARY);

                // gd.drawGlareContours(glare_mask, frame);  // Draw enclosing contours from mask

                if (debug_mode && glarePos.x >= 0) {
                    // ✅ glarePos를 기반으로 imshow에 원 표시
                    cv::circle(frame, glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
                } else {
                    cv::circle(frame, avg_glarePos, 10, cv::Scalar(0, 255, debug_color), 2);
                }

                
            }  
            else{
                avg_glarePos.x = -1;
                avg_glarePos.y = -1;
            }

            // 종현이 형 코드 추가
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

            // [추후에 활성화] 아두이노 연결시
            // // (추가) Arduino 명령 바이트 생성 및 전송
            // if (!SerialCom::sendCommandToArduino(glare_is_detected_flag, grid_coords)) {
            //         cerr << "[Main] Error: Failed to send command to Arduino via SerialCom module." << endl;
            // }
            //       

            if (priority2_valid) {
                auto end = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(end - start).count();
                
                total_time += duration;
            }
            
            avg_detect_time = detect_time/time_count;
            avg_total_time = total_time/time_count;
            if (IMAGE){
                cout << "glarePos.x: "<< glarePos.x << ", glarePos.y: "<< glarePos.y<< "\n";
            }
            else{
                cout << "avg_glarePos.x: "<< avg_glarePos.x << ", avg_glarePos.y: "<< avg_glarePos.y<< "\n";
            }
            
            // if (glare_is_detected_flag && grid_coords.first != -1) {
            // cout << " | Grid Coords: (" << grid_coords.first << ", "
            //     << grid_coords.second << ")\n";
            // cout << " | BitList: [";
            // for (size_t i = 0; i < bit_list_for_grid.size(); ++i) {
            //     cout << bit_list_for_grid[i]
            //         << (i == bit_list_for_grid.size() - 1 ? "" : ", ");
            // }
            // cout << "]\n";
            // } else if (glare_is_detected_flag) {
            // cout << " | Grid Coords: Transform Failed\n";
            // }
            
            

            imshow("glare Detection", frame);
            
            if (waitKey(1) == 27) break;
        }

        cout << "avg_detect_time: "<< avg_detect_time << "ms\n";
        cout << "avg_total_time: "<< avg_total_time << "ms\n";

        detect_time = 0;
        total_time = 0;
        time_count = 0;

        cap.release();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }   
    gp.gd.endVideo();
    destroyAllWindows();

    cout << ">>>>> Video Ended <<<<<\n";
    return 0;      
}

#endif