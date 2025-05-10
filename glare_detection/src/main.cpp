#include <return_position.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;

int main() {
    glare_position gp;
    glare_detector gd;
    position_queue pq;

    bool debug_mode = true;
    const double brightness_threshold = 0.3; 

    const char* cmd =
        "libcamera-vid -t 0 -n --width 640 --height 480 --codec mjpeg -o - 2>/dev/null | "
        "ffmpeg -f mjpeg -i - -f image2pipe -vcodec mjpeg - 2>/dev/null";

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

        auto brigthness = gd.isBrightArea(frame);

        cv::Point2f glarePos;
        cv::Point2f avg_glarePos;
        
        // check foward brightness state
        if (brigthness > brightness_threshold) {
            // Compute the photometric glare map from intensity, saturation, and contrast
            cv::Mat gphoto = gd.computePhotometricMap(frame);
            
            // Compute the geometric glare map from circle detection on gphoto
            cv::Mat ggeo = gd.computeGeometricMap(gphoto);

            // Combine all available maps into the final glare probability map
            cv::Mat glare_map = gd.combineMaps(gphoto, ggeo);

            // Threshold the glare map to obtain a binary glare mask
            cv::Mat glare_mask;
            cv::threshold(glare_map, glare_mask, 0.5, 1.0, cv::THRESH_BINARY);

            glarePos = gp.getGlareCoordinates(glare_mask);
            pq.push(glarePos);

            if (pq.shouldReturnAverage()){
                avg_glarePos = pq.avgCoord;
            }

            if (debug_mode) {
                gd.drawGlareContours(glare_mask, frame);  // Draw enclosing circles
                //cv::imshow("Glare Map", glare_map);
                //cv::imshow("Glare Mask", glare_mask);
            } 
            else {
                cv::circle(frame, avg_glarePos, 5, cv::Scalar(0, 0, 255), -1);  // Draw small dot
            }
            
        }  
        else{
            avg_glarePos.x = -1;
            avg_glarePos.y = -1;
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();    
        
        cout << "Detected glare position: (" << avg_glarePos.x << ", " << avg_glarePos.y << ")\n";
        cout << "Processing time: " << duration << " ms\n";

        imshow("glare Detection", frame);
        
        if (waitKey(1) == 27) break;
    }

    pclose(pipe);
    gp.gd.endVideo();
    destroyAllWindows();

    cout << ">>>>> Video Ended <<<<<\n";
    return 0;
}
