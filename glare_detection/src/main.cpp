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
    solar_position sp;

    const char* cmd =
        "libcamera-vid -t 0 -n --width 640 --height 480 --codec mjpeg -o - | "
        "ffmpeg -f mjpeg -i - -f image2pipe -vcodec copy -";

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
            sp.sd.startVideo(frame);
            first_frame = false;
        }

        auto start = high_resolution_clock::now();

        auto sunPos = sp.getSunCoordinates(frame);
        double area = sp.getSunArea();

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();

        cout << "Detected sun position: (" << sunPos.first << ", " << sunPos.second << ")\n";
        cout << "Processing time: " << duration << " ms\n";

        sp.sd.drawSun(frame);
        imshow("Sun Detection", frame);

        if (waitKey(10) == 27) break;
    }

    pclose(pipe);
    sp.sd.endVideo();
    destroyAllWindows();

    cout << ">>>>> Video Ended <<<<<\n";
    return 0;
}
