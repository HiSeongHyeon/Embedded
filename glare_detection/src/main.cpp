// =========================
// File: main.cpp
// =========================
// This main program implements automatic glare detection based on the paper:
// "Automatic Glare Detection via Photometric, Geometric, and Global Positioning Information"
// by Mehran Andalibi and Damon M. Chandler (2017).
//
// ⚠️ Notes on Differences from the Paper:
// - Saturation: Instead of computing S = (V - min(R,G,B)) / V manually, we use the S channel from HSV.
//   This is a common approximation for efficiency, but can be updated for full alignment.
// - Ggeo: Instead of building a Gaussian distribution explicitly at each circle center, we draw filled circles
//   and apply GaussianBlur for efficiency. This approximates the Gaussian sum described in the paper.
//
// ✅ Hyperparameters:
// - Local contrast block size: 17x17, stride: 4 (matches paper)
// - HoughCircles radius: 10 to 200
// - Gaussian blur kernel: 15x15 with σ=5 (as specified)
// - Gsun: sigma = image_height / 12 (as specified)
// - Threshold for glare mask: 0.85

#include "glare_detector.hpp"
#include "gps_utils.hpp"
#include "glare_utils.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>

int main() {
    // Open the default camera (typically the Pi Camera or USB camera)
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open the camera." << std::endl;
        return -1;
    }

    // Create instances for photometric/geometric processing and GPS sun estimation
    GlareDetector detector;
    GPSUtils gps;

    // Toggle options
    bool use_sun_map = true;   // Use GPS-based solar glare map
    bool debug_mode = true;    // Show intermediate glare maps and outlines for debugging

    // Thresholds and durations
    const double brightness_threshold = 0.7;    // Brightness level (0~1) to consider environment as bright
    const int detection_duration_ms = 3000;     // Minimum duration of brightness before glare detection (ms)

    // For tracking how long brightness has persisted
    auto bright_start_time = std::chrono::steady_clock::time_point();
    bool in_bright_zone = false;

    while (true) {
        // Capture a frame from the camera
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Convert to grayscale and compute mean brightness (0 to 1)
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        double mean_intensity = cv::mean(gray)[0] / 255.0;

        // If brightness exceeds threshold, track how long it's been bright
        auto now = std::chrono::steady_clock::now();
        if (mean_intensity > brightness_threshold) {
            if (!in_bright_zone) {
                bright_start_time = now;
                in_bright_zone = true;
            }
        } else {
            in_bright_zone = false;
        }

        // Check if brightness has lasted long enough to start glare detection
        bool glare_detect = false;
        if (in_bright_zone) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - bright_start_time).count();
            if (elapsed > detection_duration_ms) {
                glare_detect = true;
            }
        }

        if (glare_detect) {
            // Compute the photometric glare map from intensity, saturation, and contrast
            cv::Mat gphoto = detector.computePhotometricMap(frame);

            // Compute the geometric glare map from circle detection on gphoto
            cv::Mat ggeo = detector.computeGeometricMap(gphoto);

            // Optional: Compute the GPS-based solar glare map (Gsun)
            cv::Mat gsun;
            if (use_sun_map) {
                gsun = gps.computeSunMap(frame.size(), 37.5665, 126.9780, 0.0);  // Seoul location
            }

            // Combine all available maps into the final glare probability map
            cv::Mat glare_map = detector.combineMaps(gphoto, ggeo, gsun);

            // Threshold the glare map to obtain a binary glare mask
            cv::Mat glare_mask;
            cv::threshold(glare_map, glare_mask, 0.85, 1.0, cv::THRESH_BINARY);

            // Compute the centroid of the detected glare area
            cv::Point2f center = GlareUtils::computeCentroid(glare_mask);
            if (center.x >= 0 && center.y >= 0) {
                std::cout << "[GLARE DETECTED] Centroid: (" << center.x << ", " << center.y << ")" << std::endl;

                // Visualization: draw the result depending on mode
                if (debug_mode) {
                    GlareUtils::drawGlareContours(glare_mask, frame);  // Draw enclosing circles
                } else {
                    cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);  // Draw small dot
                }
            }

            // Show glare detection output
            cv::imshow("Original", frame);
            if (debug_mode) {
                cv::imshow("Glare Map", glare_map);
                cv::imshow("Glare Mask", glare_mask);
            }
        } else {
            // Show original view if not in glare detection mode
            cv::imshow("Original", frame);
            if (debug_mode) {
                cv::imshow("Grayscale", gray);
            }
        }

        // Exit loop if ESC key is pressed
        if (cv::waitKey(1) == 27) break;
    }

    // Release camera and close windows
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
