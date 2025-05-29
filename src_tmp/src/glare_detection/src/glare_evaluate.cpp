""" 
IoU 보통 0.5, 0.75, 0.95 사용

Example Output:

=== Classification Evaluation ===
TP: 18, FP: 2, FN: 1, TN: 19
Precision: 0.90
Recall: 0.95
F1 Score: 0.925
Accuracy: 0.925

=== IoU Evaluation (Glare Exists) ===
Total Images with GT: 18
Average IoU (on TP): 0.78

=== mAP Evaluation ===
IoU >= 0.5 : Precision = 0.90
IoU >= 0.55 : Precision = 0.89
IoU >= 0.6 : Precision = 0.88
IoU >= 0.65 : Precision = 0.85
IoU >= 0.7 : Precision = 0.81
IoU >= 0.75 : Precision = 0.76
IoU >= 0.8 : Precision = 0.70
IoU >= 0.85 : Precision = 0.60
IoU >= 0.9 : Precision = 0.45
IoU >= 0.95 : Precision = 0.30
mAP@[0.5:0.95] = 0.724
"""

"""
GT 있고 예측도 있음		
→ IoU ≥ t	            ap_tp[t]++	                해당 임계값 만족한 예측
→ IoU < t	            ap_fp[t]++	                임계값 미달
GT 없음 & 예측 있음		
모든 t에서 ap_fp[t]++	 GT 없이 예측은 무조건 FP	
GT 있음 & 예측 없음	      해당 없음	                    mAP는 예측 없으면 포함 안 함
GT 없음 & 예측 없음	      해당 없음	                    예측 자체가 없으므로 제외됨
"""

// glare_evaluate.cpp
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <map>
#include "glare_detector.h"
#include "return_position.h"

namespace fs = std::filesystem;

struct YoloLabel {
    float cx, cy, w, h; // normalized (0~1)
};

cv::Rect2f yoloToRect(const YoloLabel& label, int img_w, int img_h) {
    // OpenCV는 사각형을 왼쪽 위 꼭짓점 좌표 (x, y) + 크기 (w, h) 로 표현
    float x = (label.cx - label.w / 2.0f) * img_w;
    float y = (label.cy - label.h / 2.0f) * img_h;
    float w = label.w * img_w;
    float h = label.h * img_h;
    return cv::Rect2f(x, y, w, h);
}

// a: YOLO 모델이나 알고리즘이 예측한 박스 (예: getPredictedBox()의 출력)
// b: 사람이 라벨링한 정답 박스 (예: Roboflow에서 export한 GT)
float computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    // 교차 영역의 좌측 상단 꼭짓점 (left-top)
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    // 교차 영역의 우측 하단 꼭짓점 (right-bottom)
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float interArea = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float unionArea = a.area() + b.area() - interArea;

    return (unionArea > 0.0f) ? interArea / unionArea : 0.0f;
}

std::vector<cv::Rect2f> loadYoloLabels(const std::string& label_path, int img_w, int img_h) {
    std::ifstream in(label_path);
    std::vector<cv::Rect2f> boxes;
    // 파일이 존재하지 않거나 열 수 없으면 (예: glare 없는 이미지) → 빈 리스트 반환
    if (!in.is_open()) return boxes;

    int class_id;
    YoloLabel lbl;
    while (in >> class_id >> lbl.cx >> lbl.cy >> lbl.w >> lbl.h) {
        boxes.push_back(yoloToRect(lbl, img_w, img_h));
    }
    return boxes;
}

cv::Rect2f getPredictedBox(const cv::Mat& frame, glare_position& gp, glare_detector& gd) {
    cv::Mat gphoto = gd.computePhotometricMap(frame);
    cv::Mat ggeo = gd.computeGeometricMap(gphoto);
    cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);

    cv::Mat combined = gd.combineMapsbyprod(gphoto, ggeo);
    cv::Mat masked;
    combined.copyTo(masked, (priority == 1));

    cv::Mat bin;
    cv::threshold(masked, bin, 1e-5, 1.0, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_8U);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return cv::Rect2f();

    std::vector<cv::Point> maxContour;
    double maxArea = 0.0;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area > maxArea) {
            maxArea = area;
            maxContour = c;
        }
    }

    // 가장 큰 glare blob을 감싸는 OpenCV 사각형 (좌상단 + 너비/높이) 반환
    // cv::boundingRect()는 OpenCV에서 제공하는 함수로, 점들의 집합(contour)을 감싸는 가장 작은 사각형(bounding box)을 계산
    return cv::boundingRect(maxContour);
}

int main() {
    std::string img_dir = "./test_images";
    std::string label_dir = "./gt_labels";

    glare_detector gd;
    glare_position gp;

    float total_iou = 0.0f;
    int total_tp_iou = 0;
    int total_images = 0;

    int TP = 0, FP = 0, FN = 0, TN = 0;

    std::map<float, int> ap_tp;
    std::map<float, int> ap_fp;
    std::vector<float> iou_thresholds;
    // iou_thresholds = {0.50, 0.55, 0.60, ..., 0.95} 생성
    for (float t = 0.5f; t <= 0.95f; t += 0.05f) iou_thresholds.push_back(t);

    for (const auto& entry : fs::directory_iterator(img_dir)) {
        std::string img_path = entry.path().string();     // 이미지 경로
        std::string fname = entry.path().stem().string(); // 파일 이름만 추출 (확장자 제거)
        std::string label_path = label_dir + "/" + fname + ".txt"; // 대응되는 라벨 파일 경로

        cv::Mat frame = cv::imread(img_path);
        if (frame.empty()) continue;

        auto gt_boxes = loadYoloLabels(label_path, frame.cols, frame.rows);
        cv::Rect2f pred_box = getPredictedBox(frame, gp, gd); // 해당 이미지(frame)에 대해 예측된 glare 영역을 담은 bounding box를 얻는 부분

        bool gt_has_glare = !gt_boxes.empty(); // gt_boxes가 없으면 gt는 glare가 없는 것으로 간주
        bool pred_has_glare = (pred_box.area() > 1.0); // 1.0 픽셀 면적보다 크면 감지된 것으로 간주

        if (gt_has_glare && pred_has_glare) {
            float best_iou = 0.0f;
            for (const auto& gt : gt_boxes) {
                best_iou = std::max(best_iou, computeIoU(pred_box, gt));
            }

            if (best_iou >= 0.5f) {
                TP++;
                total_tp_iou++;
                total_iou += best_iou;
            } else {
                FP++;
            }

            // 여러 IoU 임계값(0.5 ~ 0.95)에 대해, 해당 best_iou가 해당 기준을 만족하면 → ap_tp[t]++, 아니면 → ap_fp[t]++
            // 이 정보는 나중에 mAP@[0.5:0.95] 계산에 사용됨
            for (float t : iou_thresholds) {
                if (best_iou >= t) ap_tp[t]++;
                else ap_fp[t]++;
            }
        }
        else if (!gt_has_glare && pred_has_glare) {
            FP++; // 예측은 했지만 실제로는 glare 없음 → 무조건 FP
            for (float t : iou_thresholds) ap_fp[t]++; // 모든 IoU 임계값에서 오탐으로 간주됨 → FP 카운트만 증가
        }
        else if (gt_has_glare && !pred_has_glare) {
            FN++; // 실제 glare는 존재하는데, 예측을 못함 → FN
        }
        else if (!gt_has_glare && !pred_has_glare) {
            TN++; // 정답도 없고 예측도 안 함 → TN
        }

        total_images++;
    }

    std::cout << "\n=== Classification Evaluation ===" << std::endl;
    std::cout << "TP: " << TP << ", FP: " << FP << ", FN: " << FN << ", TN: " << TN << std::endl;

    int total = TP + FP + FN + TN;
    float precision = (TP + FP > 0) ? (float)TP / (TP + FP) : 0;
    float recall = (TP + FN > 0) ? (float)TP / (TP + FN) : 0;
    float accuracy = (total > 0) ? (float)(TP + TN) / total : 0;
    float f1 = (precision + recall > 0) ? 2 * precision * recall / (precision + recall) : 0;

    std::cout << "Precision: " << precision << std::endl;
    std::cout << "Recall: " << recall << std::endl;
    std::cout << "F1 Score: " << f1 << std::endl;
    std::cout << "Accuracy: " << accuracy << std::endl;

    std::cout << "\n=== IoU Evaluation (Glare Exists) ===" << std::endl;
    std::cout << "Total Images with GT: " << total_tp_iou << std::endl;
    std::cout << "Average IoU (on TP): " << (total_tp_iou > 0 ? total_iou / total_tp_iou : 0.0f) << std::endl;

    std::cout << "\n=== mAP Evaluation ===" << std::endl;
    float ap_sum = 0.0f;
    int ap_count = 0;
    for (float t : iou_thresholds) {
        int tp = ap_tp[t];
        int fp = ap_fp[t];
        float ap_precision = (tp + fp > 0) ? (float)tp / (tp + fp) : 0;
        std::cout << "IoU >= " << t << " : Precision = " << ap_precision << std::endl;
        ap_sum += ap_precision;
        ap_count++;
    }
    float mAP = (ap_count > 0) ? ap_sum / ap_count : 0.0f;
    std::cout << "mAP@[0.5:0.95] = " << mAP << std::endl;

    return 0;
}

