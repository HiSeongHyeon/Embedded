// glare_evaluate.cpp (with resizing applied + arrow key navigation)
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

#define RESIZED_WIDTH 640
#define RESIZED_HEIGHT 480

struct YoloLabel {
    float cx, cy, w, h; // normalized (0~1)
};

cv::Rect2f yoloToRect(const YoloLabel& label, int img_w, int img_h) {
    float x = (label.cx - label.w / 2.0f) * img_w;
    float y = (label.cy - label.h / 2.0f) * img_h;
    float w = label.w * img_w;
    float h = label.h * img_h;
    return cv::Rect2f(x, y, w, h);
}

float computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float interArea = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float unionArea = a.area() + b.area() - interArea;

    return (unionArea > 0.0f) ? interArea / unionArea : 0.0f;
}

std::vector<cv::Rect2f> loadYoloLabels(const std::string& label_path, int img_w, int img_h) {
    std::ifstream in(label_path);
    std::vector<cv::Rect2f> boxes;
    if (!in.is_open()) return boxes;

    int class_id;
    YoloLabel lbl;
    while (in >> class_id >> lbl.cx >> lbl.cy >> lbl.w >> lbl.h) {
        boxes.push_back(yoloToRect(lbl, img_w, img_h));
    }
    return boxes;
}

std::pair<cv::Rect2f, int> getPredictedBox(const cv::Mat& frame, glare_position& gp, glare_detector& gd) {
    cv::Mat gphoto = gd.computePhotometricMap(frame);
    cv::Mat ggeo = gd.computeGeometricMap(gphoto);
    cv::Mat priority = gd.computePriorityMap(gphoto, ggeo);
    cv::Mat combined = gd.combineMapsbyprod(gphoto, ggeo);

    auto extractBox = [&](int target_priority) -> cv::Rect2f {
        cv::Mat mask = (priority == target_priority);
        cv::Mat masked;
        combined.copyTo(masked, mask);

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
        return cv::boundingRect(maxContour);
    };

    cv::Rect2f box1 = extractBox(1);
    if (box1.area() > 500.0f && box1.area() < 10000.0f) return { box1, 1 };

    cv::Rect2f box2 = extractBox(2);
    if (box2.area() > 500.0f && box2.area() < 10000.0f) return { box2, 2 };

    return { cv::Rect2f(), 0 };
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
    for (float t = 0.5f; t <= 0.96f; t += 0.05f) iou_thresholds.push_back(t);

    std::vector<std::pair<std::string, cv::Mat>> results;
    std::vector<std::string> image_names;

    for (const auto& entry : fs::directory_iterator(img_dir)) {
        std::string img_path = entry.path().string();
        std::string fname = entry.path().stem().string();
        image_names.push_back(fname);

        std::string label_path = label_dir + "/" + fname + ".txt";
        cv::Mat orig = cv::imread(img_path);
        if (orig.empty()) continue;

        cv::Mat frame;
        cv::resize(orig, frame, cv::Size(RESIZED_WIDTH, RESIZED_HEIGHT));

        auto gt_boxes = loadYoloLabels(label_path, orig.cols, orig.rows);

        std::vector<cv::Rect2f> resized_gt_boxes;
        float x_scale = static_cast<float>(RESIZED_WIDTH) / orig.cols;
        float y_scale = static_cast<float>(RESIZED_HEIGHT) / orig.rows;
        for (const auto& box : gt_boxes) {
            resized_gt_boxes.emplace_back(
                box.x * x_scale,
                box.y * y_scale,
                box.width * x_scale,
                box.height * y_scale
            );
        }

        auto [pred_box, priority_type] = getPredictedBox(frame, gp, gd);

        bool gt_has_glare = !resized_gt_boxes.empty();
        bool pred_has_glare = (pred_box.area() > 1.0);

        if (gt_has_glare && pred_has_glare) {
            float best_iou = 0.0f;
            for (const auto& gt : resized_gt_boxes) {
                best_iou = std::max(best_iou, computeIoU(pred_box, gt));
            }
            if (best_iou >= 0.5f) {
                TP++;
                total_tp_iou++;
                total_iou += best_iou;
            } else {
                FP++;
            }
            for (float t : iou_thresholds) {
                if (best_iou >= t) ap_tp[t]++;
                else ap_fp[t]++;
            }
        } else if (!gt_has_glare && pred_has_glare) {
            FP++;
            for (float t : iou_thresholds) ap_fp[t]++;
        } else if (gt_has_glare && !pred_has_glare) {
            FN++;
        } else {
            TN++;
        }

        total_images++;

        for (const auto& gt : resized_gt_boxes) {
            cv::rectangle(frame, gt, cv::Scalar(0, 0, 255), 2);
        }
        if (pred_has_glare) {
            cv::Scalar color = (priority_type == 1) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
            cv::rectangle(frame, pred_box, color, 2);
        }

        std::string text = "GT: " + std::string(gt_has_glare ? "Yes" : "No") + ", Pred: " + std::string(pred_has_glare ? "Yes" : "No");
        cv::putText(frame, text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        results.emplace_back(fname, frame);
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

    int idx = 0;
    while (true) {
        const auto& [name, img] = results[idx];
        cv::imshow("Result", img);
        std::cout << "Viewing: " << name << " - Press LEFT/RIGHT to navigate, ESC to exit." << std::endl;
        int key = cv::waitKey(0);
        if (key == 27) break;
        else if (key == 81 || key == 2424832) idx = (idx - 1 + results.size()) % results.size(); // left arrow
        else if (key == 83 || key == 2555904) idx = (idx + 1) % results.size(); // right arrow
    }

    return 0;
}
