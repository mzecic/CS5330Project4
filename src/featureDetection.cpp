/*
Matej Zecic
CS5330
Spring 2025
This file demonstrates robust feature detection.
*/

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv)
{
    // Open video capture
    cv::VideoCapture cap("../images/japan.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open the video stream" << std::endl;
        return -1;
    }

    cv::namedWindow("Feature Detection", cv::WINDOW_AUTOSIZE);

    // Feature detection variables
    int detectorType = 0; // 0 = Harris, 1 = FAST, 2 = ORB
    std::string detectorNames[] = {"Harris", "FAST", "ORB"};
    int threshold = 100;

    std::cout << "Controls:" << std::endl;
    std::cout << "  't' - Switch detection method" << std::endl;
    std::cout << "  '+' - Increase threshold" << std::endl;
    std::cout << "  '-' - Decrease threshold" << std::endl;
    std::cout << "  ESC - Exit program" << std::endl;

    cv::Mat frame, gray;

    for (;;) {
        // Read a frame from the video stream
        cap >> frame;
        // Add after reading the frame:
        cv::resize(frame, frame, cv::Size(), 0.5, 0.5); // Resize to half the original size

        if (frame.empty()) {
            std::cerr << "Error: Cannot read a frame from the video stream" << std::endl;
            break;
        }

        // Convert to grayscale for feature detection
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Create a copy for visualization
        cv::Mat display = frame.clone();

        // Detect features based on selected method
        switch (detectorType) {
            case 0: // Harris corners
                {
                    cv::Mat harris_response;
                    cv::cornerHarris(gray, harris_response, 2, 3, 0.04);

                    // Normalize for visualization
                    cv::Mat harris_normalized;
                    cv::normalize(harris_response, harris_normalized, 0, 255, cv::NORM_MINMAX);
                    harris_normalized.convertTo(harris_normalized, CV_8U);

                    // Draw detected corners
                    for (int i = 0; i < harris_normalized.rows; i++) {
                        for (int j = 0; j < harris_normalized.cols; j++) {
                            if ((int)harris_normalized.at<uchar>(i, j) > threshold) {
                                cv::circle(display, cv::Point(j, i), 5, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                }
                break;

            case 1: // FAST features
                {
                    std::vector<cv::KeyPoint> keypoints;
                    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(threshold);
                    detector->detect(gray, keypoints);
                    cv::drawKeypoints(frame, keypoints, display, cv::Scalar(0, 255, 0),
                                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                }
                break;

            case 2: // ORB features
                {
                    std::vector<cv::KeyPoint> keypoints;
                    cv::Ptr<cv::ORB> detector = cv::ORB::create(500, 1.2f, 8, 31, 0, 2,
                                                                cv::ORB::HARRIS_SCORE, 31, threshold);
                    detector->detect(gray, keypoints);
                    cv::drawKeypoints(frame, keypoints, display, cv::Scalar(255, 0, 0),
                                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                }
                break;
        }

        // Add status text
        std::string statusText = "Detector: " + detectorNames[detectorType] +
                               " | Threshold: " + std::to_string(threshold);
        cv::putText(display, statusText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // Show the result
        cv::imshow("Feature Detection", display);

        // Check for keypresses
        int key = cv::waitKey(30);
        if (key == 27)  // ESC key
            break;
        else if (key == 't') {
            // Switch detector type
            detectorType = (detectorType + 1) % 3;
            std::cout << "Switched to " << detectorNames[detectorType] << " detector" << std::endl;
        }
        else if (key == '+' || key == '=') {
            threshold += 10;
            std::cout << "Threshold increased to " << threshold << std::endl;
        }
        else if (key == '-' || key == '_') {
            threshold = std::max(10, threshold - 10);
            std::cout << "Threshold decreased to " << threshold << std::endl;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
