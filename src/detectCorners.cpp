/*
Matej Zecic
CS5330
Spring 2025
This file is responsible for detecting corners on a chessboard.
*/

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::Mat frame;
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    cv::Mat lastSuccessfulImage;
    std::vector<cv::Point2f> lastSuccessfulCorners;
    bool hasSuccessfulDetection = false;
    bool calibrated = false;

    // Camera calibration variables
    cv::Mat camera_matrix;
    std::vector<double> distortion_coefficients;
    double reprojection_error = 0.0;
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;

    // Chessboard dimensions - a 10x7 board has 9x6 inner corners
    int boardWidth = 9;  // number of inner corners along width
    int boardHeight = 6; // number of inner corners along height

    std::cout << "Looking for a chessboard with " << boardWidth << "x" << boardHeight << " inner corners" << std::endl;
    std::cout << "Controls: " << std::endl;
    std::cout << "  's' - Save current chessboard detection for calibration" << std::endl;
    std::cout << "  'c' - Calibrate camera (need at least 5 saved images)" << std::endl;
    std::cout << "  'w' - Write calibration parameters to file" << std::endl;
    std::cout << "  ESC - Exit program" << std::endl;

    // Load the video stream
    cv::VideoCapture cap(0);
    // Check if the video stream is opened successfully
    if (!cap.isOpened())
    {
        std::cout << "Error: Cannot open the video stream" << std::endl;
        return -1;
    }

    // Create a window to display the video stream
    cv::namedWindow("Video Stream", cv::WINDOW_AUTOSIZE);

    for (;;) {
        // Read a frame from the video stream
        cap >> frame;

        // Check if the frame is empty
        if (frame.empty())
        {
            std::cout << "Error: Cannot read a frame from the video stream" << std::endl;
            break;
        }

        // Convert the frame to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect corners on the chessboard
        std::vector<cv::Point2f> corners;
        bool patternFound = cv::findChessboardCorners(gray, cv::Size(boardWidth, boardHeight), corners,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH +
                                                    cv::CALIB_CB_NORMALIZE_IMAGE +
                                                    cv::CALIB_CB_FAST_CHECK);

        // Only print pattern found status when it changes
        static bool lastPatternFound = false;
        if (patternFound != lastPatternFound) {
            std::cout << "Pattern found: " << (patternFound ? "YES" : "NO") << std::endl;
            lastPatternFound = patternFound;
        }

        if (patternFound) {
            // Refine corner positions for better accuracy
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                           cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // Save the corners for the last successful detection
            frame.copyTo(lastSuccessfulImage);
            lastSuccessfulCorners = corners;
            hasSuccessfulDetection = true;
        }

        // Draw the corners on the frame
        cv::drawChessboardCorners(frame, cv::Size(boardWidth, boardHeight), corners, patternFound);

        // Add status text to the frame
        std::string statusText = "Images: " + std::to_string(corner_list.size()) +
                               " | Press 's' to save, 'c' to calibrate, 'w' to write";
        cv::putText(frame, statusText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // Display the frame
        cv::imshow("Video Stream", frame);

        // Handle keyboard input
        int key = cv::waitKey(30);
        if (key == 27)
        {
            break;
        }
        else if (key == 's') {
            if (hasSuccessfulDetection) {
                std::vector<cv::Vec3f> point_set;

                for (int i = 0; i < boardHeight; i++) {
                    for (int j = 0; j < boardWidth; j++) {
                        point_set.push_back(cv::Vec3f(j, -i, 0.0));
                    }
                }
                // Add to calibration lists
                corner_list.push_back(lastSuccessfulCorners);
                point_list.push_back(point_set);

                // Save the calibration image
                std::string filename = "calibration" + std::to_string(corner_list.size()) + ".png";
                cv::imwrite(filename, lastSuccessfulImage);

                std::cout << "Added calibration image #" << corner_list.size() << ". Total: "
                << corner_list.size() << " images." << std::endl;

                // Save image with corners drawn on it
                cv::Mat cornersImage = lastSuccessfulImage.clone();
                cv::drawChessboardCorners(cornersImage, cv::Size(boardWidth, boardHeight),
                                        lastSuccessfulCorners, true);
                cv::imwrite("corners_" + std::to_string(corner_list.size()) + ".png", cornersImage);
            }
            else {
                std::cout << "No chessboard detected! Cannot save." << std::endl;
            }
        }
        else if (key == 'c' && corner_list.size() >= 5) {
            std::cout << "Calibrating camera with " << corner_list.size() << " images..." << std::endl;

            // Initialize camera matrix with guess of camera center
            camera_matrix = cv::Mat::eye(3, 3, CV_64F);
            camera_matrix.at<double>(0, 2) = frame.cols / 2.0;  // cx
            camera_matrix.at<double>(1, 2) = frame.rows / 2.0;  // cy

            // For testing without distortion first
            distortion_coefficients = std::vector<double>();

            // Print initial camera matrix
            std::cout << "\nInitial Camera Matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;

            // Perform calibration
            reprojection_error = cv::calibrateCamera(
                point_list,              // 3D points
                corner_list,             // 2D points
                lastSuccessfulImage.size(), // Image size
                camera_matrix,           // Output camera matrix
                distortion_coefficients, // Output distortion coefficients
                rotations,               // Output rotation vectors for each view
                translations,            // Output translation vectors for each view
                cv::CALIB_FIX_ASPECT_RATIO // Flags - assume square pixels
            );

            // Print results
            std::cout << "\nCalibrated Camera Matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;

            std::cout << "\nDistortion Coefficients:" << std::endl;
            for (const auto& coeff : distortion_coefficients) {
                std::cout << coeff << " ";
            }
            std::cout << std::endl;

            std::cout << "\nReprojection Error: " << reprojection_error << " pixels" << std::endl;

            calibrated = true;
        }
        else if (key == 'c' && corner_list.size() < 5) {
            std::cout << "Need at least 5 calibration images! Currently have: "
                     << corner_list.size() << std::endl;
        }
        else if (key == 'w') {
            if (calibrated) {
                // Save calibration parameters to a file
                cv::FileStorage fs("camera_calibration.xml", cv::FileStorage::WRITE);
                fs << "camera_matrix" << camera_matrix;
                fs << "distortion_coefficients" << distortion_coefficients;
                fs << "reprojection_error" << reprojection_error;
                fs.release();
                std::cout << "Calibration parameters saved to 'camera_calibration.xml'" << std::endl;
            } else {
                std::cout << "Please calibrate the camera first (press 'c')" << std::endl;
            }
        }
    }

    return 0;
}
