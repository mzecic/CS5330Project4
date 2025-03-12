/*
Matej Zecic
CS5330
Spring 2025
This file performs camera pose estimation using a calibrated camera.
*/

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    // Load camera calibration parameters from file
    cv::FileStorage fs("camera_calibration.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open camera calibration file" << std::endl;
        return -1;
    }

    // Read calibration parameters
    cv::Mat camera_matrix;
    std::vector<double> distortion_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coeffs;
    fs.release();

    std::cout << "Loaded camera matrix:" << std::endl << camera_matrix << std::endl;
    std::cout << "Loaded distortion coefficients: ";
    for (const auto& coeff : distortion_coeffs) {
        std::cout << coeff << " ";
    }
    std::cout << std::endl;

    // Chessboard dimensions - same as used for calibration
    int boardWidth = 9;   // number of inner corners along width
    int boardHeight = 6;  // number of inner corners along height

    // Create 3D points of chessboard corners in world coordinates
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardHeight; i++) {
        for (int j = 0; j < boardWidth; j++) {
            objectPoints.push_back(cv::Point3f(j, -i, 0.0f));
        }
    }

    // Open video capture
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open the video stream" << std::endl;
        return -1;
    }

    cv::namedWindow("Camera Pose Estimation", cv::WINDOW_AUTOSIZE);

    // Variables for pose estimation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    // Variables for visualization
    cv::Mat frame, gray;
    bool lastPatternFound = false;

    for (;;) {
        // Read a frame from the video stream
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Cannot read a frame from the video stream" << std::endl;
            break;
        }

        // Convert to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Find chessboard corners
        std::vector<cv::Point2f> corners;
        bool patternFound = cv::findChessboardCorners(gray, cv::Size(boardWidth, boardHeight), corners,
                                cv::CALIB_CB_ADAPTIVE_THRESH +
                                cv::CALIB_CB_NORMALIZE_IMAGE +
                                cv::CALIB_CB_FAST_CHECK);

        if (patternFound != lastPatternFound) {
            std::cout << "Pattern found: " << (patternFound ? "YES" : "NO") << std::endl;
            lastPatternFound = patternFound;
        }

        if (patternFound) {
            // Refine corner positions for better accuracy
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // Solve for camera pose
            bool success = cv::solvePnP(objectPoints, corners, camera_matrix, distortion_coeffs,
                                       rotation_vector, translation_vector);

            if (success) {
                // Print camera pose
                std::cout << "Translation (x,y,z): ["
                    << translation_vector.at<double>(0) << ", "
                    << translation_vector.at<double>(1) << ", "
                    << translation_vector.at<double>(2) << "] units" << std::endl;

                std::cout << "Rotation Vector: ["
                    << rotation_vector.at<double>(0) << ", "
                    << rotation_vector.at<double>(1) << ", "
                    << rotation_vector.at<double>(2) << "] radians" << std::endl;

                // Convert rotation vector to Euler angles (in degrees)
                cv::Mat rotation_matrix;
                cv::Rodrigues(rotation_vector, rotation_matrix);

                // Extract Euler angles
                double roll, pitch, yaw;
                // Decompose rotation matrix to get Euler angles
                // Note: This is a simplification. More robust methods exist for all rotation cases
                pitch = -asin(rotation_matrix.at<double>(2,0));
                yaw = atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));
                roll = atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));

                // Convert to degrees
                roll = roll * 180.0 / CV_PI;
                pitch = pitch * 180.0 / CV_PI;
                yaw = yaw * 180.0 / CV_PI;

                std::cout << "Rotation (roll,pitch,yaw): ["
                    << roll << ", " << pitch << ", " << yaw << "] degrees" << std::endl;

                // Draw coordinate axes on the chessboard
                std::vector<cv::Point3f> axisPoints;
                axisPoints.push_back(cv::Point3f(0, 0, 0));  // Origin
                axisPoints.push_back(cv::Point3f(3, 0, 0));  // X axis (red)
                axisPoints.push_back(cv::Point3f(0, 3, 0));  // Y axis (green)
                axisPoints.push_back(cv::Point3f(0, 0, 3));  // Z axis (blue)

                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(axisPoints, rotation_vector, translation_vector,
                                camera_matrix, distortion_coeffs, imagePoints);

                // Draw the axes
                cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X-axis: Red
                cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y-axis: Green
                cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z-axis: Blue

                // Display pose information on the frame
                std::string positionText = "Position: [" +
                    std::to_string(translation_vector.at<double>(0)).substr(0, 5) + ", " +
                    std::to_string(translation_vector.at<double>(1)).substr(0, 5) + ", " +
                    std::to_string(translation_vector.at<double>(2)).substr(0, 5) + "]";

                std::string rotationText = "Rotation: [" +
                    std::to_string(roll).substr(0, 5) + ", " +
                    std::to_string(pitch).substr(0, 5) + ", " +
                    std::to_string(yaw).substr(0, 5) + "]";

                cv::putText(frame, positionText, cv::Point(10, 30),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
                cv::putText(frame, rotationText, cv::Point(10, 60),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
            }

            // Draw the corners for visualization
            cv::drawChessboardCorners(frame, cv::Size(boardWidth, boardHeight), corners, patternFound);
        }

        // Display the frame
        cv::imshow("Camera Pose Estimation", frame);

        // Check for exit
        if (cv::waitKey(30) == 27)  // ESC key
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
