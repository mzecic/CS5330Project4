/*
Matej Zecic
CS5330
Spring 2025
This file performs camera pose estimation using a calibrated camera.
*/

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

// Create a complex 3D model like a small village or landscape
void createComplexVirtualScene(std::vector<cv::Point3f>& vertices,
                             std::vector<std::pair<int, int>>& edges,
                             std::vector<std::tuple<int, int, int, cv::Scalar>>& faces);

// Draw 3D model with filled faces for more realistic appearance
void drawComplexObject(cv::Mat& image,
                      const std::vector<cv::Point2f>& projectedPoints,
                      const std::vector<std::pair<int, int>>& edges,
                      const std::vector<std::tuple<int, int, int, cv::Scalar>>& faces,
                      const cv::Mat& rotMatrix);

int main(int argc, char** argv)
{
    // Keypress variable for various actions
    int key;
    std::string mode;

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
        key = cv::waitKey(30);

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

                // Process keypress for mode change
                if (key == 'd') {
                    mode = "draw";
                    std::cout << "Mode: Simple 3D objects" << std::endl;
                } else if (key == 'v') {
                    mode = "village";
                    std::cout << "Mode: Complex village" << std::endl;
                }

                // Draw based on selected mode
                if (mode == "draw") {
                    // Define vertices relative to origin (0, 0, 0)
                    std::vector<cv::Point3f> objectVertices = {
                        cv::Point3f(0, 0, 0),
                        cv::Point3f(2, 0, 0),
                        cv::Point3f(2, -2, 0),
                        cv::Point3f(0, -2, 0),
                        cv::Point3f(1, -1, 3)
                    };

                    // Define edges as pairs of vertex indices
                    std::vector<std::pair<int, int>> objectEdges = {
                        {0, 1}, {1, 2}, {2, 3}, {3, 0},
                        {0, 4}, {1, 4}, {2, 4}, {3, 4}
                    };

                    float xOffset = 4.0f;
                    float yOffset = -3.0f;
                    float height = 2.0f;

                    // Transform vertices to be at right position
                    std::vector<cv::Point3f> worldVertices;
                    for (const auto &vertex : objectVertices) {
                        worldVertices.push_back(cv::Point3f(vertex.x + xOffset, vertex.y + yOffset, vertex.z + height));
                    }

                    // Project vertices to image space
                    cv::projectPoints(worldVertices, rotation_vector, translation_vector,
                                     camera_matrix, distortion_coeffs, imagePoints);

                    // Draw edges
                    for (const auto &edge : objectEdges) {
                        int start = edge.first;
                        int end = edge.second;

                        // Make sure indices are valid
                        if (start < imagePoints.size() && end < imagePoints.size()) {
                            line(frame, imagePoints[start], imagePoints[end], cv::Scalar(0, 255, 255), 2);
                        }
                    }

                    // Draw house
                    // Define house vertices relative to origin
                    std::vector<cv::Point3f> houseVertices = {
                        cv::Point3f(0, 0, 0),       // base - front left
                        cv::Point3f(2, 0, 0),       // base - front right
                        cv::Point3f(2, -2, 0),      // base - back right
                        cv::Point3f(0, -2, 0),      // base - back left
                        cv::Point3f(0, 0, 2),       // top - front left
                        cv::Point3f(2, 0, 2),       // top - front right
                        cv::Point3f(2, -2, 2),      // top - back right
                        cv::Point3f(0, -2, 2),      // top - back left
                        cv::Point3f(1, 0, 3),       // roof - front middle
                        cv::Point3f(1, -2, 3)       // roof - back middle
                    };

                    // Define house edges
                    std::vector<std::pair<int, int>> houseEdges = {
                        {0, 1}, {1, 2}, {2, 3}, {3, 0},  // base square
                        {4, 5}, {5, 6}, {6, 7}, {7, 4},  // top square
                        {0, 4}, {1, 5}, {2, 6}, {3, 7},  // vertical edges
                        {4, 8}, {5, 8}, {7, 9}, {6, 9},  // to roof peaks
                        {8, 9}                           // roof ridge
                    };

                    // Position the house to the right of the pyramid
                    float houseXOffset = 8.0f;  // Further to the right than pyramid
                    float houseYOffset = -3.0f; // Same vertical position
                    float houseHeight = 0.0f;   // Sitting on the board

                    // Transform house vertices to world position
                    std::vector<cv::Point3f> worldHouseVertices;
                    for (const auto &vertex : houseVertices) {
                        worldHouseVertices.push_back(cv::Point3f(
                            vertex.x + houseXOffset,
                            vertex.y + houseYOffset,
                            vertex.z + houseHeight
                        ));
                    }

                    // Project house vertices to image space
                    std::vector<cv::Point2f> houseImagePoints;
                    cv::projectPoints(worldHouseVertices, rotation_vector, translation_vector,
                                    camera_matrix, distortion_coeffs, houseImagePoints);

                    // Draw house edges with a different color
                    for (const auto &edge : houseEdges) {
                        int start = edge.first;
                        int end = edge.second;

                        // Make sure indices are valid
                        if (start < houseImagePoints.size() && end < houseImagePoints.size()) {
                            // Use a different color for the house (e.g., cyan)
                            line(frame, houseImagePoints[start], houseImagePoints[end],
                                cv::Scalar(255, 255, 0), 2);
                        }
                    }

                    // Different colors for different parts
                    cv::Scalar roofColor(0, 165, 255);    // Orange
                    cv::Scalar wallColor(255, 255, 0);    // Cyan
                    cv::Scalar baseColor(0, 255, 128);    // Light green

                    // Example: Add a door
                    cv::Point2f doorBottom = (houseImagePoints[0] + houseImagePoints[1]) * 0.5f;
                    cv::Point2f doorTop = (houseImagePoints[4] + houseImagePoints[5]) * 0.5f;
                    doorTop.y = doorTop.y * 0.7f + doorBottom.y * 0.3f; // Door is 70% of wall height
                    cv::line(frame, doorBottom, doorTop, cv::Scalar(139, 69, 19), 2);

                    // Add mode text
                    cv::putText(frame, "Mode: Simple 3D objects (press 'v' for village)", cv::Point(10, 90),
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                }
                else if (mode == "village") {
                    // Define data structures for our complex scene
                    std::vector<cv::Point3f> sceneVertices;
                    std::vector<std::pair<int, int>> sceneEdges;
                    std::vector<std::tuple<int, int, int, cv::Scalar>> sceneFaces;

                    // Create the complex scene
                    createComplexVirtualScene(sceneVertices, sceneEdges, sceneFaces);

                    // Position the scene in 3D space
                    float sceneOffsetX = 4.0f;
                    float sceneOffsetY = -3.0f;
                    float sceneHeight = 0.0f;

                    // Transform scene vertices to world space
                    std::vector<cv::Point3f> worldSceneVertices;
                    for (const auto &vertex : sceneVertices) {
                        worldSceneVertices.push_back(cv::Point3f(
                            vertex.x + sceneOffsetX,
                            vertex.y + sceneOffsetY,
                            vertex.z + sceneHeight
                        ));
                    }

                    // Project scene vertices to image space
                    std::vector<cv::Point2f> sceneImagePoints;
                    cv::projectPoints(worldSceneVertices, rotation_vector, translation_vector,
                                    camera_matrix, distortion_coeffs, sceneImagePoints);

                    // Get rotation matrix for back-face culling
                    cv::Mat rotMatrix;
                    cv::Rodrigues(rotation_vector, rotMatrix);

                    // Draw the complex scene with filled faces
                    drawComplexObject(frame, sceneImagePoints, sceneEdges, sceneFaces, rotMatrix);

                    // Add mode text
                    cv::putText(frame, "Mode: Village Scene (press 'd' for wireframe objects)", cv::Point(10, 90),
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                }
            }

            // Draw the corners for visualization
            cv::drawChessboardCorners(frame, cv::Size(boardWidth, boardHeight), corners, patternFound);
        }

        // Display the frame
        cv::imshow("Camera Pose Estimation", frame);

        // Check for keypresses
        if (key == 27)  // ESC key
            break;
    }

    cv::destroyAllWindows();
    return 0;
}

void createComplexVirtualScene(std::vector<cv::Point3f>& vertices,
    std::vector<std::pair<int, int>>& edges,
    std::vector<std::tuple<int, int, int, cv::Scalar>>& faces) {
// Clear any existing data
vertices.clear();
edges.clear();
faces.clear();

// Base ground platform
float groundSize = 10.0f;
vertices.push_back(cv::Point3f(-groundSize/2, -groundSize/2, 0));  // 0: ground corner
vertices.push_back(cv::Point3f(groundSize/2, -groundSize/2, 0));   // 1: ground corner
vertices.push_back(cv::Point3f(groundSize/2, groundSize/2, 0));    // 2: ground corner
vertices.push_back(cv::Point3f(-groundSize/2, groundSize/2, 0));   // 3: ground corner

// Ground edges
edges.push_back(std::make_pair(0, 1));
edges.push_back(std::make_pair(1, 2));
edges.push_back(std::make_pair(2, 3));
edges.push_back(std::make_pair(3, 0));

// Ground face with a more natural grass color
cv::Scalar grassColor(50, 180, 50);  // Darker green for grass
faces.push_back(std::make_tuple(0, 1, 2, grassColor));
faces.push_back(std::make_tuple(0, 2, 3, grassColor));

// Add a path across the village
float pathWidth = 1.0f;
int pathBaseIndex = vertices.size();

// Path from left to right
vertices.push_back(cv::Point3f(-groundSize/2, -pathWidth/2, 0.01f));
vertices.push_back(cv::Point3f(groundSize/2, -pathWidth/2, 0.01f));
vertices.push_back(cv::Point3f(groundSize/2, pathWidth/2, 0.01f));
vertices.push_back(cv::Point3f(-groundSize/2, pathWidth/2, 0.01f));

// Path face with a brown/tan color
cv::Scalar pathColor(90, 140, 180);  // Tan/dirt color
faces.push_back(std::make_tuple(pathBaseIndex, pathBaseIndex + 1, pathBaseIndex + 2, pathColor));
faces.push_back(std::make_tuple(pathBaseIndex, pathBaseIndex + 2, pathBaseIndex + 3, pathColor));

// Add a central tower
int towerBaseIndex = vertices.size();
float towerSize = 2.0f;
float towerHeight = 6.0f;

// Tower base vertices
vertices.push_back(cv::Point3f(-towerSize/2, -towerSize/2, 0)); // 4: tower base
vertices.push_back(cv::Point3f(towerSize/2, -towerSize/2, 0));  // 5
vertices.push_back(cv::Point3f(towerSize/2, towerSize/2, 0));   // 6
vertices.push_back(cv::Point3f(-towerSize/2, towerSize/2, 0));  // 7

// Tower top vertices
vertices.push_back(cv::Point3f(-towerSize/2, -towerSize/2, towerHeight)); // 8: tower top
vertices.push_back(cv::Point3f(towerSize/2, -towerSize/2, towerHeight));  // 9
vertices.push_back(cv::Point3f(towerSize/2, towerSize/2, towerHeight));   // 10
vertices.push_back(cv::Point3f(-towerSize/2, towerSize/2, towerHeight));  // 11

// Tower edges - vertical
edges.push_back(std::make_pair(towerBaseIndex + 0, towerBaseIndex + 4)); // Vertical edges
edges.push_back(std::make_pair(towerBaseIndex + 1, towerBaseIndex + 5));
edges.push_back(std::make_pair(towerBaseIndex + 2, towerBaseIndex + 6));
edges.push_back(std::make_pair(towerBaseIndex + 3, towerBaseIndex + 7));

// Tower edges - base and top
edges.push_back(std::make_pair(towerBaseIndex + 0, towerBaseIndex + 1));
edges.push_back(std::make_pair(towerBaseIndex + 1, towerBaseIndex + 2));
edges.push_back(std::make_pair(towerBaseIndex + 2, towerBaseIndex + 3));
edges.push_back(std::make_pair(towerBaseIndex + 3, towerBaseIndex + 0));

edges.push_back(std::make_pair(towerBaseIndex + 4, towerBaseIndex + 5));
edges.push_back(std::make_pair(towerBaseIndex + 5, towerBaseIndex + 6));
edges.push_back(std::make_pair(towerBaseIndex + 6, towerBaseIndex + 7));
edges.push_back(std::make_pair(towerBaseIndex + 7, towerBaseIndex + 4));

// Tower faces with more vibrant stone colors
cv::Scalar stoneDark(90, 90, 110);
cv::Scalar stoneMid(110, 110, 130);
cv::Scalar stoneLight(130, 130, 150);

faces.push_back(std::make_tuple(towerBaseIndex + 0, towerBaseIndex + 1, towerBaseIndex + 5, stoneLight));
faces.push_back(std::make_tuple(towerBaseIndex + 0, towerBaseIndex + 5, towerBaseIndex + 4, stoneLight));
faces.push_back(std::make_tuple(towerBaseIndex + 1, towerBaseIndex + 2, towerBaseIndex + 6, stoneMid));
faces.push_back(std::make_tuple(towerBaseIndex + 1, towerBaseIndex + 6, towerBaseIndex + 5, stoneMid));
faces.push_back(std::make_tuple(towerBaseIndex + 2, towerBaseIndex + 3, towerBaseIndex + 7, stoneDark));
faces.push_back(std::make_tuple(towerBaseIndex + 2, towerBaseIndex + 7, towerBaseIndex + 6, stoneDark));
faces.push_back(std::make_tuple(towerBaseIndex + 3, towerBaseIndex + 0, towerBaseIndex + 4, stoneMid));
faces.push_back(std::make_tuple(towerBaseIndex + 3, towerBaseIndex + 4, towerBaseIndex + 7, stoneMid));

// Add a tower roof (pyramid)
int roofBaseIndex = vertices.size();
float roofHeight = 2.0f;

// Roof peak
vertices.push_back(cv::Point3f(0, 0, towerHeight + roofHeight)); // Roof peak

// Roof edges
edges.push_back(std::make_pair(towerBaseIndex + 4, roofBaseIndex));
edges.push_back(std::make_pair(towerBaseIndex + 5, roofBaseIndex));
edges.push_back(std::make_pair(towerBaseIndex + 6, roofBaseIndex));
edges.push_back(std::make_pair(towerBaseIndex + 7, roofBaseIndex));

// Roof faces with deep red color
cv::Scalar roofColor(30, 30, 150);  // Deep red
faces.push_back(std::make_tuple(towerBaseIndex + 4, towerBaseIndex + 5, roofBaseIndex, roofColor));
faces.push_back(std::make_tuple(towerBaseIndex + 5, towerBaseIndex + 6, roofBaseIndex, roofColor));
faces.push_back(std::make_tuple(towerBaseIndex + 6, towerBaseIndex + 7, roofBaseIndex, roofColor));
faces.push_back(std::make_tuple(towerBaseIndex + 7, towerBaseIndex + 4, roofBaseIndex, roofColor));

// Add a small house to the side
int houseBaseIndex = vertices.size();
float houseSize = 1.5f;
float houseHeight = 1.5f;
float houseX = 3.0f;
float houseY = 2.0f;

// House base vertices
vertices.push_back(cv::Point3f(houseX, houseY, 0));
vertices.push_back(cv::Point3f(houseX + houseSize, houseY, 0));
vertices.push_back(cv::Point3f(houseX + houseSize, houseY + houseSize, 0));
vertices.push_back(cv::Point3f(houseX, houseY + houseSize, 0));

// House top vertices
vertices.push_back(cv::Point3f(houseX, houseY, houseHeight));
vertices.push_back(cv::Point3f(houseX + houseSize, houseY, houseHeight));
vertices.push_back(cv::Point3f(houseX + houseSize, houseY + houseSize, houseHeight));
vertices.push_back(cv::Point3f(houseX, houseY + houseSize, houseHeight));

// House roof peak
vertices.push_back(cv::Point3f(houseX + houseSize/2, houseY + houseSize/2, houseHeight + 1.0f));

// House edges
for (int i = 0; i < 4; i++) {
// Base square
edges.push_back(std::make_pair(houseBaseIndex + i, houseBaseIndex + ((i + 1) % 4)));
// Top square
edges.push_back(std::make_pair(houseBaseIndex + i + 4, houseBaseIndex + ((i + 1) % 4) + 4));
// Vertical edges
edges.push_back(std::make_pair(houseBaseIndex + i, houseBaseIndex + i + 4));
// Roof edges
edges.push_back(std::make_pair(houseBaseIndex + i + 4, houseBaseIndex + 8));
}

// House faces (walls) - cottage style with wood/stone appearance
cv::Scalar cottageWall(200, 220, 240);  // Light beige/tan
cv::Scalar cottageRoof(50, 80, 120);    // Brown roof

for (int i = 0; i < 4; i++) {
int next = (i + 1) % 4;
// Wall faces
faces.push_back(std::make_tuple(
houseBaseIndex + i,
houseBaseIndex + next,
houseBaseIndex + next + 4,
cottageWall
));
faces.push_back(std::make_tuple(
houseBaseIndex + i,
houseBaseIndex + next + 4,
houseBaseIndex + i + 4,
cottageWall
));

// Roof faces
faces.push_back(std::make_tuple(
houseBaseIndex + i + 4,
houseBaseIndex + next + 4,
houseBaseIndex + 8,
cottageRoof
));
}

// Add a second house on the other side
int house2BaseIndex = vertices.size();
float house2X = -3.5f;
float house2Y = -2.5f;
float house2Size = 1.8f;
float house2Height = 1.2f;

// House base vertices
vertices.push_back(cv::Point3f(house2X, house2Y, 0));
vertices.push_back(cv::Point3f(house2X + house2Size, house2Y, 0));
vertices.push_back(cv::Point3f(house2X + house2Size, house2Y + house2Size, 0));
vertices.push_back(cv::Point3f(house2X, house2Y + house2Size, 0));

// House top vertices
vertices.push_back(cv::Point3f(house2X, house2Y, house2Height));
vertices.push_back(cv::Point3f(house2X + house2Size, house2Y, house2Height));
vertices.push_back(cv::Point3f(house2X + house2Size, house2Y + house2Size, house2Height));
vertices.push_back(cv::Point3f(house2X, house2Y + house2Size, house2Height));

// Second house roof is a simple flat roof
for (int i = 0; i < 4; i++) {
// Base square
edges.push_back(std::make_pair(house2BaseIndex + i, house2BaseIndex + ((i + 1) % 4)));
// Top square
edges.push_back(std::make_pair(house2BaseIndex + i + 4, house2BaseIndex + ((i + 1) % 4) + 4));
// Vertical edges
edges.push_back(std::make_pair(house2BaseIndex + i, house2BaseIndex + i + 4));
}

// House 2 faces - different color than first house
cv::Scalar house2Wall(150, 180, 220);  // Blueish
cv::Scalar house2Roof(80, 80, 80);     // Dark gray/black roof

for (int i = 0; i < 4; i++) {
int next = (i + 1) % 4;
// Wall faces
faces.push_back(std::make_tuple(
house2BaseIndex + i,
house2BaseIndex + next,
house2BaseIndex + next + 4,
house2Wall
));
faces.push_back(std::make_tuple(
house2BaseIndex + i,
house2BaseIndex + next + 4,
house2BaseIndex + i + 4,
house2Wall
));
}

// Flat roof (single face)
faces.push_back(std::make_tuple(
house2BaseIndex + 4,
house2BaseIndex + 5,
house2BaseIndex + 6,
house2Roof
));
faces.push_back(std::make_tuple(
house2BaseIndex + 4,
house2BaseIndex + 6,
house2BaseIndex + 7,
house2Roof
));

// Add some trees to the scene
cv::Scalar trunkColor(50, 90, 120);  // Brown trunk
cv::Scalar leafColor(30, 160, 30);   // Dark green leaves

// Add 3 trees at different positions
float treePositions[][2] = {
{-3.5f, 3.0f},
{4.0f, -2.0f},
{2.0f, -4.0f}
};

for (int t = 0; t < 3; t++) {
float treeX = treePositions[t][0];
float treeY = treePositions[t][1];

int treeBaseIndex = vertices.size();

// Tree trunk (cube)
float trunkWidth = 0.3f;
float trunkHeight = 1.0f;

// Trunk base
vertices.push_back(cv::Point3f(treeX - trunkWidth/2, treeY - trunkWidth/2, 0));
vertices.push_back(cv::Point3f(treeX + trunkWidth/2, treeY - trunkWidth/2, 0));
vertices.push_back(cv::Point3f(treeX + trunkWidth/2, treeY + trunkWidth/2, 0));
vertices.push_back(cv::Point3f(treeX - trunkWidth/2, treeY + trunkWidth/2, 0));

// Trunk top
vertices.push_back(cv::Point3f(treeX - trunkWidth/2, treeY - trunkWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX + trunkWidth/2, treeY - trunkWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX + trunkWidth/2, treeY + trunkWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX - trunkWidth/2, treeY + trunkWidth/2, trunkHeight));

// Trunk edges
for (int i = 0; i < 4; i++) {
edges.push_back(std::make_pair(treeBaseIndex + i, treeBaseIndex + ((i + 1) % 4)));
edges.push_back(std::make_pair(treeBaseIndex + i + 4, treeBaseIndex + ((i + 1) % 4) + 4));
edges.push_back(std::make_pair(treeBaseIndex + i, treeBaseIndex + i + 4));
}

// Trunk faces
for (int i = 0; i < 4; i++) {
int next = (i + 1) % 4;
faces.push_back(std::make_tuple(
treeBaseIndex + i,
treeBaseIndex + next,
treeBaseIndex + next + 4,
trunkColor
));
faces.push_back(std::make_tuple(
treeBaseIndex + i,
treeBaseIndex + next + 4,
treeBaseIndex + i + 4,
trunkColor
));
}

// Tree leaves (cone or pyramid)
int leavesBaseIndex = vertices.size();
float leavesWidth = 1.2f;
float leavesHeight = 1.8f;

// Leaves base vertices
vertices.push_back(cv::Point3f(treeX - leavesWidth/2, treeY - leavesWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX + leavesWidth/2, treeY - leavesWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX + leavesWidth/2, treeY + leavesWidth/2, trunkHeight));
vertices.push_back(cv::Point3f(treeX - leavesWidth/2, treeY + leavesWidth/2, trunkHeight));

// Leaves top vertex (peak)
vertices.push_back(cv::Point3f(treeX, treeY, trunkHeight + leavesHeight));

// Leaves edges
for (int i = 0; i < 4; i++) {
edges.push_back(std::make_pair(leavesBaseIndex + i, leavesBaseIndex + ((i + 1) % 4)));
edges.push_back(std::make_pair(leavesBaseIndex + i, leavesBaseIndex + 4));
}

// Leaves faces
for (int i = 0; i < 4; i++) {
int next = (i + 1) % 4;
faces.push_back(std::make_tuple(
leavesBaseIndex + i,
leavesBaseIndex + next,
leavesBaseIndex + 4,
leafColor
));
}
// Base of leaves
faces.push_back(std::make_tuple(
leavesBaseIndex + 0,
leavesBaseIndex + 1,
leavesBaseIndex + 2,
leafColor
));
faces.push_back(std::make_tuple(
leavesBaseIndex + 0,
leavesBaseIndex + 2,
leavesBaseIndex + 3,
leafColor
));
}

// Add a small pond
int pondBaseIndex = vertices.size();
float pondX = -2.0f;
float pondY = 0.5f;
float pondRadius = 1.2f;
int pondSegments = 8;

// Center vertex of the pond
vertices.push_back(cv::Point3f(pondX, pondY, 0.01f));  // Slightly above ground

// Perimeter vertices
for (int i = 0; i < pondSegments; i++) {
float angle = 2.0f * CV_PI * i / pondSegments;
vertices.push_back(cv::Point3f(
pondX + pondRadius * cos(angle),
pondY + pondRadius * sin(angle),
0.01f
));
}

// Pond edges
for (int i = 0; i < pondSegments; i++) {
int next = (i + 1) % pondSegments;
edges.push_back(std::make_pair(pondBaseIndex + 1 + i, pondBaseIndex + 1 + next));
}

// Pond faces (triangles from center)
cv::Scalar waterColor(200, 140, 40);  // Blue for water
for (int i = 0; i < pondSegments; i++) {
int next = (i + 1) % pondSegments;
faces.push_back(std::make_tuple(
pondBaseIndex,
pondBaseIndex + 1 + i,
pondBaseIndex + 1 + next,
waterColor
));
}
}


// Draw 3D model with filled faces for more realistic appearance
void drawComplexObject(cv::Mat& image,
    const std::vector<cv::Point2f>& projectedPoints,
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<std::tuple<int, int, int, cv::Scalar>>& faces,
    const cv::Mat& rotMatrix) {
// First, determine which faces are visible (simple back-face culling)
std::vector<bool> faceVisible(faces.size(), false);

for (size_t i = 0; i < faces.size(); i++) {
int idx1 = std::get<0>(faces[i]);
int idx2 = std::get<1>(faces[i]);
int idx3 = std::get<2>(faces[i]);

if (idx1 >= projectedPoints.size() || idx2 >= projectedPoints.size() || idx3 >= projectedPoints.size()) {
continue;
}

cv::Point2f v1 = projectedPoints[idx2] - projectedPoints[idx1];
cv::Point2f v2 = projectedPoints[idx3] - projectedPoints[idx1];

// Cross product Z component for determining winding order
float crossZ = v1.x * v2.y - v1.y * v2.x;

// Draw all faces initially for debugging
// For proper backface culling, use: faceVisible[i] = (crossZ < 0);
faceVisible[i] = true; // Show all faces regardless of orientation
}

// Draw visible faces
for (size_t i = 0; i < faces.size(); i++) {
if (!faceVisible[i]) continue;

int idx1 = std::get<0>(faces[i]);
int idx2 = std::get<1>(faces[i]);
int idx3 = std::get<2>(faces[i]);
cv::Scalar color = std::get<3>(faces[i]);

if (idx1 >= projectedPoints.size() || idx2 >= projectedPoints.size() || idx3 >= projectedPoints.size()) {
continue;
}

// Create a filled triangle
std::vector<cv::Point> points;
points.push_back(cv::Point(projectedPoints[idx1]));
points.push_back(cv::Point(projectedPoints[idx2]));
points.push_back(cv::Point(projectedPoints[idx3]));

cv::fillConvexPoly(image, points, color);
}

// Draw edges (optional, for more defined appearance)
for (const auto& edge : edges) {
int start = edge.first;
int end = edge.second;

if (start < projectedPoints.size() && end < projectedPoints.size()) {
cv::line(image, projectedPoints[start], projectedPoints[end],
  cv::Scalar(0, 0, 0), 1);  // Black edges
}
}
}
