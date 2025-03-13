# AR Camera Pose Estimation Project

## Author

Matej Zecic

## Demo Video

[Watch the Demo Video for Simple Object Projection ](https://youtu.be/qV6FFnrxptQ)
[Watch the Demo Video for Feature Detection ](https://youtu.be/p7Q5biFXt38)
[Watch the Demo Video for Village ](https://youtu.be/iC1YiKxkqXs)

## Development Environment

- **Operating System**: macOS
- **IDE**: Visual Studio Code
- **Compiler**: g++ (Apple clang version)
- **Required Libraries**: OpenCV 4.11.0

## Project Overview

This project implements camera pose estimation and augmented reality visualization using OpenCV. The application detects a chessboard pattern using a webcam, calculates the camera's position and orientation in 3D space, and renders virtual 3D objects on top of the real-world scene.

## Features

- **Camera Calibration**: Uses pre-calibrated camera parameters from an XML file
- **Real-time Pose Estimation**: Calculates camera position and orientation in real-time
- **Coordinate System Visualization**: Displays X, Y, Z axes on the detected chessboard
- **Two Rendering Modes**:
  - **Simple Wireframe Mode**: Displays basic 3D shapes with wireframes
  - **Complex Village Scene**: Renders a detailed 3D village with buildings, trees, and a pond

## How It Works

### Camera Pose Estimation

1. The program captures video from the webcam and looks for a chessboard pattern
2. When a pattern is detected, it calculates the camera's position and orientation using OpenCV's solvePnP
3. The position and rotation are displayed as text on the screen, and coordinate axes are overlaid on the chessboard

### 3D Rendering

The program uses the calculated pose to render 3D objects as if they were in the real world:

1. **Wireframe Mode ('d' key)**:

   - Simple pyramid and house structures rendered as wireframe outlines
   - Objects are positioned relative to the chessboard

2. **Village Mode ('v' key)**:
   - Complex 3D environment with filled polygons
   - Includes a central tower, multiple houses, trees, and a pond
   - Uses back-face culling to properly handle object occlusion
   - Objects are colored to create a realistic village scene

### Technical Implementation

- 3D objects are defined using vertices, edges, and triangular faces
- Objects are transformed to world coordinates based on the chessboard position
- The camera's projection matrix is used to map 3D points onto the 2D image
- Filled polygons are drawn with proper depth ordering

## Instructions

### Running the Application

1. Ensure OpenCV is installed and properly linked in your build environment
2. Navigate to the project directory
3. Build the project:
   ```
   cd src
   make
   ```
4. Run the executable:
   ```
   ./bin/cameraUtil
   ```

### Usage

1. Print the chessboard pattern (9×6 inner corners) and place it in the camera's view
2. When the chessboard is detected, 3D objects will appear on screen
3. Use keyboard controls:
   - Press 'd' to switch to simple wireframe mode
   - Press 'v' to switch to village scene mode
   - Press 'ESC' to exit the program

### Testing Extensions

1. **Complex Village Rendering**:

   - Press 'v' to activate the village scene
   - Move the chessboard to see the village from different angles
   - Observe how all buildings and objects maintain proper 3D positioning

2. **Back-face Culling**:

   - In the source code, modify the `drawComplexObject` function
   - Change `faceVisible[i] = true;` to `faceVisible[i] = (crossZ < 0);`
   - Recompile and observe how only visible faces are rendered as you rotate the scene

3. **Feature Detection Modes**:

   ```
   cd src
   make
   ```

   ```
   ../bin/featureDetection
   ```

   - press '-' to decrease threshold
   - press '+' to increase threshold
   - press 't' to cycle between 'harris', 'fast', or 'orb'

## Project Structure

- **cameraUtil.cpp**: Main source file containing all functionality
- **camera_calibration.xml**: Pre-calibrated camera parameters
- **createComplexVirtualScene()**: Function that builds the 3D village model
- **drawComplexObject()**: Function that renders the 3D models with filled faces

## Implementation Notes

- The application requires a pre-calibrated camera (parameters stored in camera_calibration.xml)
- The chessboard pattern should have 9×6 inner corners
- Performance depends on camera quality and lighting conditions

## Time Travel Days

Not using any time travel days.
