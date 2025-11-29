# Aegis Radar Tracking System

A multi-threaded C++ Real-Time Radar Tracking System designed to simulate detecting and tracking aerial targets. This project demonstrates high-performance computing, real-time systems, concurrency, networking, and mathematical simulation.

## Key Features

*   **Real-Time Tracking**: Multi-threaded architecture separating UDP packet reception from track processing.
*   **Extended Kalman Filter (EKF)**: Implements a 5-state EKF (x, y, v, heading, turn_rate) to track maneuvering targets performing non-linear turns (CTRV model).
*   **Radar Simulator**: Generates synthetic targets with realistic physics, including coordinated turns and sensor noise.
*   **Data Association**: Uses Nearest Neighbor algorithm to associate incoming plots with existing tracks.
*   **DirectX 11 Visualization**: High-performance PPI (Plan Position Indicator) scope rendering using Dear ImGui and DirectX 11.
*   **Networked Architecture**: Uses raw Winsock UDP sockets for communication between the Simulator and Tracker.

## Architecture

The system consists of two main executables:

1.  **Sender.exe (Simulator)**:
    *   Simulates aerial targets.
    *   Applies a Constant Turn Rate and Velocity (CTRV) motion model.
    *   Adds Gaussian noise to measurements.
    *   Broadcasts `Plot` data structs via UDP port 5000.

2.  **Aegis.exe (Tracker)**:
    *   **Receiver Thread**: Listens on port 5000 and pushes raw packets to a thread-safe queue.
    *   **Processing Thread**: Pops packets, runs the Extended Kalman Filter (Predict/Update), and manages Track lifecycles (creation, pruning).
    *   **Main Thread (GUI)**: Renders the PPI scope, tracks, and history trails at 60 FPS.

## Build Instructions

### Prerequisites
*   Visual Studio 2022 (with C++ Desktop Development workload)
*   Windows SDK (for DirectX and Winsock)

### Building
1.  Open a terminal (Command Prompt or PowerShell).
2.  Navigate to the project directory.
3.  Run the build script:
    ```cmd
    .\build.bat
    ```
    This will compile `Aegis.exe`, `Sender.exe`, and `test_kalman.exe` into the `build/` directory.

## Running the System

1.  Start the Tracker:
    ```cmd
    .\build\Aegis.exe
    ```
    A window should appear with the radar scope.

2.  Start the Simulator (in a separate terminal):
    ```cmd
    .\build\Sender.exe
    ```
    You should see targets appearing on the Aegis scope.

## Testing
Run the unit tests:
```cmd
.\build\test_kalman.exe
```
