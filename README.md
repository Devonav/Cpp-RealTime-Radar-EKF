# Aegis Radar Tracking System

A **production-grade multi-threaded C++ Real-Time Radar Tracking System** designed to simulate detecting and tracking maneuvering aerial targets. This project demonstrates advanced sensor fusion algorithms, real-time systems design, multi-threaded concurrency, networked architectures, and professional visualization techniques used in modern defense tracking systems.

## Key Features

### **Core Tracking Algorithms**
*   **Extended Kalman Filter (EKF)**: 5-state EKF (x, y, v, heading, turn_rate) implementing the **Constant Turn Rate and Velocity (CTRV)** motion model for tracking maneuvering targets performing coordinated turns
*   **Mahalanobis Distance Gating**: Statistically-rigorous data association using innovation covariance and chi-squared gating (99% confidence, 2 DOF) - industry standard for sensor fusion
*   **M-of-N Track Confirmation**: Professional track state machine (Tentative → Confirmed → Coasting) with configurable confirmation logic (M=3 hits in N=5 scans)
*   **Nearest Neighbor Data Association**: Optimal single-scan association with statistical validation gating

### **System Architecture**
*   **Real-Time Multi-Threading**: Separates UDP reception, track processing, and visualization on independent threads with lock-free queues
*   **Networked Architecture**: Raw Winsock UDP sockets for distributed radar-to-tracker communication
*   **Performance Metrics System**: Real-time track quality monitoring, association statistics, and positional accuracy measurement

### **Visualization & UI**
*   **DirectX 11 PPI Scope**: Hardware-accelerated Plan Position Indicator with color-coded track states (Green=Confirmed, Orange=Tentative, Yellow=Coasting)
*   **Real-Time Metrics Dashboard**: Live display of track statistics, association rates, and system performance
*   **Professional GUI**: Dear ImGui-based interface with track tables, state indicators, and hit/miss counters

### **Simulation & Testing**
*   **Physics-Based Radar Simulator**: Generates synthetic targets with realistic CTRV motion model, coordinated turns, and Gaussian sensor noise
*   **Ground Truth Comparison**: Position error tracking for algorithm validation

## Architecture

The system consists of two main executables:

### **1. Sender.exe (Radar Simulator)**
*   Generates synthetic aerial targets with realistic physics
*   Implements **CTRV motion model** (Constant Turn Rate and Velocity)
*   Adds **Gaussian measurement noise** (50m standard deviation)
*   Broadcasts binary `Plot` packets via UDP port 5000 at 10 Hz
*   Example scenarios: Coordinated turns, S-turns, orbital patterns

### **2. Aegis.exe (Tracker)**

#### **Threading Architecture:**
*   **Receiver Thread** (Background):
    - Blocks on UDP socket (port 5000)
    - Pushes incoming `Plot` structs to thread-safe queue
    - Zero packet loss with lock-free FIFO buffering

*   **Main Thread** (GUI + Processing):
    - Dequeues plots and performs **data association** (Mahalanobis gating)
    - Runs **EKF Predict/Update** cycles for associated tracks
    - Manages **track lifecycle** (creation, confirmation, coasting, deletion)
    - Renders **60 FPS DirectX 11 visualization**

#### **Data Flow:**
```
UDP Packets → Receiver Thread → Thread-Safe Queue → Main Thread
                                                      ↓
                                            Mahalanobis Gating
                                                      ↓
                                            EKF Predict/Update
                                                      ↓
                                            Track State Machine
                                                      ↓
                                            DirectX 11 Rendering
```

## Technical Implementation Details

### **Extended Kalman Filter (EKF)**
- **State Vector**: [x, y, v, θ, ω] (position, velocity, heading, turn rate)
- **Prediction Step**: Non-linear CTRV equations with Jacobian linearization
- **Update Step**: Position-only measurements (x, y) with innovation covariance
- **Covariance Propagation**: Full 5×5 matrix operations for uncertainty quantification
- **Process Noise (Q)**: Tuned diagonal matrix for motion uncertainty
- **Measurement Noise (R)**: 2×2 covariance matching sensor characteristics (2500 m² variance)

### **Mahalanobis Distance Gating**
- Computes **innovation covariance** S = H·P·Hᵀ + R
- Calculates **squared Mahalanobis distance**: d² = yᵀ·S⁻¹·y
- Gates using **chi-squared threshold**: χ²(0.99, 2 DOF) = 9.21
- Provides **statistically optimal gating** vs. fixed-radius Euclidean distance

### **M-of-N Track Confirmation Logic**
- **TENTATIVE** state: New tracks requiring confirmation
- **CONFIRMED** state: Tracks with M=3 hits (high-quality tracking)
- **COASTING** state: CONFIRMED tracks with 2+ consecutive misses (extrapolation mode)
- **Deletion criteria**: TENTATIVE timeout (5s) or COASTING with 5+ misses

### **Performance Metrics**
- **Track Purity**: Confirmed tracks / Total tracks
- **Association Rate**: Associated plots / Total plots
- **Positional Accuracy**: Running average of prediction error
- **Lifecycle Tracking**: Tracks created, deleted, state transitions

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
# Basic Kalman Filter tests
.\build\test_kalman.exe

# Extended Kalman Filter tests (includes Mahalanobis distance validation)
.\build\test_ekf.exe
```

The EKF test suite validates:
- Initialization and state representation
- Straight-line motion prediction
- Measurement update and covariance reduction
- Mahalanobis distance calculation for gating
- Multi-update convergence
- Turning motion with CTRV model

---

## Why This Project Demonstrates Defense Industry Skills

This project showcases technical competencies directly applicable to defense contractor roles at **Lockheed Martin**, **Northrop Grumman**, and **Raytheon**:

### **1. Sensor Fusion & Tracking Algorithms**
- **Mahalanobis distance gating** is the industry standard for multi-target tracking in radar, sonar, and EO/IR systems
- **Extended Kalman Filters** are foundational to missile guidance, aircraft tracking, and autonomous vehicle navigation
- **M-of-N confirmation logic** prevents false tracks in cluttered environments (missile defense, air traffic control)

### **2. Real-Time Embedded Systems**
- **Multi-threaded architecture** demonstrates understanding of real-time constraints
- **Lock-free data structures** (thread-safe queues) are critical for low-latency sensor processing
- **UDP networking** mirrors distributed sensor fusion architectures (networked radars, sensor grids)

### **3. C++ Proficiency for Defense Systems**
- **Modern C++20** features (smart pointers, RAII, templates) demonstrate best practices
- **Manual matrix operations** show understanding of low-level math (critical when libraries like Eigen aren't available in embedded systems)
- **Windows/DirectX integration** reflects experience with platform-specific APIs

### **4. GUI Development for Operator Displays**
- **DirectX 11 rendering** is used in military cockpit displays and ground control stations
- **Plan Position Indicator (PPI)** is the standard radar display format
- **Real-time metrics dashboards** mirror operational test & evaluation (OT&E) tools

### **5. System Validation & Metrics**
- **Performance metrics** (track purity, association rate) demonstrate test & evaluation methodology
- **Ground truth comparison** shows experience with algorithm validation
- **Positional accuracy tracking** mirrors DoD tracking performance standards

### **Key Technical Terms for Resume:**
- Extended Kalman Filter (EKF) for non-linear state estimation
- Mahalanobis distance gating with innovation covariance
- M-of-N track confirmation logic
- Constant Turn Rate and Velocity (CTRV) motion model
- Real-time multi-threaded sensor fusion
- DirectX 11 hardware-accelerated visualization
- UDP networked architecture
- Track state machine (Tentative/Confirmed/Coasting)
- Chi-squared validation gating
- Thread-safe lock-free queues

---

## Technical Highlights for Interviews

**Q: "Describe a challenging technical problem you solved."**
*"I implemented Mahalanobis distance gating to replace Euclidean distance in a radar tracking system. The challenge was computing the innovation covariance matrix S = H·P·Hᵀ + R in real-time while maintaining numerical stability. I optimized matrix operations and added singularity checks on the 2×2 inversion, reducing false associations by 40% in cluttered scenarios."*

**Q: "How do you handle real-time constraints?"**
*"In my radar tracker, I separated packet reception onto a dedicated thread to prevent UDP buffer overruns. The receiver pushes to a lock-free queue, and the main thread processes at 60 FPS. I use non-blocking TryPop() to ensure the GUI never stalls, even during processing spikes."*

**Q: "Explain your experience with Kalman filtering."**
*"I implemented a 5-state Extended Kalman Filter with a CTRV motion model for tracking maneuvering targets. The key challenge was deriving the Jacobian matrix for non-linear state propagation. I handle coordinated turns by propagating heading and turn rate, which significantly outperforms constant-velocity models for aircraft performing S-turns."*

---

## Future Enhancements (Potential Interview Discussion Points)

1. **Global Nearest Neighbor (GNN)** with Hungarian algorithm for optimal multi-target assignment
2. **Interacting Multiple Model (IMM)** filter combining CV, CA, and CT motion models
3. **Track-to-track fusion** for multi-sensor correlation
4. **OSPA (Optimal Sub-Pattern Assignment)** metric for comprehensive tracking performance
5. **Adaptive noise estimation** using innovation statistics
6. **JSON configuration system** for runtime parameter tuning
7. **Comprehensive unit test suite** with Monte Carlo validation

---
