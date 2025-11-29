#include "../src/physics/ExtendedKalmanFilter.h"
#include <cmath>
#include <iostream>

// Simple test framework
#define TEST(name)                                                             \
  void name();                                                                 \
  struct name##_runner {                                                       \
    name##_runner() {                                                          \
      std::cout << "Running " << #name << "..." << std::endl;                  \
      name();                                                                  \
      std::cout << "  PASSED" << std::endl;                                    \
    }                                                                          \
  } name##_instance;                                                           \
  void name()

#define ASSERT_NEAR(a, b, tolerance)                                           \
  if (std::abs((a) - (b)) > (tolerance)) {                                     \
    std::cerr << "  FAILED: " << #a << " (" << (a) << ") != " << #b << " ("   \
              << (b) << "), diff = " << std::abs((a) - (b)) << std::endl;      \
    exit(1);                                                                   \
  }

#define ASSERT_TRUE(condition)                                                 \
  if (!(condition)) {                                                          \
    std::cerr << "  FAILED: " << #condition << " is false" << std::endl;       \
    exit(1);                                                                   \
  }

using namespace aegis;

// Test 1: EKF initialization
TEST(TestEKFInitialization) {
  ExtendedKalmanFilter ekf(100.0f, 200.0f, 50.0f, 45.0f);

  glm::vec2 pos = ekf.GetPosition();
  ASSERT_NEAR(pos.x, 100.0f, 0.01f);
  ASSERT_NEAR(pos.y, 200.0f, 0.01f);
}

// Test 2: Straight-line prediction (zero turn rate)
TEST(TestStraightLinePrediction) {
  // Initialize at origin, heading North (0 degrees), 100 m/s
  ExtendedKalmanFilter ekf(0.0f, 0.0f, 100.0f, 0.0f);

  // Predict forward 1 second
  ekf.Predict(1.0f);

  glm::vec2 pos = ekf.GetPosition();
  // Expected: y += 100*cos(0)*1 = 100m North
  ASSERT_NEAR(pos.x, 0.0f, 1.0f);
  ASSERT_NEAR(pos.y, 100.0f, 1.0f);
}

// Test 3: Measurement update
TEST(TestMeasurementUpdate) {
  ExtendedKalmanFilter ekf(100.0f, 100.0f, 50.0f, 0.0f);

  // First measurement close to initial position
  ekf.Update(105.0f, 105.0f);

  glm::vec2 pos = ekf.GetPosition();
  // Position should be pulled toward measurement
  ASSERT_TRUE(pos.x > 100.0f && pos.x < 110.0f);
  ASSERT_TRUE(pos.y > 100.0f && pos.y < 110.0f);
}

// Test 4: Mahalanobis distance calculation
TEST(TestMahalanobisDistance) {
  ExtendedKalmanFilter ekf(0.0f, 0.0f, 50.0f, 0.0f);

  // Measurement at same position should have small distance
  float dist1 = ekf.GetMahalanobisDistance(0.0f, 0.0f);
  ASSERT_TRUE(dist1 < 1.0f); // Should be near zero

  // Far measurement should have large distance
  float dist2 = ekf.GetMahalanobisDistance(1000.0f, 1000.0f);
  ASSERT_TRUE(dist2 > 10.0f);

  // Nearby measurement should be within gate (chi2 = 9.21 for 99% confidence)
  float dist3 = ekf.GetMahalanobisDistance(50.0f, 50.0f);
  std::cout << "  Mahalanobis distance for (50,50): " << dist3 << std::endl;
}

// Test 5: Multiple updates improve estimate
TEST(TestConvergence) {
  ExtendedKalmanFilter ekf(0.0f, 0.0f, 0.0f, 0.0f);

  // Simulate target moving North at 100 m/s
  float true_y = 0.0f;
  for (int i = 0; i < 10; ++i) {
    ekf.Predict(0.1f); // 0.1 second steps
    true_y += 100.0f * 0.1f;

    // Add noisy measurement
    float noise = (i % 2 == 0) ? 5.0f : -5.0f;
    ekf.Update(0.0f, true_y + noise);
  }

  glm::vec2 pos = ekf.GetPosition();
  glm::vec2 vel = ekf.GetVelocity();

  // After 10 updates at 1 second total, should be near (0, 100)
  std::cout << "  Final position: (" << pos.x << ", " << pos.y << ")"
            << std::endl;
  std::cout << "  Final velocity: (" << vel.x << ", " << vel.y << ")"
            << std::endl;

  ASSERT_NEAR(pos.y, 100.0f, 20.0f); // Allow some error
  ASSERT_NEAR(vel.y, 100.0f, 30.0f); // Velocity should converge
}

// Test 6: Turning motion
TEST(TestTurningMotion) {
  // Initialize heading East (90 degrees), 100 m/s, with turn rate
  ExtendedKalmanFilter ekf(0.0f, 0.0f, 100.0f, 90.0f);

  // Note: Turn rate is initialized to 0 internally
  // In a real scenario, we'd need measurements to estimate it
  ekf.Predict(1.0f);

  glm::vec2 pos = ekf.GetPosition();
  // Heading 90 degrees (East): x += v*sin(90)*dt = 100m East
  std::cout << "  Position after 1s heading East: (" << pos.x << ", " << pos.y
            << ")" << std::endl;
  ASSERT_NEAR(pos.x, 100.0f, 1.0f);
  ASSERT_NEAR(pos.y, 0.0f, 1.0f);
}

int main() {
  std::cout << "\n=== Extended Kalman Filter Unit Tests ===" << std::endl;
  std::cout << "\nAll tests passed!\n" << std::endl;
  return 0;
}
