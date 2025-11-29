#include "../src/physics/KalmanFilter.h"
#include <cassert>
#include <cmath>
#include <glm/glm.hpp>
#include <iostream>
#include <vector>


bool Check(float val, float expected, float epsilon = 0.001f) {
  return std::abs(val - expected) < epsilon;
}

void TestTracking() {
  std::cout << "Testing Tracking..." << std::endl;
  // Initial state: x=0, y=0
  aegis::KalmanFilter kf(0.0f, 0.0f);

  // Predict 1 second (should stay at 0,0 as init velocity is 0)
  kf.Predict(1.0f);
  glm::vec4 state = kf.GetState();

  if (Check(state.x, 0.0f) && Check(state.y, 0.0f)) {
    std::cout << "[PASS] Initial Prediction correct." << std::endl;
  } else {
    std::cout << "[FAIL] Initial Prediction: " << state.x << ", " << state.y
              << " Expected: 0.0, 0.0" << std::endl;
  }

  // Update with measurement at 10,0
  kf.Update(10.0f, 0.0f);
  state = kf.GetState();

  // Filter should move towards measurement (Kalman gain will determine how
  // much) It won't be exactly 10.0 immediately due to noise covariance
  std::cout << "State after update: " << state.x << ", " << state.y
            << std::endl;

  if (state.x > 0.0f && state.x <= 10.0f) {
    std::cout << "[PASS] Update moved towards measurement." << std::endl;
  } else {
    std::cout << "[FAIL] Update failed to move towards measurement."
              << std::endl;
  }

  // Predict again
  kf.Predict(1.0f);
  state = kf.GetState();
  std::cout << "State after 2nd prediction: " << state.x << ", " << state.y
            << std::endl;
}

int main() {
  TestTracking();
  std::cout << "All tests completed." << std::endl;
  return 0;
}
