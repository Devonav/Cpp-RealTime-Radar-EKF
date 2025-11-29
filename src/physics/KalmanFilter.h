#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace aegis {

class KalmanFilter {
public:
  // State: [x, y, vx, vy]
  KalmanFilter(float initialX, float initialY);

  void Predict(float dt);
  void Update(float measX, float measY);

  glm::vec4 GetState() const { return m_x; }
  glm::vec2 GetPosition() const { return glm::vec2(m_x.x, m_x.y); }
  glm::vec2 GetVelocity() const { return glm::vec2(m_x.z, m_x.w); }

private:
  glm::vec4 m_x;   // State vector
  glm::mat4 m_P;   // State covariance
  glm::mat4 m_F;   // State transition matrix
  glm::mat4 m_Q;   // Process noise covariance
  glm::mat4x2 m_H; // Measurement matrix (2x4) - Transposed in GLM as 4x2? No,
                   // GLM is column-major. H maps 4 state vars to 2 measurements
                   // (x, y). H = [1 0 0 0]
                   //     [0 1 0 0]

  // GLM doesn't have non-square matrices easily accessible for multiplication
  // with vectors in the way we want sometimes. We'll use 4x4 for everything and
  // zero out rows/cols or just do manual math for the update step to be safe
  // and fast. Actually, let's stick to 4x4 for F, P, Q. For H, R, K, we'll
  // handle them carefully.

  glm::mat2 m_R; // Measurement noise covariance
};

} // namespace aegis
