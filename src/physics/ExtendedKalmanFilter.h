#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace aegis {

class ExtendedKalmanFilter {
public:
  // State: [x, y, v, heading, turn_rate]
  ExtendedKalmanFilter(float initialX, float initialY, float initialV,
                       float initialHeading);

  void Predict(float dt);
  void Update(float measX, float measY);

  glm::vec4 GetState() const; // Returns [x, y, vx, vy] for compatibility
  glm::vec2 GetPosition() const;
  glm::vec2 GetVelocity() const;

  // Mahalanobis distance for gating (sensor fusion best practice)
  float GetMahalanobisDistance(float measX, float measY) const;

private:
  // 5D State vector: x, y, v, heading, turn_rate
  // We'll use glm::vec<5, float> if available, but GLM usually supports up to
  // vec4. We can use a custom struct or just an array, or a larger matrix.
  // Let's use Eigen if we had it, but we don't.
  // We'll use manual arrays or a custom Matrix class, or just map it to
  // glm::mat<5,1> if possible? GLM doesn't support vec5. We will implement a
  // simple 5x5 matrix / 5x1 vector wrapper or just use raw arrays for the math.
  // Actually, for simplicity and readability, let's use a struct for State and
  // helper functions for matrix math. Or, since we only need 5x5, we can use an
  // array `float m_x[5]` and `float m_P[25]`.

  // State
  float m_x[5];
  // Covariance
  float m_P[25]; // 5x5 row-major

  // Process Noise
  float m_Q[25];

  // Measurement Noise
  float m_R[4]; // 2x2 for x,y measurements

  void MatrixMultiply(const float *A, const float *B, float *C, int r1, int c1,
                      int c2) const;
  void MatrixTranspose(const float *A, float *AT, int rows, int cols) const;
  void MatrixAdd(const float *A, const float *B, float *C, int rows,
                 int cols) const;
  void MatrixSubtract(const float *A, const float *B, float *C, int rows,
                      int cols) const;
  bool MatrixInverse2x2(const float *A, float *invA)
      const; // Only need 2x2 inverse for S
};

} // namespace aegis
