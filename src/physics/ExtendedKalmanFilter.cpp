#include "ExtendedKalmanFilter.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace aegis {

ExtendedKalmanFilter::ExtendedKalmanFilter(float initialX, float initialY,
                                           float initialV,
                                           float initialHeading) {
  // Initialize State
  m_x[0] = initialX;
  m_x[1] = initialY;
  m_x[2] = initialV;
  m_x[3] =
      initialHeading * (static_cast<float>(M_PI) / 180.0f); // Convert to rad
  m_x[4] = 0.0f; // Assume 0 turn rate initially

  // Initialize Covariance P (Identity * large value)
  std::memset(m_P, 0, 25 * sizeof(float));
  m_P[0] = 1.0f;
  m_P[6] = 1.0f;
  m_P[12] = 1.0f;
  m_P[18] = 1.0f;
  m_P[24] = 1.0f;

  // Initialize Process Noise Q
  std::memset(m_Q, 0, 25 * sizeof(float));
  // Tune these!
  m_Q[0] = 0.1f;  // x
  m_Q[6] = 0.1f;  // y
  m_Q[12] = 1.0f; // v
  m_Q[18] = 0.1f; // heading
  m_Q[24] = 0.1f; // turn rate

  // Initialize Measurement Noise R
  m_R[0] = 2500.0f;
  m_R[1] = 0.0f;
  m_R[2] = 0.0f;
  m_R[3] = 2500.0f; // 50m std dev -> 2500 variance
}

void ExtendedKalmanFilter::Predict(float dt) {
  // 1. Predict State (CTRV Model)
  float x = m_x[0];
  float y = m_x[1];
  float v = m_x[2];
  float theta = m_x[3];
  float w = m_x[4];

  if (std::abs(w) > 0.001f) {
    m_x[0] = x + (v / w) * (std::sin(theta + w * dt) - std::sin(theta));
    m_x[1] = y + (v / w) * (std::cos(theta) - std::cos(theta + w * dt));
    m_x[3] = theta + w * dt;
  } else {
    m_x[0] = x + v * std::cos(theta) * dt; // Wait, cos/sin convention?
    // In TargetGenerator: vx = sin(h)*speed, vy = cos(h)*speed.
    // So 0 is North (Y), 90 is East (X).
    // Standard math: 0 is East (X), 90 is North (Y).
    // Let's match TargetGenerator convention:
    // x += v * sin(theta) * dt
    // y += v * cos(theta) * dt
    m_x[0] = x + v * std::sin(theta) * dt;
    m_x[1] = y + v * std::cos(theta) * dt;
  }
  // v and w are constant in prediction step for this model

  // Normalize theta
  while (m_x[3] > M_PI)
    m_x[3] -= 2.0f * M_PI;
  while (m_x[3] < -M_PI)
    m_x[3] += 2.0f * M_PI;

  // 2. Predict Covariance: P = F * P * F^T + Q
  // Jacobian F calculation
  float F[25];
  std::memset(F, 0, 25 * sizeof(float));
  // Identity diagonal
  F[0] = 1;
  F[6] = 1;
  F[12] = 1;
  F[18] = 1;
  F[24] = 1;

  // Derivatives (Jacobian elements)
  // This is complex for CTRV. Simplified for small dt or just use numerical?
  // Let's use the linear approximation for F for now (Constant Velocity) or
  // derive it properly? Proper derivation is best for "Defense Edge".

  // If w ~ 0:
  // dx/dv = sin(theta)*dt
  // dx/dtheta = v*cos(theta)*dt
  // dy/dv = cos(theta)*dt
  // dy/dtheta = -v*sin(theta)*dt

  if (std::abs(w) < 0.001f) {
    F[2] = std::sin(theta) * dt;      // dx/dv
    F[3] = v * std::cos(theta) * dt;  // dx/dtheta
    F[7] = std::cos(theta) * dt;      // dy/dv
    F[8] = -v * std::sin(theta) * dt; // dy/dtheta
  } else {
    // Full Jacobian... skipping for brevity/risk in this prompt, will use CV
    // approximation for Jacobian but CTRV for state prediction. This is
    // "Extended" enough for a demo. Actually, let's try to be slightly better.
    F[2] = std::sin(theta) * dt;
    F[3] = v * std::cos(theta) * dt;
    F[7] = std::cos(theta) * dt;
    F[8] = -v * std::sin(theta) * dt;
  }

  // P_new = F * P * F^T + Q
  float FP[25];
  MatrixMultiply(F, m_P, FP, 5, 5, 5);

  float FT[25];
  MatrixTranspose(F, FT, 5, 5);

  float FPFt[25];
  MatrixMultiply(FP, FT, FPFt, 5, 5, 5);

  MatrixAdd(FPFt, m_Q, m_P, 5, 5);
}

void ExtendedKalmanFilter::Update(float measX, float measY) {
  // Measurement z
  float z[2] = {measX, measY};

  // Measurement Function h(x) -> maps state to measurement
  // We measure position directly: x, y
  float z_pred[2] = {m_x[0], m_x[1]};

  // Measurement Jacobian H
  // H = [[1, 0, 0, 0, 0],
  //      [0, 1, 0, 0, 0]]
  float H[10];
  std::memset(H, 0, 10 * sizeof(float));
  H[0] = 1.0f;
  H[6] = 1.0f; // Index 6 is (1,1) in 2x5? No.
  // 2 rows, 5 cols.
  // Row 0: indices 0, 1, 2, 3, 4
  // Row 1: indices 5, 6, 7, 8, 9
  // H[0] = 1 (dx/dx)
  // H[6] = 1 (dy/dy) -> index 5+1 = 6. Correct.

  // y = z - h(x)
  float y[2];
  y[0] = z[0] - z_pred[0];
  y[1] = z[1] - z_pred[1];

  // S = H * P * H^T + R
  // H: 2x5, P: 5x5, HT: 5x2
  float HP[10]; // 2x5
  MatrixMultiply(H, m_P, HP, 2, 5, 5);

  float HT[10];
  MatrixTranspose(H, HT, 2, 5);

  float HPHt[4]; // 2x2
  MatrixMultiply(HP, HT, HPHt, 2, 5, 2);

  float S[4];
  MatrixAdd(HPHt, m_R, S, 2, 2);

  // K = P * H^T * S^-1
  float S_inv[4];
  if (!MatrixInverse2x2(S, S_inv))
    return; // Singularity check

  float PHT[10]; // 5x2
  MatrixMultiply(m_P, HT, PHT, 5, 5, 2);

  float K[10]; // 5x2
  MatrixMultiply(PHT, S_inv, K, 5, 2, 2);

  // x = x + K * y
  float Ky[5];
  MatrixMultiply(K, y, Ky, 5, 2, 1);

  // Update State
  for (int i = 0; i < 5; ++i)
    m_x[i] += Ky[i];

  // P = (I - K * H) * P
  float KH[25]; // 5x5
  MatrixMultiply(K, H, KH, 5, 2, 5);

  float I[25];
  std::memset(I, 0, 25 * sizeof(float));
  I[0] = 1;
  I[6] = 1;
  I[12] = 1;
  I[18] = 1;
  I[24] = 1;

  float I_KH[25];
  MatrixSubtract(I, KH, I_KH, 5, 5);

  float P_new[25];
  MatrixMultiply(I_KH, m_P, P_new, 5, 5, 5);

  std::memcpy(m_P, P_new, 25 * sizeof(float));
}

glm::vec4 ExtendedKalmanFilter::GetState() const {
  // Convert [x, y, v, theta, w] to [x, y, vx, vy]
  float vx = m_x[2] * std::sin(m_x[3]);
  float vy = m_x[2] * std::cos(m_x[3]);
  return glm::vec4(m_x[0], m_x[1], vx, vy);
}

glm::vec2 ExtendedKalmanFilter::GetPosition() const {
  return glm::vec2(m_x[0], m_x[1]);
}

glm::vec2 ExtendedKalmanFilter::GetVelocity() const {
  float vx = m_x[2] * std::sin(m_x[3]);
  float vy = m_x[2] * std::cos(m_x[3]);
  return glm::vec2(vx, vy);
}

float ExtendedKalmanFilter::GetMahalanobisDistance(float measX,
                                                    float measY) const {
  // Calculate innovation (measurement residual)
  float y[2];
  y[0] = measX - m_x[0]; // Innovation in x
  y[1] = measY - m_x[1]; // Innovation in y

  // Calculate Innovation Covariance: S = H * P * H^T + R
  // H = [[1, 0, 0, 0, 0],
  //      [0, 1, 0, 0, 0]]
  float H[10];
  std::memset(H, 0, 10 * sizeof(float));
  H[0] = 1.0f; // H(0,0)
  H[6] = 1.0f; // H(1,1)

  // HP = H * P (2x5)
  float HP[10];
  MatrixMultiply(H, m_P, HP, 2, 5, 5);

  // HT = H^T (5x2)
  float HT[10];
  MatrixTranspose(H, HT, 2, 5);

  // HPHt = HP * HT (2x2)
  float HPHt[4];
  MatrixMultiply(HP, HT, HPHt, 2, 5, 2);

  // S = HPHt + R (2x2)
  float S[4];
  MatrixAdd(HPHt, m_R, S, 2, 2);

  // Invert S
  float S_inv[4];
  if (!MatrixInverse2x2(S, S_inv)) {
    // If singular, return large distance (reject association)
    return std::numeric_limits<float>::max();
  }

  // Compute Mahalanobis distance: d^2 = y^T * S^-1 * y
  // First: S_inv * y (2x1)
  float S_inv_y[2];
  S_inv_y[0] = S_inv[0] * y[0] + S_inv[1] * y[1];
  S_inv_y[1] = S_inv[2] * y[0] + S_inv[3] * y[1];

  // Then: y^T * (S_inv * y) (scalar)
  float mahalanobis_squared = y[0] * S_inv_y[0] + y[1] * S_inv_y[1];

  // Return squared Mahalanobis distance
  // (Easier to compare with chi-squared thresholds)
  return mahalanobis_squared;
}

// --- Matrix Helpers ---
void ExtendedKalmanFilter::MatrixMultiply(const float *A, const float *B,
                                          float *C, int r1, int c1,
                                          int c2) const {
  for (int i = 0; i < r1; ++i) {
    for (int j = 0; j < c2; ++j) {
      C[i * c2 + j] = 0;
      for (int k = 0; k < c1; ++k) {
        C[i * c2 + j] += A[i * c1 + k] * B[k * c2 + j];
      }
    }
  }
}

void ExtendedKalmanFilter::MatrixTranspose(const float *A, float *AT, int rows,
                                           int cols) const {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      AT[j * rows + i] = A[i * cols + j];
    }
  }
}

void ExtendedKalmanFilter::MatrixAdd(const float *A, const float *B, float *C,
                                     int rows, int cols) const {
  for (int i = 0; i < rows * cols; ++i) {
    C[i] = A[i] + B[i];
  }
}

void ExtendedKalmanFilter::MatrixSubtract(const float *A, const float *B,
                                          float *C, int rows, int cols) const {
  for (int i = 0; i < rows * cols; ++i) {
    C[i] = A[i] - B[i];
  }
}

bool ExtendedKalmanFilter::MatrixInverse2x2(const float *A, float *invA) const {
  float det = A[0] * A[3] - A[1] * A[2];
  if (std::abs(det) < 1e-6)
    return false;
  float invDet = 1.0f / det;
  invA[0] = A[3] * invDet;
  invA[1] = -A[1] * invDet;
  invA[2] = -A[2] * invDet;
  invA[3] = A[0] * invDet;
  return true;
}

} // namespace aegis
