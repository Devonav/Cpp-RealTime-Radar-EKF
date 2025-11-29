#include "KalmanFilter.h"

namespace aegis {

KalmanFilter::KalmanFilter(float initialX, float initialY) {
  // Initial State
  m_x = glm::vec4(initialX, initialY, 0.0f, 0.0f);

  // Initial Covariance (High uncertainty in velocity, low in position)
  m_P = glm::mat4(1.0f);
  m_P[0][0] = 10.0f;
  m_P[1][1] = 10.0f;
  m_P[2][2] = 1000.0f;
  m_P[3][3] = 1000.0f;

  // Transition Matrix (F) - Will be updated with dt
  // [1 0 dt 0]
  // [0 1 0 dt]
  // [0 0 1  0]
  // [0 0 0  1]
  m_F = glm::mat4(1.0f);

  // Process Noise (Q) - Uncertainty in the model (e.g. wind, maneuvers)
  m_Q = glm::mat4(1.0f) * 1.0f; // Tunable

  // Measurement Noise (R) - Uncertainty in the sensor
  m_R = glm::mat2(1.0f) * 2500.0f; // Variance = 50^2 = 2500
}

void KalmanFilter::Predict(float dt) {
  // Update F with dt
  m_F[2][0] = dt; // Column 0, Row 2? No, GLM is m[col][row]
  // F =
  // 1 0 dt 0
  // 0 1 0 dt
  // 0 0 1 0
  // 0 0 0 1
  // In GLM (Column-Major):
  // Col 0: 1, 0, 0, 0
  // Col 1: 0, 1, 0, 0
  // Col 2: dt, 0, 1, 0
  // Col 3: 0, dt, 0, 1

  m_F = glm::mat4(1.0f);
  m_F[2][0] = dt; // This sets Col 2, Row 0. Wait.
  // m[col][row]
  // We want Row 0, Col 2 to be dt. So m[2][0] = dt.
  // We want Row 1, Col 3 to be dt. So m[3][1] = dt.

  m_F[2][0] = dt;
  m_F[3][1] = dt;

  // x = F * x
  m_x = m_F * m_x;

  // P = F * P * F^T + Q
  m_P = m_F * m_P * glm::transpose(m_F) + m_Q;
}

void KalmanFilter::Update(float measX, float measY) {
  glm::vec2 z(measX, measY);

  // H = [1 0 0 0; 0 1 0 0]
  // y = z - H * x
  glm::vec2 Hx(m_x.x, m_x.y); // First two components
  glm::vec2 y = z - Hx;

  // S = H * P * H^T + R
  // H * P selects the top-left 2x4 block of P?
  // H * P * H^T selects the top-left 2x2 block of P.
  glm::mat2 P_topleft(m_P[0][0], m_P[0][1], m_P[1][0], m_P[1][1]);
  glm::mat2 S = P_topleft + m_R;

  // K = P * H^T * S^-1
  // P * H^T is the first 2 columns of P?
  // P is 4x4. H^T is 4x2. Result is 4x2.
  // Col 0 of Result = P * [1 0 0 0]^T = Col 0 of P
  // Col 1 of Result = P * [0 1 0 0]^T = Col 1 of P

  // So K_num is 4x2 matrix consisting of first two columns of P.
  // But GLM doesn't have mat4x2.
  // We can compute K manually or use vectors.

  glm::mat2 S_inv = glm::inverse(S);

  // K = (First 2 cols of P) * S_inv
  // Let's compute K as 2 vectors (columns)
  glm::vec4 P_col0 = m_P[0];
  glm::vec4 P_col1 = m_P[1];

  // K is 4x2.
  // K = [P_col0 P_col1] * S_inv
  // K_col0 = P_col0 * S_inv[0][0] + P_col1 * S_inv[0][1]
  // K_col1 = P_col0 * S_inv[1][0] + P_col1 * S_inv[1][1]

  glm::vec4 K_col0 = P_col0 * S_inv[0][0] + P_col1 * S_inv[0][1];
  glm::vec4 K_col1 = P_col0 * S_inv[1][0] + P_col1 * S_inv[1][1];

  // x = x + K * y
  m_x = m_x + (K_col0 * y.x + K_col1 * y.y);

  // P = (I - K * H) * P
  // K * H is 4x4.
  // (K * H) * P
  // K * H has structure:
  // [K_col0 K_col1 0 0]

  // Let KH be 4x4
  glm::mat4 KH(0.0f);
  KH[0] = K_col0; // Col 0
  KH[1] = K_col1; // Col 1
  // Cols 2 and 3 are 0.

  glm::mat4 I(1.0f);
  m_P = (I - KH) * m_P;
}

} // namespace aegis
