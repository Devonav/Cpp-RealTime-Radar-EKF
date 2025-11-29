#include "Track.h"

namespace aegis {

Track::Track(uint32_t id, float x, float y, double timestamp)
    : m_id(id), m_lastUpdate(timestamp),
      m_kf(x, y, 0.0f, 0.0f) // Initial V=0, Heading=0.
// Note: EKF convergence might be slow if init V is wrong.
// Ideally we'd wait for 2 measurements to init V.
// For now, 0 is safe, covariance will handle it.
{
  m_history.push_back(glm::vec2(x, y));
}

void Track::Predict(double currentTime) {
  float dt = static_cast<float>(currentTime - m_lastUpdate);
  if (dt > 0.0f) {
    m_kf.Predict(dt);
    m_lastUpdate = currentTime;
  }
}

void Track::Update(float x, float y, double timestamp) {
  double dt = timestamp - m_lastUpdate;
  if (dt > 0.0001) {
    m_kf.Predict(static_cast<float>(dt));
  }

  m_kf.Update(x, y);
  m_lastUpdate = timestamp;

  m_history.push_back(m_kf.GetPosition());
  if (m_history.size() > MAX_HISTORY) {
    m_history.erase(m_history.begin());
  }
}

} // namespace aegis
