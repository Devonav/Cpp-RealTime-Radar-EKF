#include "Track.h"

namespace aegis {

Track::Track(uint32_t id, float x, float y, double timestamp)
    : m_id(id), m_lastUpdate(timestamp), m_kf(x, y, 0.0f, 0.0f),
      m_state(TrackState::TENTATIVE), m_hitCount(1),
      m_missCount(0) // Initial V=0, Heading=0.
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

  // M-of-N confirmation logic
  m_hitCount++;
  m_missCount = 0; // Reset consecutive miss count on successful update

  // Promote TENTATIVE → CONFIRMED after M hits
  if (m_state == TrackState::TENTATIVE && m_hitCount >= M_HITS_TO_CONFIRM) {
    m_state = TrackState::CONFIRMED;
  }

  // Promote COASTING → CONFIRMED on measurement
  if (m_state == TrackState::COASTING) {
    m_state = TrackState::CONFIRMED;
  }

  m_history.push_back(m_kf.GetPosition());
  if (m_history.size() > MAX_HISTORY) {
    m_history.erase(m_history.begin());
  }
}

void Track::IncrementMissCount(double currentTime) {
  m_missCount++;

  // Transition CONFIRMED → COASTING after consecutive misses
  if (m_state == TrackState::CONFIRMED && m_missCount >= 2) {
    m_state = TrackState::COASTING;
  }
}

} // namespace aegis
