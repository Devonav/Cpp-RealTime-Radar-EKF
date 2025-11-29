#pragma once

#include "../physics/ExtendedKalmanFilter.h"
#include <glm/glm.hpp>
#include <vector>

namespace aegis {

class Track {
public:
  Track(uint32_t id, float x, float y, double timestamp);

  void Predict(double currentTime);
  void Update(float x, float y, double timestamp);

  uint32_t GetId() const { return m_id; }
  glm::vec2 GetPosition() const { return m_kf.GetPosition(); }
  glm::vec2 GetVelocity() const { return m_kf.GetVelocity(); }
  double GetLastUpdate() const { return m_lastUpdate; }
  const std::vector<glm::vec2> &GetHistory() const { return m_history; }

private:
  uint32_t m_id;
  ExtendedKalmanFilter m_kf;
  double m_lastUpdate;
  std::vector<glm::vec2> m_history;

  static const size_t MAX_HISTORY = 100;
};

} // namespace aegis
