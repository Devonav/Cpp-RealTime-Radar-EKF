#pragma once

#include "../physics/ExtendedKalmanFilter.h"
#include <glm/glm.hpp>
#include <vector>

namespace aegis {

// Track state enumeration for M-of-N confirmation logic
enum class TrackState {
  TENTATIVE, // New track, not yet confirmed
  CONFIRMED, // Track has received sufficient updates (M-of-N)
  COASTING   // Track is extrapolating without measurements
};

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

  // Statistical distance for data association
  float GetMahalanobisDistance(float x, float y) const {
    return m_kf.GetMahalanobisDistance(x, y);
  }

  // Track state management for M-of-N confirmation
  TrackState GetState() const { return m_state; }
  int GetHitCount() const { return m_hitCount; }
  int GetMissCount() const { return m_missCount; }

  // Called when no measurement associates (for coasting logic)
  void IncrementMissCount(double currentTime);

private:
  uint32_t m_id;
  ExtendedKalmanFilter m_kf;
  double m_lastUpdate;
  std::vector<glm::vec2> m_history;

  // M-of-N confirmation logic (M=3 hits in N=5 scans to confirm)
  TrackState m_state;
  int m_hitCount;  // Number of successful associations
  int m_missCount; // Number of consecutive misses

  static const size_t MAX_HISTORY = 100;
  static const int M_HITS_TO_CONFIRM = 3;
  static const int N_SCANS_WINDOW = 5;
  static const int MAX_COAST_MISSES = 5; // Delete after 5 consecutive misses
};

} // namespace aegis
