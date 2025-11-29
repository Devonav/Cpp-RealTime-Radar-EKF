#pragma once

#include "PerformanceMetrics.h"
#include "Track.h"
#include <memory>
#include <mutex>
#include <vector>


namespace aegis {

class TrackManager {
public:
  TrackManager();

  void ProcessPlot(uint32_t plotId, float x, float y, double timestamp);
  void PruneTracks(double currentTime);
  void IncrementMissedTracks(double currentTime); // Mark tracks with no association

  // Thread-safe access for rendering
  std::vector<std::shared_ptr<Track>> GetTracks() const;

  // Performance metrics
  const TrackingMetrics &GetMetrics() const { return m_metrics; }
  void UpdateMetrics(); // Call periodically to update track state counts

private:
  std::vector<std::shared_ptr<Track>> m_tracks;
  mutable std::mutex m_mutex;
  uint32_t m_nextTrackId;

  // Performance metrics tracking
  TrackingMetrics m_metrics;
  int m_previousTrackCount = 0; // For tracking created/deleted

  // Chi-squared gating threshold for 2 DOF (x,y) at 99% confidence
  // Chi2(0.99, 2) = 9.21
  const float CHI_SQUARED_GATE = 9.21f;
  const double TIMEOUT_THRESHOLD = 5.0;  // Seconds
};

} // namespace aegis
