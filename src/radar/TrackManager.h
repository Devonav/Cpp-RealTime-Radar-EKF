#pragma once

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

  // Thread-safe access for rendering
  std::vector<std::shared_ptr<Track>> GetTracks() const;

private:
  std::vector<std::shared_ptr<Track>> m_tracks;
  mutable std::mutex m_mutex;
  uint32_t m_nextTrackId;

  const float GATING_THRESHOLD = 500.0f; // Meters
  const double TIMEOUT_THRESHOLD = 5.0;  // Seconds
};

} // namespace aegis
