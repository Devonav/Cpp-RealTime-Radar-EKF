#include "TrackManager.h"
#include <algorithm>
#include <limits>


namespace aegis {

TrackManager::TrackManager() : m_nextTrackId(1) {}

void TrackManager::ProcessPlot(uint32_t plotId, float x, float y,
                               double timestamp) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Nearest Neighbor Association
  std::shared_ptr<Track> bestTrack = nullptr;
  float minDist = std::numeric_limits<float>::max();

  for (auto &track : m_tracks) {
    // Predict track to current time for better association
    // Note: We don't want to modify the track state permanently here just for
    // association check? Or we do. Let's use the current state.
    glm::vec2 pos = track->GetPosition();
    float dist = glm::distance(pos, glm::vec2(x, y));

    if (dist < minDist && dist < GATING_THRESHOLD) {
      minDist = dist;
      bestTrack = track;
    }
  }

  if (bestTrack) {
    bestTrack->Update(x, y, timestamp);
  } else {
    // Create new track
    // Use plotId as track ID if available/unique, or generate one.
    // For simulation, plotId is the ground truth ID. We can use it for debug,
    // but a real system would assign its own ID.
    // Let's use our own ID to be realistic.
    m_tracks.push_back(
        std::make_shared<Track>(m_nextTrackId++, x, y, timestamp));
  }
}

void TrackManager::PruneTracks(double currentTime) {
  std::lock_guard<std::mutex> lock(m_mutex);

  m_tracks.erase(
      std::remove_if(m_tracks.begin(), m_tracks.end(),
                     [currentTime, this](const std::shared_ptr<Track> &track) {
                       return (currentTime - track->GetLastUpdate()) >
                              TIMEOUT_THRESHOLD;
                     }),
      m_tracks.end());
}

std::vector<std::shared_ptr<Track>> TrackManager::GetTracks() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_tracks;
}

} // namespace aegis
