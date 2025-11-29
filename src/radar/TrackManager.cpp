#include "TrackManager.h"
#include <algorithm>
#include <limits>


namespace aegis {

TrackManager::TrackManager() : m_nextTrackId(1) {}

void TrackManager::ProcessPlot(uint32_t plotId, float x, float y,
                               double timestamp) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Update metrics
  m_metrics.totalPlots++;

  // Nearest Neighbor Association with Mahalanobis Distance Gating
  // Uses innovation covariance for statistically-rigorous gating
  std::shared_ptr<Track> bestTrack = nullptr;
  float minDist = std::numeric_limits<float>::max();

  for (auto &track : m_tracks) {
    // Predict track to current time for better association
    // Note: We don't want to modify the track state permanently here just for
    // association check? Or we do. Let's use the current state.

    // Calculate Mahalanobis distance (statistically weighted)
    float mahalanobis_sq = track->GetMahalanobisDistance(x, y);

    // Gate using chi-squared threshold (2 DOF, 99% confidence)
    if (mahalanobis_sq < minDist && mahalanobis_sq < CHI_SQUARED_GATE) {
      minDist = mahalanobis_sq;
      bestTrack = track;
    }
  }

  if (bestTrack) {
    bestTrack->Update(x, y, timestamp);
    m_metrics.associatedPlots++;

    // Calculate position error for metrics
    glm::vec2 predicted = bestTrack->GetPosition();
    float error = glm::distance(predicted, glm::vec2(x, y));
    m_metrics.AddPositionError(error);
  } else {
    // Create new track
    // Use plotId as track ID if available/unique, or generate one.
    // For simulation, plotId is the ground truth ID. We can use it for debug,
    // but a real system would assign its own ID.
    // Let's use our own ID to be realistic.
    m_tracks.push_back(
        std::make_shared<Track>(m_nextTrackId++, x, y, timestamp));
    m_metrics.newTracks++;
    m_metrics.tracksCreated++;
  }
}

void TrackManager::IncrementMissedTracks(double currentTime) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Increment miss count for all tracks (will be reset when associated)
  for (auto &track : m_tracks) {
    track->IncrementMissCount(currentTime);
  }
}

void TrackManager::PruneTracks(double currentTime) {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Delete tracks based on state and miss count
  m_tracks.erase(
      std::remove_if(
          m_tracks.begin(), m_tracks.end(),
          [currentTime, this](const std::shared_ptr<Track> &track) {
            // Delete TENTATIVE tracks that haven't confirmed after timeout
            if (track->GetState() == TrackState::TENTATIVE &&
                (currentTime - track->GetLastUpdate()) > TIMEOUT_THRESHOLD) {
              return true;
            }

            // Delete COASTING tracks after max consecutive misses
            if (track->GetState() == TrackState::COASTING &&
                track->GetMissCount() >= 5) {
              return true;
            }

            // Standard timeout for all tracks
            return (currentTime - track->GetLastUpdate()) > TIMEOUT_THRESHOLD;
          }),
      m_tracks.end());
}

std::vector<std::shared_ptr<Track>> TrackManager::GetTracks() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_tracks;
}

void TrackManager::UpdateMetrics() {
  std::lock_guard<std::mutex> lock(m_mutex);

  // Count tracks by state
  m_metrics.totalTracks = static_cast<int>(m_tracks.size());
  m_metrics.confirmedTracks = 0;
  m_metrics.tentativeTracks = 0;
  m_metrics.coastingTracks = 0;

  for (const auto &track : m_tracks) {
    switch (track->GetState()) {
    case TrackState::CONFIRMED:
      m_metrics.confirmedTracks++;
      break;
    case TrackState::TENTATIVE:
      m_metrics.tentativeTracks++;
      break;
    case TrackState::COASTING:
      m_metrics.coastingTracks++;
      break;
    }
  }

  // Track deletion detection
  if (m_tracks.size() < static_cast<size_t>(m_previousTrackCount)) {
    m_metrics.tracksDeleted +=
        (m_previousTrackCount - static_cast<int>(m_tracks.size()));
  }
  m_previousTrackCount = static_cast<int>(m_tracks.size());
}

} // namespace aegis
