#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace aegis {

// Performance metrics for tracking system evaluation
struct TrackingMetrics {
  // Track quality metrics
  int totalTracks = 0;
  int confirmedTracks = 0;
  int tentativeTracks = 0;
  int coastingTracks = 0;

  // Association metrics
  int totalPlots = 0;
  int associatedPlots = 0;
  int newTracks = 0;

  // Ground truth comparison (requires plotId matching)
  int correctAssociations = 0; // Track ID matches ground truth
  int incorrectAssociations = 0;

  // Track lifecycle
  int tracksCreated = 0;
  int tracksDeleted = 0;

  // Positional accuracy (running average)
  float avgPositionError = 0.0f;
  int positionErrorSamples = 0;

  // Reset all metrics
  void Reset() {
    totalTracks = 0;
    confirmedTracks = 0;
    tentativeTracks = 0;
    coastingTracks = 0;
    totalPlots = 0;
    associatedPlots = 0;
    newTracks = 0;
    correctAssociations = 0;
    incorrectAssociations = 0;
    tracksCreated = 0;
    tracksDeleted = 0;
    avgPositionError = 0.0f;
    positionErrorSamples = 0;
  }

  // Update running average for position error
  void AddPositionError(float error) {
    avgPositionError = (avgPositionError * positionErrorSamples + error) /
                       (positionErrorSamples + 1);
    positionErrorSamples++;
  }

  // Calculate track purity (confirmed / total)
  float GetTrackPurity() const {
    return (totalTracks > 0) ? (float)confirmedTracks / totalTracks : 0.0f;
  }

  // Calculate association rate
  float GetAssociationRate() const {
    return (totalPlots > 0) ? (float)associatedPlots / totalPlots : 0.0f;
  }

  // False track rate
  float GetFalseTrackRate() const {
    return (totalTracks > 0) ? (float)tentativeTracks / totalTracks : 0.0f;
  }
};

} // namespace aegis
