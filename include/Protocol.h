#pragma once

#include <cstdint>

namespace aegis {

#pragma pack(push, 1)
struct Plot {
  uint32_t id; // Unique ID for the target (ground truth ID, usually unknown to
               // receiver but useful for debug)
  float x;     // X position (meters)
  float y;     // Y position (meters)
  float z;     // Z position (meters)
  float velocity;   // Speed (m/s)
  float heading;    // Heading (degrees)
  double timestamp; // Time of detection
};
#pragma pack(pop)

} // namespace aegis
