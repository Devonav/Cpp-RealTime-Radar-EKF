#pragma once

#include "Protocol.h"
#include <random>
#include <vector>

namespace aegis {

class TargetGenerator {
public:
  TargetGenerator(uint32_t id, float startX, float startY, float speed,
                  float heading, float turnRate = 0.0f);

  void Update(float dt);
  Plot GetNoisyPlot(double timestamp);

private:
  uint32_t m_id;
  float m_x, m_y, m_z;
  float m_vx, m_vy;
  float m_speed;
  float m_heading;
  float m_turnRate; // Degrees per second

  std::default_random_engine m_generator;
  std::normal_distribution<float> m_noiseDist;
};

} // namespace aegis
