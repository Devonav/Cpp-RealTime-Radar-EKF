#include "TargetGenerator.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace aegis {

TargetGenerator::TargetGenerator(uint32_t id, float startX, float startY,
                                 float speed, float heading, float turnRate)
    : m_id(id), m_x(startX), m_y(startY), m_z(1000.0f), m_speed(speed),
      m_heading(heading), m_turnRate(turnRate),
      m_noiseDist(0.0f, 50.0f) // 50m standard deviation
{
  float headingRad = heading * (static_cast<float>(M_PI) / 180.0f);
  m_vx = std::sin(headingRad) * speed;
  m_vy = std::cos(headingRad) * speed;
}

void TargetGenerator::Update(float dt) {
  // Constant Turn Rate and Velocity (CTRV) Model
  if (std::abs(m_turnRate) < 0.001f) {
    // Linear motion
    float headingRad = m_heading * (static_cast<float>(M_PI) / 180.0f);
    m_x += std::sin(headingRad) * m_speed * dt;
    m_y += std::cos(headingRad) * m_speed * dt;
  } else {
    // Turning motion
    float headingRad = m_heading * (static_cast<float>(M_PI) / 180.0f);
    float turnRateRad = m_turnRate * (static_cast<float>(M_PI) / 180.0f);

    m_x += (m_speed / turnRateRad) *
           (std::cos(headingRad) -
            std::cos(
                headingRad +
                turnRateRad *
                    dt)); // Note: X is East (Sin), Y is North (Cos) convention?
                          // Wait, standard math: X=cos, Y=sin.
    // Radar convention often: Y=North (0 deg), X=East (90 deg).
    // Let's stick to standard trig for now but rotated 90 deg if needed.
    // Previous code: vx = sin(heading) * speed. This implies 0 deg is North
    // (Y-axis) if Y is up? If heading=0, vx=0, vy=speed. So 0 is +Y (North). 90
    // is +X (East). Correct integration for this frame: x(t+dt) = x(t) + v/w *
    // (cos(theta(t)) - cos(theta(t) + w*dt)) y(t+dt) = y(t) + v/w *
    // (sin(theta(t) + w*dt) - sin(theta(t))) BUT, our heading 0 is Y axis. x =
    // x + v/w * (cos(h) - cos(h+w*dt)) -> This matches standard X axis
    // integration Let's derive carefully. vx = v * sin(h) vy = v * cos(h) x' =
    // v * sin(h) y' = v * cos(h) h(t) = h0 + w*t x(t) = integral(v*sin(h0+wt))
    // = -v/w * cos(h0+wt) + C y(t) = integral(v*cos(h0+wt)) = v/w * sin(h0+wt)
    // + C Delta X = -v/w * (cos(h+w*dt) - cos(h)) = v/w * (cos(h) -
    // cos(h+w*dt)) Delta Y = v/w * (sin(h+w*dt) - sin(h))

    m_x += (m_speed / turnRateRad) *
           (std::cos(headingRad) - std::cos(headingRad + turnRateRad * dt));
    m_y += (m_speed / turnRateRad) *
           (std::sin(headingRad + turnRateRad * dt) - std::sin(headingRad));

    m_heading += m_turnRate * dt;

    // Normalize heading to 0-360
    if (m_heading > 360.0f)
      m_heading -= 360.0f;
    if (m_heading < 0.0f)
      m_heading += 360.0f;
  }
}

Plot TargetGenerator::GetNoisyPlot(double timestamp) {
  Plot p;
  p.id = m_id;
  p.x = m_x + m_noiseDist(m_generator);
  p.y = m_y + m_noiseDist(m_generator);
  p.z = m_z + m_noiseDist(m_generator);
  p.velocity = m_speed; // Doppler velocity (simplified)
  p.heading = m_heading;
  p.timestamp = timestamp;
  return p;
}

} // namespace aegis
