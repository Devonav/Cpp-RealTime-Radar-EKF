#include "network/UdpSocket.h"
#include "radar/TargetGenerator.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

int main() {
  try {
    aegis::net::UdpSocket socket;
    // No bind needed for sender usually, but good practice to bind to 0

    std::cout << "Aegis Radar Simulator (Sender) Started" << std::endl;
    std::cout << "Sending to 127.0.0.1:5000" << std::endl;

    std::vector<aegis::TargetGenerator> targets;
    // Create a few targets
    // Target 101: South-West, moving North-East, turning right (Orbit)
    targets.emplace_back(101, -5000.0f, -5000.0f, 250.0f, 45.0f, 5.0f);

    // Target 102: North-East, moving South-West, turning left (S-Turn start)
    targets.emplace_back(102, 5000.0f, 5000.0f, 150.0f, 225.0f, -3.0f);

    // Use system time for timestamp to ensure synchronization with receiver
    auto startTime = std::chrono::high_resolution_clock::now();
    const float dt = 0.1f; // 10 Hz update rate

    while (true) {
      auto now = std::chrono::high_resolution_clock::now();
      double timestamp =
          std::chrono::duration<double>(now.time_since_epoch()).count();

      for (auto &target : targets) {
        target.Update(dt);
        aegis::Plot plot = target.GetNoisyPlot(timestamp);

        socket.SendTo("127.0.0.1", 5000, &plot, sizeof(plot));

        std::cout << "Sent Plot ID: " << plot.id << " X: " << plot.x
                  << " Y: " << plot.y << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
