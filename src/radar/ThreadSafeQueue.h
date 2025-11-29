#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>


namespace aegis {

template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;

  void Push(const T &value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push(value);
    m_cond.notify_one();
  }

  // Blocking pop
  T Pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cond.wait(lock, [this] { return !m_queue.empty(); });
    T value = m_queue.front();
    m_queue.pop();
    return value;
  }

  // Non-blocking pop
  std::optional<T> TryPop() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
      return std::nullopt;
    }
    T value = m_queue.front();
    m_queue.pop();
    return value;
  }

  bool Empty() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }

private:
  std::queue<T> m_queue;
  mutable std::mutex m_mutex;
  std::condition_variable m_cond;
};

} // namespace aegis
