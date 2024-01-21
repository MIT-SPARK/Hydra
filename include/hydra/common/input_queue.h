/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>

namespace hydra {

template <typename T>
struct InputQueue {
  using Ptr = std::shared_ptr<InputQueue<T>>;
  std::queue<T> queue;
  mutable std::mutex mutex;
  mutable std::condition_variable cv;
  size_t max_size;

  explicit InputQueue(size_t max_size) : max_size(max_size) {}

  InputQueue() : InputQueue(0) {}

  bool empty() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.empty();
  }

  const T& front() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.front();
  }

  const T& back() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.back();
  }

  /**
   * @brief wait for the queue to have data
   */
  bool poll(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return !queue.empty(); });
  }

  /**
   * @brief wait for the queue to not have any data
   */
  bool block(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return queue.empty(); });
  }

  bool push(const T& input) {
    bool added = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!max_size || queue.size() < max_size) {
        queue.push(input);
        added = true;
      }
    }

    cv.notify_all();

    return added;
  }

  T pop() {
    std::unique_lock<std::mutex> lock(mutex);
    auto value = queue.front();
    queue.pop();
    return value;
  }

  size_t size() const {
    std::unique_lock<std::mutex> lock(mutex);
    return queue.size();
  }

  void clear() {
    std::unique_lock<std::mutex> lock(mutex);
    queue = {};
  }
};

}  // namespace hydra
