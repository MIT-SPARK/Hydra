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
#include <list>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace hydra {

template <typename T>
struct MessageQueue {
  using Ptr = std::shared_ptr<MessageQueue<T>>;
  std::list<T> queue;
  mutable std::mutex mutex;
  mutable std::condition_variable cv;
  size_t max_size;

  /**
   * @brief Construct a queue with a size limit
   * @param max_size Maximum size of the queue. If 0, size limit is disabled
   */
  explicit MessageQueue(size_t max_size) : max_size(max_size) {}

  /**
   * @brief Construct a queue without a size limit
   */
  MessageQueue() : MessageQueue(0) {}

  /**
   * @brief Check whether the queue is empty
   */
  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.empty();
  }

  /**
   * @brief Get a reference to the first element in the queue
   * @throw std::out_of_range Throws an exception if the queue is empty
   */
  const T& front() const {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty()) {
      throw std::out_of_range("queue is empty");
    }

    return queue.front();
  }

  /**
   * @brief Get a reference to the last element in the queue
   * @throw std::out_of_range Throws an exception if the queue is empty
   */
  const T& back() const {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty()) {
      throw std::out_of_range("queue is empty");
    }

    return queue.back();
  }

  /**
   * @brief Wait for the queue to have data
   */
  bool poll(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return !queue.empty(); });
  }

  /**
   * @brief Wait for the queue to not have any data
   */
  bool block(int wait_time_us = 1000) const {
    std::chrono::microseconds wait_duration(wait_time_us);
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, wait_duration, [&] { return queue.empty(); });
  }

  /**
   * @brief Push a new element to the queue
   *
   * Note that the blocking behavior is on by default but will not take effect unless
   * the queue max size is set.
   *
   * @param input New element for queue
   * @param blocking Wait until queue has room for new element
   * @param wait_time_us Max time to wait for room in the queue in microseconds
   * @returns Whether the element was added to the queue or not
   */
  bool push(T input, bool blocking = true, int wait_time_us = 0) {
    bool added = false;
    {  // start critical section
      std::unique_lock<std::mutex> lock(mutex);
      if (!max_size || queue.size() < max_size) {
        queue.push_back(std::move(input));
        added = true;
      }

      if (!added && blocking) {
        if (wait_time_us) {
          const std::chrono::microseconds wait_duration(wait_time_us);
          cv.wait_for(lock, wait_duration, [this] { return queue.size() < max_size; });
        } else {
          cv.wait(lock, [this] { return queue.size() < max_size; });
        }

        // condition variable wait succeeded
        if (queue.size() < max_size) {
          queue.push_back(std::move(input));
          added = true;
        }
      }
    }  // end critical section

    // let writers know that queue has a new element
    if (added) {
      cv.notify_all();
    }

    return added;
  }

  /**
   * @brief Get the first element from the queue
   *
   * This will notify any other threads attempting to modify the queue
   *
   * @throw std::out_of_range Throws an exception if the queue is empty
   * @returns The first element of the queue.
   */
  T pop() {
    T value;
    {  // start critical section
      std::lock_guard<std::mutex> lock(mutex);
      if (queue.empty()) {
        throw std::out_of_range("queue is empty");
      }

      value = std::move(queue.front());
      queue.pop_front();
    }  // end critical section

    // let writers know that queue has decreased in size
    cv.notify_all();
    return value;
  }

  /**
   * @brief Get size of the queue
   * @returns The size of the queue
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
  }

  /**
   * @brief Clear the queue (i.e., remove all elements)
   *
   * This will notify any other threads attempting to modify the queue
   */
  void clear() {
    {  // start critical section
      std::lock_guard<std::mutex> lock(mutex);
      queue.clear();
    }  // end critical section

    // let writers know that queue is now empty
    cv.notify_all();
  }
};

}  // namespace hydra
