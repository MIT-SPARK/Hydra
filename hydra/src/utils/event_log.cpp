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
#include "hydra/utils/event_log.h"

#include <glog/logging.h>

#include <chrono>

namespace hydra {

EventLog::EventLog(const std::filesystem::path& path, const std::string& header)
    : path_(path), header_(header) {}

void EventLog::append(const std::vector<std::string>& lines) {
  if (lines.empty()) {
    return;
  }

  std::string buffer;
  for (const auto& line : lines) {
    buffer += line;
    buffer += '\n';
  }

  std::lock_guard<std::mutex> lock(mutex_);
  std::error_code ec;
  if (!out_.is_open() || !std::filesystem::exists(path_, ec)) {
    out_.close();
    out_.clear();
    if (path_.has_parent_path()) {
      std::filesystem::create_directories(path_.parent_path(), ec);
    }

    out_.open(path_, std::ios::out | std::ios::app | std::ios::binary);
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    buffer =
        "{\"ev\": \"open\", \"version\": 1, \"wall_ns\": " + std::to_string(wall_ns) +
        (header_.empty() ? "" : ", " + header_) + "}\n" + buffer;
  }

  out_ << buffer;
  out_.flush();
  if (!out_) {
    LOG_EVERY_N(ERROR, 100) << "Failed to write event log '" << path_.string() << "'";
    out_.close();  // retry opening on the next append
  }
}

}  // namespace hydra
