// Portions of the following code and their modifications are originally from
// https://github.com/ethz-asl/voxblox and are licensed under the following license:
//
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modifications are also subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.

#pragma once

#include <glog/logging.h>

#include <deque>
#include <queue>
#include <vector>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra {
// TODO(@Nathan): Check whether this is needed.
template <typename T>
class BucketQueue {
 public:
  BucketQueue() : last_bucket_index_(0) {}
  explicit BucketQueue(int num_buckets, double max_val)
      : num_buckets_(num_buckets),
        max_val_(max_val),
        last_bucket_index_(0),
        num_elements_(0) {
    buckets_.resize(num_buckets_);
  }

  /// WARNING: will CLEAR THE QUEUE!
  void setNumBuckets(int num_buckets, double max_val) {
    max_val_ = max_val;
    num_buckets_ = num_buckets;
    buckets_.clear();
    buckets_.resize(num_buckets_);
    num_elements_ = 0;
  }

  void push(const T& key, double value) {
    CHECK_NE(num_buckets_, 0);
    if (value > max_val_) {
      value = max_val_;
    }
    int bucket_index = std::floor(std::abs(value) / max_val_ * (num_buckets_ - 1));
    if (bucket_index >= num_buckets_) {
      bucket_index = num_buckets_ - 1;
    }
    if (bucket_index < last_bucket_index_) {
      last_bucket_index_ = bucket_index;
    }
    buckets_[bucket_index].push(key);
    num_elements_++;
  }

  void pop() {
    if (empty()) {
      return;
    }
    while (buckets_[last_bucket_index_].empty() && last_bucket_index_ < num_buckets_) {
      last_bucket_index_++;
    }
    if (last_bucket_index_ < num_buckets_) {
      buckets_[last_bucket_index_].pop();
      num_elements_--;
    }
  }

  T front() {
    CHECK_NE(num_buckets_, 0);
    CHECK(!empty());
    while (buckets_[last_bucket_index_].empty() && last_bucket_index_ < num_buckets_) {
      last_bucket_index_++;
    }
    return buckets_[last_bucket_index_].front();
  }

  bool empty() { return num_elements_ == 0; }

  void clear() {
    buckets_.clear();
    buckets_.resize(num_buckets_);
    last_bucket_index_ = 0;
    num_elements_ = 0;
  }

 private:
  int num_buckets_;
  double max_val_;
  std::vector<std::queue<T>> buckets_;

  /// Speeds up retrivals.
  int last_bucket_index_;
  /// This is also to speed up empty checks.
  size_t num_elements_;
};

}  // namespace hydra

