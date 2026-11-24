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
#include <optional>
#include <string>

#include "hydra/common/config_utilities.h"

namespace hydra {

struct VerbosityConfig {
  //! An unset value inherits the current run default
  std::optional<int> verbosity = std::nullopt;
  std::string log_prefix = "";

  VerbosityConfig() = default;
  VerbosityConfig(int verbosity);
  VerbosityConfig(const std::string& prefix,
                  std::optional<int> verbosity = std::nullopt);

  //! Make a copy of the verbosity config with a prefix derived from the provided name
  VerbosityConfig with_name(const std::string& name) const;

  static VerbosityConfig named(const std::string& name = "");

  bool enabled(int level) const { return get_verbosity() >= level; }

  int get_verbosity() const;
};

void declare_config(VerbosityConfig& config);

// NOTE(nathan) you need to include <glog/logging.h> for this to work, but
// I don't want have <glog/logging.h> included in a bunch of header files
#define MLOG_WITH(C, V) LOG_IF(INFO, (C).enabled(V)) << (C).log_prefix
#define MLOG(V) MLOG_WITH(config, V)

}  // namespace hydra
