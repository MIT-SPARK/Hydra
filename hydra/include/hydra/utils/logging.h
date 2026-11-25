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
  std::string log_name;
  std::string log_prefix = "[";
  std::string log_suffix = "] ";

  //! Use short snake_case names for default component log labels
  VerbosityConfig(const std::string& name = "",
                  std::optional<int> verbosity = std::nullopt);

  //! Copy with a different name while preserving verbosity and delimiters
  VerbosityConfig with_name(const std::string& name) const;

  //! Render the component label, or an empty string when the name is empty
  std::string get_log_prefix() const;

  bool enabled(int level) const { return get_verbosity() >= level; }

  int get_verbosity() const;
};

void declare_config(VerbosityConfig& config);

// NOTE(nathan) you need to include <glog/logging.h> for this to work, but
// I don't want have <glog/logging.h> included in a bunch of header files
// Levels 1-3 cover summaries, per-update output, and normal diagnostic details.
// Reserve 4 for very detailed traces and 5 for data dumps or test diagnostics.
#define MLOG_WITH(C, V) LOG_IF(INFO, (C).enabled(V)) << (C).get_log_prefix()
#define MLOG(V) MLOG_WITH(config, V)

// Severity-based module messages do not use the verbosity threshold.
#define HYDRA_MLOG_LEVEL_INFO INFO
#define HYDRA_MLOG_LEVEL_WARN WARNING
#define HYDRA_MLOG_LEVEL_ERROR ERROR

#define HYDRA_MLOG_NAMED(C, LEVEL) LOG(LEVEL) << (C).get_log_prefix()
#define MLOG_NAMED_WITH(C, LEVEL) HYDRA_MLOG_NAMED(C, HYDRA_MLOG_LEVEL_##LEVEL)
#define MLOG_NAMED(LEVEL) MLOG_NAMED_WITH(config, LEVEL)

}  // namespace hydra
