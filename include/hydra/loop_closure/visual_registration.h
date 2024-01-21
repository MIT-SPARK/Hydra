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
#include <config_utilities/factory.h>
#include <teaser/registration.h>

#include <opencv2/features2d.hpp>

#include "hydra/loop_closure/sensor_registration.h"

namespace hydra::lcd {

using TeaserParams = teaser::RobustRegistrationSolver::Params;

class TwoViewTeaserSolver : public SensorRegistrationSolver {
 public:
  struct Config : public SensorRegistrationConfig {
    TeaserParams teaser;
    std::string timer_prefix;
    std::string matcher_type = "BruteForce-Hamming";
    size_t min_correspondences = 10;
    float lowe_ratio = 0.9;
    size_t min_inliers = 10;
  };

  explicit TwoViewTeaserSolver(const Config& config);

  virtual ~TwoViewTeaserSolver() = default;

  RegistrationSolution solve(const DynamicSceneGraph& dsg,
                             const SensorRegistrationInput& match) const override;

 public:
  const Config config;
  std::string timer_prefix;

 protected:
  // registration call mutates the solver
  mutable teaser::RobustRegistrationSolver solver_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<SensorRegistrationSolver,
                                     TwoViewTeaserSolver,
                                     Config>("TwoViewTeaserSolver");
};

void declare_config(TwoViewTeaserSolver::Config& config);

}  // namespace hydra::lcd
