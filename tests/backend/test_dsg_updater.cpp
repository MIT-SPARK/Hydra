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
#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <gtest/gtest.h>
#include <hydra/backend/dsg_updater.h>

#include "hydra_test/shared_dsg_fixture.h"

namespace hydra {
namespace {

struct CallRecord {
  std::string functor;
  bool node_active;
  bool node_new;
};

std::vector<CallRecord>& callRecords() {
  static std::vector<CallRecord> records;
  return records;
}

std::map<std::string, size_t>& cleanupCounts() {
  static std::map<std::string, size_t> counts;
  return counts;
}

NodeId trackedNode() { return spark_dsg::NodeSymbol('p', 0); }

// Records, per functor invocation, how the shared per-spin bookkeeping (forced-active
// flags and new-node status on the source graph) looks from inside the functor.
struct RecordingFunctor : UpdateFunctor {
  struct Config {
    std::string name;
  } const config;

  explicit RecordingFunctor(const Config& config) : config(config) {}

  Hooks hooks() const override {
    auto my_hooks = UpdateFunctor::hooks();
    my_hooks.cleanup = [name = config.name](const UpdateInfo::ConstPtr&,
                                            DynamicSceneGraph&,
                                            SharedDsgInfo*) {
      ++cleanupCounts()[name];
    };
    return my_hooks;
  }

  void call(const DynamicSceneGraph& unmerged,
            SharedDsgInfo&,
            const UpdateInfo::ConstPtr&) const override {
    const auto& attrs = unmerged.getNode(trackedNode()).attributes();
    callRecords().push_back({config.name,
                             attrs.is_active,
                             unmerged.checkNode(trackedNode()) == NodeStatus::NEW});
  }

  inline static const auto registration_ =
      config::RegistrationWithConfig<UpdateFunctor, RecordingFunctor, Config>(
          "RecordingFunctor");
};

void declare_config(RecordingFunctor::Config& config) {
  config::name("RecordingFunctor::Config");
  config::field(config.name, "name");
}

}  // namespace

TEST(DsgUpdater, PerSpinBookkeepingSharedAcrossFunctors) {
  callRecords().clear();
  cleanupCounts().clear();

  auto target = test::makeSharedDsg();
  DynamicSceneGraph::Ptr source = target->graph->clone();

  // a NEW node that archived before the backend ever saw it
  auto attrs = std::make_unique<PlaceNodeAttributes>();
  attrs->is_active = false;
  source->emplaceNode(DsgLayers::PLACES, trackedNode(), std::move(attrs));

  DsgUpdater::Config config;
  config.update_functors.emplace_back(
      "a", DsgUpdater::Config::FunctorConfig(RecordingFunctor::Config{"a"}));
  config.update_functors.emplace_back(
      "b", DsgUpdater::Config::FunctorConfig(RecordingFunctor::Config{"b"}));

  DsgUpdater updater(config, source, target);
  UpdateInfo::ConstPtr info(new UpdateInfo{0});
  updater.callUpdateFunctions(0, info);

  // every functor in the spin should observe the forced-active flag and the new-node
  // status; both are per-spin state that must only reset after all functors ran
  ASSERT_EQ(callRecords().size(), 2u);
  EXPECT_EQ(callRecords()[0].functor, "a");
  EXPECT_TRUE(callRecords()[0].node_active);
  EXPECT_TRUE(callRecords()[0].node_new);
  EXPECT_EQ(callRecords()[1].functor, "b");
  EXPECT_TRUE(callRecords()[1].node_active);
  EXPECT_TRUE(callRecords()[1].node_new);

  // each cleanup hook runs exactly once per spin
  EXPECT_EQ(cleanupCounts()["a"], 1u);
  EXPECT_EQ(cleanupCounts()["b"], 1u);

  // after the spin the forced-active flag is restored
  EXPECT_FALSE(source->getNode(trackedNode()).attributes().is_active);
}

}  // namespace hydra
