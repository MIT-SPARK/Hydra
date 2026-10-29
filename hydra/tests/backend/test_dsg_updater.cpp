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
#include <spark_dsg/node_attributes.h>

#include "hydra_test/config_guard.h"
#include "hydra_test/shared_dsg_fixture.h"
#include "hydra_test/temp_directory.h"

#ifdef HYDRA_ENABLE_IMAGE_STORE
#include <hydra/utils/image_store.h>

#include "hydra_test/image_store_query.h"
#endif

namespace hydra {

using spark_dsg::DsgLayers;
using spark_dsg::NodeAttributes;
using spark_dsg::NodeId;
using spark_dsg::SceneGraph;
using MergeUpdate = SharedDsgInfo::MergeUpdate;

namespace {

//! Update functor that proposes the merges set by the test
struct ScriptedMergeFunctor : public UpdateFunctor {
  struct Config {};

  explicit ScriptedMergeFunctor(const Config&) {}

  Hooks hooks() const override {
    Hooks my_hooks;
    my_hooks.find_merges = [](const auto&, const auto&) {
      auto proposed = proposals();
      proposals().clear();
      return proposed;
    };
    return my_hooks;
  }

  void call(const SceneGraph&,
            SharedDsgInfo&,
            const UpdateInfo::ConstPtr&) const override {}

  static MergeList& proposals() {
    static MergeList merges;
    return merges;
  }
};

void declare_config(ScriptedMergeFunctor::Config&) {
  config::name("ScriptedMergeFunctor::Config");
}

static const auto registration =
    config::RegistrationWithConfig<UpdateFunctor,
                                   ScriptedMergeFunctor,
                                   ScriptedMergeFunctor::Config>(
        "ScriptedMergeFunctor");

DsgUpdater::Config makeConfig() {
  DsgUpdater::Config config;
  config.update_functors.emplace_back(
      "objects", DsgUpdater::Config::FunctorConfig(ScriptedMergeFunctor::Config{}));
  return config;
}

SceneGraph::Ptr makeSource() {
  auto source = test::makeSharedDsg()->graph;
  for (NodeId node_id = 0; node_id < 4; ++node_id) {
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position = Eigen::Vector3d(node_id, 0.0, 0.0);
    attrs->is_active = true;
    attrs->last_update_time_ns = 10u;
    source->emplaceNode(DsgLayers::OBJECTS, node_id, std::move(attrs));
  }

  return source;
}

void runRound(DsgUpdater& updater, const MergeList& proposals) {
  ScriptedMergeFunctor::proposals() = proposals;
  UpdateInfo::ConstPtr info(new UpdateInfo{10});
  updater.callUpdateFunctions(10, info);
}

using Updates = std::vector<MergeUpdate>;

}  // namespace

TEST(DsgUpdater, ReportsNewMerges) {
  auto source = makeSource();
  auto target = test::makeSharedDsg();
  DsgUpdater updater(makeConfig(), source, target);

  runRound(updater, {{1, 0}});
  EXPECT_EQ(target->new_merges, Updates({{1, 0}}));
  EXPECT_FALSE(target->graph->hasNode(1));

  // nothing new: the report is cleared
  runRound(updater, {});
  EXPECT_TRUE(target->new_merges.empty());

  // merging the parent re-parents its children
  runRound(updater, {{0, 3}});
  EXPECT_EQ(target->new_merges, Updates({{0, 3}, {1, 3}}));
  const std::map<NodeId, NodeId> expected{{0, 3}, {1, 3}};
  EXPECT_EQ(target->merges, expected);

  // resetting the backend graph undoes every merge
  updater.reset(10);
  runRound(updater, {});
  EXPECT_EQ(target->new_merges, Updates({{0, std::nullopt}, {1, std::nullopt}}));
  EXPECT_TRUE(target->merges.empty());
  EXPECT_TRUE(target->graph->hasNode(0));
  EXPECT_TRUE(target->graph->hasNode(1));

  runRound(updater, {});
  EXPECT_TRUE(target->new_merges.empty());
}

#ifdef HYDRA_ENABLE_IMAGE_STORE
TEST(DsgUpdater, MirrorsMergesIntoImageStore) {
  const test::TempDirectory tmp("dsg_updater_store");
  const auto aliases = [&]() {
    ImageStore::open(tmp.path)->flush();
    return test::queryStore(
        tmp.path, "SELECT child_id, parent_id FROM node_aliases ORDER BY child_id");
  };

  auto source = makeSource();
  auto target = test::makeSharedDsg();
  const auto guard = test::ConfigGuard::WithImageStore(tmp.path);
  DsgUpdater updater(makeConfig(), source, target);

  runRound(updater, {{1, 0}, {2, 0}});
  EXPECT_EQ(aliases(), test::StoreRows({{"1", "0"}, {"2", "0"}}));

  // re-parenting
  runRound(updater, {{0, 3}});
  EXPECT_EQ(aliases(), test::StoreRows({{"0", "3"}, {"1", "3"}, {"2", "3"}}));

  // removal after the merges are recomputed
  updater.reset(10);
  runRound(updater, {{2, 3}});
  EXPECT_EQ(aliases(), test::StoreRows({{"2", "3"}}));
  EXPECT_EQ(target->new_merges, Updates({{0, std::nullopt}, {1, std::nullopt}}));
}
#endif

}  // namespace hydra
