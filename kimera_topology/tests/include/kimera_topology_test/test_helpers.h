#pragma once
#include <gtest/gtest.h>
#include <kimera_topology/gvd_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>

namespace kimera {
namespace topology {
namespace test_helpers {

using voxblox::EsdfIntegrator;

GvdIntegratorConfig gvdConfigFromEsdfConfig(const EsdfIntegrator::Config& esdf);

}  // namespace test_helpers
}  // namespace topology
}  // namespace kimera
