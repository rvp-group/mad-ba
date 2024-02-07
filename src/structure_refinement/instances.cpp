#include "instances.h"
#include "point_cloud_proc.h"
#include "surfel.h"

namespace structure_refinement {

  void registerTypes() {
    BOSS_REGISTER_CLASS(PointCloudProc);
    BOSS_REGISTER_CLASS(Surfel);
  }
} // namespace structure_refinement
