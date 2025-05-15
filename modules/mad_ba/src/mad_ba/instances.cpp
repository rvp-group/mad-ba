#include "instances.h"
#include "point_cloud_proc.h"
#include "surfel.h"

namespace mad_ba {

  void registerTypes() {
    BOSS_REGISTER_CLASS(PointCloudProc);
    BOSS_REGISTER_CLASS(Surfel);
  }
} // namespace mad_ba
