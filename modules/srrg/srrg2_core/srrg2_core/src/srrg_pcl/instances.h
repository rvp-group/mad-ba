#pragma once
#include "normal_computator.h"
#include "point_projector_lidar3d_types.h"
#include "point_projector_types.h"
#include "point_types_data.h"
#include "point_unprojector_lidar3d_types.h"
#include "point_unprojector_types.h"

namespace srrg2_core {

  void point_cloud_registerTypes() __attribute__((constructor));
}
