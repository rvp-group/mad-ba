#pragma once
namespace srrg2_core {

  enum TRANSFORM_CLASS {
    Isometry            = 0x0,
    PinholeProjection   = 0x1,
    PinholeUnprojection = 0x2,
    PolarProjection     = 0x3,
    PolarUnprojection   = 0x4
  };

  enum POINT_STATUS { Valid, OutOfRange, BehindObserver, Invalid };

} // namespace srrg2_core
