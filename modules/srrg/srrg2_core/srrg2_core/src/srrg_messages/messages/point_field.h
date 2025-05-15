#pragma once
#include "srrg_property/property_container.h"

namespace srrg2_core {

  class PointField : public PropertyContainerIdentifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointField();
    PointField(const PointField& cp_);
    static const uint8_t INT8;
    static const uint8_t UINT8;
    static const uint8_t INT16;
    static const uint8_t UINT16;
    static const uint8_t INT32;
    static const uint8_t UINT32;
    static const uint8_t FLOAT32;
    static const uint8_t FLOAT64;
    PropertyString name;
    PropertyUnsignedInt offset;
    PropertyUInt8 datatype;
    PropertyUnsignedInt count;
  };

  std::ostream& operator<<(std::ostream& stream_, const PointField& point_field_);

} // namespace srrg2_core
