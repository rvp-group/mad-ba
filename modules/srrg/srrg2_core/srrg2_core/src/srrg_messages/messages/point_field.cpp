#include <srrg_messages/messages/point_field.h>

namespace srrg2_core {

  const uint8_t PointField::INT8    = 1;
  const uint8_t PointField::UINT8   = 2;
  const uint8_t PointField::INT16   = 3;
  const uint8_t PointField::UINT16  = 4;
  const uint8_t PointField::INT32   = 5;
  const uint8_t PointField::UINT32  = 6;
  const uint8_t PointField::FLOAT32 = 7;
  const uint8_t PointField::FLOAT64 = 8;

  PointField::PointField() :
    SETUP_PROPERTY(name, ""),
    SETUP_PROPERTY(offset, 0),
    SETUP_PROPERTY(datatype, 0),
    SETUP_PROPERTY(count, 0) {
    // ia nothin to do here
  }

  PointField::PointField(const PointField& cp_) :
    SETUP_PROPERTY(name, cp_.name.value()),
    SETUP_PROPERTY(offset, cp_.offset.value()),
    SETUP_PROPERTY(datatype, cp_.datatype.value()),
    SETUP_PROPERTY(count, cp_.count.value()) {
    // ia nothin to do here
  }

  std::ostream& operator<<(std::ostream& stream_, const PointField& point_field_) {
    stream_ << "name [ " << point_field_.name.value() << " ] - datatype [ "
            << (int) (point_field_.datatype.value()) << " ] - offset [ "
            << point_field_.offset.value() << " ] - count [ " << point_field_.count.value() << " ]";
    return stream_;
  }

} // namespace srrg2_core
