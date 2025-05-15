#include "static_transform.h"

namespace srrg2_core {

  StaticTransform::StaticTransform(const std::string& from_frame_id_,
                                   const std::string& to_frame_id_,
                                   const Isometry3f& transform_):
    _from_frame_id(from_frame_id_),
    _to_frame_id(to_frame_id_),
    _transform(transform_) {
  }

  StaticTransform::~StaticTransform() {
  }

  void StaticTransform::serialize(ObjectData& data, IdContext& context) {
    data.setString("from_frame_id", _from_frame_id);
    data.setString("to_frame_id", _to_frame_id);
    geometry3d::t2v(_transform).toBOSS(data, "transform");
  }

  void StaticTransform::deserialize(ObjectData& data, IdContext& context) {
    _from_frame_id = data.getString("from_frame_id");
    _to_frame_id = data.getString("to_frame_id");
    Vector6f transform_v;
    transform_v.fromBOSS(data, "transform");
    _transform = geometry3d::v2t(transform_v);
  }

  BOSS_REGISTER_CLASS(StaticTransform);

} /* namespace srrg2_core */
