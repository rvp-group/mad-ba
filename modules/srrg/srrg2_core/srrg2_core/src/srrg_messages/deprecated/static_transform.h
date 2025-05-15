#pragma once

#include <string>
#include <memory>
#include <srrg_boss/identifiable.h>
#include <srrg_boss/eigen_boss_plugin.h>  //mc needs to be called before geometry definition
#include <srrg_geometry/geometry_defs.h>
#include <srrg_geometry/geometry3d.h>

namespace srrg2_core {

  class StaticTransform: public Serializable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StaticTransform(const std::string& from_frame_id_ = "",
                    const std::string& to_frame_id_ = "",
                    const Isometry3f& transform_ = Isometry3f::Identity());
    virtual ~StaticTransform();

    inline void setFromFrameId(const std::string& from_frame_id_) {
      _from_frame_id = from_frame_id_;
    }

    inline void setToFrameId(const std::string& to_frame_id_) {
      _to_frame_id = to_frame_id_;
    }

    inline void setTransform(const Isometry3f& transform_) {
      _transform = transform_;
    }

    inline const std::string& fromFrameId() const {
      return _from_frame_id;
    }

    inline const std::string& toFrameId() const {
      return _to_frame_id;
    }

    inline const Isometry3f& transform() const {
      return _transform;
    }

    virtual void serialize(ObjectData& data, IdContext& context) override;
    virtual void deserialize(ObjectData& data, IdContext& context) override;

  private:
    std::string _from_frame_id, _to_frame_id;
    Isometry3f _transform;
  };

} /* namespace srrg2_core */

