#pragma once

#include <map>
#include "static_transform.h"

namespace srrg2_core {
  class StaticTransformTree: public Serializable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::map<std::string,
                     StaticTransform,
                     std::less<std::string>,
                     Eigen::aligned_allocator<std::pair<std::string, StaticTransform> > > StringTransformMap;
    StaticTransformTree();
    virtual ~StaticTransformTree();

    bool addStaticTransform(const StaticTransform& static_transform_);
    bool updateTransform(const StaticTransform& static_transform_);

    inline const std::string& rootFrameId() const {
      return _root_frame_id;
    }

    inline const StringTransformMap& offsetMap() const {
      return _offset_map;
    }

    const StaticTransform& getTransfromByFrame(const std::string& to_frame_id_) const;
    bool isWellFormed(bool verbose_ = true);
    bool getOffsetFromRoot(const std::string& to_frame_id_, Isometry3f& offset_);
    bool isPresent(const std::string& to_frame_id_);
    void clear();

    virtual void serialize(ObjectData& data, IdContext& context) override;
    virtual void deserialize(ObjectData& data, IdContext& context) override;

  private:
    void replaceTransform(const StaticTransform& static_transform_);
    StringTransformMap _offset_map;
    std::string _root_frame_id;
  };

} /* namespace srrg2_core */

