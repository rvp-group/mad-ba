#include <iostream>
#include "static_transform_tree.h"

namespace srrg2_core {

  StaticTransformTree::StaticTransformTree(): _root_frame_id("") {
  }

  StaticTransformTree::~StaticTransformTree() {
    if (_offset_map.size() > 0) {
      clear();
    }
  }

  bool StaticTransformTree::addStaticTransform(const StaticTransform& tf_) {
    const std::string& from_frame_id = tf_.fromFrameId();
    const std::string& to_frame_id = tf_.toFrameId();

    if (isPresent(to_frame_id)) {
      std::cerr << "error, another transform leading to frame_id " << to_frame_id << " exists" << std::endl;
      return false;
    }
    if (!isPresent(from_frame_id)) {
      _root_frame_id = from_frame_id;
    }
    _offset_map.insert(std::make_pair(to_frame_id, tf_));

    return true;
  }

  bool StaticTransformTree::updateTransform(const StaticTransform& tf_) {
    const std::string& to_frame_id = tf_.toFrameId();

    if (!isPresent(to_frame_id)) {
      std::cerr << "Transformation does not exists: call addStaticTransform instead.\n";
      return false;
    }

    const StaticTransform& other = getTransfromByFrame(to_frame_id);

    if (other.fromFrameId() != tf_.fromFrameId()) {
      printf("Calling updateTransform for different links\n");
      printf("input tf is  : [%s->%s]\n", other.fromFrameId().c_str(), other.toFrameId().c_str());
      printf("tf in tree is: [%s->%s]\n", tf_.fromFrameId().c_str(), tf_.toFrameId().c_str());
      printf("Nothing to do here\n");
      return false;
    }

    replaceTransform(tf_);

    if (!isWellFormed(false)) {
      std::cerr << "Not well formed." << std::endl;
      return false;
    }
    return true;
  }

  void StaticTransformTree::replaceTransform(const StaticTransform& static_transform_) {
    StringTransformMap::iterator it = _offset_map.find(static_transform_.toFrameId());
    if (it->second.transform().matrix() == static_transform_.transform().matrix()) {
      return;
    }
    _offset_map.erase(it);
    addStaticTransform(static_transform_);
  }

  bool StaticTransformTree::isWellFormed(bool verbose_) {
    if (!_offset_map.size()) {
      std::cerr << "empty map" << std::endl;
      return true;
    }
    if (verbose_) {
      std::cerr << "there are " << _offset_map.size() << " static transforms" << std::endl;
    }

    for (StringTransformMap::iterator it = _offset_map.begin(); it != _offset_map.end(); ++it) {
      Isometry3f offset;
      if (!getOffsetFromRoot(it->first, offset)) {
        if (verbose_) {
          std::cerr << "missing linking between " << _root_frame_id << " and " << it->second.toFrameId() << std::endl;
        }
      } else {
        if (verbose_) {
          std::cerr << "transform from [" << _root_frame_id << "] to ["
                    << it->second.toFrameId() << "] " << geometry3d::t2v(offset).transpose() << std::endl;
        }
      }
    }
    if (verbose_) {
      std::cerr << "StaticTransfromTree is well formed." << std::endl;
    }
    return true;
  }

  bool StaticTransformTree::getOffsetFromRoot(const std::string& to_frame_id_, Isometry3f& offset_) {
    Isometry3f T;
    T.setIdentity();

    std::string to_frame_id = to_frame_id_;
    StaticTransform static_transform = getTransfromByFrame(to_frame_id);
    while (isPresent(to_frame_id)) {
      T = static_transform.transform() * T;
      if (static_transform.fromFrameId() == _root_frame_id) {
        offset_ = T;
        return true;
      }
      to_frame_id = static_transform.fromFrameId();
      static_transform = getTransfromByFrame(to_frame_id);
    }
    return false;
  }

  const StaticTransform& StaticTransformTree::getTransfromByFrame(const std::string& to_frame_id_) const {
    StringTransformMap::const_iterator it = _offset_map.find(to_frame_id_);
    return it->second;
  }

  void StaticTransformTree::clear() {
    _offset_map.clear();
  }

  void StaticTransformTree::serialize(ObjectData& data, IdContext& context) {
    data.setString("root_frame_id", _root_frame_id);

    ArrayData* tree_array = new ArrayData;
    for (StringTransformMap::iterator it = _offset_map.begin(); it != _offset_map.end(); ++it) {
      ObjectData* odata = new ObjectData;
      StaticTransform tf = it->second;
      tf.serialize(*odata, context);

      tree_array->add(odata);
    }

    data.setField("transforms_map", tree_array);
  }

  void StaticTransformTree::deserialize(ObjectData& data, IdContext& context) {
    clear();
    _root_frame_id = data.getString("root_frame_id");
    ArrayData& odata_vector = data.getField("transforms_map")->getArray();
    for (int i = 0; i < odata_vector.size(); ++i) {
      ObjectData& odata = odata_vector[i].getObject();
      StaticTransform tf;
      tf.deserialize(odata, context);
      addStaticTransform(tf);
    }
  }

  bool StaticTransformTree::isPresent(const std::string& to_frame_id_) {
    StringTransformMap::const_iterator it = _offset_map.find(to_frame_id_);
    if (it == _offset_map.end()) {
      return false;
    }
    return true;
  }

  BOSS_REGISTER_CLASS(StaticTransformTree);

} /* namespace srrg2_core */
