#include "grid_map_2d.h"
#include "srrg_boss/object_data.h"
#include <limits>
#include <utility>

namespace srrg2_core {

  GridMap2DHeader::~GridMap2DHeader() {
  }

  GridMap2DHeader::CoordinatesType GridMap2DHeader::boundingBox() const {
    float xmin = std::numeric_limits<float>::max();
    float xmax = -std::numeric_limits<float>::max();
    float ymin = std::numeric_limits<float>::max();
    float ymax = -std::numeric_limits<float>::max();
    for (int r = 0; r < 1; ++r) {
      for (int c = 0; c < 1; ++c) {
        IndicesType idx = IndicesType(0);
        if (r)
          idx(r) = _size(r);
        if (c)
          idx(c) = _size(c);
        CoordinatesType p = indices2local(Vector2i(0, 0));
        xmin              = std::min(xmin, p.x());
        ymin              = std::min(ymin, p.y());
        xmax              = std::max(xmax, p.x());
        ymax              = std::max(ymax, p.y());
      }
    }
    return CoordinatesType(xmax - xmin, ymax - ymin);
  }

  void GridMap2DHeader::setSize(const IndicesType& size_) {
    _size   = size_;
    _center = _resolution * _size.cast<float>() * 0.5;
    _resizeGrids(_size);
  }

  void GridMap2DHeader::setSize(const CoordinatesType& metric_size_) {
    _size = (metric_size_ * _inverse_resolution).cast<int>();
    setSize(_size);
  }

  void GridMap2DHeader::setResolution(float resolution_) {
    _inverse_resolution = 1. / resolution_;
    _resolution         = resolution_;
    _center             = _size.cast<float>() * _resolution * 0.5;
  }

  void GridMap2DHeader::setOrigin(const Eigen::Isometry2f& origin_) {
    _origin         = origin_;
    _inverse_origin = _origin.inverse();
  }

  void GridMap2DHeader::serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) {
    odata.setEigen<Isometry2f::MatrixType>("origin", _origin.matrix());
    odata.setEigen<Vector2i>("size", _size);
    odata.setFloat("resolution", _resolution);
  }

  void GridMap2DHeader::deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) {
    _origin.matrix() = odata.getEigen<Isometry2f::MatrixType>("origin");
    _size            = odata.getEigen<Vector2i>("size");
    _resolution      = odata.getFloat("resolution");
    setOrigin(_origin);
    setSize(_size);
    setResolution(_resolution);
  }

  void GridMap2DHeader::cornersInGlobal(Eigen::Matrix<float, 2, 4>& items) {
    items.col(0) = indices2global(Vector2i(0, 0));
    items.col(1) = indices2global(Vector2i(_size.x() - 1, 0));
    items.col(2) = indices2global(Vector2i(_size.x() - 1, _size.y() - 1));
    items.col(3) = indices2global(Vector2i(0, _size.y() - 1));
  }

  GridMap2D::~GridMap2D() {
  }

  void GridMap2D::_resizeGrids(const Eigen::Vector2i& size_) {
    for (auto& it : this->_properties) {
      PropertyBase* p                    = it.second;
      Property_<FrequencyGridType>* freq = dynamic_cast<Property_<FrequencyGridType>*>(p);
      if (freq) {
        freq->value().resize(size_.x(), size_.y());
        continue;
      }
      Property_<OccupancyGridType>* occ = dynamic_cast<Property_<OccupancyGridType>*>(p);
      if (occ) {
        occ->value().resize(size_.x(), size_.y());
        continue;
      }
      Property_<ImageOccupancyType>* im_occ = dynamic_cast<Property_<ImageOccupancyType>*>(p);
      if (im_occ) {
        im_occ->value().resize(size_.x(), size_.y());
        continue;
      }
    }
  }

  GridMap2DHeader GridMap2D::gridMapHeader() {
    GridMap2DHeader map_header;
    map_header.setSize(this->GridMap2DHeader::size());
    map_header.setResolution(this->resolution());
    map_header.setOrigin(this->origin());
    return map_header;
  }

  void initGridMap2D() {
    BOSS_REGISTER_CLASS(GridMap2DHeader);
    BOSS_REGISTER_CLASS(GridMap2D);
  }

} // namespace srrg2_core
