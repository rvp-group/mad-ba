#pragma once
#include "srrg_boss/serializable.h"
#include "srrg_data_structures/matrix.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_property/property_container.h"

namespace srrg2_core {
  class GridMap2DHeader : public Serializable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using IndicesType     = Vector2i;
    using CoordinatesType = Vector2f;

    virtual ~GridMap2DHeader();
    inline CoordinatesType global2local(const CoordinatesType& global) const {
      return _inverse_origin * global;
    }

    inline CoordinatesType local2global(const CoordinatesType& local) const {
      return _origin * local;
    }

    inline IndicesType local2indices(const CoordinatesType& local) const {
      return ((local + _center) * _inverse_resolution).cast<int>();
    }

    inline CoordinatesType local2floatIndices(const CoordinatesType& local) const {
      return ((local + _center) * _inverse_resolution);
    }

    inline IndicesType global2indices(const CoordinatesType& global) const {
      return ((global2local(global) + _center) * _inverse_resolution).cast<int>();
    }

    inline CoordinatesType global2floatIndices(const CoordinatesType& global) const {
      return ((global2local(global) + _center) * _inverse_resolution);
    }

    inline CoordinatesType indices2local(const IndicesType& indices) const {
      return indices.cast<float>() * _resolution - _center;
    }

    inline CoordinatesType floatIndices2local(const CoordinatesType& indices) const {
      return indices * _resolution - _center;
    }

    inline CoordinatesType floatIndices2global(const CoordinatesType& indices) const {
      return local2global(floatIndices2local(indices));
    }

    inline CoordinatesType indices2global(const IndicesType& indices) const {
      return local2global(indices2local(indices));
    }

    inline CoordinatesType boundingBoxLocal() const {
      return _size.cast<float>() * _resolution;
    }

    inline const Eigen::Isometry2f& origin() const {
      return _origin;
    }
    inline float resolution() const {
      return _resolution;
    }
    inline IndicesType size() const {
      return _size;
    }
    CoordinatesType boundingBox() const;
    void setSize(const IndicesType& size_);
    void setSize(const CoordinatesType& metric_size_);
    void setResolution(float resolution_);
    void setOrigin(const Eigen::Isometry2f& origin_);
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
    void cornersInGlobal(Eigen::Matrix<float, 2, 4>& items); // takes a 4 array
    // bb get origin for debug
    inline CoordinatesType center() const {
      return _center;
    }

  protected:
    virtual void _resizeGrids(const Eigen::Vector2i& size_) {
    }
    Eigen::Isometry2f _origin         = Eigen::Isometry2f::Identity(); // center of the local map
    Eigen::Isometry2f _inverse_origin = Eigen::Isometry2f::Identity(); // center of the local map
    IndicesType _size                 = IndicesType::Zero();
    float _resolution                 = 0.05;
    float _inverse_resolution         = 1. / 0.05;
    CoordinatesType _center           = CoordinatesType::Zero();
  };

  using FrequencyGridType =
    srrg2_core::Matrix_<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

  using OccupancyGridType = srrg2_core::Matrix_<float>;

  using ImageOccupancyType = srrg2_core::Matrix_<uint8_t>;

  // the grid map is just a header and a bunch of properties
  struct GridMap2D : public PropertyContainerDerived_<GridMap2DHeader> {
    virtual ~GridMap2D();
    virtual void _resizeGrids(const Eigen::Vector2i& size_);
    virtual GridMap2DHeader gridMapHeader();
  };

  using GridMap2DPtr = std::shared_ptr<GridMap2D>;
  void initGridMap2D() __attribute__((constructor));

} // namespace srrg2_core
