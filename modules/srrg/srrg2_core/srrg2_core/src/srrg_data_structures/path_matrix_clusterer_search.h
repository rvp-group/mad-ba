#pragma once
#include "path_matrix.h"
#include "path_matrix_search_base.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_image/image.h"
#include <stdexcept>

namespace srrg2_core {

  typedef Vector2_<int> Vector2i;
  typedef std::vector<Vector2i, Eigen::aligned_allocator<Vector2i>>
    Vector2iVector;
  typedef Vector3_<uint8_t> Vector3uc;

  class ClustererPathSearch : public PathMatrixSearchBase {
  public:
    struct Cluster {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      PathMatrixCell* cell;
      Vector2i lower;
      Vector2i upper;
      float mean_r;
      float mean_c;
      int point_count;
      Vector3uc color;
      Vector2iVector pixels;
    };
    typedef std::vector<Cluster, Eigen::aligned_allocator<Cluster>>
      ClusterVector;
    ClustererPathSearch();

    //! @param regions image: this is the input.
    // cell to cluster should be set to 0
    // each cell to ignore should be set to -1;
    inline void setRegionsImage(const ImageInt& regions_image) {
      _regions_image = &regions_image;
    }
    inline void setOutputPathMatrix(PathMatrix& path_matrix_) {
      _path_map = &path_matrix_;
    }

    void setColor(int color_);
    const Vector3uc& color() {
      return _color;
    }

    virtual void init(); //< call this once after setting indices image

    virtual bool compute();

    // returns a vector of cluster centroids
    inline const ClusterVector& clusters() const {
      return _clusters;
    }
    inline const ImageVector3uc& clusterImage() const {
      return _cluster_image;
    }

  protected:
    void fillFromImage();
    void expandRegion(PathMatrixCell* cell);

    int _max_index;
    size_t _num_operations;
    Vector3uc _color;
    const ImageInt* _regions_image;
    // New RGB image in srrg2_core
    ImageVector3uc _cluster_image;
    PathMatrix* _path_map;
    ClusterVector _clusters;
  };

} // namespace srrg2_core
