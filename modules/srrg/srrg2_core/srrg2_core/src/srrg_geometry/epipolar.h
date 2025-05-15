#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

namespace srrg2_core{
  namespace epipolar {
    using Vector3f=Eigen::Vector3f;
    using Vector3fVector=std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;
    using Isometry3f=Eigen::Isometry3f;
    using Vector2f=Eigen::Vector2f;
    using Matrix3f=Eigen::Matrix3f;
    using Matrix2f=Eigen::Matrix2f;

    struct Line3f {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Vector3f point=Vector3f::Zero();
      Vector3f direction=Vector3f::Zero();
      Line3f(){}
      Line3f(const Vector3f& point_,
             const Vector3f& direction_):
        point(point_),
        direction(direction_){}
    
      inline Line3f transform(const Isometry3f& iso) const {
        return Line3f(iso*point, iso.linear()*direction);
      }
      
      inline const Line3f& transformInPlace(const Isometry3f& iso) {
        point=iso*point;
        direction=iso.linear()*direction;
        return *this;
      }
      
    };


    float triangulatePoint(Vector3f& p,
                           const Line3f& a,
                           const Line3f& b,
                           Vector2f& s);
    
    float triangulatePoint(Vector3f& p,
                           const Line3f& a,
                           const Line3f& b);
    
    int triangulateDirections(Vector3fVector& points,
                              std::vector<float>& results,
                              const Isometry3f& iso1,
                              const Isometry3f& iso2,
                              const Vector3fVector& directions1,
                              const Vector3fVector& directions2);

    void projectPointsOnSphere(Vector3fVector& dirs,
                               const Isometry3f& iso,
                               const Vector3fVector& points);
    
    Matrix3f iso2essential(const Eigen::Isometry3f& iso);

    void essential2iso(Isometry3f iso[2],
                       const Matrix3f& E);
    
    float eightPointEstimate(Matrix3f& dest,
                             const Vector3fVector& dirs1,
                             const Vector3fVector& dirs2,
                             bool normalize_directions=true);
    
    float estimateTransformFromDirections(Isometry3f& iso,
                                          const Vector3fVector& dirs1,
                                          const Vector3fVector& dirs2,
                                          bool normalize_directions=true);
    
    int scoreEssential(std::vector<float>& errors,
                       const Matrix3f& E,
                       const Vector3fVector& dirs1,
                       const Vector3fVector& dirs2,
                       const float inlier_threshold);

    int estimateEssentialRANSACEightPts(Matrix3f& E,
                                        std::vector<float>& errors,
                                        const Vector3fVector& dirs1,
                                        const Vector3fVector& dirs2,
                                        float inlier_threshold,
                                        int num_rounds,
                                        bool normalize_directions=true,
                                        const std::vector<double>& weights_=std::vector<double>());

    float  estimateRotationTwoPts(Matrix3f& R,
                                  const Vector3fVector& dirs1,
                                  const Vector3fVector& dirs2,
                                  int idx_1,
                                  int idx_2,
                                  const float s_min=0.1,
                                  const float epsilon=1e-3);

    float estimateRotation(Matrix3f& R,
                           Vector3fVector& dirs1,
                           Vector3fVector& dirs2);
    
    int scoreRotation(std::vector<float>& errors,
                      const Matrix3f& R,
                      const Vector3fVector& dirs1,
                      const Vector3fVector& dirs2,
                      const float inlier_threshold);
    
    int estimateRotationRANSACTwoPts(Matrix3f& R,
                                     std::vector<float>& errors,
                                     const Vector3fVector& dirs1,
                                     const Vector3fVector& dirs2,
                                     float inlier_threshold,
                                     int num_rounds,
                                     const std::vector<double>& weights_=std::vector<double>());

    int estimateTransformRANSAC(Isometry3f& iso,
                                std::vector<float>& errors,
                                const Vector3fVector& dirs1,
                                const Vector3fVector& dirs2,
                                float inlier_threshold,
                                int num_rounds,
                                bool normalize_directions=true,
                                const std::vector<double>& weights_=std::vector<double>());
  }


}
