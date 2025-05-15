#include "epipolar.h"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include "permutation_sampler.h"
#include "srrg_geometry/geometry3d.h"

namespace srrg2_core{
  using namespace std;
  
  namespace epipolar {
    using Matrix3_2f = Eigen::Matrix<float, 3, 2>;
    using Matrix9f   = Eigen::Matrix<float,9,9>;
    using Matrix3_9f = Eigen::Matrix<float, 3, 9>;
    using Vector9f   = Eigen::Matrix<float, 9, 1>;
    
    float triangulatePoint(Vector3f& p,
                           const Line3f& a,
                           const Line3f& b,
                           Vector2f& s) {
      const float epsilon=1e-9;
      Matrix3_2f D;
      D.col(0)=-a.direction;
      D.col(1)=b.direction;
      Matrix2f DtD=D.transpose()*D;
      if (fabs(DtD.determinant())<=epsilon)
        return -3.;
      s=DtD.inverse()*D.transpose()*(a.point-b.point);
      if (s(0)<0)
        return -1.;
      if (s(1)<0)
        return -2;
      Vector3f pa=a.point+a.direction*s(0);
      Vector3f pb=b.point+b.direction*s(1);
      p=0.5*(pa+pb);
      return (pa-pb).norm();
    }

    float triangulatePoint(Vector3f& p,
                           const Line3f& a,
                           const Line3f& b) {
      Vector2f s;
      return triangulatePoint(p,a,b,s);
    }

    int triangulateDirections(Vector3fVector& points,
                              std::vector<float>& results,
                              const Isometry3f& iso1,
                              const Isometry3f& iso2,
                              const Vector3fVector& directions1,
                              const Vector3fVector& directions2){
      assert(directions1.size()==directions2.size() && "directions size mismatch");
      size_t size=directions1.size();
      points.resize(size);
      results.resize(size);
      Line3f l1(iso1.translation(), Vector3f::Zero());
      Line3f l2(iso2.translation(), Vector3f::Zero());
      int good=0;
      for (size_t i=0; i<size; ++i) {
        l1.direction = iso1.linear()*directions1[i];
        l2.direction = iso2.linear()*directions2[i];
        results[i]=triangulatePoint(points[i], l1, l2);
        if (results[i]>=0)
          ++good;
      }
      return good;
    }


    void projectPointsOnSphere(Vector3fVector& dirs,
                               const Isometry3f& iso,
                               const Vector3fVector& points){
      dirs.resize(points.size());
      Isometry3f invT=iso.inverse();
      for (size_t i=0; i<points.size(); ++i) {
        dirs[i]=(invT*points[i]);
        dirs[i].normalize();
      }
    }

    Matrix3f iso2essential(const Eigen::Isometry3f& iso){
      const Vector3f t=iso.translation();
      return iso.linear().transpose()*geometry3d::skew(t);
    }

    void essential2iso(Isometry3f iso[2],
                       const Matrix3f& E) {
      Matrix3f W;
      W << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
      Eigen::JacobiSVD<Matrix3f> svd(E, Eigen::ComputeFullU|Eigen::ComputeFullV);
      Eigen::Matrix3f R=svd.matrixV()*W*svd.matrixU().transpose();
      if(R.determinant()<0) {
        svd=Eigen::JacobiSVD<Matrix3f>(-E, Eigen::ComputeFullU|Eigen::ComputeFullV);
        R=svd.matrixV()*W*svd.matrixU().transpose();
      }
      iso[0].setIdentity();
      iso[0].linear()=R;
      Matrix3f t_cross=R*E;
      iso[0].translation()=Vector3f(t_cross(2,1)-t_cross(1,2),
                                    t_cross(0,2)-t_cross(2,0),
                                    t_cross(1,0)-t_cross(0,1));

      iso[1].setIdentity();
      R=svd.matrixV()*W.transpose()*svd.matrixU().transpose();
      iso[1].linear()=R;
      t_cross=R*E;
      iso[1].translation()=Vector3f(t_cross(2,1)-t_cross(1,2),
                                    t_cross(0,2)-t_cross(2,0),
                                    t_cross(1,0)-t_cross(0,1));
    }

    inline Matrix3f meanR(const Vector3f& v) {
      using namespace geometry3d;
      float alpha_y=atan2(-v.x(), v.z());
      float q=-sin(alpha_y)*v.x()+cos(alpha_y)*v.z();
      float alpha_x=atan2(v.y(), q);
      return rotationX(alpha_x)*rotationY(alpha_y);
    }
    
    float eightPointEstimate(Matrix3f& dest,
                             const Vector3fVector& dirs1,
                             const Vector3fVector& dirs2,
                             bool normalize_directions) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch");
      float n=dirs1.size();
      using Matrix9f=Eigen::Matrix<float, 9,9>;
      using Vector9f=Eigen::Matrix<float, 9,1>;
      Matrix9f H;
      H.setZero();

      Eigen::Matrix3f Omega1=Eigen::Matrix3f::Identity();
      Eigen::Matrix3f Omega2=Eigen::Matrix3f::Identity();
      Eigen::Matrix3f R1=Eigen::Matrix3f::Identity();
      Eigen::Matrix3f R2=Eigen::Matrix3f::Identity();
      
      if (normalize_directions) {
        //rebalance covariances;
        Vector3f mu1=Vector3f::Zero(), mu2=Vector3f::Zero();
        for (size_t i=0; i<dirs1.size(); ++i) {
          const Vector3f& d1=dirs1[i];
          const Vector3f& d2=dirs2[i];
          mu1+=d1;
          mu2+=d2;
        }
        mu1.normalize();
        mu2.normalize();
      
        Matrix3f Sigma1=Matrix3f::Zero(), Sigma2=Matrix3f::Zero();
        for (size_t i=0; i<dirs1.size(); ++i) {
          const Vector3f d1=R1*dirs1[i];
          const Vector3f d2=R2*dirs2[i];
          Sigma1+=d1*d1.transpose();
          Sigma2+=d2*d2.transpose();
        }
        Sigma1*=1./(n-1);
        Sigma2*=1./(n-1);
        Omega1=Sigma1.inverse();
        Omega2=Sigma2.inverse();
      }
      for (size_t i=0; i<dirs1.size(); ++i) {
        const Vector3f d1=Omega1*dirs1[i];
        const Vector3f d2=Omega2*dirs2[i];
        Vector9f a;
        int k=0;
        for (int c=0; c<3; ++c)
          for (int r=0; r<3; ++r, ++k) {
            a[k]=d1(c)*d2(r);
          }
        H.noalias()+=a*a.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 9,9> > eig(H);
      int k=0;
      for (int c=0; c<3; ++c){
        for (int r=0; r<3; ++r, ++k) {
          dest(r,c)=eig.eigenvectors()(k,0);
        }
      }
      dest=Omega2*dest*Omega1;
      dest.normalize();
      
      return eig.eigenvalues()(0);
    }

    float estimateTransformFromDirections(Isometry3f& iso,
                                          const Vector3fVector& dirs1,
                                          const Vector3fVector& dirs2,
                                          bool normalize_directions) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch");
      Matrix3f E;
      /*float lambda=*/eightPointEstimate(E, dirs1, dirs2, normalize_directions);
      Isometry3f iso_v[2];
      essential2iso(iso_v,E);
      Vector3fVector temp_pts(dirs1.size());
      std::vector<float> temp_errors;
      int best_points=0;
      for (int sol=0; sol<2; ++sol) {
        Isometry3f temp_iso=iso_v[sol];
        int temp_good = triangulateDirections(temp_pts,
                                              temp_errors, 
                                              Isometry3f::Identity(),
                                              temp_iso,
                                              dirs1, dirs2);
        if (temp_good>best_points) {
          best_points=temp_good;
          iso=temp_iso;
        }
        temp_iso.translation()=-temp_iso.translation();
        temp_good = triangulateDirections(temp_pts,
                                          temp_errors,
                                          Isometry3f::Identity(),
                                          temp_iso,
                                          dirs1, dirs2);
        if (temp_good>best_points) {
          best_points=temp_good;
          iso=temp_iso;
        }
      }
      return best_points;
    }

    int scoreEssential(std::vector<float>& errors,
                       const Matrix3f& E,
                       const Vector3fVector& dirs1,
                       const Vector3fVector& dirs2,
                       const float inlier_threshold) {
      int num_inliers=0;
      assert(dirs1.size()==dirs2.size());
      const size_t s=dirs1.size();
      errors.resize(s);
      float total_error=0;
      for (size_t i=0; i<s; ++i) {
        errors[i]=fabs(dirs2[i].transpose()*E*dirs1[i]);
        total_error+=errors[i];
        if (fabs(errors[i])<inlier_threshold)
          ++num_inliers;
      }
      return num_inliers;
    }

    int estimateEssentialRANSACEightPts(Matrix3f& E,
                                        std::vector<float>& errors,
                                        const Vector3fVector& dirs1,
                                        const Vector3fVector& dirs2,
                                        float inlier_threshold,
                                        int num_rounds,
                                        bool normalize_directions,
                                        const std::vector<double>& weights_) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch");
      size_t s=dirs1.size();
      assert(s>=8 && "not enough correspondences for 8 pt estimate");
      int best_num_inliers=0;
      Vector3fVector dirs1_current(8);
      Vector3fVector dirs2_current(8);
      std::vector<float> errors_current(s);
      std::vector<double> weights(s, 1.f);
      if (weights_.size()) {
        assert(weights.size()==s && "weights size mismatch");
        weights=weights_;
      }
      PermutationSampler sampler(weights);
      for (int round_num=0; round_num<num_rounds; ++round_num) {
        // std::cerr << "RANSAC:, round: " << round_num << "[ ";
        for (size_t i=0; i<8; ++i) {
          double r=drand48()*sampler.getSum();
          int idx=sampler.sampleWithRemoval(r);
          dirs1_current[i]=dirs1[idx];
          dirs2_current[i]=dirs2[idx];
          // std::cerr << idx << " ";
        }
        sampler.recoverWeights();
        Matrix3f E_current;
        E_current.setZero();
        eightPointEstimate(E_current, dirs1_current, dirs2_current, normalize_directions);
        int current_inliers=scoreEssential(errors_current, E_current, dirs1, dirs2, inlier_threshold);
        // std::cerr << "], inliers: " << current_inliers << std::endl;
        if (current_inliers>best_num_inliers) {
          E=E_current;
          errors=errors_current;
          best_num_inliers=current_inliers;
        }
      }
      return best_num_inliers;
    }

    float estimateRotation(Matrix3f& R,
                           Vector3fVector& dirs1,
                           Vector3fVector& dirs2) {
      assert (dirs1.size()==dirs2.size() &&" dirs size differ");
      assert (dirs1.size()>2 &&" too few points");
      Matrix9f H;
      H.setZero();
      Vector9f b;
      b.setZero();
      Vector9f x;
      x.setZero();
      Matrix3_9f J;
      J.setZero();
      for (size_t i=0; i<dirs1.size(); ++i) {
        const Vector3f& d1=dirs1[i];
        const Vector3f& d2=dirs2[i];
        J.block<1,3>(0,0)=d2.transpose();
        J.block<1,3>(1,3)=d2.transpose();
        J.block<1,3>(2,6)=d2.transpose();
        H+=J.transpose()*J;
        b+=J.transpose()*d1;
      }
      x=H.ldlt().solve(b);
      int k=0;
      for (int r=0; r<3; ++r)
        for (int c=0; c<3; ++c, ++k)
          R(r,c)=x(k);

      // // renormalize solution
      Eigen::JacobiSVD<Matrix3f> svd(R, Eigen::ComputeFullU|Eigen::ComputeFullV);
      R=svd.matrixU()*svd.matrixV().transpose();
      double chi=0;
      for (size_t i=0; i<dirs1.size(); ++i) {
        const Vector3f& d1=dirs1[i];
        const Vector3f& d2=dirs2[i];
        Vector3f e=d1-R*d2;
        chi+=e.squaredNorm();
      }
      return chi;
    }

    
    float  estimateRotationTwoPts(Matrix3f& R,
                                  const Vector3fVector& dirs1,
                                  const Vector3fVector& dirs2,
                                  int idx_1,
                                  int idx_2,
                                  const float s_min,
                                  const float epsilon) {
      using namespace std;
      R.setIdentity();
      assert (dirs1.size()==dirs2.size() &&" dirs size differ");
      assert (dirs1.size()>= 2 &&" need at least 2 dirs");
      Vector3f v1=dirs1[idx_1].cross(dirs1[idx_2]);
      Vector3f v2=dirs2[idx_1].cross(dirs2[idx_2]);
      // construct two rotation matrices
      float n1=v1.norm();
      float n2=v2.norm();
      if(n1<s_min || n2<s_min) {
        return -1;
      }
      if (fabs(n1-n2)>epsilon) {
        return -2;
      }
      v1.normalize();
      v2.normalize();
      Vector3fVector pts1(3), pts2(3);
      pts1[0]=dirs1[idx_1];
      pts1[1]=dirs1[idx_2];
      pts1[2]=v1;

      pts2[0]=dirs2[idx_1];
      pts2[1]=dirs2[idx_2];
      pts2[2]=v2;

      return estimateRotation(R, pts1, pts2);
    }

    int scoreRotation(std::vector<float>& errors,
                      const Matrix3f& R,
                      const Vector3fVector& dirs1,
                      const Vector3fVector& dirs2,
                      const float inlier_threshold) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch");
      errors.resize(dirs1.size());
      int num_inliers=0;
      for (size_t i=0; i<dirs1.size(); ++i) {
        const Vector3f& d1=dirs1[i];
        const Vector3f& d2=dirs2[i];
        Vector3f e=R*d2-d1;
        errors[i]=e.squaredNorm();
        if (errors[i]<inlier_threshold)
          ++num_inliers;
      }
      return num_inliers;
    }

    int estimateRotationRANSACTwoPts(Matrix3f& R,
                                     std::vector<float>& errors,
                                     const Vector3fVector& dirs1,
                                     const Vector3fVector& dirs2,
                                     float inlier_threshold,
                                     int num_rounds,
                                     const std::vector<double>& weights_) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch");
      size_t s=dirs1.size();
      assert(s>=2 && "not enough correspondences for 2 pt estimate");
      int best_num_inliers=0;
      Vector3fVector dirs1_current(2);
      Vector3fVector dirs2_current(2);
      std::vector<float> errors_current(s);
      std::vector<double> weights(s, 1.f);
      if (weights_.size()) {
        assert(weights.size()==s && "weights size mismatch");
        weights=weights_;
      }
      PermutationSampler sampler(weights);
      for (int round_num=0; round_num<num_rounds; ++round_num) {
        // std::cerr << "RANSAC:, round: " << round_num << "[ ";
        for (size_t i=0; i<2; ++i) {
          double r=drand48()*sampler.getSum();
          int idx=sampler.sampleWithRemoval(r);
          dirs1_current[i]=dirs1[idx];
          dirs2_current[i]=dirs2[idx];
          // std::cerr << idx << " ";
        }
        sampler.recoverWeights();
        Matrix3f R_current;
        R_current.setIdentity();
        float error=estimateRotationTwoPts(R_current, dirs1_current, dirs2_current, 0, 1);
        if (error<0)
          continue;
        int current_inliers=scoreRotation(errors_current, R_current, dirs1, dirs2, inlier_threshold);
        // std::cerr << "], inliers: " << current_inliers << std::endl;
        if (current_inliers>best_num_inliers) {
          R=R_current;
          errors=errors_current;
          best_num_inliers=current_inliers;
        }
      }
      return best_num_inliers;
    }

    int estimateTransformRANSAC(Isometry3f& iso,
                                std::vector<float>& errors,
                                const Vector3fVector& dirs1,
                                const Vector3fVector& dirs2,
                                float inlier_threshold,
                                int num_rounds,
                                bool normalize_directions,
                                const std::vector<double>& weights_) {
      assert(dirs1.size()==dirs2.size() && "dir size mismatch"); 
      Matrix3f E;
      vector<float> errors_essential;
      int inliers_essential =
        estimateEssentialRANSACEightPts(E, errors_essential, dirs1, dirs2, inlier_threshold, num_rounds, normalize_directions, weights_);
      //cerr << "Essential (est) :" << endl << E << endl;
      Vector3fVector d1, d2;
      int k=0;
      for (size_t i=0; i<errors_essential.size(); ++i) {
        if (errors_essential[i]<inlier_threshold) {
          d1.push_back(dirs1[i]);
          d2.push_back(dirs2[i]);
          ++k;
        }
      }
      //cerr << "compute transform " << k << endl;
      inliers_essential = estimateTransformFromDirections(iso, d1, d2);
      //cerr << "Iso (est, essential): " << endl << iso.matrix() << endl;
      Matrix3f R;
      vector<float> errors_rotation;
      int inliers_rotation =
        estimateRotationRANSACTwoPts(R, errors_rotation, dirs1, dirs2, inlier_threshold, num_rounds, weights_);

      //cerr << "inliers rotation: " << inliers_rotation;
      //cerr << "inliers essential: " << inliers_essential;
      
      if (inliers_rotation>inliers_essential) {
        iso.setIdentity();
        iso.linear()=R;
        errors=errors_rotation;
        return inliers_rotation;
      } else {
        errors=errors_essential;
        return inliers_essential;
      }
    }
  }
}
