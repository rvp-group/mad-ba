#include <Eigen/Cholesky>
namespace srrg2_solver{

  template <int Dim_>
  struct Unscented_ {
    static constexpr int Dim=Dim_;
    static constexpr int NumPoints=2*Dim+1;
    static constexpr float alpha=1e-3;
    static constexpr float beta =2;
    static constexpr float lambda = alpha*alpha*Dim;
    static constexpr float wn= 0.5/(Dim+lambda);
    static constexpr float wm0= lambda/(Dim+lambda);
    static constexpr float wc0= wm0 + (1-alpha*alpha+beta);
   
    using VectorType = Eigen::Matrix<float, Dim, 1>;
    using MatrixType = Eigen::Matrix<float, Dim, Dim>;
    struct SigmaPoint {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VectorType x;
      float wc;
      float wm;
    };

    SigmaPoint sigma_points[Dim*2+1];

    void compute(const VectorType& mu, const MatrixType& sigma) {
      Eigen::LLT<MatrixType> llt;
      llt.compute(sigma*(Dim+lambda));
      MatrixType L=llt.matrixL();
      sigma_points[0].x=mu;
      sigma_points[0].wc=wc0;
      sigma_points[0].wm=wm0;
      int k=1;
      for (int i=0; i<Dim; ++i) {
        sigma_points[k].x=mu + L.col(i);
        sigma_points[k].wc=sigma_points[k].wm=wn;
        ++k;
        sigma_points[k].x=mu - L.col(i);
        sigma_points[k].wc=sigma_points[k].wm=wn;
        ++k;
      }
    }

    static void recover(VectorType& mu,
                        MatrixType& sigma,
                        SigmaPoint* sigma_points_,
                        const int n) {
      mu.setZero();
      sigma.setZero();
      if (n<Dim*2+1) {
        throw std::runtime_error("unscented recovery fail, too few sigma points");
      }
      
      for (int i=0; i<n; ++i) {
        mu.noalias()+=sigma_points_[i].wm*sigma_points_[i].x;
      }
      for (int i=0; i<n; ++i) {
        VectorType dm=sigma_points_[i].x-mu;
        sigma.noalias()+=sigma_points_[i].wc*dm*dm.transpose();
      }
    }
  };
}
