#pragma once
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"

#include <Eigen/Cholesky>

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  struct FactorNoiseAdderBase {
    default_random_engine& rnd_gen;
    std::normal_distribution<float>& norm_gen;
    FactorNoiseAdderBase(default_random_engine& rnd_gen_,
                         std::normal_distribution<float>& norm_gen_):
      rnd_gen(rnd_gen_),
      norm_gen(norm_gen_){}
    virtual void addNoise(FactorBase* factor)=0;
  };

  using FactorNoiseAdderPtr = std::shared_ptr<FactorNoiseAdderBase>;
  
  template <typename FactorType_>
  struct FactorNoiseAdder_: public FactorNoiseAdderBase {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
  
    static constexpr int ErrorDim=FactorType_::ErrorDim;
    FactorNoiseAdder_(default_random_engine& rnd_gen_,
                      std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdderBase (rnd_gen_, norm_gen_){
      setInformationMatrix(InformationMatrixType::Identity());
    }

    void setInformationMatrix(const InformationMatrixType& info_) {
      info=info_;
      Eigen::LLT<InformationMatrixType> llt;
      llt.compute(info);
      InformationMatrixType info_l=llt.matrixL();
      info_il=info_l.transpose().inverse();
    }

    void addNoise(FactorBase* factor_) override {
      FactorType* factor=dynamic_cast<FactorType*>(factor_);
      setInformationMatrix(factor->informationMatrix());
      if (! factor)
        return;
      ErrorVectorType unit_noise;
      for (int i=0; i<unit_noise.rows(); ++i){
        unit_noise(i)=norm_gen(rnd_gen);
      }
      ErrorVectorType noise=info_il*unit_noise;
      applyPerturbation(*factor, noise);
    };

    virtual void applyPerturbation(FactorType& ptr, const ErrorVectorType& noise) = 0;
    InformationMatrixType info;
    InformationMatrixType info_il; // inverse of chol decomp of info, for random generation
  };

  template <typename FactorType_>
  struct FactorNoiseAdderSE3Quat_: public FactorNoiseAdder_<FactorType_> {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
    FactorNoiseAdderSE3Quat_(default_random_engine& rnd_gen_,
                             std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdder_<FactorType_> (rnd_gen_, norm_gen_){}
    void applyPerturbation(FactorType& factor, const ErrorVectorType& noise) override {
      factor.setMeasurement(factor.measurement()*geometry3d::v2t(noise));
    }
  };

  template <typename FactorType_>
  struct FactorNoiseAdderSim3_: public FactorNoiseAdder_<FactorType_> {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
    FactorNoiseAdderSim3_(default_random_engine& rnd_gen_,
                             std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdder_<FactorType_> (rnd_gen_, norm_gen_){}
    void applyPerturbation(FactorType& factor, const ErrorVectorType& noise) override {
      factor.setMeasurement(factor.measurement()*geometry3d::v2s(noise));
    }
  };

  template <typename FactorType_>
  struct FactorNoiseAdderSE2_: public FactorNoiseAdder_<FactorType_> {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
    FactorNoiseAdderSE2_(default_random_engine& rnd_gen_,
                             std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdder_<FactorType_> (rnd_gen_, norm_gen_){}
    void applyPerturbation(FactorType& factor, const ErrorVectorType& noise) override {
      factor.setMeasurement(factor.measurement()*geometry2d::v2t(noise));
    }
  };

  template <typename FactorType_>
  struct FactorNoiseAdderEuclidean_: public FactorNoiseAdder_<FactorType_> {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
    FactorNoiseAdderEuclidean_(default_random_engine& rnd_gen_,
                               std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdder_<FactorType_> (rnd_gen_, norm_gen_){}
    void applyPerturbation(FactorType& factor, const ErrorVectorType& noise) override {
      factor.setMeasurement(factor.measurement()+noise);
    }
  };

  template <typename FactorType_>
  struct FactorNoiseAdderNormalize_: public FactorNoiseAdder_<FactorType_> {
    using FactorType=FactorType_;
    using InformationMatrixType = typename FactorType_::InformationMatrixType;
    using ErrorVectorType       = typename FactorType_::ErrorVectorType;
    FactorNoiseAdderNormalize_(default_random_engine& rnd_gen_,
                               std::normal_distribution<float>& norm_gen_):
      FactorNoiseAdder_<FactorType_> (rnd_gen_, norm_gen_){}
    void applyPerturbation(FactorType& factor, const ErrorVectorType& noise) override {
      ErrorVectorType m=factor.measurement()+noise;
      m.normalize();
      factor.setMeasurement(m);
    }
  };

}
