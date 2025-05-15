#include "imu_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver {

      //! @brief how to compute the error
    ImuErrorFactorAD::ADErrorVectorType
    ImuErrorFactorAD::operator()(ImuErrorFactorAD::VariableTupleType& vars) {
      const Isometry3_<DualValuef>& Ti   = vars.at<0>()->adEstimate(); // from
      const Matrix3_<DualValuef>& Ri     = Ti.linear();
      const Vector3_<DualValuef>& pi     = Ti.translation();
      const Vector3_<DualValuef>& vi     = vars.at<1>()->adEstimate();
      const Isometry3_<DualValuef>& Tj   = vars.at<2>()->adEstimate(); // to
      const Matrix3_<DualValuef>& Rj     = Tj.linear();
      const Vector3_<DualValuef>& pj     = Tj.translation();
      const Vector3_<DualValuef>& vj     = vars.at<3>()->adEstimate();
      const Vector6_<DualValuef>& bias_i = vars.at<4>()->adEstimate();
      const Vector6_<DualValuef>& bias_j = vars.at<5>()->adEstimate();

      // ldg prediction
      const Matrix3_<DualValuef> R_hat = Ri.transpose() * Rj;
      const Vector3_<DualValuef> p_hat =
        Ri.transpose() * (pj - pi - (vi + 0.5 * _g * _delta_t) * _delta_t);
      const Vector3_<DualValuef> v_hat = Ri.transpose() * (vj - vi - _g * _delta_t);

      // tg bias variation
      const Vector3_<DualValuef>& delta_bias_omega = bias_i.head<3>() - _bias_omega_hat;
      const Vector3_<DualValuef>& delta_bias_acc   = bias_i.tail<3>() - _bias_acc_hat;
      // ldg delta measurement due to bias variation
      const Vector3_<DualValuef> delta_R_bias = _dRij_dbg * delta_bias_omega;
      const Vector3_<DualValuef> delta_p_bias =
        _dpij_dba * delta_bias_acc + _dpij_dbg * delta_bias_omega;
      const Vector3_<DualValuef> delta_v_bias =
        _dvij_dba * delta_bias_acc + _dvij_dbg * delta_bias_omega;
      // tg apply to measurements
      const Matrix3_<DualValuef> R_meas = _R_meas_biased * geometry3d::expMapSO3(delta_R_bias);
      const Vector3_<DualValuef> p_meas = _p_meas_biased + delta_p_bias;
      const Vector3_<DualValuef> v_meas = _v_meas_biased + delta_v_bias;

      ADErrorVectorType error;
      Matrix3_<DualValuef> R_error = R_meas.transpose() * R_hat;
      error.segment<3>(0)          = geometry3d::logMapSO3(R_error);
      error.segment<3>(3)          = v_hat - v_meas;
      error.segment<3>(6)          = p_hat - p_meas;
      error.segment<6>(9)          = bias_j - bias_i;
      return error;
    }

    //! @brief converts the measurement in dual values
    // ldg convert each damn matrix
    void ImuErrorFactorAD::setMeasurement(const PreintegratedImuMeasurements& meas_) {
      convertMatrix(_R_meas_biased, meas_.delta_Rij());
      convertMatrix(_p_meas_biased, meas_.delta_pij());
      convertMatrix(_v_meas_biased, meas_.delta_vij());
      _delta_t = meas_.delta_t();
      convertMatrix(_bias_acc_hat, meas_.biasAccelerometer());
      convertMatrix(_bias_omega_hat, meas_.biasGyroscope());
      convertMatrix(_dRij_dbg, meas_.dRij_dbg());
      convertMatrix(_dpij_dba, meas_.dpij_dba());
      convertMatrix(_dpij_dbg, meas_.dpij_dbg());
      convertMatrix(_dvij_dba, meas_.dvij_dba());
      convertMatrix(_dvij_dbg, meas_.dvij_dbg());
      this->setInformationMatrix(meas_.informationMatrix());
    }


  void ImuErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePoseGeodesicQuaternionErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] =
      reinterpret_cast<const VariableSE3QuaternionRight*>(variable(0))->estimate().translation();
    coords[1] =
      reinterpret_cast<const VariableSE3QuaternionRight*>(variable(2))->estimate().translation();

    float lw = 0.5;
    if (fabs(variableId(0) - variableId(1)) == 1) {
      lw *= 2;
    }
    lw *= (level() * 3 + 1);
    canvas_->pushColor();
    canvas_->pushLineWidth();
    canvas_->setLineWidth(lw);
    float fading   = 1. - 0.5 * level();
    Vector3f color = srrg2_core::ColorPalette::color3fBlue() * fading;
    canvas_->setColor(color);
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
    canvas_->popAttribute();
  }

  INSTANTIATE(ImuErrorFactorAD)
} // namespace srrg2_solver
