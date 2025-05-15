#pragma once
#include "ad.h"
#include <Eigen/Core>

namespace srrg2_core {

  template <typename Scalar_ , int InputSize_, int OutputSize_>
  class MultivariateFunction{
    public:
      typedef Scalar_ Scalar;
      static const int InputSize=InputSize_;
      static const int OutputSize=OutputSize_;

      //virtual void operator()(Scalar_* output, const Scalar_* input)=0;
  };


  template <typename BaseScalar_, template <typename _T> class FunctorType_>
  class ADMultivariateFunction: public  FunctorType_< DualValue_<BaseScalar_> >{
    public:
      static const int InputSize=FunctorType_< DualValue_<BaseScalar_> >::InputSize;
      static const int OutputSize=FunctorType_< DualValue_<BaseScalar_> >::OutputSize;

    // if input not given assumed to be the zero vector
      void operator()(BaseScalar_* output, const BaseScalar_* input=0){
        DualValue_<BaseScalar_> ad_input[InputSize];
        DualValue_<BaseScalar_> ad_output[OutputSize];
        for (size_t c=0; c<InputSize; c++){
          ad_input[c].value= (input )? input[c]: 0;
          ad_input[c].derivative=0;
        }
        FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output,ad_input);
        for (size_t r=0; r<OutputSize; r++){
          output[r]=ad_output[r].value;
        }
      }
    
      typedef
          typename Eigen::Matrix<BaseScalar_, OutputSize, InputSize>
      JacobianType;

    // if input not given assumed to be the zero vector
      JacobianType jacobian(const BaseScalar_* input=0){
        JacobianType jacobian=JacobianType::Zero();
        const int rows=FunctorType_<BaseScalar_>::OutputSize;
        const int cols=FunctorType_<BaseScalar_>::InputSize;
        DualValue_<BaseScalar_> ad_input[cols];
        DualValue_<BaseScalar_> ad_output[rows];
        for (size_t c=0; c<cols; c++){
          ad_input[c].value=(input) ? input[c] : 0.;
          ad_input[c].derivative=0;
        }
        for (size_t c=0; c<cols; c++){
          ad_input[c].derivative=1.0;
          FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output, ad_input);
          for (size_t r=0; r<rows; r++){
            jacobian(r,c)=ad_output[r].derivative;
          }
          ad_input[c].derivative=0.0;
        }
        return jacobian;
      }

      JacobianType numericJacobian(const BaseScalar_* input=0, const BaseScalar_& epsilon=1e-6){
        JacobianType jacobian=JacobianType::Zero();
        const int rows=FunctorType_<BaseScalar_>::OutputSize;
        const int cols=FunctorType_<BaseScalar_>::InputSize;
        DualValue_<BaseScalar_> ad_input_plus[cols], ad_input_minus[cols];
        DualValue_<BaseScalar_> ad_output_plus[rows], ad_output_minus[rows];
        for (size_t c=0; c<cols; c++){
          BaseScalar_ x = input ? input[c]: 0.;
          ad_input_plus[c].value=x;
          ad_input_plus[c].derivative=0;
          ad_input_minus[c].value=x;
          ad_input_minus[c].derivative=0;
        }
        for (size_t c=0; c<cols; c++){
          BaseScalar_ x = input ? input[c]: 0.;

          ad_input_plus[c].value+=epsilon;
          ad_input_minus[c].value-=epsilon;

          FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output_plus, ad_input_plus);
          FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output_minus, ad_input_minus);
          for (size_t r=0; r<rows; r++){
            jacobian(r,c)=ad_output_plus[r].value-ad_output_minus[r].value;
          }

          ad_input_plus[c].value=x;
          ad_input_minus[c].value=x;

        }
        return jacobian*(.5/epsilon);
      }

  };

  template <typename Scalar_ , int InputSize_, int OutputSize_>
  class MultivariateZeroFunction_{
    public:
      typedef Scalar_ Scalar;
      static const int InputSize=InputSize_;
      static const int OutputSize=OutputSize_;
      typedef typename Eigen::Matrix<Scalar, OutputSize, 1> OutputType;
      typedef typename Eigen::Matrix<Scalar, InputSize, 1>  InputType;
      
  };

  template <typename BaseScalar_, template <typename _T> class MultivariateZeroFunction_>
  class ADZeroFunction_: public MultivariateZeroFunction_< DualValue_<BaseScalar_> > {
  public:
    static const int InputSize=MultivariateZeroFunction_< DualValue_<BaseScalar_> >::InputSize;
    static const int OutputSize=MultivariateZeroFunction_< DualValue_<BaseScalar_> >::OutputSize;

    typedef typename Eigen::Matrix<BaseScalar_, OutputSize, InputSize> JacobianType;
    typedef typename Eigen::Matrix<BaseScalar_, OutputSize, 1> OutputType;
    typedef typename Eigen::Matrix<DualValue_<BaseScalar_>, InputSize, 1> ADPerturbationType;
    typedef typename Eigen::Matrix<DualValue_<BaseScalar_>, OutputSize, 1> ADOutputType;
    typedef typename Eigen::Matrix<DualValue_<BaseScalar_>, OutputSize, InputSize> ADJacobianType;
    typedef MultivariateZeroFunction_< DualValue_<BaseScalar_> > FunctionType;
    void errorAndJacobian(OutputType& e, JacobianType& J) const {
      ADPerturbationType pert;
      pert.setZero();
      ADOutputType e_=FunctionType::operator()(pert);
      convertMatrix(e,e_);
      for (int c=0; c<InputSize; ++c){
        pert[c].derivative=BaseScalar_(1);
        e_=FunctionType::operator()(pert);
        for (int r=0; r<OutputSize; ++r)
          J(r,c)=e_(r).derivative;
        pert[c].derivative=BaseScalar_(0);
      }
    }
    

  };
  

}
