#pragma once
namespace srrg2_core {
  
  template <typename ValueType_>
  struct KahanAdderBase_{
    using ValueType = ValueType_;
    void add(const ValueType& src) {
      ValueType y = src-_c;
      ValueType t = _sum + y;
      _c = (t - _sum) - y;
      _sum=t;
    }
    inline const ValueType& sum() const {return _sum;}
  protected:
    ValueType _sum;
    ValueType _c;
  };

  template <typename ValueType_>
  struct KahanAdder_: public KahanAdderBase_<ValueType_> {
    void reset(){
      this->_sum.setZero();
      this->_c.setZero();
    }
  };

  template <>
  struct KahanAdder_<float>: public KahanAdderBase_<float> {
    void reset(){
      this->_sum=0.f;
      this->_c=0.f;
    }
  };

  template <>
  struct KahanAdder_<double>: public KahanAdderBase_<double> {
    void reset(){
      this->_sum=0.f;
      this->_c=0.f;
    }
  };

  
}
