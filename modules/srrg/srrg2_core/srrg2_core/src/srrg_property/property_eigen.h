#pragma once
#include <Eigen/Core>

#include "property.h"

namespace srrg2_core {

  struct PropertyContainerBase;

  struct PropertyEigenBase : public PropertyBase {
    PropertyEigenBase(const std::string& name_,
                      const std::string& description_,
                      PropertyContainerBase* config_,
                      bool* flag = 0) :
      PropertyBase(name_, description_, config_, flag){};

    virtual int rows() const                            = 0;
    virtual int cols() const                            = 0;
    virtual float valueAt(int r, int c) const           = 0;
    virtual void setValueAt(int r, int c, float value_) = 0;
    virtual void resize(int r, int c)                   = 0;
    virtual int sizeAtCompileTime() const               = 0;
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;
  };

  template <typename EigenType_>
  class PropertyEigen_ : public PropertyEigenBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using ValueType = EigenType_;
    using ThisType  = Property_<EigenType_>;

    PropertyEigen_(const std::string& name_,
                   const std::string& description_,
                   PropertyContainerBase* config_,
                   const EigenType_& value_,
                   bool* flag = 0) :
      PropertyEigenBase(name_, description_, config_, flag),
      _value(value_) {
    }

    PROPERTY_ADD_SET_GET;

    int rows() const override {
      return _value.rows();
    }
    int cols() const {
      return _value.cols();
    }
    float valueAt(int r, int c) const override {
      return _value.matrix()(r, c);
    }
    void setValueAt(int r, int c, float value_) override {
      _value.matrix()(r, c) = value_;
    }
    virtual int sizeAtCompileTime() const override {
      return this->_value.matrix().SizeAtCompileTime;
    }
    void resize(int r, int c) override {
      if (sizeAtCompileTime() == Eigen::Dynamic) {
        _value.matrix().resize(r, c);
      } else {
        throw std::runtime_error(
          "PropertyEigen| resize(): cannot resize fixed size eigen object");
      }
    }

  protected:
    ValueType _value;
  };
} // namespace srrg2_core
