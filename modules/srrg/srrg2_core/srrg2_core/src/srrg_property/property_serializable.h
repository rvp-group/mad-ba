#pragma once
#include <Eigen/Core>

#include "property.h"
#include "srrg_boss/id_context.h"

namespace srrg2_core {

  template <typename SerializableType_>
  class PropertySerializable_ : public PropertyNonCopiable_<SerializableType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using ValueType = SerializableType_;
    using ThisType  = PropertySerializable_<SerializableType_>;

    PropertySerializable_(const std::string& name_,
                          const std::string& description_,
                          PropertyContainerBase* config_,
                          bool* flag = 0) :
      PropertyNonCopiable_<SerializableType_>(name_, description_, config_) {
    }

    PROPERTY_ADD_SET_GET;

    void serialize(ObjectData& odata, IdContext& context) override {
      ObjectData* inner_data = new ObjectData;
      this->_value.serialize(*inner_data, context);
      odata.setField(this->name(), inner_data, this->description().c_str());
    }

    void deserialize(ObjectData& odata, IdContext& context) override {
      ObjectData* inner_data =
        dynamic_cast<ObjectData*>(odata.getField(this->name()));
      this->_value.deserialize(*inner_data, context);
    }

    void copyTo(PropertyBase& other_) override {
      ThisType& other = dynamic_cast<ThisType&>(other_);
      ObjectData odata;
      IdContext context;
      serialize(odata, context);
      other.deserialize(odata, context);
    }
  };

  template <typename SerializableType_>
  class PropertySerializableVector_
    : public PropertyNonCopiable_<
        std::vector<SerializableType_,
                    Eigen::aligned_allocator<SerializableType_>>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = std::vector<SerializableType_,
                                  Eigen::aligned_allocator<SerializableType_>>;
    using ThisType  = PropertySerializable_<SerializableType_>;

    PropertySerializableVector_(const std::string& name_,
                                const std::string& description_,
                                PropertyContainerBase* config_,
                                bool* flag = 0) :
      PropertyNonCopiable_<ValueType>(name_, description_, config_) {
    }

    inline size_t size() const {
      return this->_value.size();
    }

    inline void resize(size_t s) {
      this->_value.resize(s);
      this->touch();
    }

    inline void clear() {
      this->_value.clear();
      this->touch();
    }

    inline const typename ValueType::value_type& value(int idx) const {
      return this->_value[idx];
    }

    inline typename ValueType::value_type& value(int idx) {
      return this->_value[idx];
    }

    inline void pushBack(const typename ValueType::value_type& v) {
      return this->_value.push_back(v);
    }

    inline void setValue(int idx, typename ValueType::value_type& v) {
      this->_value[idx] = v;
      this->touch();
    }
    
    void serialize(ObjectData& odata, IdContext& context) override {
      ArrayData* adata = new ArrayData;
      for (size_t i = 0; i < this->_value.size(); ++i) {
        ObjectData* inner_data = new ObjectData;
        this->_value[i].serialize(*inner_data, context);
        adata->add(inner_data);
      }
      odata.setField(this->name(), adata, this->description().c_str());
    }

    void deserialize(ObjectData& odata, IdContext& context) override {
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField(this->name()));
      this->_value.resize(adata->size());
      for (size_t i = 0; i < adata->size(); ++i) {
        ObjectData& inner_data = dynamic_cast<ObjectData&>((*adata)[i]);
        this->_value[i].deserialize(inner_data, context);
      }
    }

    void copyTo(PropertyBase& other_) override {
      ThisType& other = dynamic_cast<ThisType&>(other_);
      ObjectData odata;
      IdContext context;
      serialize(odata, context);
      other.deserialize(odata, context);
    }
  };

} // namespace srrg2_core
