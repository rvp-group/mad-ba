#include "property_eigen.h"

namespace srrg2_core {

  void PropertyEigenBase::serialize(ObjectData& odata, IdContext& context) {
    ArrayData* adata = new ArrayData;
    for (int r = 0; r < rows(); ++r) {
      for (int c = 0; c < cols(); ++c) {
        adata->add(this->valueAt(r, c));
      }
    }
    if (sizeAtCompileTime() == Eigen::Dynamic) {
      ObjectData* sdata = new ObjectData;
      sdata->setInt("rows", rows());
      sdata->setInt("cols", cols());
      sdata->setField("values", adata);
      odata.setField(this->name(), sdata);
    } else {
      odata.setField(this->name(), adata);
    }
  }

  void PropertyEigenBase::deserialize(ObjectData& odata, IdContext& context) {
    ValueData* inner_data = odata.getField(this->name());
    ArrayData* adata      = 0;
    if (sizeAtCompileTime() == Eigen::Dynamic) {
      ObjectData* sdata = dynamic_cast<ObjectData*>(inner_data);
      int rows_         = sdata->getInt("rows");
      int cols_         = sdata->getInt("cols");
      this->resize(rows_, cols_);
      adata = dynamic_cast<ArrayData*>(sdata->getField("values"));
    } else {
      adata = dynamic_cast<ArrayData*>(odata.getField(this->name()));
    }
    assert(adata && (int) adata->size() == this->rows() * this->cols() &&
           "size mismatch");

    int k = 0;
    for (int r = 0; r < rows(); ++r) {
      for (int c = 0; c < cols(); ++c, ++k) {
        ValueData& v = (*adata)[k];
        this->setValueAt(r, c, v.getFloat());
      }
    }
  }

} // namespace srrg2_core
