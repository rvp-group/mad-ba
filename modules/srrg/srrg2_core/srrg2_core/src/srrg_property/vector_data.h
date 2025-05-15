#pragma once
#include <Eigen/Core>

#include "srrg_boss/blob.h"

namespace srrg2_core {

  template <typename VectorType_>
  struct VectorData_ : public VectorType_, public BLOB {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using VectorType = VectorType_;
    using ValueType  = typename VectorType::value_type;

    VectorData_() {
      // std::cerr << "constructed!" << this << std::endl;
    }

    virtual ~VectorData_() {
      // std::cerr << "destroying!" << this << std::endl;
    }

    virtual bool read(std::istream& is) {
      //      std::cerr << "loading: " << this << std::endl;
      this->clear();
      while (1) {
        ValueType pt;
        is.read((char*) &pt, sizeof(ValueType));
        size_t read_size = is.gcount();
        if (read_size == 0)
          return true;

        if (read_size != sizeof(ValueType))
          return false;

        this->push_back(pt);
      }
      //      std::cerr << "loaded: " << this->size() << " points" << std::endl;
      return true;
    }

    virtual void write(std::ostream& os) const {
      // we get the size of the buffer
      char* start = (char*) &((*this)[0]);
      char* end   = (char*) &((*this)[this->size()]);
      // we read
      os.write((char*) this->data(), end - start);
    }
  };

} // namespace srrg2_core
