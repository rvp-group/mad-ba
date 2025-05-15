#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <Eigen/Core>

#include "identifiable.h"

namespace srrg2_core {

  class BaseBLOBReference;

  template <class T> class BLOBReference;

  class BLOB {
    template <class T>
    friend class BLOBReference;
  public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BLOB(): _ref(0) {}
    virtual ~BLOB() {}
    virtual bool read(std::istream& is) = 0;
    virtual void write(std::ostream& os) const = 0 ;
    virtual int serializedSize() const { return -1;}
    virtual const std::string& extension();
  protected:
    const BaseBLOBReference* _ref;
  };

  class BaseBLOBReference: public Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseBLOBReference(BLOB* instance, int id, IdContext* context):
      Identifiable(id, context) {
      _instance.reset(instance);
      _nameAttribute = "unknown";
    }
    inline int index() const {return _index;}
    inline void setIndex(int i) {_index=i;}
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    virtual BLOB* get() = 0;
    virtual void set(BLOB*);
    const std::string& extension();
    inline const std::string& nameAttribute() {return _nameAttribute;}
    inline void setNameAttribute(const std::string& nameAttribute_) {_nameAttribute = nameAttribute_;}

    void setFileName(const std::string& fname) {
      _fileName = fname;
    }

    const std::string& getFileName() {
      return _fileName;
    }

    inline BLOB* instance() {return _instance.get();}
  protected:
    bool load(BLOB& instance) const; //gg: hack
    std::string _inputDirectory;
    std::string _fileName;
    std::string _nameAttribute;
    mutable std::unique_ptr<BLOB> _instance;
    int _index=-1;
  };

  template<typename T>
  class BLOBReference: public BaseBLOBReference {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BLOBReference(T* instance = 0, int id = -1, IdContext* context = 0):
      BaseBLOBReference(instance, id, context) {}

    virtual T* get();
    virtual const T* get() const;
    virtual T* fetch();
  };

  template <typename T>
  T* BLOBReference<T>::get() {
    //  std::cerr << "accessing " << _instance.get() << std::endl;
    if (!_instance && getContext()) {
      _instance.reset(new T());
      //    std::cerr << "created instance: " << _instance.get() << std::endl;
      if (load(*_instance)) {
        _instance->_ref = this;
      } else {
        _instance.reset(0);
      }
    }
    return (T*) _instance.get();
  }

  template <typename T>
  const T* BLOBReference<T>::get() const {
    //  std::cerr << "accessing " << _instance.get() << std::endl;
    if (!_instance && getContext()) {
      _instance.reset(new T());
      //    std::cerr << "created instance: " << _instance.get() << std::endl;
      if (load(*_instance)) {
        _instance->_ref = this;
      } else {
        _instance.reset(0);
      }
    }
    return (const T*) _instance.get();
  }

  template <typename T>
  T* BLOBReference<T>::fetch() {
    //std::cerr << "accessing " << _instance.get() << std::endl;
    if (!_instance && getFileName().length()) {
      _instance.reset(new T());
      std::ifstream is(_inputDirectory + _fileName);
      if (is.good() && _instance->read(is)) {
        _instance->_ref=this;
      } else {
        _instance.reset(0);
      }
    }
    return (T*) _instance.get();
  }
}

#define BOSS_REGISTER_BLOB(class_name) \
  static AutoRegisterer<BLOBReference<class_name > > _reg_##class_name(#class_name,typeid(BLOBReference<class_name >).name());
