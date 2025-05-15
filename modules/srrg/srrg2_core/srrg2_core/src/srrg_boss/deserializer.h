#pragma once

#include <map>
#include <set>
#include <vector>
#include <istream>
#include <string>

#include "id_context.h"
#include "serialization_context.h"

namespace srrg2_core {
  
class Serializable;
class ValueData;
class ObjectParser;
class ObjectData;

  class Deserializer: virtual public IdContext {
public:
  Deserializer(SerializationContext* serContext = 0);
  /*
   * Set file path for main data file.
   * Binary files use this path as base directory.
   */
  void setFilePath(const std::string& fpath);
  
  /*
   * Set data file format (default is "JSON").
   */
  bool setFormat(const std::string& format);



    SerializablePtr readObjectShared();
    SerializablePtr readObjectShared(const std::string& line);

    
  
    bool readObjectData(ObjectData*& objectData, std::string& type);

    
  virtual ~Deserializer();
protected:

  /*!
   * \brief Read a single object and create the related Serializable instance.
   * \details Read and parse a single object, returning a pointer to the instance, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of returned pointer.
   * \return the read object
   */
  Serializable* readObject();

  
  /*!
   * \brief Read a single object and create the related Serializable instance.
   * \details Read and parse a single object, returning a pointer to the instance, if any; a null pointer
   * is returned in case of error. <br/>
   * The caller takes ownership of returned pointer.
   * \return the read object
   */
    Serializable* readObject(const std::string& line);

    
  ObjectParser* _objectParser;
  
  //Map with Identifiable objects that refers to unresolved pointers
  std::map<Serializable*,std::set<int> > _waitingInstances;
  //Reverse map
  std::map<int,std::set<Serializable*> > _danglingReferences;
};

}
