#pragma once

#include <string>
#include <iostream>

#include "blob.h"

namespace srrg2_core {

  class Serializer;

  class SerializationContext {  
  public:
    SerializationContext();
    virtual ~SerializationContext();
    virtual std::string createBinaryFilePath(BaseBLOBReference& instance);
    virtual std::ostream* getBinaryOutputStream(const std::string& fname);
    virtual std::istream* getBinaryInputStream(const std::string& fname) const;
    virtual void setInputFilePath(const std::string& str);
    virtual void setOutputFilePath(const std::string& str);
    virtual void setBinaryPath(const std::string& str);
    void replaceEnvTags(std::string& str);
    void loadCurrentTime();
    void makeInputStream();
    void makeOutputStream();

    void setInputStream(std::istream* is, const std::string& filename="");
    void setOutputStream(std::ostream* os, const std::string& filename="");

    void destroyInputStream();
    void destroyOutputStream();

    inline std::istream* inputStream() {return _inputStream;}
    inline std::ostream* outputStream() {return _outputStream;}
    inline const std::string& inputDataFilename() const {return _inputDataFileName;}
  protected:
    std::string _inputDataFileName;
    std::string _outputDataFileName;
    std::string _blobFileName;
    std::map<std::string, std::string> _envMap;
    std::istream* _inputStream;
    std::ostream* _outputStream;
  };

}
