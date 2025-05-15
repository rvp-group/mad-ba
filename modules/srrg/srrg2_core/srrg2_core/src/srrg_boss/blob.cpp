#include <memory>

#include "blob.h"
#include "object_data.h"
#include "serialization_context.h"
#include "id_context.h"

using namespace srrg2_core;
using namespace std;

static string DEFAULT_EXTENSION("dat");

const string& BLOB::extension() {
  return DEFAULT_EXTENSION;
}

const string& BaseBLOBReference::extension() {
  if (_instance) {
    return _instance->extension();
  }
  return DEFAULT_EXTENSION;
}

void BaseBLOBReference::serialize(ObjectData& data, IdContext& context) {
  Identifiable::serialize(data, context);
  if (_instance) {
    //Check if binary file serialization is supported

    SerializationContext* fileContext = context.serializationContext();
    if (fileContext) {
      _fileName = fileContext->createBinaryFilePath(*this);
      std::ostream* os = fileContext->getBinaryOutputStream(_fileName);
      if (os) {
        if (_index<0) {
          _instance->write(*os);
        } else {
          if (_instance->serializedSize()<0) {
            throw std::runtime_error("indexed writing, size error");
          }
          std::ofstream* os_fstream = dynamic_cast<std::ofstream*>(os);
          if (! os_fstream)
            throw std::runtime_error("indexed writing on something that is not a file");
          size_t pos=_index*_instance->serializedSize();
          if (! os_fstream->good()) {
            throw std::runtime_error("indexed writing on file failed");
          }
          os_fstream->seekp(pos, ios::beg);
          _instance->write(*os_fstream);
          
        }
        std::ofstream* os_fstream = dynamic_cast<std::ofstream*>(os);
        if (os_fstream)
          os_fstream->close();
        delete os;
      }
    }
  }
  
  data << field("path_name", _fileName);
  data << field("index", _index);
}

void BaseBLOBReference::deserialize(ObjectData& data, IdContext& context) {
  Identifiable::deserialize(data, context);
  data >> field("path_name", _fileName);
  SerializationContext* fileContext = context.serializationContext();
  _inputDirectory = "";
  if (fileContext) {
    auto pos=fileContext->inputDataFilename().find_last_of("/");
    if (pos!=string::npos) {
      _inputDirectory = fileContext->inputDataFilename().substr(0,pos+1);
    }
    ValueData* index_data=data.getField("index");
    if (index_data) {
      _index=index_data->getInt();
    } else {
      _index=-1;
    }
  }
  //cerr << "Path name: " << _fileName << endl;
  _instance = 0;
}

bool BaseBLOBReference::load(BLOB& instance) const {
  //Check if binary file serialization is supported
  cerr << "Load: Path name: " << _fileName << endl;
  cerr << "instance: " << &instance << endl;

  const SerializationContext* fileContext = getContext()->serializationContext();
  bool result = false;
  if (fileContext) {
    std::istream* is = fileContext->getBinaryInputStream(_inputDirectory + _fileName);
//    cerr << "istream: " << is << endl;
    if (is) {
      if (_index>=0) {
        if (instance.serializedSize()<0) {
          throw std::runtime_error("attempted seek with a negative size. please specify the constant size of the serialized object in the blob");
        }
        size_t pos=_index*instance.serializedSize();
        cerr << "reading at pos: " << pos << endl;
        ifstream* ifs=dynamic_cast<ifstream*>(is);
        if (! ifs) {
          throw std::runtime_error("attempted seek on a stream which is not a file");
        }
        ifs->seekg(pos, ios::beg);
        if (! ifs->good()) {
          throw std::runtime_error("stream ended, index out of bounds");
        } 
      }
      result = instance.read(*is);
      delete is;
    }
  }
  return result;
}

void BaseBLOBReference::set(BLOB* instance) {
  _instance.reset(instance);
}
