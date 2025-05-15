#include <ctime>
#include <sstream>

#include "serializer.h"
#include "json_object_writer.h"
#include "object_data.h"

using namespace std;
using namespace srrg2_core;


 
ValueData* processDataForWrite(ValueData* vdata, IdContext& context) {
  switch (vdata->type()) {
    case OBJECT: {
      ObjectData* o=static_cast<ObjectData*>(vdata);
      const vector<string>& fields=o->fields();
      for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();f_it++) {
        ValueData* fdata=o->getField(*f_it);
        ValueData* replaceData=processDataForWrite(fdata, context);
        if (replaceData) {
          o->setField(*f_it,replaceData, replaceData->description());
        }
      }
      break;
    }
    case ARRAY: {
      ArrayData* v_array=static_cast<ArrayData*>(vdata);
      for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++) {
        ValueData* replaceData=processDataForWrite(*v_it, context);
        if (replaceData) {
          v_array->set(v_it-v_array->begin(),replaceData);
        }
      }
      break;
    }
    case POINTER: {
      Identifiable* identifiable=vdata->getPointer();
      if (identifiable) {
	identifiable->ensureValidId(&context);
      }
      ObjectData* pointerObject=new ObjectData();
      pointerObject->_description=vdata->description();
      pointerObject->setInt("#pointer",identifiable?identifiable->getId():-1);
      return pointerObject;
    }
    default:
      //Nothing to do
      break;
  }
  return 0;
}

Serializer::Serializer(SerializationContext *sc){
  _objectWriter=new JSONObjectWriter();
  _serializationContext = sc;
}

void Serializer::setFilePath(const string& fpath) {
  if (! _serializationContext)
    _serializationContext = new SerializationContext;
  _serializationContext->setOutputFilePath(fpath);
}

void Serializer::setBinaryPath(const string& fpath) {
  if (! _serializationContext)
    _serializationContext = new SerializationContext;
  _serializationContext->setBinaryPath(fpath);
}

bool Serializer::setFormat(const string& format) {
  //TODO Implementation
  return format=="JSON";
}

bool Serializer::writeObject(Serializable& instance) {
  //mc if something goes wrong revert to ObjectData* data and change accordingly
  ObjectData data;
  instance.serialize(data,*this);

  processDataForWrite(&data,*this);
  _serializationContext->makeOutputStream();
  if (*_serializationContext->outputStream()) {
    _objectWriter->writeObject(*_serializationContext->outputStream(),instance.className(),data, _comments_enabled);
    //TODO Change writer to get status flag
//    _serializationContext->destroyOutputStream();
    return true;
  }
  return false;

}

bool Serializer::writeObject(std::string& output, Serializable& instance) {
  //mc if something goes wrong revert to ObjectData* data and change accordingly
  ObjectData data;
  instance.serialize(data, *this);
  ostringstream os;
  processDataForWrite(&data, *this);
  _objectWriter->writeObject(os,instance.className(), data, _comments_enabled);
  output = os.str();
  return true;
}

Serializer::~Serializer() {
  delete _objectWriter;
}

