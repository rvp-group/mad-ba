#include <fstream>
#include <memory>
#include <stdexcept>

#include "deserializer.h"

#include "id_placeholder.h"
#include "identifiable.h"
#include "json_recursive_object_parser.h"
#include "object_data.h"

using namespace std;
using namespace srrg2_core;

static ValueData* processData(ValueData* vdata,
                              IdContext& context,
                              vector<int>& danglingRefs,
                              vector<int>& declaredIDs) {
  switch (vdata->type()) {
    case OBJECT: {
      ObjectData* data  = static_cast<ObjectData*>(vdata);
      ValueData* pfield = data->getField("#pointer");
      if (pfield) {
        int id = pfield->getInt();

        // Negative IDs map to null pointer
        Identifiable* pointer = 0;
        if (id >= 0) {
          pointer = context.getById(id);
          if (!pointer) {
            pointer = context.createPlaceHolder(id);
          }
          danglingRefs.push_back(id);
        }
        return new PointerReference(pointer);
      }
      ValueData* idfield = data->getField("#id");
      if (idfield) {
        declaredIDs.push_back(idfield->getInt());
      }

      const vector<string>& fields = data->fields();
      for (vector<string>::const_iterator f_it = fields.begin(); f_it != fields.end(); f_it++) {
        ValueData* fdata       = data->getField(*f_it);
        ValueData* replaceData = processData(fdata, context, danglingRefs, declaredIDs);
        if (replaceData) {
          data->setField(*f_it, replaceData);
        }
      }
      break;
    }
    case ARRAY: {
      ArrayData* v_array = static_cast<ArrayData*>(vdata);
      for (vector<ValueData*>::const_iterator v_it = v_array->begin(); v_it != v_array->end();
           v_it++) {
        ValueData* replaceData = processData(*v_it, context, danglingRefs, declaredIDs);
        if (replaceData) {
          v_array->set(v_it - v_array->begin(), replaceData);
        }
      }
      break;
    }
    default:
      // Nothing to do
      break;
  }
  return 0;
}

#define UNKNOWN_OBJECT_CLASS_ASSERTION 0

static Serializable* createInstance(ObjectData* odata,
                                    const string& type,
                                    IdContext& context,
                                    map<Serializable*, std::set<int>>& waitingInstances,
                                    map<int, std::set<Serializable*>>& danglingReferences,
                                    map<Identifiable*, IdentifiablePtr>& instances_shared,
                                    bool make_shared = false) {
  ValueData* idValue = odata->getField("#id");

  // Identifiable objects are overwritten if another object
  // with the same ID is read, so first check if the ID is already in the context
  Serializable* instance = 0;
  if (idValue) {
    instance = context.getById(idValue->getInt());
    if (instance) {
      IdPlaceholder* placeHolder = dynamic_cast<IdPlaceholder*>(instance);
      if (placeHolder) {
        // Found instance is a placeholder, replace it
        instance = 0;
      }
    }
  }

  if (instance) {
    // Just to ensure that the found instance has right type
    if (instance->className() != type) {
      stringstream msg;
      msg << "Trying to overwrite " << instance->className() << " (ID " << idValue->getInt()
          << ") with " << type;
      throw logic_error(msg.str());
    }
  } else {
    try {
      instance = Serializable::createInstance(type);
      if (make_shared) {
        Identifiable* ident_raw = dynamic_cast<Identifiable*>(instance);
        if (ident_raw) {
          instances_shared[ident_raw] = IdentifiablePtr(ident_raw);
          // std::cerr << "obmap populated: " << ident_raw <<  " " << instances_shared[ident_raw] <<
          // std::endl; ident_raw->setContext(&context);
        }
      }
    } catch (logic_error& e) {
      cerr << "EE Cannot create an instance of type '" << type << "'" << endl;
      std::cerr << e.what() << std::endl;
      exit(-1);
      // TODO Notify this occurrence
      return 0;
    }
  }

  vector<int> danglingPointers;
  vector<int> declaredIDs;

  processData(odata, context, danglingPointers, declaredIDs);

  instance->deserialize(*odata, context);

  for (vector<int>::iterator dp_it = danglingPointers.begin(); dp_it != danglingPointers.end();
       dp_it++) {
    waitingInstances[instance].insert(*dp_it);
    danglingReferences[*dp_it].insert(instance);
  }
  for (vector<int>::iterator id_it = declaredIDs.begin(); id_it != declaredIDs.end(); id_it++) {
    // Further check, just in case the ID was a fake field
    if (context.getById(*id_it)) {
      map<int, set<Serializable*>>::iterator entry = danglingReferences.find(*id_it);
      if (entry != danglingReferences.end()) {
        set<Serializable*>& instSet = (*entry).second;
        for (set<Serializable*>::iterator instance_it = instSet.begin();
             instance_it != instSet.end();
             instance_it++) {
          waitingInstances[*instance_it].erase(*id_it);
          if (waitingInstances[*instance_it].empty()) {
            waitingInstances.erase(*instance_it);
            (*instance_it)->deserializeComplete();
          }
        }
        danglingReferences.erase(*id_it);
      }
    }
  }
  if (danglingPointers.empty()) {
    instance->deserializeComplete();
  }
  return instance;
}

Deserializer::Deserializer(SerializationContext* sc) {
  _objectParser         = new JSONRecursiveObjectParser();
  _serializationContext = sc;
}

Serializable* Deserializer::readObject() {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return 0;
  }

  string type;
  unique_ptr<ObjectData> odata(
    _objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  return createInstance(
    odata.get(), type, *this, _waitingInstances, _danglingReferences, _instances_shared, false);
}

Serializable* Deserializer::readObject(const std::string& line) {
  _serializationContext->setInputStream(new istringstream(line.c_str()));
  string type;
  unique_ptr<ObjectData> odata(
    _objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  return createInstance(
    odata.get(), type, *this, _waitingInstances, _danglingReferences, _instances_shared, false);
}

SerializablePtr Deserializer::readObjectShared() {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return 0;
  }

  string type;
  unique_ptr<ObjectData> odata(
    _objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  Serializable* s = createInstance(
    odata.get(), type, *this, _waitingInstances, _danglingReferences, _instances_shared, true);

  Identifiable* i = dynamic_cast<Identifiable*>(s);
  if (i) {
    if (!i->getSharedPtr()) {
      throw std::runtime_error("pointer not managed! be consistent with readObject calls");
    }
    return i->getSharedPtr();
  }
  return SerializablePtr(s);
}

SerializablePtr Deserializer::readObjectShared(const std::string& line) {
  _serializationContext->setInputStream(new istringstream(line.c_str()));
  string type;
  unique_ptr<ObjectData> odata(
    _objectParser->readObject(*_serializationContext->inputStream(), type));
  if (!odata.get()) {
    return 0;
  }
  Serializable* s = createInstance(
    odata.get(), type, *this, _waitingInstances, _danglingReferences, _instances_shared, true);

  Identifiable* i = dynamic_cast<Identifiable*>(s);
  if (i) {
    if (!i->getSharedPtr()) {
      throw std::runtime_error("pointer not managed! be consistent with readObject calls");
    }
    return i->getSharedPtr();
  }
  return SerializablePtr(s);
}

bool Deserializer::readObjectData(ObjectData*& objdata, string& type) {
  _serializationContext->makeInputStream();
  if (!(*_serializationContext->inputStream())) {
    return false;
  }

  objdata = _objectParser->readObject(*_serializationContext->inputStream(), type);
  return true;
}

void Deserializer::setFilePath(const string& fpath) {
  if (!_serializationContext)
    _serializationContext = new SerializationContext;
  _serializationContext->setInputFilePath(fpath);
}

Deserializer::~Deserializer() {
  if (_serializationContext)
    delete _serializationContext;
  delete _objectParser;
}
