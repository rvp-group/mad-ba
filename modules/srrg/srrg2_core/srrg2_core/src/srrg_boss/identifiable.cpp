/*
    Copyright (C) 2013  Daniele Baldassari <daniele@dikappa.org>
    Copyright (C) 2013  Giorgio Grisetti   <grisetti@diag.uniroma1.it>
*/

#include <stdexcept>
#include <sstream>

#include "identifiable.h"
#include "id_context.h"
#include "object_data.h"

using namespace srrg2_core;
using namespace std;

Identifiable::Identifiable(int id, IdContext* context): _id(id), _context(context) {
  if (_context) {
    if (_id<0) {
      ostringstream msg;
      msg << "invalid id value within context: " << _id;
      throw logic_error(msg.str());
    }
    if (!_context->add(this)) {
      ostringstream msg;
      msg << "duplicate id value within context: " << _id;
      throw logic_error(msg.str());
    }   
  }
}

Identifiable::~Identifiable() {
  if (_context) {
    _context->remove(this);
  }
}

bool Identifiable::setContext(IdContext* context) {
  if (_context==context) {
    return true;
  }
  if (context) {
    if (_id<0) {
      ostringstream msg;
      msg << "invalid id value within context: " << _id;
      throw logic_error(msg.str());
    }
    if (!context->add(this)) {
      return false;
    }
  }
  if (_context) {
    _context->remove(this);
  }
  _context=context;
  //cerr << "SetContext this: " << this << " context: " << context << endl;
  return true;
}

bool Identifiable::setId(int id, IdContext* context) {
  if (id==_id) {
    return setContext(context);
  }
  
  int oldId=_id;
  IdContext* oldContext=_context;
  _id=id;
  _context=context;
  
  bool ok=true;
  if (_context) {
    if (_id<0) {
      ostringstream msg;
      msg << "invalid id value within context: " << _id;
      throw logic_error(msg.str());
    }
    if (_context==oldContext) {
      ok=_context->update(this,oldId);
    } else {
      ok=_context->add(this);
    }
  }
  
  if (!ok) {
    _id=oldId;
    _context=oldContext;
    return false;
  }
  
  if (oldContext&&oldContext!=_context) {
    oldContext->remove(this);
  }
  
  return true;
}

int Identifiable::getId() const {
  return _id;
}

void Identifiable::ensureValidId(IdContext* context) {
  //Ensure that the current ID is valid in the serialization context and register
  //this instance
  if (_id<0||!setContext(context)) {
    setId(context->generateId(),context);
  }
}

void Identifiable::serialize(ObjectData& data, IdContext& context) {
  ensureValidId(&context);
  data << field("#id",_id);
}

void Identifiable::deserialize(ObjectData& data, IdContext& context) {
  data >> field("#id",_id);
  setContext(&context);
}

IdentifiablePtr& Identifiable::getSharedPtr(){
  if (!_context) {
    throw std::runtime_error("disaster ptr");
  }
  return _context->getSharedPtr(this);
}

