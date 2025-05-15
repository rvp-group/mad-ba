#include "id_context.h"

#include "identifiable.h"
#include "id_placeholder.h"
#include <stdexcept>

using namespace srrg2_core;
using namespace std;

IdContext::IdContext(): _lastGeneratedID(0) {
  _serializationContext = 0;
}

bool IdContext::add(Identifiable* obj) {
  lock_guard<mutex> lock(_instances_lock);
  return addImpl(obj);
}

bool IdContext::add(std::shared_ptr<Identifiable> obj) {
  lock_guard<mutex> lock(_instances_lock);
  if (! obj)
    return true;
  if (obj->getId()<0)
    obj->setId(generateId());
  Identifiable*& instance=_instances[obj->getId()];
  if (instance) {
    IdPlaceholder* placeHolder=dynamic_cast<IdPlaceholder*>(instance);
    if (placeHolder) {
      throw std::logic_error("IdContext::add| instance already existing");
    }
    return false;
  }
  _instances[obj->getId()]=(obj.get());
  _instances_shared[instance]=obj;
  return true;
}

bool IdContext::addImpl(Identifiable* obj) {
  //cerr << "add obj:" << obj << " context:" << this;
  Identifiable*& instance=_instances[obj->getId()];
  if (instance) {
    IdPlaceholder* placeHolder=dynamic_cast<IdPlaceholder*>(instance);
    if (placeHolder) {
      placeHolder->resolve(obj);
      placeHolder->_context=0;
      delete placeHolder;
    } else {
      return true;
      //return false;
    } 
  }
  instance=obj;
  //cerr << " true" << endl;
  return true;
}

bool IdContext::remove(Identifiable* obj) {
//  lock_guard<mutex> lock(_instances_lock);
  if (_instances.erase(obj->getId())>0){
    _instances_shared.erase(obj);
    return true;
  };
  return false;
}

bool IdContext::remove(std::shared_ptr<Identifiable> obj) {
  return remove(obj.get());
}

bool IdContext::update(Identifiable* obj, int oldId) {
  if (obj->getId()==oldId) {
    return true;
  }
  lock_guard<mutex> lock(_instances_lock);
  if (addImpl(obj)) {
    _instances.erase(oldId);
    return true;
  }
  return false;
}

Identifiable* IdContext::getById(int id) {
  lock_guard<mutex> lock(_instances_lock);
  map<int, Identifiable*>::iterator instance_pair=_instances.find(id);
  if (instance_pair==_instances.end()) {
    return 0;
  }
  return (*instance_pair).second;
}

void IdContext::setStartingId(int id){
  if (id<0)
    throw std::runtime_error("ids cannot be less than 0");
  _lastGeneratedID=id;
}

int IdContext::generateId() {
  lock_guard<mutex> lock(_instances_lock);
  int minId=_lastGeneratedID+1;
  if (!_instances.empty()) {
    int curId=(*_instances.rbegin()).first+1;
    if (curId>minId) {
      minId=curId;
    }
  }
  return _lastGeneratedID=minId;
}

IdPlaceholder* IdContext::createPlaceHolder(int id) {
  return new IdPlaceholder(id,this);
}

IdContext::~IdContext() {
  lock_guard<mutex> lock(_instances_lock);
  for (map<int, Identifiable*>::iterator ipair=_instances.begin();ipair!=_instances.end();ipair++) {
    Identifiable* instance=(*ipair).second;
    if (dynamic_cast<IdPlaceholder*>(instance)) {
      delete instance;
    } else {
      instance->_context=0;
    }
  }
  _instances.clear();
}

IdentifiablePtr& IdContext::getSharedPtr(Identifiable* obj){
  auto it=_instances_shared.find(obj);
  if (it==_instances_shared.end()) {
    throw std::runtime_error("disaster ptr");
  }
  return it->second;
}
