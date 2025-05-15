#include <cassert>
#include <dlfcn.h>
#include <gnu/lib-names.h>
#include <iostream>

#include "srrg_boss/deserializer.h"
#include "srrg_boss/json_object_writer.h"
#include "srrg_boss/serializer.h"
#include "srrg_system_utils/env.h"
#include "srrg_system_utils/shell_colors.h"
#include "srrg_system_utils/system_utils.h"

#include "property_container_manager.h"
#include "property_identifiable.h"

#define DEBUG(var) \
  if (var)         \
  std::cerr << #var << ": "

extern char** environ;

namespace srrg2_core {
  using namespace std;
  
  void _register_dynamic_loader() __attribute__((constructor));

  void _register_dynamic_loader() {
    BOSS_REGISTER_CLASS(DynamicLoaderConfig);
  }

  static const bool module_manager_debug = false;

  void DynamicLoaderConfig::deserializeComplete() {
    std::vector<std::string> complete_paths;
    const size_t total_so_names = so_names.value().size();
    size_t so_names_found       = 0;

    for (const auto& so_it : so_names.value()) {
      bool found = false;
      std::string complete_path;

      // ia for each path
      for (const auto& p_it : so_paths.value()) {
        // construct full path and replaces the env vars
        complete_path = p_it + "/" + so_it;
        replaceEnvTags(complete_path);
        //#ifndef NDEBUG
        std::cerr << "DynamicLoaderConfig|looking for file [" << complete_path << " ] ... ";
        //#endif

        // ia check if path exists ( fastest way :) )
        if (srrg2_core::isAccessible(complete_path)) {
          complete_paths.push_back(complete_path);
          found = true;
          ++so_names_found;
#ifndef NDEBUG
          std::cerr << "FOUND" << std::endl;
#endif
        } else {
#ifndef NDEBUG
          std::cerr << FG_RED("NOT_FOUND") << std::endl;
#endif
        }
      }
      if (!found) {
        std::cerr << "DynamicLoaderConfig|unable to find [ " << FG_RED(so_it) << " ] in paths" << endl;
      }
    }

    std::cerr << "DynamicLoaderConfig|found [ "
              << FG_YELLOW(so_names_found << "/" << total_so_names) << " ] libraries\n";
    PropertyContainerManager::initFactory(complete_paths);
  }

  void PropertyContainerManager::initFactory(const std::string& loader_config_filename) {
    Deserializer des;
    des.setFilePath(loader_config_filename);
    SerializablePtr o = 0;
    while ((o = des.readObjectShared())) {
      std::shared_ptr<DynamicLoaderConfig> loader_ptr =
        std::dynamic_pointer_cast<DynamicLoaderConfig>(o);
      if (loader_ptr) {
        std::cerr << "PropertyContainerManager::initFactory|loaded config paths from file [ "
                  << loader_config_filename << " ]" << std::endl;
        break;
      }
    }
  }

  void PropertyContainerManager::makeFactoryStub(const std::string& loader_config_filename) {
    Serializer ser;
    ser.setFilePath(loader_config_filename);
    DynamicLoaderConfig loader;
    ser.writeObject(loader);
  }

  void PropertyContainerManager::initFactory(const std::vector<std::string>& library_paths) {
    const size_t total_so_to_open = library_paths.size();
    size_t so_opened              = 0;

    for (const auto& it : library_paths) {
      std::cerr << "PropertyContainerManager::initFactory|opening library [ " << FG_YELLOW(it)
                << "] ... ";
      void* handle = dlopen(it.c_str(), RTLD_LAZY);
      if (!handle) {
        std::cerr << FG_RED("ERROR") << std::endl;
        std::cerr << "PropertyContainerManager::initFactory|library [ " << FG_BWHITE(it)
                  << " ] -- error:\n"
                  << FG_RED(dlerror()) << std::endl;
      } else {
        ++so_opened;
        std::cerr << FG_GREEN("SUCCESS") << std::endl;
      }
    }

    // ia log something
    std::cerr << "PropertyContainerManager::initFactory|opened [ "
              << FG_YELLOW(so_opened << "/" << total_so_to_open) << " ] libraries\n";

    // now we try a cast to initiate the types
    std::vector<PropertyContainerIdentifiablePtr> configurables;
    std::vector<std::string> class_names = srrg2_core::getClassNames();
    for (auto it = class_names.begin(); it != class_names.end(); ++it) {
      srrg2_core::Serializable* ser = srrg2_core::Serializable::createInstance(*it);
      srrg2_core::PropertyContainerIdentifiable* c =
        dynamic_cast<srrg2_core::PropertyContainerIdentifiable*>(ser);
      if (c) {
        configurables.push_back(PropertyContainerIdentifiablePtr(c));
      } else {
        delete ser;
      }
    }

    // we recurse in each configuration, and we list the configurable properties
    // for each of these properties, we attempt a cast with all instances of the
    // existing instances
    for (auto c : configurables) {
      DEBUG(module_manager_debug) << "name: " << c->className() << std::endl;
      for (auto prop_it : c->_properties_identifiable) {
        const std::string& prop_name = prop_it->name();
        DEBUG(module_manager_debug) << "\tproperty: " << prop_name << std::endl;
        PropertyIdentifiablePtrInterfaceBase& prop = *prop_it;
        std::vector<std::string>& assignables      = prop.assignableTypes();
        assignables.clear();
        for (auto t : configurables) {
          if (prop.canAssign(t)) {
            DEBUG(module_manager_debug) << "\t\tcandidate: " << t->className() << std::endl;
            assignables.push_back(t->className());
          }
        }
      }
    }
  }

  // lists all possible types of config that can be created
  std::vector<std::string> PropertyContainerManager::listTypes() {
    // now we try a cast to initiate the types
    std::vector<std::string> available;
    std::vector<std::string> class_names = srrg2_core::getClassNames();
    for (const std::string& name : class_names) {
      // DEBUG(module_manager_debug) << *it << " ";
      srrg2_core::Serializable* ser = srrg2_core::Serializable::createInstance(name);
      srrg2_core::PropertyContainerIdentifiable* c =
        dynamic_cast<srrg2_core::PropertyContainerIdentifiable*>(ser);
      if (c) {
        // DEBUG(module_manager_debug) << "OK" << endl;
        available.push_back(name);
      }
      // else
      //   DEBUG(module_manager_debug) << "--" << endl;
      delete ser;
    }
    return available;
  }

  void PropertyContainerManager::read(const std::string& filename) {
    _named_instances.clear();
    _instances.clear();
    _objects.clear();
    Deserializer des;
    des.setFilePath(filename);
    SerializablePtr o = 0;
    while ((o = des.readObjectShared())) {
      _objects.insert(o);
      PropertyContainerIdentifiablePtr c =
        std::dynamic_pointer_cast<PropertyContainerIdentifiable>(o);
      if (!c) {
        continue;
      }
      _instances.insert(c);
      if (!c->name().length()) {
        continue;
      }
      auto it = _named_instances.find(c->name());
      if (it != _named_instances.end()) {
        DEBUG(module_manager_debug) << "error: a config with the same name: [" << c->name() << "] "
                                    << " already exists in the system" << std::endl;
        continue;
      }
      _named_instances.insert(std::make_pair(c->name(), c));
    }
  }

  void PropertyContainerManager::refresh() {
    std::set<PropertyContainerIdentifiablePtr> all_instances;
    std::map<std::string, PropertyContainerIdentifiablePtr> named_instances;

    for (auto ident : _instances) {
      if (! ident)
        continue;
      all_instances.insert(ident);
      ident->getReacheableContainers(all_instances);
      if (ident->name()!="") {
        named_instances[ident->name()]=ident;
      }
    }
    _instances=all_instances;
    _named_instances=named_instances;
  }
  
  
  void PropertyContainerManager::write(const std::string& filename, bool enable_comments) {
    refresh();
    // we need to add to the pool all configurations that
    // might be automatically generated on compute
    _objects.clear();
    // clear all boss IDs
    for (auto it : _instances) {
      Identifiable* ident = dynamic_cast<Identifiable*>(it.get());
      if (ident) {
        ident->setId(-1, 0);
      }
      _objects.insert(it);
    }
    
    
    Serializer ser;
    ser.setCommentsEnabled(enable_comments);
    ser.setFilePath(filename);
    for (auto it : _objects) {
      ser.writeObject(*it);
    }
  }

  // renames a config
  void PropertyContainerManager::rename(PropertyContainerIdentifiablePtr conf,
                                        const std::string& name) {
    if (conf->name().length()) {
      auto it = _named_instances.find(conf->name());
      if (it == _named_instances.end()) {
        throw("PropertyContainerManager::rename|config name mismatch");
      }
      _named_instances.erase(it);
    }
    if (name.length()) {
      auto it = _named_instances.find(name);
      if (it != _named_instances.end()) {
        throw("PropertyContainerManager::rename|config name already existing");
      }
      _named_instances.insert(std::make_pair(name, conf));
    }
    conf->setName(name);
    DEBUG(module_manager_debug) << "all right" << std::endl;
  }

  // retrieves a configurable whose hame is config_name
  PropertyContainerIdentifiablePtr
  PropertyContainerManager::getByName(const std::string& config_name) {
    auto it = _named_instances.find(config_name);
    if (it == _named_instances.end()) {
      return PropertyContainerIdentifiablePtr();
    }
    return it->second;
  }

  PropertyContainerIdentifiablePtr PropertyContainerManager::create(const std::string& classname,
                                                                    const std::string& name) {
    Serializable* s = Serializable::createInstance(classname);
    if (!s) {
      return 0;
    }
    PropertyContainerIdentifiable* c = dynamic_cast<PropertyContainerIdentifiable*>(s);
    if (!s) {
      return 0;
    }
    PropertyContainerIdentifiablePtr p(c);
    p->setName(name);
    if (add(p)) {
      return p;
    }
    std::cerr << "PropertyContainerManager::create|WARNING, cannot create object [ " << classname
              << " ] with name [ " << name << " ] (maybe name is already in use?)\n";
    return 0;
  }

  // adds to the managed system a new configurable (and all connected objects)
  bool PropertyContainerManager::add(PropertyContainerIdentifiablePtr c) {
    std::cerr << "PropertyContainerManager::add|adding container [ " << c->className() << " ]"
              << std::endl;
    if (_instances.count(c)) {
      return false;
    }
    _instances.insert(c);
    _objects.insert(c);
    if (c->name().length()) {
      auto it = _named_instances.find(c->name());
      if (it != _named_instances.end()) {
        return false;
      }
      _named_instances.insert(std::make_pair(c->name(), c));
    }
    std::set<PropertyContainerIdentifiablePtr> reachable;
    c->getReacheableContainers(reachable);
    std::cerr << "PropertyContainerManager::add|container [ " << c->className() << " ] added"
              << std::endl;
    for (PropertyContainerIdentifiablePtr r : reachable) {
      std::cerr << "PropertyContainerManager::add|renaming [ " << r << " ] to ''" << std::endl;
      r->setName(""); // ds TODO is overwriting the name intended?
      _instances.insert(r);
      _objects.insert(r);
    }
    return true;
  }

  // erases a configurable from the managed system
  // detaching it from all connected confs
  void PropertyContainerManager::erase(PropertyContainerIdentifiablePtr erased) {
    DEBUG(module_manager_debug) << "object " << std::endl;
    auto o_it = _objects.find(erased);
    if (o_it == _objects.end()) {
      DEBUG(module_manager_debug) << "no object: " << std::endl;
      return;
    }
    _objects.erase(o_it);

    DEBUG(module_manager_debug) << "names" << std::endl;
    if (erased->name().length()) {
      _named_instances.erase(erased->name());
    }

    for (PropertyContainerIdentifiablePtr other : _instances) {
      DEBUG(module_manager_debug) << other->className() << std::endl;

      for (auto prop_it : other->_properties_identifiable) {
        DEBUG(module_manager_debug) << "  " << prop_it->name() << std::endl;

        PropertyIdentifiablePtrVectorInterface* v_prop =
          dynamic_cast<PropertyIdentifiablePtrVectorInterface*>(prop_it);
        if (v_prop) {
          std::vector<IdentifiablePtr> resized_vec;
          resized_vec.reserve(v_prop->size());
          for (size_t k = 0; k < v_prop->size(); ++k) {
            IdentifiablePtr ptr = v_prop->getSharedPtr(k);
            if (ptr == erased) {
              continue;
              //              v_prop->assign(k, IdentifiablePtr());
            }
            resized_vec.emplace_back(ptr);
          }
          v_prop->assign(resized_vec);
          continue;
        }
        PropertyIdentifiablePtrInterface* prop =
          dynamic_cast<PropertyIdentifiablePtrInterface*>(prop_it);
        IdentifiablePtr ptr = prop->getSharedPtr();
        if (ptr == erased) {
          prop->assign(IdentifiablePtr());
        }
      }
    }
    _instances.erase(erased);
  }

} // namespace srrg2_core
