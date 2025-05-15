#pragma once
#include "property_container.h"
#include "property_vector.h"
namespace srrg2_core {

  class DynamicLoaderConfig : public PropertyContainerSerializable {
  public:
    PropertyVector_<std::string> so_paths =
      PropertyVector_<std::string>("so_paths", "path where to look for the so", this);

    PropertyVector_<std::string> so_names =
      PropertyVector_<std::string>("so_names", "names of the .so to load in this manager", this);
    void deserializeComplete() override;
  };

  class PropertyContainerManager : public PropertyContainerSerializable {
  public:
    //! loads a a set of *.so files whose names are passed as argument
    static void
    initFactory(const std::vector<std::string>& library_paths = std::vector<std::string>());
    static void initFactory(const std::string& loader_config_filename);
    static void makeFactoryStub(const std::string& loader_config_filename);
    static std::vector<std::string> listTypes();

    // reads a config from a file
    void read(const std::string& filename);

    // retrieves a configurable whose hame is config_name
    PropertyContainerIdentifiablePtr getByName(const std::string& config_name);

    // creates from the managed system a new configurable instance
    PropertyContainerIdentifiablePtr create(const std::string& config_classname,
                                            const std::string& config_name = "");

    // creates a configurable from a class
    template <typename ContainerType>
    std::shared_ptr<ContainerType> create(const std::string& name = "") {
      std::shared_ptr<ContainerType> c(new ContainerType);
      c->setName(name);
      if (add(c)) {
        return c;
      }
      std::cerr << "PropertyContainerManager::create|WARNING, cannot create object [ "
                << c->className() << " ] with name [ " << name
                << " ] (maybe name is already in use?)\n";
      return 0;
    }

    // erases a configurable from the managed system
    // detaching it from all connected confs
    void erase(PropertyContainerIdentifiablePtr configurable);

    void rename(PropertyContainerIdentifiablePtr conf, const std::string& name);

    void write(const std::string& filename, bool enable_comments=true);

    // directly instantiates a configurable or terminates the program
    template <typename IdentifiableType_>
    std::shared_ptr<IdentifiableType_> getByName(const std::string& name) {
      PropertyContainerIdentifiablePtr c = getByName(name);
      if (!c) {
        std::cerr << "no object with name [" << name << "] found in file" << std::endl;
        return 0;
      }
      std::shared_ptr<IdentifiableType_> m = std::dynamic_pointer_cast<IdentifiableType_>(c);
      if (!m) {
        std::cerr << "cast_fail [ " << name << " ]" << std::endl;
        std::cerr << "conf_name [ " << c->className() << " ]" << std::endl;
        return 0;
      }
      return m;
    }

    inline std::set<PropertyContainerIdentifiablePtr>& instances() {
      return _instances;
    }
    inline std::map<std::string, PropertyContainerIdentifiablePtr>& namedInstances() {
      return _named_instances;
    }
    void refresh();
  protected:
    bool add(PropertyContainerIdentifiablePtr c);

    std::map<std::string, PropertyContainerIdentifiablePtr> _named_instances;
    std::set<PropertyContainerIdentifiablePtr> _instances;
    std::set<SerializablePtr> _objects;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_core
