#include <stdint.h>
#include "srrg_boss/serializer.h"
#include "srrg_boss/deserializer.h"
#include "srrg_property/property_container.h"
#include "srrg_property/property_identifiable.h"
#include "srrg_property/property_eigen.h"
#include "srrg_property/property_vector.h"
#include "srrg_property/property.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_system_utils/parse_command_line.h"
#include <iostream>

using namespace srrg2_core;
using namespace std;

/* Boss serialization system example */

// Step 1 : Derive your class from Serializable

class AnObject: public PropertyContainerIdentifiable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PropertyInt count;
  PropertyEigen_<Eigen::Vector3f> point;
  PropertyVector_<int> values;
  PropertyIdentifiablePtr_<std::weak_ptr<AnObject> > next;
  PropertyIdentifiablePtr_<std::weak_ptr<AnObject> > previous;
  PropertyIdentifiablePtrVector_<std::weak_ptr<AnObject> > all_of_them;
  AnObject():
    count("count", "", this, 0, 0),
    point("point", "", this, Eigen::Vector3f(0,0,0), 0),
    values("values", "", this, 0),
    next("next", "", this, std::weak_ptr<AnObject>(), 0),
    previous("prev", "", this, std::weak_ptr<AnObject>(), 0),
    all_of_them("all", "", this,0)
  {}
  // serialization comes for free
};

// Step 2 : Register the class in the boss serialization system

BOSS_REGISTER_CLASS(AnObject)

const char * banner[]={
    "srrg_boss_example: illustrates how to serialize and deserialize a simple object with boss",
    "usage: srrg_boss_example [-w] -f out.json",
    0
};

int main(int argc, char** argv) {

  ParseCommandLine cmd(argv, banner);
  ArgumentFlag write(&cmd,"w","write","Serialize class, if not set it will deserialize");
  ArgumentString filename(&cmd,"f","filename","Filename where to read/write","boss_example.json");
  cmd.parse();

  if (write.isSet()) {
    std::vector< std::shared_ptr<AnObject> > v;

    for (int i=0; i<10; ++i){
      std::shared_ptr<AnObject> o(new AnObject);
      v.push_back(o);
      o->count.setValue(i);
      for (int i=0; i<100; ++i){ 
        o->values.pushBack(i);
      }
      o->point.setValue(Eigen::Vector3f(.1, .2, .3));
      if (i>0){
        o->previous.setValue(v[i-1]);
        o->previous.value().lock()->next.setValue(o);
      }
      for (int j=0; j<i; ++j){
        o->all_of_them.pushBack(v[j]);
      }
    }


    // Instanciate a serializer
    Serializer ser;
    cerr << "writing on file [" << filename.value() << "]" << endl;
    // Set filename
    ser.setFilePath(filename.value());
    for (auto& o: v){
      // Serialize the object
      ser.writeObject(*o.get());
    }

  } else {
    // Instanciate a deserializer
    Deserializer des;
    // set filename
    des.setFilePath(filename.value());
    cerr << "reading from file [" << filename.value() << "]" << endl;
    // Deserialize object
    SerializablePtr o;
    std::vector<SerializablePtr> v;
    while ((o=des.readObjectShared())){
      v.push_back(o);
    }

    for (auto o: v) {
      cerr << "Object pointer : " << o << endl;
      cerr << "Type : " << o->className() << endl;
      // Dynamic cast to proper type
      std::shared_ptr<AnObject> ao=std::dynamic_pointer_cast<AnObject>(o);
      if (ao) {
        cerr << "Vector size : " << ao->values.size()  << endl;
        cerr << "Count : " << ao->count.value() << endl;
        cerr << "Point : " << ao->point.value().transpose() << endl;
        std::shared_ptr<AnObject> prev=ao->previous.value().lock();
        std::shared_ptr<AnObject> next=ao->next.value().lock();
        if (prev)
          cerr << "prev_id : " << prev->getId() << endl;
        if (next)
          cerr << "next_id : " << next->getId() << endl;

        std::cerr << "all: [";
        for (size_t j=0; j<ao->all_of_them.size(); ++j){
          std::cerr << ao->all_of_them.value(j).lock()->getId() << " ";
        }
        std::cerr << " ]" << std::endl;

      }
    }
  }
}
