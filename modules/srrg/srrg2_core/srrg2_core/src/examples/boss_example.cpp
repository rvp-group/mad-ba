#include <stdint.h>
#include "srrg_boss/object_data.h"
#include "srrg_boss/serializer.h"
#include "srrg_boss/deserializer.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_system_utils/parse_command_line.h"
#include <iostream>

using namespace srrg2_core;
using namespace std;

/* Boss serialization system example */

// Step 1 : Derive your class from Serializable

class AnObject: public Serializable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  int count;
  Eigen::Vector3f point;
  std::vector<int> values;
  void serialize(ObjectData& data, IdContext& context) override;
  void deserialize(ObjectData& data, IdContext& context) override;
};

// Step 2 : Implement the serialize/deserialize methods

void AnObject::serialize(ObjectData& data, IdContext& context) {
  // serialize string ...
  data.setString("name", name);
  // An Eigen Matrix ...
  data.setEigen<Eigen::Vector3f>("point", point);
  // Basic types ...
  data.setInt("count", count);
  // An std::vector
  ArrayData* adata=new ArrayData;
  adata->reserve(values.size());
  for (size_t i=0; i<values.size(); ++i)
    adata->add(values[i]);
  data.setField("values", adata);
}

void AnObject::deserialize(ObjectData& data, IdContext& context){
  // deserialize a string...
  name=data.getString("name");
  // an integer...
  count= data.getInt("count");
  point=data.getEigen<Eigen::Vector3f>("point");
  // An std::vector ...
  ArrayData* adata=dynamic_cast<ArrayData*>(data.getField("values"));
  if(! adata)
    throw std::runtime_error("deserialization error");
  values.resize(adata->size());
  for (size_t i=0; i<values.size(); ++i)
    values[i]=(*adata)[i].getInt();
}

// Step 3 : Register the class in the boss serialization system

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
    AnObject o;
    o.name="mirco_colosi";
    o.count=0;
    for (int i=0; i<100; ++i){ 
      o.values.push_back(i);
    }
    o.point << .1, .2, .3;

    // Instanciate a serializer
    Serializer ser;
    cerr << "writing on file [" << filename.value() << "]" << endl;
    // Set filename
    ser.setFilePath(filename.value());
    for (int i=0; i<10; ++i){
      // Serialize the object
      ser.writeObject(o);
      o.count++;
    }

  } else {
    // Instanciate a deserializer
    Deserializer des;
    // set filename
    des.setFilePath(filename.value());
    cerr << "reading from file [" << filename.value() << "]" << endl;
    // Deserialize object
    SerializablePtr o;
    while ((o=des.readObjectShared())){
      cerr << " " << endl;
      cerr << "Object pointer : " << o << endl;
      cerr << "Type : " << o->className() << endl;
      // Dynamic cast to proper type
      std::shared_ptr<AnObject> ao=std::dynamic_pointer_cast<AnObject>(o);
      if (ao) {
        cerr << "Vector size : " << ao->values.size()  << endl;
        cerr << "Name : " << ao->name << endl;
        cerr << "Count : " << ao->count << endl;
        cerr << "Point : " << ao->point.transpose() << endl;
      }
    }
  }
}
