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

class AnObject: public Identifiable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  int count;
  Eigen::Vector3f point;
  std::vector<int> values;
  std::weak_ptr<AnObject> next;
  std::weak_ptr<AnObject> previous;
  
  void serialize(ObjectData& data, IdContext& context) override;
  void deserialize(ObjectData& data, IdContext& context) override;
};

// Step 2 : Implement the serialize/deserialize methods

void AnObject::serialize(ObjectData& data, IdContext& context) {
  Identifiable::serialize(data, context);
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
  std::shared_ptr<AnObject> ptr=previous.lock();
  data.setPointer("previous", ptr);
  ptr=next.lock();
  data.setPointer("next", ptr);
}

void AnObject::deserialize(ObjectData& data, IdContext& context){
  Identifiable::deserialize(data, context);
  // deserialize a string...
  name=data.getString("name");
  // an integer...
  count= data.getInt("count");
  // An Eigen Matrix...
  point = data.getEigen<Eigen::Vector3f>("point");
  // An std::vector ...
  ArrayData* adata=dynamic_cast<ArrayData*>(data.getField("values"));
  if(! adata)
    throw std::runtime_error("deserialization error");
  values.resize(adata->size());
  for (size_t i=0; i<values.size(); ++i)
    values[i]=(*adata)[i].getInt();
  //std::cerr << "getting ref for previous" << std::endl;
  data.getReference("previous").bind(previous);
  //std::cerr << "getting ref for next" << std::endl;
  data.getReference("next").bind(next);
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
    std::vector< std::shared_ptr<AnObject> > v;

    for (int i=0; i<10; ++i){
      std::shared_ptr<AnObject> o(new AnObject);
      v.push_back(o);
      o->name="mirco_colosi";
      o->count=0;
      for (int i=0; i<100; ++i){ 
        o->values.push_back(i);
      }
      o->point << .1, .2, .3;
      if (i>0){
        o->previous=v[i-1];
        o->previous.lock()->next=o;
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
        cerr << "Name : " << ao->name << endl;
        cerr << "Count : " << ao->count << endl;
        cerr << "Point : " << ao->point.transpose() << endl;
        std::shared_ptr<AnObject> prev=ao->previous.lock();
        std::shared_ptr<AnObject> next=ao->next.lock();
        if (prev)
          cerr << "prev_id : " << prev->getId() << endl;
        if (next)
          cerr << "next_id : " << next->getId() << endl;
      }

    }
  }
}
