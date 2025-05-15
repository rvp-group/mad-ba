#include <iostream>
#include "srrg_data_structures/abstract_map.h"
#include "srrg_data_structures/abstract_ptr_map.h"

using namespace srrg2_core;
using namespace std;

#warning "TODO: Explain something about this example."

using AbstractIntFloatMap=AbstractMap_<int, float>;

using AbstractIntFloatPtrMap=AbstractPtrMap_<int, float, std::shared_ptr<float> >;

using AbstractIntFloatRawPtrMap=AbstractMap_<int, float*>;

int main(int argc, char** argv) {
  {
    AbstractIntFloatMap m;
    cerr << "inserting" << endl;
  
    m.insert(std::make_pair(1,0.1f));
    m.insert(std::make_pair(5,0.5f));
    m.insert(std::make_pair(2,0.2f));

    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << (*it).first << " " << (*it).second << endl;
    }

    cerr << "searching" << endl;
    auto it=m.find(3);
    if (it==m.end())
      cerr << "not found" << endl;

    cerr << "erasing" << endl;
    it=m.find(5);
    cerr << it.key() << " " << it.value() << endl;
  
    m.erase(it);
  
    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << it.key() << " " << it.value() << endl;
    }
  }

  {
    AbstractIntFloatPtrMap m;
    cerr << "inserting" << endl;
  
    m.insert(std::make_pair(1,std::shared_ptr<float>(new float(0.1f))));
    m.insert(std::make_pair(5,std::shared_ptr<float>(new float(0.5f))));
    m.insert(std::make_pair(2,std::shared_ptr<float>(new float(0.2f))));

    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << (*it).first << " " << *(*it).second << endl;
    }

    cerr << "searching" << endl;
    auto it=m.find(3);
    if (it==m.end())
      cerr << "not found" << endl;

    cerr << "erasing" << endl;
    it=m.find(5);
    cerr << it.key() << " " << *it.value() << endl;
  
    m.erase(it);
  
    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << it.key() << " " << *it.value() << endl;
    }

  }

  {
    AbstractIntFloatRawPtrMap m;
    cerr << "inserting" << endl;
  
    m.insert(std::make_pair(1,new float(0.1f)));
    m.insert(std::make_pair(5,new float(0.5f)));
    m.insert(std::make_pair(2,new float(0.2f)));

    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << (*it).first << " " << *(*it).second << endl;
    }

    cerr << "searching" << endl;
    auto it=m.find(3);
    if (it==m.end())
      cerr << "not found" << endl;

    cerr << "erasing" << endl;
    it=m.find(5);
    cerr << it.key() << " " << *it.value() << endl;
  
    m.erase(it);
  
    cerr << "printing" << endl;
    for (auto it=m.begin(); it!=m.end(); ++it) {
      cerr << it.key() << " " << *it.value() << endl;
    }

  }

}
