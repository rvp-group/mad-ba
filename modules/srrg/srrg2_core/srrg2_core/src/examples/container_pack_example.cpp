#include <iostream>
#include <list>
#include <vector>
#include "srrg_data_structures/container_pack.h"

using namespace srrg2_core;
using namespace std;

using MyContainerPack=ContainerPack_<int, std::list<int>, std::vector<int>, std::list<int> >;

int main(int argc, char** argv) {
  MyContainerPack pack;
  cerr << "constructed" << endl;
  pack.field<0>().push_back(10);
  pack.field<0>().push_back(20);
  pack.field<0>().push_back(30);
  pack.field<1>().push_back(40);
  pack.field<1>().push_back(50);
  pack.field<2>().push_back(60);
  pack.field<2>().push_back(70);

  cerr << "num_fields: " << MyContainerPack::NumFields << endl;

  cerr << pack.size() << endl;

  pack.updateBeginEnd();

  for (auto v : pack) {
    cerr << v << endl;
  }
  pack.field<2>().push_front(-20);
  pack.updateBeginEnd();

  for (auto v : pack) {
    cerr << v << endl;
  }

}
