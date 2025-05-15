#include <iostream>
#include "srrg_data_structures/field_pack.h"
#include <vector>
#include <string>
#include <sstream>
#include "srrg_data_structures/generic_class_method_caller.h"

using namespace srrg2_core;
using namespace std;

using MyFieldPack=FieldPack_<int, string, float>;

using MyFieldPackDerived=FieldPackDerived_<MyFieldPack, float, float, string>;

using MyFieldPackDerivedDerived=FieldPackDerived_<MyFieldPackDerived, string>;


template <typename FieldPackType_, int i>
struct PrintAt_ {
  static void print_(const FieldPackType_& pack) {
    PrintAt_<FieldPackType_, i-1>::print_(pack);
    std::cerr << pack.template field<i>() << ", ";
  }
};

template <typename FieldPackType_>
struct PrintAt_<FieldPackType_, 0> {
  static void print_(const FieldPackType_& pack) {
    std::cerr << pack.template field<0>() <<", ";
  }
};


struct SimpleRunner {
  bool methodOne(int i , string s, float f) {
    std::cerr << "MethodOne" << i << " " << s << " " << f << std::endl;
    return true;
  }
};
  
int main(int argc, char** argv) {
  MyFieldPack pack;
  pack.field<0>() = 5;
  pack.field<1>() = std::string("pack_test");
  pack.field<2>() = 5.5;

  MyFieldPackDerived dpack;
  dpack.field<0>() = 50;
  dpack.field<1>() = std::string("dpack_test");
  dpack.field<2>() = 55.5;
  dpack.field<3>() = 0.1;
  dpack.field<4>() = 0.7;
  dpack.field<5>() = std::string("dpack_test_final");

  cerr << "num_fields: " << MyFieldPack::NumFields << endl;

  MyFieldPackDerivedDerived ddpack;
  ddpack.field<0>() = 500;
  ddpack.field<1>() = std::string("ddpack_test");
  ddpack.field<2>() = 555.5;
  ddpack.field<3>() = 0.01;
  ddpack.field<4>() = 0.07;
  ddpack.field<5>() = std::string("ddpack_test_middle");
  ddpack.field<6>() = std::string("ddpack_test_final");

  std::cerr<< "base: " << endl;
  PrintAt_<MyFieldPack, MyFieldPack::NumFields-1>::print_(pack);
  cerr << endl;

  std::cerr<< "derived: " << endl;
  PrintAt_<MyFieldPackDerived, MyFieldPackDerived::NumFields-1>::print_(dpack);
  cerr << endl;

  std::cerr << "base=derived" << std::endl;
  pack=dpack;
  PrintAt_<MyFieldPack, MyFieldPack::NumFields-1>::print_(pack);
  cerr << endl;

  PrintAt_<MyFieldPack, MyFieldPack::NumFields-1>::print_(pack);

  SimpleRunner runner;
  GenericClassMethodCaller_<SimpleRunner,
                            typeof(&SimpleRunner::methodOne),
                            int, std::string, float> caller;

  std::vector<std::string> tokens;
  tokens.push_back("1");
  tokens.push_back("i_am_a_string");
  tokens.push_back("1.1");
  caller.parsePackFromTokens<0>(tokens);
  caller.execute<0>(runner, &SimpleRunner::methodOne);
  
}
