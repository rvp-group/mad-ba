#include "srrg_autodiff/ad.h"
#include <iostream>

using namespace std;
using namespace srrg2_core;

DualValuef fancyFn(const DualValuef& x){
  return sin(x)*exp(-x*x)/(DualValuef(2.0)+cos(DualValuef(2.0)+log(x)));
}

int main(int argc, char** argv) {
  DualValuef result=fancyFn(DualValuef (0.5,1));
  cerr << "value of the function in the point: " << result.value << endl;
  cerr << "value of the derivtive in the point: " << result.derivative << endl;
}
