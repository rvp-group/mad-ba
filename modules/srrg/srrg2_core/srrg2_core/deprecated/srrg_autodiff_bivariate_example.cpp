#include "srrg_autodiff/ad.h"
#include <iostream>

using namespace std;
using namespace srrg2_core;

DualValuef fancyFn(const DualValuef& x, const DualValuef& y){
  return sin(y)*exp(-x*x)/(DualValuef(2.0)+cos(DualValuef(2.0)+log(y)));
}

typedef DualValue_<float> DualValuef;
int main(int argc, char** argv) {
  DualValuef x(0.5);
  DualValuef y(0.1);

  DualValuef result=fancyFn(x,y);
  cerr << "value of the function in the point: " << result.value << endl;
  
  x.derivative=1;
  result = fancyFn(x,y);
  cerr << "value of the derivtive in the point w.r.t. x: " << result.derivative << endl;
  x.derivative=0;

  y.derivative=1;
  result = fancyFn(x,y);
  cerr << "value of the derivtive in the point w.r.t. y: " << result.derivative << endl;
  y.derivative=0;
  
}
