#pragma once
#include <cstdlib>
#include <iostream>
#include <utility>
#include <cassert>
namespace srrg2_core {
  using namespace std;

  struct LineActionPrint{
    bool operator()(int x, int y){
      std::cerr << "(" << x << "," << y << ")" << std::endl;
      return true;
    }
  };

  template <typename ActionType>
  bool traverseLine(int x1, int y1, int x2, int y2, ActionType& action)  {
    bool swapped=false;
    if (abs(y2-y1)>abs(x2-x1)) {
      swapped=true;
      std::swap(x1,y1);
      std::swap(x2,y2);
    }
    int dx=(x2 - x1);
    int dy=(y2 - y1);
    int inc = (x2-x1)>=0 ? 1: -1;
    float slope=0.f;
    if (dx) {
      slope = (float)dy / (float)dx;
    }
    dx+=inc;
    
    for (int d=0; d!=dx; d+=inc) {
      int x=x1+d;
      int y=y1+d*slope;
      if (swapped) {
        if (! action(y,x) )
          return false;
      } else {
        if (! action(x,y) )
          return false;
      }
    }
    return true;
  }

}
