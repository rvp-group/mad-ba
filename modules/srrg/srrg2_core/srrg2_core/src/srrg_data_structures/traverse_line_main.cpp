#include "traverse_line_bresenham.h"
#include <iostream>
#include <cstdlib>

using namespace srrg2_core;

int main(int argc, char** argv) {
  int x1=atoi(argv[1]);
  int y1=atoi(argv[2]);
  int x2=atoi(argv[3]);
  int y2=atoi(argv[4]);
  LineActionPrint action;
  traverseLine(x1, y1, x2, y2, action);
  
}
