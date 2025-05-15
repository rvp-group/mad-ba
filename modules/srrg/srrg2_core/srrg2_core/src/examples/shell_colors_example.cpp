#include <iostream>
#include <srrg_system_utils/shell_colors.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::shell_colors_example| "

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  EXAMPLE_LOG << "testing shell color" << endl << endl;
  string s;

  s = "color"; 
  EXAMPLE_LOG << FG_BLACK(s) << endl;
  EXAMPLE_LOG << FG_RED(s) << endl;
  EXAMPLE_LOG << FG_GREEN(s) << endl;
  EXAMPLE_LOG << FG_YELLOW(s) << endl;
  EXAMPLE_LOG << FG_BLUE(s) << endl;
  EXAMPLE_LOG << FG_MAGENTA(s) << endl;
  EXAMPLE_LOG << FG_CYAN(s) << endl;
  EXAMPLE_LOG << FG_WHITE(s) << endl;
  EXAMPLE_LOG << endl;

  s = "bold color";
  EXAMPLE_LOG << FG_BBLACK(s) << endl;
  EXAMPLE_LOG << FG_BRED(s) << endl;
  EXAMPLE_LOG << FG_BGREEN(s) << endl;
  EXAMPLE_LOG << FG_BYELLOW(s) << endl;
  EXAMPLE_LOG << FG_BBLUE(s) << endl;
  EXAMPLE_LOG << FG_BMAGENTA(s) << endl;
  EXAMPLE_LOG << FG_BCYAN(s) << endl;
  EXAMPLE_LOG << FG_BWHITE(s) << endl;
  EXAMPLE_LOG << endl;

  s = "underlined color";
  EXAMPLE_LOG << FG_ULBLACK(s) << endl;
  EXAMPLE_LOG << FG_ULRED(s) << endl;
  EXAMPLE_LOG << FG_ULGREEN(s) << endl;
  EXAMPLE_LOG << FG_ULYELLOW(s) << endl;
  EXAMPLE_LOG << FG_ULBLUE(s) << endl;
  EXAMPLE_LOG << FG_ULMAGENTA(s) << endl;
  EXAMPLE_LOG << FG_ULCYAN(s) << endl;
  EXAMPLE_LOG << FG_ULWHITE(s) << endl;
  EXAMPLE_LOG << endl;

  s = "italic color";
  EXAMPLE_LOG << FG_IBLACK(s) << endl;
  EXAMPLE_LOG << FG_IRED(s) << endl;
  EXAMPLE_LOG << FG_IGREEN(s) << endl;
  EXAMPLE_LOG << FG_IYELLOW(s) << endl;
  EXAMPLE_LOG << FG_IBLUE(s) << endl;
  EXAMPLE_LOG << FG_IMAGENTA(s) << endl;
  EXAMPLE_LOG << FG_ICYAN(s) << endl;
  EXAMPLE_LOG << FG_IWHITE(s) << endl;
  EXAMPLE_LOG << endl;

  /*
  s = "reversed color";
  EXAMPLE_LOG << FG_REVBLACK(s) << endl;
  EXAMPLE_LOG << FG_REVRED(s) << endl;
  EXAMPLE_LOG << FG_REVGREEN(s) << endl;
  EXAMPLE_LOG << FG_REVYELLOW(s) << endl;
  EXAMPLE_LOG << FG_REVBLUE(s) << endl;
  EXAMPLE_LOG << FG_REVMAGENTA(s) << endl;
  EXAMPLE_LOG << FG_REVCYAN(s) << endl;
  EXAMPLE_LOG << FG_REVWHITE(s) << endl;
  EXAMPLE_LOG << endl;
  */
  return 0;
}
