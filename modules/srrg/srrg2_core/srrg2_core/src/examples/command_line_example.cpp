#include "srrg_system_utils/parse_command_line.h"
#include <iostream>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::command_line_example| "

using namespace srrg2_core;
using namespace std;
const char *banner[]={
  "this program",
  "does absolutely nothing",
  "besides showing how to use the command line",
  0
};

int main(int argc, char** argv) {

  //   Declare a command line argument
  //   with the arguments and optional banner explaining
  //   what the program does
  ParseCommandLine cmd_line(argv, banner);

  //   Install the parameters in the command line by construction
  //   1st arg: command line parser
  //   2nd option (without "-")
  //   3rd long option( without "--")
  //   4th description
  //   5th (for all but flags, the default value)
  ArgumentInt    a_i(&cmd_line,    "i", "integer", "this is an integer", -1);
  ArgumentDouble a_d(&cmd_line,    "d", "double",  "this is a double",   2.5);
  ArgumentString a_s(&cmd_line,    "s", "string",  "this is a string",   "boh");
  ArgumentFlag   a_flag(&cmd_line, "f", "flag",    "this is a flag");
  ArgumentFloatVector_<3> fv_flag(& cmd_line, "vf", "float vector", "this is a vector", {0,0,0});
  
  //   Call the parse, that will go through the command line
  //   if -h or --help are called, the banner will be printed with the options
  //       and the program will terminate
  //   if an unrecognized option (starting with - or with --) is detected
  //      program terminates
  //   parsing will stop at the first non option parameter
  //   the remaining tokens are accessible by lastParsedArgv;
  cmd_line.parse();

  // You can query the parameters by asking the
  // isSet() method
  // value()
  // stringValue() 
  EXAMPLE_LOG << "a_i: " << a_i.isSet() << " " << a_i.value() <<  " " << a_i.stringValue() << endl; 
  EXAMPLE_LOG << "a_d: " << a_d.isSet() << " " << a_d.value() <<  " " << a_d.stringValue() << endl; 
  EXAMPLE_LOG << "a_s: " << a_s.isSet() << " " << a_s.value() <<  " " << a_s.stringValue() << endl; 

  // This shows all non parsed arguments
  const vector<string>& other_args= cmd_line.lastParsedArgs();
  for (const string& s : other_args){
    EXAMPLE_LOG << "\t" << s << endl;
  }
  
  // this prints the summary of the parsing
  EXAMPLE_LOG << cmd_line.summary();
    
  return 0;
}
