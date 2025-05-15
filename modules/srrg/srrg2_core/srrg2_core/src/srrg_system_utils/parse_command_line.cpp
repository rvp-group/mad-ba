#include "parse_command_line.h"
#include <iostream>
#include <cstring>
#include "env.h"
namespace srrg2_core {

  
  ArgumentFlag::ArgumentFlag(ParseCommandLine* cmd_line,
                             const std::string& option_,
                             const std::string& long_option_,
                             const std::string& explanation_,
                             int num_fields_):
    _option(std::string("-")+option_),
    _long_option(std::string("--")+long_option_),
    _explanation(explanation_),
    _num_fields(num_fields_){
    cmd_line->addArgument(this);
  }
      
  char** ArgumentFlag::parseArgument(char** argv){
    return argv;
  }

  std::string ArgumentFlag::stringValue() const {
    if (_is_set)
      return "set";
    else
      return "not-set";
  }

  ParseCommandLine::ParseCommandLine(char** argv_,const char** what_does_):
    _argv(argv_),
    _what_does(what_does_),
    _help_arg(this, "h", "help", "displays this help message")
  {}
  
  void ParseCommandLine::addArgument(ArgumentFlag* arg) {
    auto it=_arg_map.find(arg->option());
    if(it!=_arg_map.end()){
      throw std::runtime_error("argument already registered");
    }
    it=_long_arg_map.find(arg->longOption());
    if(it!=_long_arg_map.end()){
      throw std::runtime_error("long argument already registered");
    }
    _arg_map.insert(std::make_pair(arg->option(), arg));
    _long_arg_map.insert(std::make_pair(arg->longOption(), arg));
  }

  std::string ParseCommandLine::options() {
    std::ostringstream os;
    os << prog_name << std::endl;
    const char** wd=_what_does;
    while(_what_does &&*wd){
      os << *wd << std::endl;
      wd++;
    }
    os << std::endl << std::endl;
    for (auto it:_arg_map) {
      ArgumentFlag* arg=it.second;
      os << "\t" << arg->explanation() << std::endl; 
      os << "\t" << arg->option()
         << " (" << arg->longOption() << ")" 
         << ", default: [" << arg->stringValue() << "]"<< std::endl << std::endl;
    }
    return os.str();
  }

  std::string ParseCommandLine::summary() {
    std::ostringstream os;
    os << prog_name << std::endl;
    for (auto it:_long_arg_map) {
      ArgumentFlag* arg=it.second;
      if (arg->isSet()) {
        os << "\t" << arg->option()
           << "\t" << arg->stringValue() << std::endl;
      }
    }
    os  << "other args: " << std::endl;
    for (const std::string& s : lastParsedArgs()) {
      os << "\t" << s << std::endl;
    }
    return os.str();
  }

  void ParseCommandLine::parse() {
    prog_name=_argv[0];
    char** last_parsed_argv=_argv+1;
    while(*last_parsed_argv) {
      char** prev_parsed_argv=last_parsed_argv;
      ArgumentFlag* arg=0;
      
      // we seek in the long option map
      if (!strncmp("--",*last_parsed_argv, 2)){
        auto it=_long_arg_map.find(*last_parsed_argv);
        if (it!=_long_arg_map.end()) {
          arg=it->second;
          last_parsed_argv++;
        } else {
          std::cerr << "unknown arg: [" << *last_parsed_argv << "]" << std::endl;
          exit(-1);
        }
        
      } // we seek in the short option map
      else if (! strncmp("-",*last_parsed_argv, 1)) {
        auto it=_arg_map.find(*last_parsed_argv);
        if (it!=_arg_map.end()) {
          arg=it->second;
          last_parsed_argv++;
        } else {
          std::cerr << "unknown arg:  [" << *last_parsed_argv << "]" << std::endl;
          exit(-1);
        }
      }
      if (arg==&_help_arg) {
        std::cerr << options() << std::endl;
        exit(0);
      }
      if (arg) {
        arg->_is_set=true;
        if (arg->numFields())
          last_parsed_argv=arg->parseArgument(last_parsed_argv);
      }
      
      if (last_parsed_argv==prev_parsed_argv)
        break;
    }
    char** larg=last_parsed_argv;
    while(*larg) {
      _last_parsed_args.push_back(std::string(*larg));
      ++larg;
    }
    initEnvMap();
    envMapAddCmdArgs(_last_parsed_args);
  }
}
