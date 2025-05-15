#pragma once
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

namespace srrg2_core {

  class ParseCommandLine;
  
  struct ArgumentFlag {
    friend class ParseCommandLine;
    ArgumentFlag(ParseCommandLine* cmd_line,
                 const std::string& option_,
                 const std::string& long_option_,
                 const std::string& explanation_,
                 int num_fields_=0);
      
    virtual char** parseArgument(char** argv);
    virtual std::string stringValue() const;
    inline bool isSet() const {return _is_set;}
    inline const std::string& option() const {return _option;}
    inline const std::string& longOption() const {return _long_option;}
    inline const std::string& explanation() const {return _explanation;}
    inline const int numFields() const {return _num_fields;}
    
  protected:
    std::string _option;
    std::string _long_option;
    std::string _explanation;
    int _num_fields;
    bool _is_set=false;
  };

  struct ParseCommandLine{

    ParseCommandLine(char** argv_, const char** what_does_=0);
    void addArgument(ArgumentFlag* arg);
    std::string options();
    std::string summary();
    inline const std::vector<std::string>& lastParsedArgs() const {return _last_parsed_args;}
    void parse();
  protected:
    std::string prog_name;
    char** _argv;
    const char** _what_does;
    std::map<std::string,ArgumentFlag*> _arg_map;
    std::map<std::string,ArgumentFlag*> _long_arg_map;
    std::vector<std::string> _last_parsed_args;

    ArgumentFlag _help_arg;
  };

  template <typename ArgumentType_, int NumFields_=1>
  struct Argument_: public ArgumentFlag {
      
    using ArgumentType=ArgumentType_;
    const ArgumentType_& value() {return _value;}
    static constexpr int NumFields=NumFields_;
  
    Argument_(ParseCommandLine* cmd_line,
              const std::string& option,
              const std::string& long_option,
              const std::string& explanation,
              const ArgumentType_& default_value):
      ArgumentFlag(cmd_line,option,long_option, explanation, NumFields)  {
      if constexpr(NumFields==1) {
          _value=default_value;
        } else {
        for (int i=0; i<NumFields; ++i)
          _value[i]=default_value[i];
      }

    }

    char** parseArgument(char** argv) override {
      if constexpr(NumFields==1) {
          if (!*argv)
            return argv;
          std::istringstream is(*argv);
          is >> _value;
          return argv+1;
        } else {
        for (int i=0; i<NumFields; ++i) {
          if (!*(argv+i))
            return argv+i;
          std::istringstream is(*(argv+i));
          is >> _value[i];
          if (! is)
            return argv+i;
        }
        return argv+NumFields;
      }
    }

    std::string stringValue() const override {
      std::ostringstream os;
      if constexpr(NumFields==1) {
          os << _value;
        }
      else {
        for (int i=0; i<NumFields; ++i) {
          os << _value[i] << " ";
        }
      }
      return os.str();
    }
    
    inline const ArgumentType& value() const {return _value;}
    inline void  setValue(const ArgumentType& value_)  {_value = value_;}
  protected:
    ArgumentType _value;
  };

  using ArgumentInt=Argument_<int> ;
  using ArgumentFloat=Argument_<float> ;
  using ArgumentDouble=Argument_<double> ;
  using ArgumentString=Argument_<std::string>;

  template <int NumFields_>
  using ArgumentFloatVector_=Argument_<float[NumFields_], NumFields_>;

  template <int NumFields_>
  using ArgumentDoubleVector_=Argument_<double[NumFields_], NumFields_>;

  template <int NumFields_>
  using ArgumentIntVector_=Argument_<int[NumFields_], NumFields_>;
}
