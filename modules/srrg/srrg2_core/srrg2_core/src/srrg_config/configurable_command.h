#pragma once
#include "configurable.h"
#include "srrg_data_structures/generic_class_method_caller.h"
namespace srrg2_core{


  template<typename ConfigurableType_,
           typename MethodType_,
           typename ... MethodArgs_>
  class ConfigurableCommand_: public Configurable::CommandBase {
  public:
    using ConfigurableType=ConfigurableType_;
    using MethodType=MethodType_;
    using CallerType=GenericClassMethodCaller_<ConfigurableType, MethodType, MethodArgs_...>;
    ConfigurableType* _module;
    MethodType _method;
    CallerType _caller;
    static constexpr int num_args=sizeof...(MethodArgs_);
    ConfigurableCommand_(ConfigurableType_* configurable_,
                         const std::string& tag_,
                         const std::string& help_message_,
                         MethodType method_):
      Configurable::CommandBase(configurable_, tag_, help_message_),
      _module(configurable_),
      _method(method_) {}
    
    bool execute(ConfigurableShell* shell_,
                 std::string& response,
                 const std::vector<std::string>& tokens) {
      if (tokens.size()<num_args) {
        response=this->_module->className() + "|command " + this->_tag
          + " requires  " + std::to_string(num_args) + " arguments\n"
          + " you provided " +  std::to_string(tokens.size()-1) + " aborting\n";
        return false;
       }
      bool parse_ok=_caller.template parsePackFromTokens<0>(tokens);
      if (! parse_ok){
        response=this->_module->className() + "|command " + this->_tag + " parse error";
        return false;
      }
      bool exec_ok=_caller.template execute<0>(*(this->_module), this->_method);
      if (exec_ok) {
        response=_caller.pack.template field<0>();
      } else {
        response=this->_module->className() + "|command " + this->_tag + " fail";
      }
      return exec_ok;
    }
  };


  
  
}
