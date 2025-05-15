#pragma once
#include <sstream>
#include <string>
#include <vector>
#include "field_pack.h"

namespace srrg2_core{

  template <typename ClassType_,
            typename MethodType_,
            typename ... MethodArgs_> 
  struct GenericClassMethodCaller_ {
    using ClassType=ClassType_;
    using MethodType=MethodType_;
    using PackType=FieldPack_<MethodArgs_ ...>;
    PackType pack;
  

    template <int arg_num=0>
    bool parsePackFromTokens(const std::vector<std::string>& tokens) {
      if ((size_t)arg_num > tokens.size()) {
        return false;
      }
      if constexpr (arg_num == PackType::NumFields){
          return true;
        }
      if constexpr (arg_num <PackType::NumFields) {
          std::istringstream is(tokens[arg_num].c_str());
          is >> pack.template field<arg_num>();
          return parsePackFromTokens<arg_num+1>(tokens);
        }
    }

    template< const int num_args_unrolled=0,
              typename ... ArgsUnrolled_>
    bool execute(ClassType& my_class,
                 MethodType my_method,
                 ArgsUnrolled_... my_args) {
      if constexpr(num_args_unrolled==PackType::NumFields) {
          bool retval=(my_class.*my_method)(my_args ...);
          return retval;
        }
      if constexpr(num_args_unrolled<PackType::NumFields) {
          return this->execute < num_args_unrolled+1,
                          ArgsUnrolled_ ...,
                          typename PackType::template TypeAt<num_args_unrolled>&
                          >
            (my_class, my_method, my_args..., pack.template field<num_args_unrolled>());
        }
      return false;
                  
    }
  };
}
