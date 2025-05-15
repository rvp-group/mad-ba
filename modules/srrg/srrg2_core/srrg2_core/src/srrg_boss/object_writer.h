#pragma once

#include<iostream>
#include<string>

namespace srrg2_core {

class ObjectData;

class ObjectWriter {
public:
  virtual void writeObject(std::ostream& os,
                           const std::string& type,
                           ObjectData& object,
                           bool enable_comments=false)=0;
  virtual ~ObjectWriter() {}
protected:
};

}
