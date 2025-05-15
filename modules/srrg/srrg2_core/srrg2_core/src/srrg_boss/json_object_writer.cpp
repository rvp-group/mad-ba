/*
    JSON object writer implementation
    Copyright (C) 2013  Daniele Baldassari <daniele@dikappa.org>
    Copyright (C) 2013  Giorgio Grisetti   <grisetti@diag.uniroma1.it>
*/


#include <limits>
#include <stdexcept>

#include "json_object_writer.h"
#include "object_data.h"

using namespace std;
using namespace srrg2_core;

static const int max_items_on_line=20;

static string escapeJSONString(const string& str) {
  string escaped;
  for (string::const_iterator c_it=str.begin();c_it!=str.end();c_it++) {
    char c=*c_it;
    switch (c) {
    case '\\':
    case '"':
      escaped.push_back('\\');
      escaped.push_back(c);
      break;
    case '\n':
      escaped.push_back('\\');
      escaped.push_back('n');
      break;
    case '\b':
      escaped.push_back('\\');
      escaped.push_back('b');
      break;
    case '\f':
      escaped.push_back('\\');
      escaped.push_back('f');
      break;
    case '\r':
      escaped.push_back('\\');
      escaped.push_back('r');
      break;
    case '\t':
      escaped.push_back('\\');
      escaped.push_back('t');
      break;
    default:
      escaped.push_back(c);
    }
  }
  return escaped;
}

void printSpaces(ostream& os, int indentation_level){
  for (int i=0; i<indentation_level; ++i)
    os << " ";
}
static void writeJSONData(ostream& os, ValueData* val, bool comments_enabled, int indentation_level=0) {
  switch (val->type()) {
  case BOOL:
    os << val->getBool();
    break;
  case NUMBER:
    os.precision(static_cast<NumberData*>(val)->precision());
    os << val->getDouble();
    break;
  case STRING:
    os << '"' << escapeJSONString(val->getString()) << '"';
    break;
  case ARRAY: {
    os << "[ ";
    ArrayData* v_array=static_cast<ArrayData*>(val);
    size_t pos=0;
    int items_on_line=0;
    for (vector<ValueData*>::const_iterator v_it=v_array->begin();v_it!=v_array->end();v_it++, pos++) {
      items_on_line++;
      writeJSONData(os,*v_it, comments_enabled);
      if (pos!=v_array->size()-1) {
        os << ", ";
      }
      if (items_on_line>max_items_on_line && pos!=v_array->size()-1){
        os << endl;
        items_on_line=0;
      }
    }
    os << " ]";
    break;
  }
  case OBJECT: {
    os << "{ " << endl;
    ObjectData* o=static_cast<ObjectData*>(val);
    const vector<string>& fields=o->fields();
    size_t pos=0;
    for (vector<string>::const_iterator f_it=fields.begin();f_it!=fields.end();++f_it, ++pos) {
      ValueData* fdata=o->getField(*f_it);
      if(comments_enabled
         && fdata->description()
         && fdata->description()[0]){
        os << endl;
        printSpaces(os, indentation_level+2);
        os << "// " << fdata->description() <<  endl;
      }
      printSpaces(os, indentation_level+2);
      os << '"' << escapeJSONString(*f_it) << "\" : ";
      writeJSONData(os, fdata, comments_enabled, indentation_level+2);
      if (pos!=fields.size()-1) {
        os << ", ";
      }
      os << endl;
      printSpaces(os, indentation_level);
    }
    os << " }";
    break;
  }
  case POINTER: {
    os << val->getPointer();
    break;
  }
  default:
    throw logic_error("unexpected value type: "+val->typeName());
  }
}

void JSONObjectWriter::writeObject(ostream& os, const string& type, ObjectData& object, bool comments_enabled) {
  int precision=os.precision();
  os << "\"" << escapeJSONString(type) << "\" : ";
  writeJSONData(os,&object, comments_enabled);
  os << endl << endl;
  
  os.precision(precision);
}
