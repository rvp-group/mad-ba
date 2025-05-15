#include <ctime>
#include <sstream>
#include "serialization_context.h"
#include "serializer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace srrg2_core;
using namespace std;


static const string DEFAULT_DATA_FILE="data.log";
static const string DEFAULT_BLOB_FILE="binary/<classname>.<nameAttribute>.<id>.<ext>";
static const string INDEXED_BLOB_FILE="binary/<classname>.<nameAttribute>.<ext>";
static const string CURRENT_DIR=".";


static string toString(int i, size_t width, char padding) {
  stringstream ss;
  ss.width(width);
  ss.fill(padding);
  ss <<  i;
  return ss.str();
}

void SerializationContext::loadCurrentTime() {
  time_t currentTime=time(0);
  struct tm* timeStruct=localtime(&currentTime);
  _envMap["yyyy"]=toString(timeStruct->tm_year+1900,4,'0');
  _envMap["yy"]=toString((timeStruct->tm_year+1900)%100,2,'0');
  _envMap["mm"]=toString(timeStruct->tm_mon+1,2,'0');
  _envMap["dd"]=toString(timeStruct->tm_mday,2,'0');
  _envMap["hh"]=toString(timeStruct->tm_hour,2,'0');
  _envMap["mi"]=toString(timeStruct->tm_min,2,'0');
  _envMap["ss"]=toString(timeStruct->tm_sec,2,'0');
}

void SerializationContext::replaceEnvTags(string& str) {
  string::iterator beginTag=str.end();
  for (string::iterator it=str.begin();it!=str.end();it++) {
    if (*it=='<') {
      beginTag=it;
      continue;
    }
    if (*it=='>'&&beginTag<it) {
      string replacement=_envMap[str.substr(beginTag+1-str.begin(),it-beginTag-1)];
      size_t newpos=beginTag-str.begin()+replacement.length()-1;
      str.replace(beginTag,it+1,replacement);
      it=str.begin()+newpos;
    }
  }
}


static void adjustBinaryPath(string& fname, const map<string,string>& envMap) {
  //Check if it's an absolute path
  if (fname[0]!='/') {
    fname.insert(0,"/");
    auto it=envMap.find("datadir");
    if (it!=envMap.end())
      fname.insert(0,it->second);
    else
      throw std::runtime_error("BOSS: no data dir");
  }
}

SerializationContext::SerializationContext() {
  _inputStream = 0;
  _outputStream = 0;
  setOutputFilePath(DEFAULT_DATA_FILE);
  setBinaryPath(DEFAULT_BLOB_FILE);
}

SerializationContext::~SerializationContext() {
  destroyInputStream();
  destroyOutputStream();
}


string SerializationContext::createBinaryFilePath(BaseBLOBReference& instance) {
  _envMap["classname"]=instance.className();
  _envMap["nameAttribute"]=instance.nameAttribute();
  _envMap["id"]=toString(instance.getId(),7,'0');
  if(instance.index()>=0) {
    _envMap["id"]="idx";
  }
  _envMap["ext"]=instance.extension();
  string str=_blobFileName;
  replaceEnvTags(str);

  //cerr << "Binary file path: " << str << endl;
  return str;
}


ostream* SerializationContext::getBinaryOutputStream(const string& fname) {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);

  //*** Boost dependent
  // if (!path(str).parent_path().empty()) {
  //   create_directories(path(str).parent_path());
  //}

  //****************************** bdc
  std::size_t found = str.find_last_of("/");
  string path = str.substr(0,found);
  DIR* dir = opendir(path.c_str());
  if(!dir)
    mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  closedir(dir);
  //****************************** bdc

  return new ofstream(str.c_str());
}

istream* SerializationContext::getBinaryInputStream(const string& fname) const {
  if (fname.empty()) {
    return 0;
  }
  string str=fname;
  adjustBinaryPath(str,_envMap);
  return new ifstream(str.c_str());
}

void SerializationContext::makeInputStream() {
  if (!_inputStream) {
    if (_inputDataFileName == "stdin"){
      _inputStream=&cin;
    } else
      _inputStream=new ifstream(_inputDataFileName.c_str());
  }
}

void  SerializationContext::makeOutputStream(){
  if (!_outputStream) {
    string str=_outputDataFileName;
    if (str=="stdout"){
      _outputStream = &cout;
    } else
      if (str=="stderr"){
        _outputStream = &cerr;
      } else {
        replaceEnvTags(str);

        //*** Boost dependent
        //      if (!path(str).parent_path().empty()) {
        //  create_directories(path(str).parent_path());
        //      }

        //****************************** bdc
        std::size_t found = str.find_last_of("/");
        string path = str.substr(0,found);
        DIR* dir = opendir(path.c_str());
        if(dir)
          mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        closedir(dir);
        //****************************** bdc

        _outputStream=new ofstream(str.c_str());
      }
  }
}

void SerializationContext::setInputStream(std::istream* is, const std::string& filename){
  destroyInputStream();
  if (is == &cin){
    _inputDataFileName = "stdin";
  } else if (_inputDataFileName!="")
    _inputDataFileName = filename;
  _inputStream = is;
}

void SerializationContext::setOutputStream(std::ostream* os, const std::string& filename){
  destroyOutputStream();
  if (os == &cout) {
    _outputDataFileName = "stdout";
  } else if (os == &cerr) {
    _outputDataFileName = "stderr";
  } else if (_outputDataFileName!="") {
    _outputDataFileName = filename;
  }
  _outputStream = os;
}

void SerializationContext::destroyInputStream() {
  if (_inputStream && _inputStream!=&cin) {
    delete _inputStream;
  }
  _inputStream=0;

}

void SerializationContext::destroyOutputStream() {
  if (_outputStream && _outputStream!=&cerr && _outputStream!=&cout) {
    delete _outputStream;
  }
  _outputStream = 0;
}

void SerializationContext::setInputFilePath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _inputDataFileName=fpath;
  destroyInputStream();
}

void SerializationContext::setOutputFilePath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _outputDataFileName=fpath;
  loadCurrentTime();
  //Extract directory
  size_t pos=_outputDataFileName.rfind('/');
  if (pos==string::npos) {
    _envMap["datadir"]=CURRENT_DIR;
  } else {
    _envMap["datadir"]=_outputDataFileName.substr(0,pos);
  }
  destroyOutputStream();
}

void SerializationContext::setBinaryPath(const string& fpath) {
  if (fpath.length()==0) {
    return;
  }
  _blobFileName=fpath;
}
