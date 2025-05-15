#include "stream.h"
#include <stdexcept>
#include <fstream>

namespace srrg2_core {

  InputOutputStream::~InputOutputStream(){
  }
  
  std::string InputOutputStream::url() const { return "";}


  InputStream::InputStream() {
    _is = 0;
  }

  InputStream::~InputStream(){
    if (_is)
      delete _is;
  }

  std::istream& InputStream::stdStream() {
    if (! _is)
      throw std::runtime_error("Attempt to access an uninitialized input stream");
    return *_is;
  }

  OutputStream::OutputStream() {
  }
  
  OutputStream::~OutputStream() {
    if (_os)
      delete _os;
  }
  
  std::ostream& OutputStream::stdOutputStream(){
    if (! _os)
      throw std::runtime_error("Attempt to access an uninitialized output stream");
    return *_os;
  }


  FileInputStream::FileInputStream() {
    _fis = 0;
    _url="";
  }

  FileInputStream::~FileInputStream() {
    if (_fis)
      _fis->close();
  }

  void FileInputStream::open(const std::string& url_) {
    if (_fis) {
      delete _fis;
      _fis=0;
      _is=0;
    }
    _fis = new std::ifstream (url_.c_str());
    if (!_fis->good()){
      delete _fis;
      _fis=0;
      _is=0;
      _url="";
    } else {
      _url=url_;
      _is=_fis;
    }
  }

  std::string FileInputStream::url() const {
    return _url;
  }

  FileOutputStream::FileOutputStream() {
    _fos = 0;
    _url="";
  }

  FileOutputStream::FileOutputStream(const std::string& url_) {
    _fos = new std::ofstream (url_.c_str());
    if (!_fos->good()){
      delete _fos;
      _fos=0;
      _os=0;
      _url="";
    } else {
      _url=url_;
      OutputStream::_os=_fos;
    }
  }

  FileOutputStream::~FileOutputStream() {
    if (_fos)
      _fos->close();
  }

  void FileOutputStream::open(const std::string& url_) {
    if (_fos) {
      delete _fos;
      _fos=0;
      OutputStream::_os=0;
    }
    _fos = new std::ofstream (url_.c_str());
    if (!_fos->good()){
      delete _fos;
      _fos=0;
      _os=0;
      _url="";
    } else {
      _url=url_;
      OutputStream::_os=_fos;
    }
  }

  std::string FileOutputStream::url() const {
    return _url;
  }



}
