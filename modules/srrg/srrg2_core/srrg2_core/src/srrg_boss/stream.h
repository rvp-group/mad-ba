# pragma once
#include <iostream>
#include <fstream>

namespace srrg2_core {

  class InputOutputStream {
  public:
    virtual ~InputOutputStream();
    virtual void open(const std::string& url) = 0;
    virtual std::string url() const;
    virtual std::istream& stdInputOutputStream() = 0;
  };

  class InputStream : public InputOutputStream {
  public:
    InputStream();
    InputStream(const std::string& url_);
    virtual ~InputStream();
    virtual  std::istream& stdStream() = 0;
  protected:
    std::istream* _is;
  };


  class OutputStream : public InputOutputStream  {
  public:
    OutputStream();
    ~OutputStream();
    virtual  std::ostream& stdOutputStream()= 0;
  protected:
    std::ostream* _os;
  };

  class FileInputStream: public InputStream {
  protected:
    std::ifstream* _fis;
    std::string _url;
  public:
    FileInputStream();
    FileInputStream(const std::string& url_);
    ~FileInputStream();
    void open(const std::string& url) = 0;
    virtual std::string url() const;
  };

  class FileOutputStream: public OutputStream {
  protected:
    std::ofstream* _fos;
    std::string _url;
  public:
    FileOutputStream();
    FileOutputStream(const std::string& url_);
    ~FileOutputStream();
    void open(const std::string& url) = 0;
    virtual std::string url() const;
  };

}
