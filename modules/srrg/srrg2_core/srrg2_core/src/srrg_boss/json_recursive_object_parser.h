#include <iostream>
#include <string>
#include <regex>
#include <vector>
#include <fstream>
#include <list>
#include "object_data.h"
#include "object_parser.h"

namespace srrg2_core {
  
  class JSONRegexLexer {
  public:
    struct Token{
      enum Type {
	EndOfFile=-1,
	Unknown=0x0,
	SquareOpenBracket=0x1,
	SquareClosedBracket=0x2,
	CurlyOpenBracket=0x3,
	CurlyClosedBracket=0x4,
	Comma=0x5,
	Colon=0x6,
	Boolean=0x7,
	String=0x8,
	Number=0x9,
        Comment=0x10
      };

    
      Token(Type type_=Unknown, const std::string& string_value="")
	:_type(type_), _string_value(string_value){}
    
      inline Type type() const {return _type;} 
      std::string getString() const;    
      int getInt() const;
      float getFloat() const;
      double getDouble() const;
      bool getBool() const;
      
      Type _type;
      std::string _string_value;
    };
  
  public:
    JSONRegexLexer();
    void setLine(const std::string& line);
    void setStream(std::istream& stream);
    inline bool hasStream() const {return _istream && _istream->good();}
    Token nextToken();
    bool hasTokens() const { return !_tokens_put_back.empty() || _words_begin!=_words_end;}
    void putBack(const Token& token);
  
  protected:
    
    std::vector<std::pair<std::string, Token::Type> > _grammar;
    std::string _regular_expression;
    std::regex _regex_lexer;
    std::string _line;
    std::sregex_iterator _words_begin, _words_end;
    std::list<Token> _tokens_put_back;
    std::istream* _istream;
  };

  struct JSONRecursiveParser{

    JSONRecursiveParser(JSONRegexLexer* lexer_);
    JSONRegexLexer* _lexer;
    using Token=JSONRegexLexer::Token;
    ObjectData* parseLine(std::string& class_name);
    ObjectData* parseStruct();
    ValueData* parseFieldPair(std::string& field_name);
    ValueData* parseFieldValue();
    ArrayData* parseArray();
  };

  class JSONRecursiveObjectParser: virtual public ObjectParser {
  public:
    JSONRecursiveObjectParser();
    virtual ObjectData* readObject(std::istream& is, std::string& type);
    virtual ObjectData* readObject(const std::string& s, std::string& type);
  protected:
    JSONRegexLexer _lexer;
    JSONRecursiveParser _parser;  
  };

}
