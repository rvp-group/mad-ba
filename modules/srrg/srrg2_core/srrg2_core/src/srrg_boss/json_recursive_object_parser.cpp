#include <string>
#include <regex>
#include <iterator>
#include <vector>
#include <iomanip>
#include <stdexcept>
#include <limits>
#include "json_recursive_object_parser.h"
#include <cstdio>

namespace srrg2_core {

  using namespace std;

  static const int LINE_LENGTH=65535;

  std::string JSONRegexLexer::Token::getString() const {
    if (_type!=String)
      throw std::runtime_error("attempt to get a string object from a non-string token");
    return _string_value.substr(1,_string_value.length()-2);
  }
    
  int JSONRegexLexer::Token::getInt() const {
    if (_type!=Number)
      throw std::runtime_error("attempt to get an int object from a non-number token");
    return atoi(_string_value.c_str());
  }

  float JSONRegexLexer::Token::getFloat() const {
    if (_type!=Number)
      throw std::runtime_error("attempt to get a float object from a non-number token");
    return (float) atof(_string_value.c_str());
  }

  double JSONRegexLexer::Token::getDouble() const {
    if (_type!=Number)
      throw std::runtime_error("attempt to get a double object from a non-number token");
    return atof(_string_value.c_str());
  }

  bool JSONRegexLexer::Token::getBool() const {
    if (_type!=Boolean)
      throw std::runtime_error("attempt to get a bool object from a non-bool token");
    return _string_value=="true";
  }

  
  JSONRegexLexer::JSONRegexLexer(){
    _grammar.push_back(std::make_pair("//.*", Token::Comment));
    _grammar.push_back(std::make_pair("\"[^\"]*\"", Token::String));
    _grammar.push_back(std::make_pair("\\[", Token::SquareOpenBracket));
    _grammar.push_back(std::make_pair("\\]", Token::SquareClosedBracket));
    _grammar.push_back(std::make_pair("\\{", Token::CurlyOpenBracket));
    _grammar.push_back(std::make_pair("\\}", Token::CurlyClosedBracket));
    _grammar.push_back(std::make_pair("\\:", Token::Colon));
    _grammar.push_back(std::make_pair("\\,", Token::Comma));
    _grammar.push_back(std::make_pair("([+-]?)[0-9]+([.][0-9]*)?([eE][+-]?[0-9]+)?",Token::Number));
    _grammar.push_back(std::make_pair("(true|false)", Token::Boolean));
    for(auto const& x : _grammar)
      _regular_expression += "(" + x.first + ")|"; // parenthesize the submatches
    _regular_expression.pop_back();
    _regex_lexer=std::regex(_regular_expression);
    _istream=0;
  }

  void JSONRegexLexer::setLine(const std::string& line){
    _line=line;
    _words_begin = std::sregex_iterator(_line.begin(), _line.end(), _regex_lexer);
    _words_end = std::sregex_iterator();
  }

  void JSONRegexLexer::setStream(std::istream& stream){
    _istream=&stream;
  }
  
  JSONRegexLexer::Token JSONRegexLexer::nextToken(){

    // no tokens we read a line of the file
    while (! hasTokens() && _tokens_put_back.empty()) {
      if (!_istream)
	throw std::runtime_error("no tokens in lexer");

      if(!_istream->good())
	return Token(Token::EndOfFile);
      
      char buf[LINE_LENGTH];
      _istream->getline(buf, LINE_LENGTH);
      setLine(std::string(buf));

      // we populate the list of tokens
      while(_words_begin!=_words_end) {
        size_t index=0;
        for( ; index < _words_begin->size(); ++index) {
          if(!_words_begin->str(index + 1).empty()) { // determine which submatch was matched
            Token token(_grammar[index].second, _words_begin->str());
            if (token.type()!=Token::Comment)
              _tokens_put_back.push_back(Token(_grammar[index].second, _words_begin->str()));
            break;
          }
        }
        _words_begin++;
      }
    }
    if (!_tokens_put_back.empty()){
      auto token = _tokens_put_back.front();
      _tokens_put_back.pop_front();
      return token;
    }
    return Token(Token::EndOfFile);
  }

  void JSONRegexLexer::putBack(const Token& token){
    _tokens_put_back.push_front(token);
  }


  
  JSONRecursiveParser::JSONRecursiveParser(JSONRegexLexer* lexer_): _lexer(lexer_){}
  

  ObjectData* JSONRecursiveParser::parseLine(std::string& class_name) {
    //cerr << __PRETTY_FUNCTION__ << endl;
    if (! _lexer->hasTokens() && !_lexer->hasStream())
      return nullptr;
    Token token=_lexer->nextToken();
    if (token.type()!=Token::String){
      return nullptr;
    }
    class_name=token.getString();
    token = _lexer->nextToken();
    if (token.type() == Token::CurlyOpenBracket) {
      _lexer->putBack(token);
    } else if ( ! (token.type() == Token::Colon) ) {
      return nullptr;
    }
    return parseStruct();
  }

  ObjectData* JSONRecursiveParser::parseStruct(){
    //cerr << __PRETTY_FUNCTION__ << endl;
    Token token=_lexer->nextToken();
    if (token.type()==Token::Colon) {
      token = _lexer->nextToken();
    }
    if (token.type()!=Token::CurlyOpenBracket){
      return nullptr;
    }
    ObjectData* odata = new ObjectData;
    ValueData* field_value=nullptr;
    do {
      std::string field_name;
      field_value=parseFieldPair(field_name);
      if (field_value){
	odata->setField(field_name, field_value); 
	token=_lexer->nextToken();
	if (token.type()==Token::CurlyClosedBracket)
	  return odata;
	if (token.type()!=Token::Comma) {
	  delete odata;
	  return nullptr; // parse error
	}
      }
    }  while(field_value);
    return nullptr;
  }

  ValueData* JSONRecursiveParser::parseFieldPair(std::string& field_name) {
    Token token=_lexer->nextToken();
    if (token.type()==Token::CurlyOpenBracket){
      _lexer->putBack(token);
      return nullptr;
    }
    if (token.type()==Token::String){
      field_name=token.getString();
    }
    token=_lexer->nextToken();
    if (token.type()!=Token::Colon){
      return nullptr;
    }
    return parseFieldValue();
  }

  ValueData* JSONRecursiveParser::parseFieldValue() {
    //cerr << __PRETTY_FUNCTION__ << endl;
    Token token=_lexer->nextToken();
    switch(token.type()){
    case JSONRegexLexer::Token::Boolean:
      return new BoolData(token.getBool());
    case JSONRegexLexer::Token::String:
      return new StringData(token.getString());
    case JSONRegexLexer::Token::Number:
      return new NumberData(token.getDouble());
    case JSONRegexLexer::Token::CurlyOpenBracket:
      _lexer->putBack(token);
      return parseStruct();
    case JSONRegexLexer::Token::SquareOpenBracket:
      _lexer->putBack(token);
      return parseArray();  
    default:
      _lexer->putBack(token);
      return nullptr;
    }
  }

  ArrayData* JSONRecursiveParser::parseArray(){
    //cerr << __PRETTY_FUNCTION__ << endl;
    Token token=_lexer->nextToken();
    if (token.type()!=Token::SquareOpenBracket){
      return nullptr;
    }
    ArrayData* adata=new ArrayData;
    ValueData* value=nullptr;
    do {
      value=parseFieldValue();
      if (! value ) {
	token=_lexer->nextToken();
        if (token.type()==Token::SquareClosedBracket)
	  return adata;
	if (token.type()!=Token::Comma)
	  return nullptr;
      } else {
        adata->add(value);
      } 
    }  while(1);
    return 0;
  }


  JSONRecursiveObjectParser::JSONRecursiveObjectParser():
    _parser(&_lexer){}
  
  ObjectData* JSONRecursiveObjectParser::readObject(std::istream& is, std::string& type){
    using namespace std;
//    cerr << __PRETTY_FUNCTION__ << endl;
    _lexer.setStream(is);
    return _parser.parseLine(type);
  }

  ObjectData* JSONRecursiveObjectParser::readObject(const std::string& s, std::string& type){
    _lexer.setLine(s);
    return _parser.parseLine(type);
  }

}
