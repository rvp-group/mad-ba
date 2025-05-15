#include "configurable.h"
#include "configurable_shell.h"
#include "configurable_command.h"
#include <sstream>

namespace srrg2_core {

  Configurable::CommandBase::~CommandBase() {
  }


  Configurable::Configurable() {
    addCommand (new ConfigurableCommand_
                < Configurable, typeof(&Configurable::cmdHelp), std::string>
                (this,
                 "help",
                 "prints all available commands of this module",
                 &Configurable::cmdHelp));
    addCommand (new ConfigurableCommand_
                < Configurable, typeof(&Configurable::cmdReset), std::string>
                (this,
                 "reset",
                 "resets this module and all its descendants",
                 &Configurable::cmdReset));
  }

  void Configurable::reset() {
    std::cerr << className() << "| reset not implemented" << std::endl;
  }
  
  void Configurable::addCommand(CommandBase* cmd) {
    if (command(cmd->tag())) {
      std::string message;
      message += "command " + cmd->tag();
      message += "already in map";
      throw std::runtime_error(message.c_str());
    }
    _command_map.insert(std::make_pair(cmd->tag(), CommandBasePtr(cmd)));
  }

  Configurable::CommandBase* Configurable::command(const std::string& tag_) {
    auto it = _command_map.find(tag_);
    if (it != _command_map.end()) {
      return it->second.get();
    }
    return nullptr;
  }

  bool Configurable::handleCommand(ConfigurableShell* shell_,
                                   std::string& response,
                                   const std::vector<std::string>& tokens) {
    if (!tokens.size()) {
      response = "unknown empty command";
      return false;
    }
    CommandBase* cmd = command(tokens[0]);
    if (!cmd) {
      response = std::string("unknown command[") + tokens[0] + "]";
      return false;
    }
    return cmd->execute(shell_, response, tokens);
  }

  bool Configurable::cmdReset(std::string& response) {
    response="";
    reset();
    return true;
  }

  bool Configurable::cmdHelp(std::string& response) {
    std::ostringstream os;
    for (auto& it : _command_map) {
      os << "\t" << it.second->tag() << ": " << it.second->helpMessage() << std::endl;
    }
    response=os.str();
    return true;
  }

} // namespace srrg2_core
