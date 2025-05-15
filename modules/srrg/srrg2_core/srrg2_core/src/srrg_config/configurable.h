#pragma once
#include "srrg_property/property_container.h"

#include "param_macros.h"

namespace srrg2_core {

  class ConfigurableShell;
  // module that can have a configuration
  class Configurable : public PropertyContainerIdentifiable {
  public:
    friend struct ConfigurableCommandHelp;

    struct CommandBase {
      CommandBase(Configurable* configurable_,
                  const std::string& tag_,
                  const std::string& help_message_) :
        _configurable(configurable_),
        _tag(tag_),
        _help_message(help_message_) {
      }

      virtual ~CommandBase();
      inline const std::string& tag() const {
        return _tag;
      }
      inline const std::string& helpMessage() const {
        return _help_message;
      }
      virtual bool execute(ConfigurableShell* shell_,
                           std::string& response,
                           const std::vector<std::string>& tokens) = 0;

    protected:
      Configurable* _configurable = nullptr;
      std::string _tag;
      std::string _help_message;
    };

    Configurable();
    void addCommand(CommandBase* cmd);

    CommandBase* command(const std::string& tag_);

    virtual bool handleCommand(ConfigurableShell* shell_,
                               std::string& response,
                               const std::vector<std::string>& tokens);

    virtual void reset();
    // shell interface for commands: requires static wrappers
    bool cmdReset(std::string& response);
    bool cmdHelp(std::string& response);
  protected:
    using CommandBasePtr = std::unique_ptr<CommandBase>;
    using CommandMap     = std::map<std::string, CommandBasePtr>;
    CommandMap _command_map;
  };

  using ConfigurablePtr = std::shared_ptr<Configurable>;

} // namespace srrg2_core
