#pragma once
#include <srrg_config/configurable.h>
#include <srrg_config/configurable_manager.h>
#include "srrg_viewer/active_drawable.h"
#include "srrg_viewer/viewer_core_base.h"
#include "preemptible.h"

namespace srrg2_core {
  using StringVector = std::vector<std::string>;

  class ConfigurableShell: public PreemptibleController {
  public:
    ConfigurableShell(ConfigurableManager& manager_);
    virtual ~ConfigurableShell();
    struct CommandBase {
      CommandBase(ConfigurableShell* shell_,
                  const std::string& tag_,
                  const std::string& help_message_) :
        shell(shell_),
        tag(tag_),
        help_message(help_message_) {
      }

      virtual ~CommandBase() = default;

      ConfigurableShell* shell = nullptr;
      std::string tag;
      std::string help_message;
      virtual bool execute(const std::vector<std::string>& args) = 0;
      virtual void completions(StringVector& completions, const StringVector& args);
    };
    void run(const StringVector& lines = StringVector());
    static void registerAllInstances();
    static void runStatic(const StringVector& lines = StringVector());
    volatile bool _viewers_changed=false;
    
    using CommandBasePtr = std::unique_ptr<CommandBase>;
    using CommandMap     = std::map<std::string, CommandBasePtr>;
    // all commands that match prefix
    void confCommandCompletion(StringVector& completions, const std::string& prefix);

    // all types that match prefix
    void confTypesCompletion(StringVector& completions, const std::string& prefix);

    // all pointers/named objects that match prefix
    void confPtrCompletion(StringVector& completions, const std::string& prefix);

    // all pointers/named objects that match prefix
    void confFileCompletion(StringVector& completions,
                            const std::string& prefix,
                            bool is_dir  = true,
                            bool is_file = true);

    void confPreemptibleCompletion(StringVector& completions,
                                   const std::string& prefix);
    
    // all fields in c that match prefix
    void
    confFieldCompletion(StringVector& completions, const std::string& prefix, ConfigurablePtr c);

    // generates completions for an entire line
    void confLineCompletions(StringVector& completions, const char* line);

    ConfigurablePtr getConfigurable(const std::string& name);
    srrg2_core::ConfigurableManager& _manager;
    void addCommand(CommandBase* cmd);
    void removeCommand(const std::string& tag_);
    CommandMap _command_map;
    void parseLine(std::istream& is);
    void clear();

    bool _run = true;
    volatile bool _ready = false;

        // ia pointer to the canvas (to draw things)
    std::map<std::string, ViewerCanvasPtr> canvases;
    std::map<ViewerCanvasPtr, ConfigurablePtr> canvas_config_map;

    // ia RAW pointer to the viewer core (it's instanciated in the main so raw pointer is ok to
    // avoid wrong deletion ordering)
    ViewerCoreBase* viewer_core;

    //! @brief completion helper for canvases
    void canvasCompletion(StringVector& completions, const std::string& partial);

    void handleViewer();
    void stopViewer();
  };

} // namespace srrg2_core
