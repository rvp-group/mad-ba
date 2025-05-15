#include <csignal>
#include <dirent.h>
#include <dlfcn.h>
#include <gnu/lib-names.h>
#include <iomanip>
#include <sstream>
#include <sys/types.h>
#include <ucontext.h>
#include <unistd.h>

#include "srrg_boss/json_object_writer.h"
#include "srrg_config/property_configurable_vector.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_system_utils/env.h"

#include "srrg_messages/message_handlers/message_file_source.h"
#include "srrg_messages/message_handlers/message_filter_base.h"
#include "srrg_messages/message_handlers/message_sink_base.h"
#include "srrg_messages/message_handlers/message_source_platform.h"

#include "configurable_shell.h"
#include "linenoise.h"

namespace srrg2_core {
  using namespace std;
    
  struct CommandHelp : public ConfigurableShell::CommandBase {
    CommandHelp(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "help", "lists available commands") {
    }
    virtual ~CommandHelp() {
    }
    bool execute(const std::vector<std::string>& args) override {
      std::cerr << FG_BCYAN("COMMANDS") << std::endl;

      for (auto& it : shell->_command_map) {
        std::cerr << std::left << std::setw(6) << " " << CYAN << std::setw(18) << it.first << RESET
                  << it.second->help_message << std::endl;
      }
      return true;
    }
  };

  struct CommandEcho : public ConfigurableShell::CommandBase {
    CommandEcho(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "echo", "prints something on the terminal") {}
    bool execute(const std::vector<std::string>& args) override {
      cerr << left;
      for (const auto& s: args)
        cerr << s << " ";
      cerr << endl;
      return true;
    }
  };

  struct CommandStepMode : public ConfigurableShell::CommandBase {
    CommandStepMode(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "step_mode", "sets/queries the step mode") {
    }
    virtual ~CommandStepMode() = default;
    bool execute(const std::vector<std::string>& args) override {
      if (args.size()>0) {
        shell->setStepMode(atoi(args[0].c_str()));
      }
      cerr << RESET << "STEP_MODE = " << shell->stepMode() << endl;
      return true;
    }
  };

  struct CommandLs : public ConfigurableShell::CommandBase {
    CommandLs(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "ls", "lists the files in the current directory") {
    }
    virtual ~CommandLs() {
    }
    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0], true, false);
    }

    bool execute(const std::vector<std::string>& args) override {
      StringVector paths;
      std::string prefix = "";
      if (args.size() > 0)
        prefix = args[0];
      shell->confFileCompletion(paths, prefix);
      for (const auto& it : paths) {
        std::cerr << it << std::endl;
      }
      return true;
    }
  };

  struct CommandPwd : public ConfigurableShell::CommandBase {
    CommandPwd(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "pwd", "prints working directory") {
    }
    virtual ~CommandPwd() {
    }

    bool execute(const std::vector<std::string>& args) override {
      std::cerr << "current_dir:[" << getcwd(0, 0) << "]" << std::endl;
      return true;
    }
  };

  struct CommandCd : public ConfigurableShell::CommandBase {
    CommandCd(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "cd", "changes the working directory") {
    }
    virtual ~CommandCd() {
    }
    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0], true, false);
    }

    bool execute(const std::vector<std::string>& args) override {
      StringVector paths;
      std::string prefix = "";
      if (args.size() > 0)
        prefix = args[0];
      if (chdir(prefix.c_str())) {
        std::cerr << "error in changing directory" << std::endl;
        return false;
      }
      std::cerr << "current dir is [" << getcwd(0, 0) << "]" << std::endl;
      return true;
    }
  };

  struct CommandDLConfig : public ConfigurableShell::CommandBase {
    CommandDLConfig(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(
                                     shell_,
                                     "dl_config",
                                     "dl_config <filename>, dynamically loads a library from a stub file") {
    }

    virtual ~CommandDLConfig() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening library [" << FG_YELLOW(filepath) << "]" << std::endl;
      ConfigurableManager::initFactory(filepath);
      return true;
    }
  };

  struct CommandDLOpen : public ConfigurableShell::CommandBase {
    CommandDLOpen(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_,
                                     "dlopen",
                                     "dlopen <library filename>, dynamically loads a library") {
    }
    virtual ~CommandDLOpen() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening library [" << FG_YELLOW(filepath) << "]" << std::endl;
      void* handle = dlopen(filepath.c_str(), RTLD_LAZY);
      if (!handle) {
        std::cerr << FG_RED(*dlerror()) << std::endl;
        return false;
      }
      ConfigurableManager::initFactory();
      return true;
    }
  };

  struct CommandOpen : public ConfigurableShell::CommandBase {
    CommandOpen(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "open", "open <filename>, opens a file") {
    }
    virtual ~CommandOpen() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confFileCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::string filepath(args[0]);
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '\''), filepath.end());
      filepath.erase(std::remove(filepath.begin(), filepath.end(), '"'), filepath.end());
      std::cerr << "opening file [" << FG_YELLOW(filepath) << "]" << std::endl;
      shell->_manager.read(filepath);
      return true;
    }
  };

  struct CommandNames : public ConfigurableShell::CommandBase {
    CommandNames(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "names", "lists the named objects in the manager") {
    }
    virtual ~CommandNames() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 0) {
        return false;
      }
      std::cerr << "named objects: " << std::endl;
      for (auto& map_it : shell->_manager.namedInstances()) {
        ConfigurablePtr c = std::dynamic_pointer_cast<Configurable>(map_it.second);
        if (!c) {
          continue;
        }
        if (args.size() == 1 && args[0] != c->className()) {
          continue;
        }

        std::stringstream ss;
        ss << c.get();

        std::cerr << "ptr: " << ULCYAN << ss.str() << RESET << " class: " << std::left
                  << std::setw(70) << c->className() << " name: " << ULCYAN << c->name() << RESET
                  << std::endl;
      }
      return true;
    }
  };

  struct CommandWrite : public ConfigurableShell::CommandBase {
    CommandWrite(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "write", "<filename> <bool>, writes a conf file, if bool is 0 the comments are not written") {
    }
    virtual ~CommandWrite() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      bool enable_comments;
      if (args.size()==2)
        enable_comments=atoi(args[1].c_str());
      std::cerr << "writing file [" << FG_YELLOW(args[0]) << "], comments: " << enable_comments << std::endl;
      shell->_manager.write(args[0], enable_comments);
      return true;
    }
  };

  struct CommandClear : public ConfigurableShell::CommandBase {
    CommandClear(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "clear", "deletes everything") {
    }
    virtual ~CommandClear() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 0) {
        return false;
      }
      shell->clear();
      return true;
    }
  };

  struct CommandQuit : public ConfigurableShell::CommandBase {
    CommandQuit(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "quit", "terminares shell") {
    }
    virtual ~CommandQuit() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }
      std::cerr << FG_RED("quitting...") << std::endl;
      shell->_run = false;
      return true;
    }
  };

  struct CommandSleep : public ConfigurableShell::CommandBase {
    CommandSleep(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "sleep", "sleeps for x seconds") {
    }
    virtual ~CommandSleep() {
    }

    bool execute(const std::vector<std::string>& args) override {
      int duration=0;
      if (args.size()>0) {
        duration=atoi(args[0].c_str());
      }
      cerr << "Sleeping for " << duration << "milliseconds " << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
      return true;
    }
  };
    
  struct CommandInstances : public ConfigurableShell::CommandBase {
    CommandInstances(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_,
                                     "instances",
                                     "[<type prefix>] shows the available instances") {
    }
    virtual ~CommandInstances() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 1) {
        return false;
      }
      std::cerr << "available instances: " << std::endl;
      for (IdentifiablePtr id : shell->_manager.instances()) {
        ConfigurablePtr c = std::dynamic_pointer_cast<Configurable>(id);
        if (!c) {
          continue;
        }
        if (args.size() == 1 && args[0] != c->className()) {
          continue;
        }

        std::stringstream ss;
        ss << c.get();

        std::cerr << "ptr: " << ULCYAN << ss.str() << RESET << " class: " << std::left
                  << std::setw(70) << c->className() << " name: " << ULCYAN << c->name() << RESET
                  << std::endl;
      }
      return true;
    }
  };

  struct CommandTypes : public ConfigurableShell::CommandBase {
    CommandTypes(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "types", "[<type prefix>] lists available types") {
    }
    virtual ~CommandTypes() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 1) {
        return false;
      }

      std::string prefix;
      if (args.size()) {
        prefix = args[0];
      }

      StringVector completions;
      shell->confTypesCompletion(completions, prefix);

      std::cerr << "available types starting with [" << prefix << "]: " << std::endl;
      for (auto& s : completions) {
        std::cerr << s << std::endl;
      }
      return true;
    }
  };

  struct CommandSetName : public ConfigurableShell::CommandBase {
    CommandSetName(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_,
                               "set_name",
                               "<name/ptr> <new_name>,  sets the name of an object ") {
    }
    virtual ~CommandSetName() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 1) {
        shell->confPtrCompletion(completions, args[0]);
        return;
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "object [" << args[0] << "] unknown, cannot set name" << std::endl;
        return false;
      }
      if (args.size() > 1) {
        auto it = shell->_manager.namedInstances().find(args[1]);
        if (it != shell->_manager.namedInstances().end() && it->second != conf) {
          std::cerr << "cannot rename, object with the same name exists : " << it->second
                    << std::endl;
        }

        shell->_manager.rename(conf, args[1]);
        std::cerr << "renamed object " << conf << " new name: [" << conf->name() << "]"
                  << std::endl;
      } else {
        shell->_manager.rename(conf, "");
        std::cerr << "erased name for object " << conf << std::endl;
      }
      return true;
    }
  };

  struct CommandShow : public ConfigurableShell::CommandBase {
    CommandShow(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "show", "<name/ptr>,  shows the fields of an object ") {
    }
    virtual ~CommandShow() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() != 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "object [" << args[0] << "] unknown, cannot show" << std::endl;
        return false;
      }
      IdContext id_context;
      ObjectData odata;
      odata.setPointer("pointer", conf.get());
      conf->serialize(odata, id_context);
      JSONObjectWriter writer;
      writer.writeObject(std::cerr, conf->className(), odata);
      return true;
    }
  };

  struct CommandCreate : public ConfigurableShell::CommandBase {
    CommandCreate(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "create", "<class_name>,  creates a new instance") {
    }
    virtual ~CommandCreate() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confTypesCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      std::vector<std::string> types = ConfigurableManager::listTypes();
      bool found                     = std::binary_search(types.begin(), types.end(), args[0]);
      if (!found) {
        std::cerr << "unknown type [" << args[0] << "] unknown, cannot create" << std::endl;
        return false;
      }
      IdentifiablePtr ptr = shell->_manager.create(args[0]);
      std::cerr << "created object of type " << args[0] << "ptr: " << ptr << std::endl;
      return true;
    }
  };

  struct CommandErase : public ConfigurableShell::CommandBase {
    CommandErase(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "erase", "<class_name/ptr>, erases an instance") {
    }
    virtual ~CommandErase() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      for (const std::string& s : args) {
        ConfigurablePtr module = shell->getConfigurable(s);
        if (!module) {
          std::cerr << "object [" << FG_CYAN(s) << "] unknown, cannot erase" << std::endl;
          return false;
        }
        shell->_manager.erase(module);
        std::cerr << "destroyed object " << FG_CYAN(s) << std::endl;
      }
      return true;
    }
  };

  struct CommandExec : public ConfigurableShell::CommandBase {
    CommandExec(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_,
                               "exec",
                               "<class_name/ptr> [<args>], executes a module a command") {
    }
    virtual ~CommandExec() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() > 1) {
        return;
      }
      shell->confPtrCompletion(completions, args[0]);
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 1) {
        return false;
      }
      ConfigurablePtr module=nullptr;
      for (const std::string& s : args) {
        if (! module) {
          module = shell->getConfigurable(s);
        }
        if (!module) {
          std::cerr << "object [" << FG_CYAN(s) << "] unknown, cannot exec" << std::endl;
          return false;
        }
        std::string response;
        std::vector<std::string> tokens;
        for (size_t i = 1; i < args.size(); ++i) {
          tokens.push_back(args[i]);
        }
        if (! tokens.size()) {
          response = "no command specified for object" + args[0];
          return false;
        }
        std::cerr <<  "exec| class: " << module->className()
                  << " instance: " << args[0]
                  << " command:" << tokens[0] << std::endl;
        bool ok = module->handleCommand(shell, response, tokens);
        std::cerr << response << std::endl;
        if (ok) {
          return true;
        }
      }
      return true;
    }
  };

  struct CommandSet : public ConfigurableShell::CommandBase {
    CommandSet(ConfigurableShell* shell_) :
      CommandBase::CommandBase(shell_, "set", "<class_name>, sets a field in a config") {
    }
    virtual ~CommandSet() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() < 1) {
        return;
      }
      if (args.size() == 1) {
        shell->confPtrCompletion(completions, args[0]);
        return;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        return;
      }

      if (args.size() == 2) {
        shell->confFieldCompletion(completions, args[1], conf);
        return;
      }
      auto prop_it = conf->properties().find(args[1]);
      if (prop_it == conf->properties().end()) {
        return;
      }
      PropertyBase* prop = prop_it->second;

      if (!prop) {
        return;
      }

      PropertyIdentifiablePtrInterfaceBase* pc =
        dynamic_cast<PropertyIdentifiablePtrInterfaceBase*>(prop);
      if (!pc) {
        return;
      }
      const std::string& last_arg = args[args.size() - 1];

      StringVector candidates;
      shell->confPtrCompletion(candidates, last_arg);
      // we check the class name of each candidate, and if not valid we erase it
      for (const std::string& s : candidates) {
        ConfigurablePtr conf = shell->getConfigurable(s);
        if (pc->canAssign(conf)) {
          completions.push_back(s);
        }
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() < 3) {
        return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[0]);
      if (!conf) {
        std::cerr << "target object [" << args[0] << "] unknown, cannot set" << std::endl;
        return false;
      }
      auto prop_it = conf->properties().find(args[1]);
      if (prop_it == conf->properties().end()) {
        std::cerr << "class [" << conf->className() << "] does not have field [" << args[1] << "]"
                  << std::endl;
        return false;
      }
      PropertyIdentifiablePtrVectorInterface* pcv =
        dynamic_cast<PropertyIdentifiablePtrVectorInterface*>(prop_it->second);
      if (pcv) {
        std::vector<ConfigurablePtr> src_modules;
        for (size_t i = 2; i < args.size(); ++i) {
          ConfigurablePtr src_module = shell->getConfigurable(args[i]);
          if (src_module && !pcv->canAssign(src_module)) {
            std::cerr << "incompatible objects" << std::endl;
            return false;
          }
          src_modules.push_back(src_module);
        }
        pcv->assign(src_modules);
        return true;
      }

      PropertyIdentifiablePtrInterface* pc =
        dynamic_cast<PropertyIdentifiablePtrInterface*>(prop_it->second);
      if (pc) {
        if (args[2] == "0") {
          ConfigurablePtr null_module;
          std::cerr << "set successful" << std::endl;
          pc->assign(null_module);
          return true;
        }
        ConfigurablePtr val = shell->getConfigurable(args[2]);
        if (!val) {
          std::cerr << "src object [" << args[2] << "] unknown, cannot set" << std::endl;
          return false;
        }
        if (!pc->assign(val)) {
          std::cerr << "incompatible objects" << std::endl;
          return false;
        }
        std::cerr << "set successful" << std::endl;
        return true;
      }
      std::vector<std::string> tokens;
      for (size_t i = 2; i < args.size(); ++i)
        tokens.push_back(args[i]);
      bool result = prop_it->second->fromTokens(tokens);
      if (!result) {
        std::cerr << "parse error on field" << std::endl;
        return false;
      }
      std::cerr << "set successful" << std::endl;
      return true;
    }
  };

      
  struct CommandRun : public ConfigurableShell::CommandBase {
    CommandRun(ConfigurableShell* shell_):
      ConfigurableShell::CommandBase(shell_, "run", "starts a task") {
    }


    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();

      if (args.size() == 1) {
        shell->confPreemptibleCompletion(completions, args[0]);
        return;
      }
      return;
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      ConfigurablePtr c=shell->getConfigurable(args[0]);
      PreemptiblePtr pipeline = dynamic_pointer_cast<Preemptible>(c);
      if (!pipeline) {
        std::cerr << "pipeline [" << args[0] << "] unknown, cannot run" << std::endl;
        return false;
      }
      auto current_runner=shell->getRunner();
      if (current_runner && current_runner->status()==Preemptible::Running) {
        std::cerr << "already running" << std::endl;
        std::cerr << "call [kill] to suppress execution" << std::endl;
        return false;
      }
      shell->PreemptibleController::run(pipeline);
      return true;

    }
  };


  struct CommandKill : public ConfigurableShell::CommandBase {
    CommandKill(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "kill", "kills a running session") {
    }
    virtual ~CommandKill() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }

      auto runner=shell->getRunner();
      if (!shell->getRunner()) {
        std::cerr << "no process active, cannot kill" << std::endl;
        return false;
      }
      std::cerr << "waiting for proper termination" << std::endl;
      shell->terminate();
      runner->reset();
      shell->setStepMode(false);
      return true;
    }
  };

  struct CommandForeground : public ConfigurableShell::CommandBase {
    CommandForeground(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "fg", "resumes a running process") {
    }
    virtual ~CommandForeground() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size()) {
        return false;
      }

      auto runner=shell->getRunner();
      if (!runner) {
        std::cerr << "no process active, cannot kill" << std::endl;
        return false;
      }
      shell->setStepMode(false);
      shell->resume();
      return true;
    }
  };

  static void completionCallback(const char* line, linenoiseCompletions* completions) {
    auto& instance = PreemptibleController::instance();
    if (!instance)
      return;
    PreemptibleController* controller=instance.get();
    ConfigurableShell* shell=dynamic_cast<ConfigurableShell*>(controller);
    StringVector comp;
    shell->confLineCompletions(comp, line);

    for (const std::string& s : comp) {
      linenoiseAddCompletion(completions, s.c_str());
    }
  }

  void ConfigurableShell::clear() {
    std::cerr << "ConfigurableShell::clear|destroying all objects" << std::endl;
    const size_t total_objects = _manager.instances().size();
    size_t k                   = 0;
    while (!_manager.instances().empty()) {
      PropertyContainerIdentifiablePtr c = *_manager.instances().begin();
      if (c) {
        std::cerr << "\tdestroyed object ptr: " << FG_ULRED(c.get()) << std::endl;
        _manager.erase(c);
        ++k;
      }
    }
    std::cerr << "ConfigurableShell::clear|destroyed [" << RED << k << "/" << total_objects << RESET
              << "] objects" << std::endl;
  }

  void ConfigurableShell::CommandBase::completions(StringVector& completions,
                                                   const StringVector& args) {
    completions.clear();
  }

  struct CommandListCanvases : public ConfigurableShell::CommandBase {
    CommandListCanvases(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "ls_canvases", "lists all canvases") {
    }
    virtual ~CommandListCanvases() {
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() > 0) {
        return false;
      }
      if (!shell){
          std::cerr << "no visual shell " << std::endl;
          return false;
      }
      std::cerr << "available canvases: " << std::endl;
      
      for (const auto& c : shell->canvases) {
        auto cd_it = shell->canvas_config_map.find(c.second);
        if (cd_it != shell->canvas_config_map.end()) {
          std::stringstream ss, ss_conf;
          ss << c.second.get();
          ss_conf << cd_it->second.get();
          auto configurable       = shell->getConfigurable(ss_conf.str());
          std::string config_disp = "";
          if (configurable && configurable->name().length()) {
            config_disp = configurable->name();
          }
          std::cerr << std::left << "canvas : " << ULCYAN << ss.str() << RESET << " " << YELLOW
                    << std::setw(16) << c.first << RESET << " module: " << ULCYAN << ss_conf.str()
                    << RESET << " " << YELLOW << config_disp << RESET << std::endl;
        } else {
          std::cerr << "no module paired with canvas " << c.first << std::endl;
          return false;
        }
      }
      return true;
    }
  };

  struct CommandRemoveCanvas : public ConfigurableShell::CommandBase {
    CommandRemoveCanvas(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_, "rm_canvas", "<canvas_name>, remove a canvas") {
    }
    virtual ~CommandRemoveCanvas() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 1) {
        if (auto s = dynamic_cast<ConfigurableShell*>(shell)) {
          s->canvasCompletion(completions, args[0]);
        }
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 1) {
        return false;
      }
      if (!shell){
          std::cerr << "no visual shell " << std::endl;
          return false;
      }
      auto c_it = shell->canvases.find(args[0]);
      if (c_it == shell->canvases.end()) {
        std::cerr << "no canvas named [" << c_it->second << "]" << endl;
        return false;
      }
      auto cd_it = shell->canvas_config_map.find(c_it->second);
      if (cd_it == shell->canvas_config_map.end()) {
        std::cerr << "no module paired with canvas named [" << c_it->second << "]" << endl;
        return false;
      }
      //shell->viewer_core->detachCanvas(args[0]);
      shell->canvas_config_map.erase(cd_it);
      shell->canvases.erase(c_it);
      shell->_viewers_changed=true;
      return true;
    }
  };

  struct CommandAddCanvas : public ConfigurableShell::CommandBase {
    CommandAddCanvas(ConfigurableShell* shell_) :
      ConfigurableShell::CommandBase(shell_,
                                     "add_canvas",
                                     "<canvas_name> <name/ptr>, add a new canvas") {
    }

    virtual ~CommandAddCanvas() {
    }

    void completions(StringVector& completions, const StringVector& args) override {
      completions.clear();
      if (args.size() == 2) {
        shell->confPtrCompletion(completions, args[1]);
      }
    }

    bool execute(const std::vector<std::string>& args) override {
      if (args.size() != 2) {
        return false;
      }
      ConfigurableShell* shell=dynamic_cast<ConfigurableShell*>(this->shell);
      if (!shell){
          std::cerr << "no visual shell " << std::endl;
          return false;
      }
      ConfigurablePtr conf = shell->getConfigurable(args[1]);
      if (!conf) {
        std::cerr << "object [" << ULCYAN << args[1] << RESET << "] unknown, cannot set the canvas"
                  << std::endl;
        return false;
      }

      // srrg check if is drawable
      ActiveDrawablePtr drawable = std::dynamic_pointer_cast<ActiveDrawable>(conf);
      if (!drawable) {
        std::cerr << "trying to put a canvas to a non-drawable object" << std::endl;
        return false;
      }

      srrg2_core::ViewerCanvasPtr canvas =
        shell->viewer_core->getCanvas(args[0]);

      std::cerr << "adding new canvas " << YELLOW << args[0] << RESET << " (" << ULCYAN << canvas
                << RESET << ") for module " << ULCYAN << args[1] << RESET << std::endl;

      auto ret = shell->canvases.insert(std::make_pair(args[0], canvas));
      if (ret.second == false) {
        std::cerr << "canvas with name " << YELLOW << args[0] << RESET
                  << " already exists, aborting" << std::endl;
        return false;
      }

      auto ret2 = shell->canvas_config_map.insert(std::make_pair(ret.first->second, conf));
      if (ret2.second == false) {
        std::cerr << "module " << ULCYAN << args[1] << RESET << " has already a canvas with name"
                  << ret2.first->first->name() << std::endl;
        return false;
      }
      drawable->addCanvas(canvas);
      shell->_viewers_changed=true;
      return true;
    }
  };

  void ConfigurableShell::run(const StringVector& lines) {
    if (lines.size()) {
      std::cerr << "ConfigurableShell::run|shell is in batch_mode" << std::endl;
    }

    for (const std::string& s : lines) {
      std::istringstream is(s);
      parseLine(is);
      handleViewer();
    }

    std::cerr << "ConfigurableShell::run|shell is in interactive_mode" << std::endl;
    _ready=true;
    while (_run) {
      char* line = linenoise("SRRG> ");
      if (!line) {
        continue;
      }
      std::istringstream is(line);
      parseLine(is);
      if (line[0]) {
        linenoiseHistoryAdd(line);
      }
      free(line);
      handleViewer();
    }
    stopViewer();
  }

  void ConfigurableShell::addCommand(CommandBase* cmd) {
    // srrg remove warning
    //    const auto& ret =
    _command_map.insert(std::make_pair(cmd->tag, std::unique_ptr<CommandBase>(cmd)));
  }

  void ConfigurableShell::removeCommand(const std::string& tag_) {
    auto ret = _command_map.find(tag_);
    if (ret != _command_map.end()) {
      _command_map.erase(ret);
    }
  }

  ConfigurablePtr ConfigurableShell::getConfigurable(const std::string& name) {
    // first we seek in the manager named map
    auto map_it = _manager.namedInstances().find(name);
    if (map_it != _manager.namedInstances().end()) {
      return std::dynamic_pointer_cast<Configurable>(map_it->second);
    }
    void* ptr_val;
    std::istringstream is(name.c_str());

    is >> ptr_val;

    for (IdentifiablePtr p : _manager.instances()) {
      if (p.get() == ptr_val) {
        return std::dynamic_pointer_cast<Configurable>(p);
      }
    }
    return 0;
  }

  void ConfigurableShell::confPtrCompletion(StringVector& completions, const std::string& partial) {
    std::ostringstream os;
    for (PropertyContainerIdentifiablePtr c : _manager.instances()) {
      std::ostringstream os;
      os << c;
      std::string ptr_name = os.str();
      auto res             = std::mismatch(partial.begin(), partial.end(), ptr_name.begin());
      if (res.first == partial.end()) {
        completions.push_back(ptr_name);
      }
      if (!c->name().length()) {
        continue;
      }
      auto res2 = std::mismatch(partial.begin(), partial.end(), c->name().begin());
      if (res2.first == partial.end()) {
        completions.push_back(c->name());
      }
    }
  }

  void ConfigurableShell::confFileCompletion(StringVector& completions,
                                             const std::string& prefix_,
                                             bool is_dir,
                                             bool is_file) {
    std::string path;
    std::string suffix;
    std::string prefix;
    if (prefix_.empty() || prefix_[0] != '/') {
      prefix = std::string(getcwd(NULL, 0)) + "/" + prefix;
    }
    if (!prefix_.empty() && prefix_[0] == '/') {
      prefix = prefix_;
    }

    std::size_t pos = prefix.find_last_of('/');
    if (pos != std::string::npos) {
      path   = prefix.substr(0, pos + 1);
      suffix = prefix.substr(pos + 1);
    }
    DIR* dp;
    struct dirent* ep;
    dp = opendir(path.c_str());
    if (dp == NULL)
      return;

    while ((ep = readdir(dp))) {
      std::string entry_path(ep->d_name);
      if (((ep->d_type == DT_DIR) && !is_dir) || ((ep->d_type != DT_DIR) && !is_file))
        continue;

      if (ep->d_type == DT_DIR)
        entry_path += "/";

      if (suffix.empty()) {
        completions.push_back(path + entry_path);
        continue;
      }
      auto res = std::mismatch(suffix.begin(), suffix.end(), entry_path.begin());
      if (res.first == suffix.end()) {
        completions.push_back(path + entry_path);
        continue;
      }
    }
    closedir(dp);
  }

  void ConfigurableShell::confTypesCompletion(StringVector& completions,
                                              const std::string& partial) {
    std::vector<std::string> types = ConfigurableManager::listTypes();
    for (const std::string& t : types) {
      auto res = std::mismatch(partial.begin(), partial.end(), t.begin());
      if (res.first == partial.end()) {
        completions.push_back(t);
        continue;
      }
    }
  }

  void ConfigurableShell::confFieldCompletion(StringVector& completions,
                                              const std::string& partial,
                                              ConfigurablePtr c) {
    for (auto it : c->properties()) {
      const std::string& field_name = it.first;
      auto res = std::mismatch(partial.begin(), partial.end(), field_name.begin());
      if (res.first == partial.end()) {
        completions.push_back(field_name);
        continue;
      }
    }
  }

  void ConfigurableShell::confCommandCompletion(StringVector& completions,
                                                const std::string& partial) {
    for (auto& it : _command_map) {
      const std::string& cmd = it.first;
      auto res               = std::mismatch(partial.begin(), partial.end(), cmd.begin());
      if (res.first == partial.end()) {
        completions.push_back(cmd);
      }
    }
  }

  void ConfigurableShell::confPreemptibleCompletion(StringVector& completions,
                                 const std::string& partial) {
    for (PropertyContainerIdentifiablePtr c : _manager.instances()) {
      Preemptible* p=dynamic_cast<Preemptible*>(c.get());
      if (!p)
        continue;
      std::ostringstream os;
      os << c;
      std::string ptr_name = os.str();
      auto res             = std::mismatch(partial.begin(), partial.end(), ptr_name.begin());
      if (res.first == partial.end()) {
        completions.push_back(ptr_name);
      }
      if (!c->name().length()) {
        continue;
      }
      auto res2 = std::mismatch(partial.begin(), partial.end(), c->name().begin());
      if (res2.first == partial.end()) {
        completions.push_back(c->name());
      }
    }
  }


  void ConfigurableShell::confLineCompletions(StringVector& completions, const char* line) {
#define MAX_LINE_LENGTH 10240
    completions.clear();
    // tokenize the line
    StringVector tokens;
    const char* c         = line;
    const int line_length = strlen(line);
    if (line_length > MAX_LINE_LENGTH)
      throw std::runtime_error("shell line buffer overrun");
    char temp_buffer[MAX_LINE_LENGTH];
    char* t = temp_buffer;
    while (*c) {
      *t = *c;
      if (*t == ' ' || !*t) {
        *t = 0;
        if (t != temp_buffer) {
          tokens.push_back(std::string(temp_buffer));
        }
        t = temp_buffer;
      } else {
        ++t;
      }
      ++c;
    }
    if (t != temp_buffer) {
      *t = 0;
      tokens.push_back(temp_buffer);
    }

    // handle last dummy token
    if (!tokens.size() || (line_length && line[line_length - 1] == ' ')) {
      tokens.push_back(std::string());
    }

    // if 1 token we need to complete the commands
    if (tokens.size() == 1) {
      confCommandCompletion(completions, tokens[0]);
      return;
    }

    // we seek for a command
    auto it = _command_map.find(tokens[0]);
    if (it == _command_map.end()) {
      return;
    }

    StringVector args = tokens;

    args.erase(args.begin());

    // ask the command for the proper completions
    // of the __word__

    StringVector word_completions;
    it->second->completions(word_completions, args);

    std::string s_base = tokens[0];
    for (size_t i = 1; i < tokens.size() - 1; ++i) {
      s_base += std::string(" ");
      s_base += tokens[i];
    }

    for (std::string& w : word_completions) {
      completions.push_back(s_base + " " + w);
    }
  }

  void ConfigurableShell::parseLine(std::istream& is) {
    char buffer[1024];
    is.getline(buffer, 1024);
    std::istringstream ss(buffer);
    std::string tag;
    ss >> tag;
    replaceEnvTags(tag);
    
    if (!ss) {
      return;
    }
    std::vector<std::string> args;
    while (ss) {
      std::string arg;
      ss >> arg;
      if (ss) {
        replaceEnvTags(arg);
        args.push_back(arg);
      }
    }
    auto cmd_it = _command_map.find(tag);
    if (cmd_it == _command_map.end()) {
      std::cerr << "ConfigurableShell::parseLine|unknown command [" << FG_YELLOW(tag) << "]"
                << std::endl;
      return;
    }
    bool result = cmd_it->second->execute(args);

    if (!result) {
      std::cerr << "ConfigurableShell::parseLine|usage: " << cmd_it->second->help_message
                << std::endl;
    }
    return;
  }

  static void ctrlJCallback() {
    auto& instance = PreemptibleController::instance();
    if (! instance)
      return;
    auto runner=instance->getRunner();
    if (!runner) {
      return;
    }
    instance->setStepMode(true);
    instance->resume();
  }

  ConfigurableShell::ConfigurableShell(ConfigurableManager& manager_) : _manager(manager_) {
    std::vector<std::string> types;
    for (auto s : types) {
      std::cerr << s << std::endl;
    }

    linenoiseSetCompletionCallback(completionCallback);
    linenoiseSetCtrlJCallback(ctrlJCallback);
    addCommand(new CommandHelp(this));
    addCommand(new CommandEcho(this));
    addCommand(new CommandPwd(this));
    addCommand(new CommandLs(this));
    addCommand(new CommandCd(this));
    addCommand(new CommandDLConfig(this));
    addCommand(new CommandDLOpen(this));
    addCommand(new CommandOpen(this));
    addCommand(new CommandQuit(this));
    addCommand(new CommandInstances(this));
    addCommand(new CommandTypes(this));
    addCommand(new CommandSetName(this));
    addCommand(new CommandShow(this));
    addCommand(new CommandCreate(this));
    addCommand(new CommandErase(this));
    addCommand(new CommandWrite(this));
    addCommand(new CommandSet(this));
    addCommand(new CommandNames(this));
    addCommand(new CommandClear(this));
    addCommand(new CommandRun(this));
    addCommand(new CommandKill(this));
    addCommand(new CommandExec(this));
    addCommand(new CommandForeground(this));
    addCommand(new CommandAddCanvas(this));
    addCommand(new CommandRemoveCanvas(this));
    addCommand(new CommandListCanvases(this));
    addCommand(new CommandStepMode(this));
    addCommand(new CommandSleep(this));

  }

  ConfigurableShell::~ConfigurableShell() {
    clear();
    linenoiseSetCompletionCallback(0);
  }

  void ConfigurableShell::runStatic(const StringVector& lines) {
    auto& instance = PreemptibleController::instance();
    if (! instance)
      return;
    ConfigurableShell* shell=dynamic_cast<ConfigurableShell*>(instance.get());
    if (! shell)
      return;
    shell->run(lines);
  }

  void ConfigurableShell::stopViewer() {
    if (viewer_core) {
      std::cerr << "ConfigurableShell::terminationCallback|stopping viewer" << std::endl;
      viewer_core->stop();
      // awaken the canvas with a flush;
      for (const auto& canvas : canvases) {
        canvas.second->flush();
      }
    }
  }

  void ConfigurableShell::handleViewer()  {
    if (!this->_viewers_changed)
      return;
    cerr << "viewer changed!" << endl;
    this->_viewers_changed=false;
    if (!viewer_core || canvases.empty())
      return;

    // ia busy waiting that viewer is online
    while (!viewer_core->isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (const auto& canvas : canvases) {
      cerr << "writing message" << endl;
      canvas.second->putText("waiting...");
      canvas.second->flush();
    }
  }


  void ConfigurableShell::canvasCompletion(StringVector& completions,
                                                 const std::string& partial) {
    std::ostringstream os;
    for (const auto c : canvases) {
      const std::string& canvas = c.first;
      auto res                  = std::mismatch(partial.begin(), partial.end(), canvas.begin());
      if (res.first == partial.end()) {
        completions.push_back(canvas);
      }
    }
  }

} // namespace srrg2_core
