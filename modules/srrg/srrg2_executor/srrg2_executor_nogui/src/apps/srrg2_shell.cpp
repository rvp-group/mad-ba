#include <dlfcn.h>
#include <stdlib.h>
#include <gnu/lib-names.h>
#include <thread>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_system_utils/profiler.h>
#include <srrg_boss/json_object_writer.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_config/configurable_shell.h>
#include <srrg_config/linenoise.h>
#include <srrg_viewer/drawable_base.h>

//#include <multi_graph_sm/instances.h>

using namespace srrg2_core;
using namespace std;

// ia logging
#define LOG std::cerr << std::string(environ[0]) + "|"

const char* banner[] = {"dynamic executor", 0};

int main(int argc, char** argv) {
  srrgInit(argc, argv, "shell");
  Profiler::enable_logging = true;

  ParseCommandLine cmd_line(argv, banner);
  ArgumentString conf_file(&cmd_line, "c", "config-file", "config file to load on start", "");
  ArgumentString dl_stub_file(&cmd_line, "dlc", "dl-config", "stub where to read/write the stub", "dl.conf");
  ArgumentString dl_path_file(&cmd_line, "dlp", "dl-path", "path where to search for the stub", "./");
  ArgumentFlag   no_sigint(&cmd_line, "ns", "no-sigint", "if set the shell executes in batch mode after run");
  cmd_line.parse();
  const std::string exe_name = argv[0];

  ConfigurableManager manager;
  std::string dl_stub_path=crawlForFile(dl_stub_file.value(), dl_path_file.value());
  ifstream is(dl_stub_path);
  if (is.good()) {
    ConfigurableManager::initFactory(dl_stub_path);
  } else {
    ConfigurableManager::makeFactoryStub(dl_stub_path);
  }
  
  StringVector commands;
  if (conf_file.isSet()) {
    std::string open_command = std::string("open ") + conf_file.value();
    commands.push_back(open_command);
  }
  if(! cmd_line.lastParsedArgs().empty()) {
    ifstream is(cmd_line.lastParsedArgs()[0]);
    while (is) {
      char buf[1024];
      buf[0] = '#';
      buf[1] = 0;
      is.getline(buf, 1024);
      if (buf[0] && buf[0] != '#')
        commands.push_back(buf);
    }
  }
  commands.push_back(std::string("help"));

  // ds allocate a shell with visualization support
  cerr << "shell_init" << endl;
  ConfigurableShell* shell=new ConfigurableShell(manager);
  shell->removeCommand("add_canvas");
  shell->removeCommand("remove_canvas");
  shell->removeCommand("ls_canvases");
  
  ConfigurableShell::init(shell, !no_sigint.isSet());
  cerr << "done" << endl;
  shell->run(commands);
  PreemptibleController::fini();
  return 0;
}
