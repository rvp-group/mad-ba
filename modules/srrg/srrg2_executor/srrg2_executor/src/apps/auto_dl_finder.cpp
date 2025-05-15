#include <cstdio>
#include <cstdlib>
#include <srrg_boss/serializer.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <unistd.h>

#ifdef DEVEL_SPACE
std::string devel_space = DEVEL_SPACE;
#endif

#define MAX_PATH_LEN 512

using namespace srrg2_core;
// srrg getting bash output [link:
// https://www.jeremymorgan.com/tutorials/c-programming/how-to-capture-the-output-of-a-linux-command-in-c/]
std::vector<std::string> getStdOutFromCommand(std::string& cmd) {
  std::vector<std::string> data;
  FILE* stream;
  const int max_buffer = 2048;
  char buffer[max_buffer];
  cmd += " 2>&1";

  stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, max_buffer, stream) != NULL) {
        data.push_back(buffer);
      }
    }
    pclose(stream);
  }
  return data;
}
const char* banner[] = {"auto dl finder", "assumption: using catkin as building system", 0};

int main(int argc, char** argv) {
#ifndef DEVEL_SPACE
  std::cerr << "NO DEVEL SPACE DEFINED" << std::endl;
  return -1;
#endif

  srrgInit(argc, argv, "auto_dl_finder");

  char cwd[MAX_PATH_LEN];
  if (!getcwd(cwd, MAX_PATH_LEN)) {
    throw std::runtime_error("unable to get the current working directory");
  }
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString dl_stub_file(
    &cmd_line, "dlc", "dl-config", "stub where to read/write the stub", "dl.conf");
  cmd_line.parse();

  std::string stripped_devel_space = devel_space.substr(0, devel_space.find("devel") + 5);
  std::cerr << "looking for dynamic libraries in " << stripped_devel_space << std::endl;

  std::string output_file(cwd);
  output_file += "/" + dl_stub_file.value();
  Serializer ser;
  ser.setFilePath(output_file);
  DynamicLoaderConfig loader;

  loader.so_paths.pushBack(stripped_devel_space);
  if (chdir(stripped_devel_space.c_str()) != 0) {
    std::string error_message = "unable to chdir to " + stripped_devel_space;
    throw std::runtime_error(error_message.c_str());
  };
  std::string command = "find * \\( -type l -o -type f \\) -name '*.so' ";
  // srrg exclude already loaded libraries
  //  command += "! -path '*_core*' ! -path '*qgl_viewport*' ";
  std::vector<std::string> things = getStdOutFromCommand(command);

  for (auto& a : things) {
    a.erase(std::remove(a.begin(), a.end(), '\n'), a.end());
    loader.so_names.pushBack(a);
  }

  for (size_t i = 0; i < loader.so_names.size(); ++i) {
    std::cerr << loader.so_names.value(i) << std::endl;
  }

  std::cerr << "serializing " << output_file << std::endl;
  ser.writeObject(loader);
  return 0;
}
