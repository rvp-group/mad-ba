#include <string>
#include <map>
#include <vector>

namespace srrg2_core { 

  void initEnvMap();
  
  // returns the environment variables
  const std::map<std::string, std::string>& envMap();

  void envMapAddCmdArgs(const std::vector<std::string>& cmd_args);

  // replaces in a string the tags "<VAR_KEY>" with "VAR_VALUE"
  // according to the environment map;
  void replaceEnvTags(std::string& str);
  
}
