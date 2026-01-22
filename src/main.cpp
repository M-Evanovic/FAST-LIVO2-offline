#include "LIVMapper.h"

int main(int argc, char **argv)
{
  std::string config_dir = std::string(ROOT_DIR) + "config/"; 
  
  if (argc == 2) {
    std::cout << "using config from: " << argv[1] << std::endl;
    config_dir = std::string(argv[1]);
  } else {
    std::cout << "using default config: " << config_dir << std::endl;
  }

  LIVMapper mapper(config_dir);
  mapper.run();
  return 0;
}