#include "H1Control.h"

#include <mc_rtc/config.h>

#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace bfs = boost::filesystem;

namespace
{

/* Checking the existence of the file */
/* Return value: true if the file exists, false otherwise */
bool file_exists(const std::string& str)
{
  std::ifstream fs(str);
  return fs.is_open();
}

} // namespace

/* Main function of the interface */
int main(int argc, char * argv[])
{
  /* Set command line arguments options */
  /* Usage example: MCControlUnitree2 -h simulation -f @ETC_PATH@/mc_unitree/mc_rtc_xxxxx.yaml */
  std::string conf_file;
  std::string network;
  po::options_description desc(std::string("MCControlH1 options"));
  
  bfs::path config_path = mc_rtc::user_config_directory_path("mc_rtc.conf");
  // Load user's local configuration if it exists
  if(!bfs::exists(config_path)) { config_path.replace_extension(".yaml"); }
  std::string check_file = config_path.string();
  if(!file_exists(check_file))
  {
    check_file = "";
  }
  
  // clang-format off
  desc.add_options()
    ("help", "display help message")
    ("network,n", po::value<std::string>(&network)->default_value("lo"), "name of network adaptor")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(check_file), "configuration file");
  // clang-format on
  
  /* Parse command line arguments */
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  mc_rtc::log::info("[mc_unitree] Reading additional configuration from {}", conf_file);

  /* Create global controller */
  mc_control::MCGlobalController g_controller(conf_file, nullptr);

  /* Check that the interface can work with the main controller robot */
  std::string module_name;
  module_name.resize(mc_unitree::ROBOT_NAME.size());
  std::transform(mc_unitree::ROBOT_NAME.begin(), mc_unitree::ROBOT_NAME.end(), module_name.begin(), ::tolower);
  if(g_controller.robot().name() != module_name)
  {
    mc_rtc::log::error(
        "[mc_unitree] This program can only handle '" + mc_unitree::ROBOT_NAME + "' at the moment");
    return 1;
  }
  
  /* Create MCControlUnitree2 interface */
  mc_unitree::MCControlUnitree2<mc_unitree::H1Control,
                                mc_unitree::H1SensorInfo,
                                mc_unitree::H1CommandData,
                                mc_unitree::H1ConfigParameter> mc_control_unitree(g_controller,
                                                                                  network);
  
  mc_rtc::log::info("[mc_unitree] Terminated");
  
  return 0;
}
