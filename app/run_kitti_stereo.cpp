#include <gflags/gflags.h>
#include "visual_odometry.hpp"

DEFINE_string(config_file_path, "../config/config.yaml", "config file path");

int main(int argc, char **argv) {
//   google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  VisualOdometry::Ptr vo(new VisualOdometry(FLAGS_config_file_path));
  assert(vo->Init() == true);
  vo->Run();

  return 0;
}