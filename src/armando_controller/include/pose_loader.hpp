#pragma once
#include <string>
#include <vector>
#include <map>

struct Pose {
  std::string name;
  std::vector<double> positions;
};

std::map<std::string, std::vector<double>> load_poses(const std::string& yaml_file_path);
