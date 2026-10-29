#include "pose_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

std::map<std::string, std::vector<double>> load_poses(const std::string& yaml_file_path) {
    std::map<std::string, std::vector<double>> poses;
    try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        if (config["poses"]) {
            for (const auto& pose : config["poses"]) {
                std::string name = pose.first.as<std::string>();
                std::vector<double> positions = pose.second["positions"].as<std::vector<double>>();
                poses[name] = positions;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load poses: " << e.what() << std::endl;
    }
    return poses;
}
