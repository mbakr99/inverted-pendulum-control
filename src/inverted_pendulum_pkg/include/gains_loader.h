#ifndef GAINS_LOADER_H
#define GAINS_LOADER_H

#include <filesystem>
#include <vector>
#include <regex>
#include <yaml-cpp/yaml.h>
#include <optional>


namespace fs = std::filesystem;


class gainLoader{
private:
    fs::path _gains_directory;
    std::regex _gains_file_pattern;
    YAML::Node _gains_yaml_nd;

public:
    gainLoader();
    gainLoader(fs::path gains_directory);
    ~gainLoader();
    std::optional<std::vector<float>> get_latest_gains();
    void set_gains_file_pattern(std::regex gains_file_pattern);

private:
    std::string get_latest_gains_file();
    void set_gains_direcotry(fs::path gains_directory);

};

#endif