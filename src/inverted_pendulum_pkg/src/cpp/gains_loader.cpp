#include "gains_loader.h"
#include <iostream>
#include <algorithm>
#include "ros/ros.h"


gainLoader::gainLoader() : _gains_directory ("/root/.ros/controller_gains"),
                           _gains_file_pattern("^controller_gains_.*\\.yaml$"){
    if (fs::exists(_gains_directory)){
        ROS_INFO("[gain_loader/construct]: gains directory found and set");
    }
    else{
        ROS_FATAL("[gain_loader/construct]: no gains direcotry found at %s", _gains_directory.c_str());
    }
}

gainLoader::gainLoader(fs::path gains_directory) : _gains_directory(gains_directory),
                                                   _gains_file_pattern("^controller_gains_.*\\.yaml$"){
    if (fs::exists(gains_directory)){
        ROS_INFO("[gain_loader/construct]: gains directory found and set");
    }
    else{
        ROS_FATAL("[gain_loader/construct]: no gains direcotry found at %s", gains_directory.c_str());
    }
}

gainLoader::~gainLoader(){
    ROS_INFO("[gainLoader/destructor]: object destroyed");
}

void gainLoader::set_gains_direcotry(fs::path gains_directory){
    _gains_directory = gains_directory;
    if (fs::exists(gains_directory)){
        ROS_INFO("[gain_loader/set_gains_directory]: gains directory found and set");
    }
    else{
        ROS_ERROR("[gain_loader/set_gains_directory]: no gains direcotry found at %s", gains_directory.c_str());
    }
}


std::string gainLoader::get_latest_gains_file(){

    // is gains directory set
    if (_gains_directory.empty()){
        ROS_ERROR("[gain_loader/get_latest_gains]: no gains directory set");
        return "";
    }

    // check if the directory is empty
    if (fs::is_empty(_gains_directory)){
        ROS_ERROR("[gain_loader/get_latest_gains]: no files found in the directory");
        return "";
    }

    // iterate through the direcotry
    fs::directory_iterator gains_dir_it(_gains_directory);
    std::vector<fs::path> gain_files;

    for (const auto& dir_entry : gains_dir_it){
        if (std::regex_match(dir_entry.path().filename().string(), _gains_file_pattern)){
            gain_files.push_back(dir_entry.path());
        }
    }

    // find the latest file
    auto latest_gains_file = std::max_element(gain_files.begin(), gain_files.end());

    return latest_gains_file->string();
}


std::optional<std::vector<float>> gainLoader::get_latest_gains(){
    
    std::vector<float> best_gains; 

    // get the latest file
    auto latest_gains_file = get_latest_gains_file();
    _gains_yaml_nd = YAML::LoadFile(latest_gains_file);

    // extract gains 
    if (_gains_yaml_nd["gains"]){
        best_gains = _gains_yaml_nd["gains"].as<std::vector<float>>();
        ROS_INFO("[gain_loader/get_latest_gains_file]: best cart gains: %f, %f, %f",
                    best_gains[0], best_gains[1], best_gains[2]); 
        ROS_INFO("[gain_loader/get_latest_gains_file]: best pendulum gains: %f, %f, %f",
                    best_gains[3], best_gains[4], best_gains[5]);
        return best_gains;
    }
    else{
        ROS_INFO("[gain_loader/get_latest_gains_file]: porblem in the yaml format. No \"gains\" entry found.");
        return std::nullopt; 
    }
    

}





//  // set gains direcotry 
//  fs::path gains_directory = "/root/.ros/controller_gains"; 
//  fs::directory_iterator dir_it(gains_directory);
//  if (fs::exists(gains_directory)){
//      std::cout << "Controller directory exists" << std::endl;
//  }

//  // store all gain files
//  std::vector<fs::path> gain_files;
//  for (const auto& dir_entry : dir_it){
//      std::cout << dir_entry.path() << std::endl;
//      gain_files.push_back(dir_entry.path());
//  }

//  // get the latest gain file
//  auto latest_gains_file = std::max_element(gain_files.begin(), gain_files.end());
//  std::cout << "latest gains file: " << *latest_gains_file << std::endl;
//  YAML::Node best_gains_nd = YAML::LoadFile(latest_gains_file->string());   

//  // extract the gains 
//  std::vector<float> best_gains;
//  if (best_gains_nd["gains"]){
//      best_gains = best_gains_nd["gains"].as<std::vector<float>>(); 
//      std::cout << "best gains:" << best_gains_nd["gains"] << std::endl; // cart_gains - pendulum gains 
//  }

//  else{
//      std::cout << "there is a problem" << std::endl;
//  }