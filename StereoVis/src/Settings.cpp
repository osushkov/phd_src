
#include "Settings.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <SDL/SDL.h>


Settings& Settings::instance(void){
    static Settings my_settings;
    return my_settings;
}


bool Settings::load(std::string filename){
    std::string cur_section = "";    
    
    std::ifstream settings_file(filename.c_str());
    if(!settings_file.is_open() || !settings_file.good()){ 
        std::cerr << "Failed to load settings file: " << filename << std::endl;
        return false; 
    }
    
    std::string tmp;
    while(!settings_file.eof()){
        settings_file >> tmp;
        if(tmp == "["){
            settings_file >> cur_section;
            settings_file >> tmp; // read the closing ']'
            if(tmp != "]"){ std::cerr << "Warning, incorrect settings file format" << std::endl; }
        }
        else if(!settings_file.eof()){
            std::string field_name = tmp;
            settings_file >> tmp;
            if(tmp != "="){ std::cerr << "Warning, incorrect settings file format" << std::endl; }
            settings_file >> tmp;
            std::string field_value = tmp;

            if (field_name[0] != '#')  {
                data[cur_section][field_name] = field_value;
                std::cout << "[" << cur_section << "] " << field_name << " : " << field_value << std::endl;
            } else {
                std::cout << "ignored [" << cur_section << "] " << field_name << " : " << field_value << std::endl;
            }
        }
    }


    loaded = true;
    return loaded;
}

int Settings::getIntValue(std::string section, std::string field){
    int result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;

}

float Settings::getFloatValue(std::string section, std::string field){
    float result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;
}

std::string Settings::getStringValue(std::string section, std::string field){
    std::string result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;
}

Settings::Settings() : loaded(false) {

}

Settings::~Settings(){

}




