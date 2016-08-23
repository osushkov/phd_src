
#include "Settings.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>


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
    return true;
}

int Settings::getIntValue(std::string section, std::string field){
    assert(loaded);

    int result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;

}

float Settings::getFloatValue(std::string section, std::string field){
    assert(loaded);

    float result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;
}

std::string Settings::getStringValue(std::string section, std::string field){
    assert(loaded);

    std::string result;
    std::stringstream tmp_stream(data[section][field]);
    tmp_stream >> result;
    return result;
}

bool Settings::addSetting(std::string section, std::string field, std::string value){
    data[section][field] = value;
    loaded = true; // if we havent loaded from a file, adding at least 1 value will be good enough
    return true;
}

bool Settings::addSetting(std::string section, std::string field, int value){
    std::stringstream tmp_stream;
    tmp_stream << value;
    return addSetting(section, field, tmp_stream.str());
}

bool Settings::addSetting(std::string section, std::string field, float value){
    std::stringstream tmp_stream;
    tmp_stream << value;
    return addSetting(section, field, tmp_stream.str());
}

Settings::Settings() : loaded(false) {

}

Settings::~Settings(){

}




