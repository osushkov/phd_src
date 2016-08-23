
#ifndef _Settings_H_
#define _Settings_H_

#include <map>
#include <string>


class Settings {
  public:
    static Settings& instance(void);
    bool load(std::string filename);
    
    int getIntValue(std::string section, std::string field);
    float getFloatValue(std::string section, std::string field);
    std::string getStringValue(std::string section, std::string field);

  private:
    bool loaded;
    std::string settings_filename;
    std::map<std::string, std::map<std::string, std::string> > data;
    

    // Private constructor and destructor since singleton class.
    Settings();
    ~Settings();
};


#endif

