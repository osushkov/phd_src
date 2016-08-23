
#ifndef _Console_H_
#define _Console_H_

#include <string>
#include <vector>
#include <list>


class Console {

  public:
    static Console& instance(void);

    bool prompt(void);

  private:
    std::vector<std::string> last_command;

    bool parseCommand(std::string command);
    std::vector<std::string> tokenise(std::string input, char delimiter);
    void sanitise(std::string &input);

    void printHelp(void);
    void setCommand(std::vector<std::string> args);
    void learnCommand(std::vector<std::string> args);
    void examineCommand(std::vector<std::string> args);
    void generateSIFTCommand(std::vector<std::string> args);
    void handCommand(std::vector<std::string> args);
    void armCommand(std::vector<std::string> args);
    void testCommand(std::vector<std::string> args);
    void reconstruct(std::vector<std::string> args);
    void generateMovement(std::vector<std::string> args);
    void rotate(std::vector<std::string> args);
    void sceneMatchObject(std::vector<std::string> args);
    void stitchTogether(std::vector<std::string> args);
    void fitShape(std::vector<std::string> args);
    void evalMatch(std::vector<std::string> args);
    void finalisePointCloud(std::vector<std::string> args);

    void printSearchHelp(void);
    void printLearnHelp(void);


    void fixPath(std::string &path);
};

#endif

