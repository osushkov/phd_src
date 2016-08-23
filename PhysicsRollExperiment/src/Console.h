
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
    void examineCommand(std::vector<std::string> args);
    void sceneMatchObject(std::vector<std::string> args);
    void physicsTest(std::vector<std::string> args);
    void experimentOnObject(std::vector<std::string> args);

    void openHand(std::vector<std::string> args);
    void closeHand(std::vector<std::string> args);
    void releaseHand(std::vector<std::string> args);

    void test(std::vector<std::string> args);

    void printSearchHelp(void);
    void printLearnHelp(void);


    void fixPath(std::string &path);
};

#endif

