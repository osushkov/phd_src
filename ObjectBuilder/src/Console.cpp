
#include "Console.h"

#include <iostream>

#include "MainController.h"

Console& Console::instance(void){
    static Console console;
    return console;
}

bool Console::prompt(void){
    std::cout << "::> " << std::flush;

    std::string command;
    std::getline(std::cin, command);

    bool should_continue = parseCommand(command);
    return should_continue;
}

/**
 * Given a string type into the console, parse it and call the relevant
 * command with the specified arguments.
 */
bool Console::parseCommand(std::string command){
    bool valid_command = true;
    std::vector<std::string> tokens = tokenise(command, ' ');

    if (tokens.size() == 0) {
		return true;
	}

    sanitise(tokens[0]);

    if(tokens[0] == "p"){
    	tokens = last_command;
    }

    // TODO: maybe do a better way of a lookup table here.
    if(tokens[0] == "help" || tokens[0] == "?"){
        printHelp();
    }
    else if(tokens[0] == "quit" || tokens[0] == "q"){
        return false;
    }
    else if(tokens[0] == "build_object" || tokens[0] == "bo"){
    	buildObject(tokens);
    }
    else if(tokens[0] == "test"){
    	test(tokens);
    }
    else{
        std::cout << "Unknown Command" << std::endl;
        valid_command = false;
    }

    if(valid_command){
        last_command = tokens;
    }

    return true;
}

/**
 * Turn a line of text type into a console into an array of tokens.
 */
std::vector<std::string> Console::tokenise(std::string input, char delimiter){
    std::vector<std::string> result;
    std::string cur_token;

    for(unsigned i = 0; i < input.size(); i++){
        if(input[i] == delimiter && cur_token.size() > 0){
            result.push_back(cur_token);
            cur_token.clear();
        }
        else{
            cur_token.push_back(input[i]);
        }
    }

    if(cur_token.size() > 0){
        result.push_back(cur_token);
    }

    return result;
}

void Console::sanitise(std::string &input){
    for(unsigned i = 0; i < input.size(); i++){
        input[i] = tolower(input[i]);
    }
}

void Console::printHelp(void){
    std::cout << "Available Commands:" << std::endl;
    std::cout << "search" << std::endl;
    std::cout << "learn" << std::endl;
    std::cout << "correlate" << std::endl;
    std::cout << "generate_sift" << std::endl;
    std::cout << "pickup" << std::endl;
    std::cout << "quit" << std::endl;
}

void Console::buildObject(std::vector<std::string> args){
    MainController::instance().buildObject(args[1]);
}

void Console::test(std::vector<std::string> args){
	//MainController::instance().test();
}

void Console::printSearchHelp(void){
    std::cout << "search\t\t\t- Search for an object to pick up from the live camera" << std::endl;
    std::cout << "search video_path \t- Search for an object from a recorded stream" << std::endl;
}

void Console::printLearnHelp(void){
    std::cout << "learn object_name sample_frames\t\t\t- Learn an object's appearance from "
              << "the live camera" << std::endl;
    std::cout << "learn video_path object_name sample_frames\t- Learn an objects appearance from "
              << "a recorded stream" << std::endl;

}

void Console::fixPath(std::string &path){
    if(path[path.size()-1] != '/'){
        path = path + "/";
    }
}
