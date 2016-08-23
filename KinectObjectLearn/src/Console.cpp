
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
    else if(tokens[0] == "learn" || tokens[0] == "l"){
        learnCommand(tokens);
    }
    else if(tokens[0] == "examine"){
    	examineCommand(tokens);
    }
    else if(tokens[0] == "generate_sift" || tokens[0] == "gs"){
        generateSIFTCommand(tokens);
    }
    else if(tokens[0] == "test"){
        testCommand(tokens);
    }
    else if(tokens[0] == "reconstruct"){
        reconstruct(tokens);
    }
    else if(tokens[0] == "generate_movement"){
        generateMovement(tokens);
    }
    else if(tokens[0] == "rotate"){
        rotate(tokens);
    }
    else if(tokens[0] == "scene_match" || tokens[0] == "sm"){
        sceneMatchObject(tokens);
    }
    else if(tokens[0] == "stitch_together" || tokens[0] == "st"){
        stitchTogether(tokens);
    }
    else if(tokens[0] == "fit_shape" || tokens[0] == "fs"){
        fitShape(tokens);
    }
    else if(tokens[0] == "eval_match" || tokens[0] == "em"){
        evalMatch(tokens);
    }
    else if(tokens[0] == "fpc"){
        finalisePointCloud(tokens);
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

void Console::setCommand(std::vector<std::string> args){

}

void Console::learnCommand(std::vector<std::string> args){
    if(args.size() == 2){
        MainController::instance().learnObject(args[1]);
    }
    else{
        std::cout << "Incorrect number of arguments" << std::endl;
    }
}

void Console::examineCommand(std::vector<std::string> args){
    if(args.size() == 2){
        MainController::instance().examineObject(args[1]);
    }
    else{
        std::cout << "Incorrect number of arguments" << std::endl;
    }
}

void Console::generateSIFTCommand(std::vector<std::string> args){
    if(args.size() != 3){
        std::cout << "Incorrect number of arguments" << std::endl;
        return;
    }

    MainController::instance().generateSIFT(args[1], args[2]);
}

void Console::handCommand(std::vector<std::string> args){

}

void Console::armCommand(std::vector<std::string> args){

}

void Console::testCommand(std::vector<std::string> args){
    MainController::instance().test();
}

void Console::reconstruct(std::vector<std::string> args){
    if(args.size() != 2){
        std::cout << "Incorrect number of arguments" << std::endl;
        return;
    }

    MainController::instance().reconstruct(args[1]);
}

void Console::generateMovement(std::vector<std::string> args){
    if (args.size() != 2) {
        std::cerr << "Incorrect number of arguments" << std::endl;
        std::cerr << "Expected: generate_movement record_path" << std::endl;
        return;
    }

    if(args.size() == 2) {
        MainController::instance().generateMovement(args[1]);
    }
    else{
        MainController::instance().generateMovement("");
    }
}

void Console::rotate(std::vector<std::string> args) {
    if (args.size() > 2) {
        std::cerr << "Incorrect number of arguments" << std::endl;
        std::cerr << "Expected: rotate record_path" << std::endl;
        return;
    }

    if(args.size() == 2) {
        MainController::instance().rotate(args[1]);
    }
    else{
        MainController::instance().rotate("");
    }
}

void Console::sceneMatchObject(std::vector<std::string> args){
    if(args.size() != 2){
        std::cerr << "Incorrect number of arguments" << std::endl;
        std::cerr << "Expected: scene_match object_name" << std::endl;
        return;
    }

    MainController::instance().sceneMatchObject(args[1]);
}

void Console::stitchTogether(std::vector<std::string> args){
    if(args.size() != 3){
        std::cerr << "Incorrect number of arguments" << std::endl;
        std::cerr << "Expected: stitch_togerther object0 object1" << std::endl;
        return;
    }

    MainController::instance().stitchTogether(args[1], args[2]);
}

void Console::fitShape(std::vector<std::string> args){
    if(args.size() != 2){
        std::cerr << "Incorrect number of arguments" << std::endl;
        std::cerr << "Expected: fit_shape object0" << std::endl;
        return;
    }

    MainController::instance().fitShape(args[1]);
}

void Console::evalMatch(std::vector<std::string> args){
    if(args.size() != 2){
        std::cerr << "Incorrect number of arguments" << std::endl;
        return;
    }

    MainController::instance().evalMatch(args[1]);
}

void Console::finalisePointCloud(std::vector<std::string> args){
    if(args.size() != 2){
        std::cerr << "Incorrect number of arguments" << std::endl;
        return;
    }

    MainController::instance().finalisePointCloud(args[1]);
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
