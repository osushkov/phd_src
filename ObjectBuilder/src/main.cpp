
#include <iostream>
#include <cstdlib>

#include "MainController.h"
#include "Console.h"
#include "Settings.h"

int main(int agrc, char **argv){
    Settings::instance().load("data/settings.cfg");

    MainController::instance().initialise();

    // Start up the console. Keep prompting the user for an input command.
    // When the user performs the quet command, prompt() returns false and we
    // exit.
    while(Console::instance().prompt());
    


    // TODO: maybe print out some statistics/summary here of the session
    return 0;
}