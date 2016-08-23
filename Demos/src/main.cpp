

#include "MainController.h"
#include "Arm/Arm.h"
#include "Settings.h"
#include "Console.h"
#include "Control.h"

int main(int argc, char **argv){
    Settings::instance().load("data/settings.cfg");
    //Control::moveArmOutOfTheWay();
    //return 0;

    Arm::getArm(); // basically connect to the arm
    MainController::instance().initialise();

    // Start up the console. Keep prompting the user for an input command.
    // When the user performs the quet command, prompt() returns false and we
    // exit.
    while(Console::instance().prompt());

    // TODO: maybe print out some statistics/summary here of the session
    return 0;
}
