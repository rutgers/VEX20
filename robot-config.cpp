#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftFront = motor(PORT1, ratio18_1, false);
motor LeftBack = motor(PORT11, ratio18_1, false);
motor RightFront = motor(PORT10, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
motor LeftBackLift = motor(PORT15, ratio18_1, true);
motor RightBackLift = motor(PORT17, ratio18_1, false);
motor Front = motor(PORT4, ratio18_1, false);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}