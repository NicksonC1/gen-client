#include "main.h"
#include "gen/motion.h"
#include "gen/colorsort.h"
#include "gen/electronics.h"
#include "pros/optical.hpp"

using DriveMode = gen::Controller::DriveMode;
using Button = gen::Controller::Button;
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
gen::Controller controller(DriveMode::Arcade2Stick, 3, 10.0, true, pros::E_CONTROLLER_MASTER);
gen::Piston pistonA('A'); 
pros::adi::DigitalIn autonSwitch('H');

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); 

namespace Motor{
  pros::Motor intakeF(10, pros::MotorGearset::blue); 
  pros::Motor intakeU(-1, pros::MotorGearset::blue); 
} // namespace Motor

namespace Sensor{
  pros::Distance d_front(4); 
  pros::Distance d_left(3); 
  pros::Optical o_colorSort(8); 
  pros::Optical o_crossed(22); 
} // namspace Sensor

namespace Auton{
    void main(){}
    void leftB(){}
    void leftR(){}
    void rightB(){}
    void rightR(){}
    void soloB(){}
    void soloR(){}
    void skills(){}
} // namespace Auton


int autonState = 0;

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
  {"Default Auton", Auton::main},
  
  {"Blue Left Qual", Auton::leftB},
  {"Red Left Qual", Auton::leftR},

  {"Blue Right Qual", Auton::rightB},
  {"Red Right Qual", Auton::rightR},

  {"Blue Solo Qual", Auton::soloB},
  {"Red Solo Qual", Auton::soloR},

  {"Skills", Auton::skills},
};

void selector() {
    if (autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
    pros::lcd::set_text(4, autonRoutines[autonState].first);
    pros::delay(10);
}

// <----------------------------------------------------------- Main Functions ------------------------------------------------------------>
void initialize() {
  pros::Task t_Select(selector);
	pros::lcd::initialize();
  pros::Task autonSelect([]{ while(1){ selector(); pros::delay(10); }});
}
void disabled() {}
void competition_initialize() {}
void autonomous() {
  (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::main();
}

void opcontrol() {
	while (1) {
    controller.arcade_two_stick();
    if(controller.pressing(Button::A)) { pistonA.toggle(); }
		pros::delay(10);
	}
}