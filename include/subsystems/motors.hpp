#include "main.h"

std::shared_ptr<ChassisController> drive;
std::shared_ptr<OdomChassisController> driveOdom;


// Joystick to read analog values for tank or arcade control.
// Master controller by default.
Controller controller;
