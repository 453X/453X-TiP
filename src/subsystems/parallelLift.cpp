#include "main.h"
    

namespace parallelLift{

    Motor lift(7);

    void init()
    {
        lift.setBrakeMode(AbstractMotor::brakeMode::hold);
    }

    void opcontrol()
    {
        Controller master;
        ControllerButton up(ControllerDigital::up);
        ControllerButton down(ControllerDigital::down);


            if (up.isPressed())
                lift.moveVoltage(12000);
            else if (down.isPressed())
                lift.moveVoltage(-12000);
            else
                lift.moveVelocity(0);
    }
}