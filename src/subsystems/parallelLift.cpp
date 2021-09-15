#include "main.h"
    

namespace parallel{

    Motor lift(6);

    void init()
    {
        lift.setBrakeMode(AbstractMotor::brakeMode::hold);
    }

    void opcontrol()
    {
        Controller master;
        ControllerButton up(ControllerDigital::R1);
        ControllerButton down(ControllerDigital::R2);


            if (up.isPressed())
                lift.moveVoltage(12000);
            else if (down.isPressed())
                lift.moveVoltage(-12000);
            else
                lift.moveVelocity(0);
    }
}