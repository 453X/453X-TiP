#include "main.h"

namespace mogoLift
{

    Motor lift(5);
    ADIButton limit('G');

    void init()
    {
        lift.setBrakeMode(AbstractMotor::brakeMode::brake);
    }

    void opcontrol()
    {
        Controller master;
        ControllerButton R1(ControllerDigital::R1);
        ControllerButton R2(ControllerDigital::R2);

        if (R2.isPressed())
        {
            lift.moveVoltage(12000);
        }
        else if (R1.isPressed())
        {
            if (!limit.isPressed())
            {
                lift.moveVoltage(-12000);
            }
            else
            {
                lift.moveVelocity(0);
            }
        }
        else
        {
            lift.moveVelocity(0);
        }
    }
}