#include "main.h"

namespace parallelLift
{

    Motor lift(7);
    Motor claw(-8);

    void init()
    {
        lift.setBrakeMode(AbstractMotor::brakeMode::hold);
        claw.setBrakeMode(AbstractMotor::brakeMode::hold);
    }

    void opcontrol()
    {
        Controller master;
        ControllerButton R1(ControllerDigital::R1);
        ControllerButton x(ControllerDigital::X);
        ControllerButton b(ControllerDigital::B);
        ControllerButton shift(ControllerDigital::L2);

        if (shift.isPressed())
        {
            if (R1.isPressed())
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
            if (R1.isPressed())
            {
                lift.moveVoltage(12000);
            }
            else
            {
                lift.moveVelocity(0);
            }
        }

        if (x.isPressed())
            claw.moveVoltage(12000);
        else if (b.isPressed())
            claw.moveVoltage(-12000);
        else
            claw.moveVelocity(0);
    }
}