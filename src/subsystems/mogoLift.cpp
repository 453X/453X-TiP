#include "main.h"

namespace mogoLift
{

    Motor lift(5);
    ADIButton frontLimit('G');
    ADIButton backLimit('H');

    void init()
    {
        lift.setBrakeMode(AbstractMotor::brakeMode::hold);
        lift.setGearing(AbstractMotor::gearset::red);
        lift.tarePosition();
    }

    void opcontrol()
    {
        Controller master;
        ControllerButton R2(ControllerDigital::R2);
        ControllerButton up(ControllerDigital::up);
        ControllerButton shift(ControllerDigital::L2);

        if (shift.isPressed())
        {
            if (R2.isPressed() && !backLimit.isPressed())
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
            if (R2.isPressed() && !frontLimit.isPressed())
            {
                lift.moveVoltage(12000);
            }
            else
            {
                lift.moveVelocity(0);
                lift.tarePosition();
            }
        }
    }
}