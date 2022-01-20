#include "main.h"

namespace rollers
{

    Motor roller(-6);

    void init()
    {
        roller.setBrakeMode(AbstractMotor::brakeMode::coast);
    }

    

    void opcontrol()
    {
        Controller master;
        ControllerButton L1(ControllerDigital::L1);
        ControllerButton shift(ControllerDigital::L2);

        if (shift.isPressed())
        {
            if (L1.isPressed())
            {
                roller.moveVoltage(-12000);
            }
            else
            {
                roller.moveVelocity(0);
            }
        }
        else
        {

            if (L1.isPressed())
            {
                roller.moveVoltage(12000);
            }
            else
            {
                roller.moveVelocity(0);
            }
        }
    }
}
