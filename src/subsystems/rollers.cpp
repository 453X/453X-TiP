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
        ControllerButton L2(ControllerDigital::L2);

        if (L1.isPressed())
            roller.moveVoltage(12000);
        else if (L2.isPressed())
            roller.moveVoltage(-12000);
        else
            roller.moveVelocity(0);
    }
}