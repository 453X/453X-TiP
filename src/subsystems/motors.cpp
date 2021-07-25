#include "main.h"

// Chassis Controller - lets us drive the robot around with open- or closed-loop control
    std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
            .withMotors(
                {-1, -2}, // Left motors are 1 & 2 (reversed)
                {3, 4})
            // Green gearset, 4 in wheel diam, 11.5 in wheel track
            .withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5BlueTPR})
            .build();

    


