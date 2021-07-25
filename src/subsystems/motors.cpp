#include "main.h"

// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto drive =
    ChassisControllerBuilder()
        .withMotors(
            {1, -3}, // Left motors are 1 & 2
            {2, -4}) // Right motors are 3 & 4

        // Green gearset, external ratio of (84.0 / 60.0), 4 inch wheel diameter, 11.5 inch wheel track
        .withDimensions({AbstractMotor::gearset::blue, (84.0 / 60.0)}, {{4_in, 11.5_in}, imev5BlueTPR}) //NEED SETUP!

        // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
        // track        = distance between 2 tracking wheels
        // wheel track  = distance between 2 wheels on either side

        // .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
        // .buildOdometry();
        .build();

auto driveOdom =
    ChassisControllerBuilder()
        .withMotors(
            {1, -3}, // Left motors are 1 & 2
            {2, -4}) // Right motors are 3 & 4

        // Green gearset, external ratio of (84.0 / 60.0), 4 inch wheel diameter, 11.5 inch wheel track
        .withDimensions({AbstractMotor::gearset::blue, (84.0 / 60.0)}, {{4_in, 11.5_in}, imev5BlueTPR}) //NEED SETUP!

        // track        = distance between 2 tracking wheels
        // wheel track  = distance between 2 wheels on either side

        // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
        .withOdometry({{2.75_in, 7_in}, quadEncoderTPR}) //NEED SETUP!
        .buildOdometry();
