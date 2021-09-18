#include "main.h"

namespace drive
{

    Motor LF(1), LB(-2), RF(3), RB(-4);

    MotorGroup left({LF, LB});
    MotorGroup right({RF, RB});

    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
    auto drive =
        ChassisControllerBuilder()
            .withMotors(
                {1, -2}, // Left motors are 1 & 2
                {3, -4}) // Right motors are 3 & 4

            // Blue gearset, external ratio of (84.0 / 30.0), 4 inch wheel diameter, 11.5 inch wheel track
            .withDimensions({AbstractMotor::gearset::blue, (84.0 / 30.0)}, {{4_in, 11.5_in}, imev5BlueTPR}) //NEED SETUP!

            // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
            // track        = distance between 2 tracking wheels
            // wheel track  = distance between 2 wheels on either side

            // .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
            // .buildOdometry();
            .build();

    auto driveOdom = ChassisControllerBuilder()
                         .withMotors(
                             {-1, 2}, // Left motors are 1 & 2
                             {3, -4}) // Right motors are 3 & 4

                         // Blue gearset, external ratio of (84.0 / 30.0), 4 inch wheel diameter, 11.5 inch wheel track
                         .withDimensions({AbstractMotor::gearset::blue, (84.0 / 30.0)}, {{4_in, 11.5_in}, imev5BlueTPR}) //NEED SETUP!

                         // track        = distance between 2 tracking wheels
                         // wheel track  = distance between 2 wheels on either side

                         // Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
                         .withOdometry({{2.75_in, 14_cm}, quadEncoderTPR}) //NEED SETUP!
                         .buildOdometry();

    void init()
    {
        LF.setBrakeMode(AbstractMotor::brakeMode::hold);
        LB.setBrakeMode(AbstractMotor::brakeMode::hold);
        RF.setBrakeMode(AbstractMotor::brakeMode::hold);
        RB.setBrakeMode(AbstractMotor::brakeMode::hold);
    }

    void opcontrol()
    {

        Controller master;

        // Arcade drive with the left stick.
        driveOdom->getModel()->arcade(master.getAnalog(ControllerAnalog::leftY),
                                      master.getAnalog(ControllerAnalog::rightX));
    }
}
