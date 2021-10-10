#include "main.h"

    Motor LF(1), LB(-2), RF(3), RB(-4);

    MotorGroup left({LF, LB});
    MotorGroup right({RF, RB});

    ADIEncoder leftEncoder('A', 'B');
    ADIEncoder rightEncoder('C', 'D', true);
    ADIEncoder middleEncoder('E', 'F');

    // pros::Imu inertial();

    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
    auto driveOdom = ChassisControllerBuilder()
                         .withMotors(
                             {-1, 2}, // Left motors are 1 & 2
                             {3, -4}) // Right motors are 3 & 4

                         // Blue gearset, external ratio of (36.0 / 84.0), 4 inch wheel diameter, 35.4 cm wheel track
                         .withDimensions({AbstractMotor::gearset::blue, (36.0 / 84.0)}, {{4_in, 35.4_cm}, imev5BlueTPR})

                         // track        = distance between the center of the 2 tracking wheels
                         // wheel track  = distance between the center of the 2 wheels on either side
                        
                         .withSensors(leftEncoder, rightEncoder, middleEncoder)
                         // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
                        // specify the middle encoder distance (1 in) and diameter (2.75 in)
                         .withOdometry({{2.75_in, 13.2_cm, 0_in, 2.75_in}, quadEncoderTPR}) 
                         .buildOdometry();

namespace drive
{

    void init()
    {
        LF.setBrakeMode(AbstractMotor::brakeMode::coast);
        LB.setBrakeMode(AbstractMotor::brakeMode::coast);
        RF.setBrakeMode(AbstractMotor::brakeMode::coast);
        RB.setBrakeMode(AbstractMotor::brakeMode::coast);

        leftEncoder.reset();
        rightEncoder.reset();
        middleEncoder.reset();
    }

    void opcontrol()
    {

        Controller master;

        // Arcade drive with the left stick.
        driveOdom->getModel()->arcade(master.getAnalog(ControllerAnalog::rightY),
                                      master.getAnalog(ControllerAnalog::leftX));
    }

}

namespace auton
{
    void redLeft(){
        driveOdom->setState({0_in, 0_in, 0_deg});
        driveOdom->driveToPoint({0_in, 160_cm});
    }
    
}
