#include "main.h"

Motor LF(-1), LB(2), RF(3), RB(-4);

Motor lift(7);
Motor claw(-8);

MotorGroup left({LF, LB});
MotorGroup right({RF, RB});

ADIEncoder middleEncoder('A', 'B');
ADIEncoder leftEncoder('C', 'D', true);
ADIEncoder rightEncoder('E', 'F', false);
IMU inertial(11);

// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto driveOdom = ChassisControllerBuilder()
                     //  .withMotors(-1,3)
                     .withMotors(
                         {-1, 2}, // Left motors are 1 & 2
                         {3, -4}) // Right motors are 3 & 4

                     .withGains(
                         {0.00075, 0.00000, 0}, // Distance controller gains
                         {0.00025, 0.00000, 0}, // Turn controller gains
                         {0.00025, 0.00000, 0}  // Angle controller gains (helps drive straight)
                         )

                     // Blue gearset, external ratio of (36.0 / 84.0), 4 inch wheel diameter, 35.4 cm wheel track

                     .withDimensions({AbstractMotor::gearset::blue, (84.0 / 36.0)}, {{4_in, 35.4_cm}, imev5BlueTPR})

                     // track        = distance between the center of the 2 tracking wheels
                     // wheel track  = distance between the center of the 2 wheels on either side

                     .withSensors(leftEncoder, rightEncoder /*, middleEncoder*/)
                     // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
                     // specify the middle encoder distance (1 in) and diameter (2.75 in)
                     .withOdometry({{2.75_in, 13.2_cm /*, 1_in, 2.75_in*/}, quadEncoderTPR})
                     .buildOdometry();

namespace drive
{

    void init()
    {
        LF.setBrakeMode(AbstractMotor::brakeMode::coast);
        LB.setBrakeMode(AbstractMotor::brakeMode::coast);
        RF.setBrakeMode(AbstractMotor::brakeMode::coast);
        RB.setBrakeMode(AbstractMotor::brakeMode::coast);

        pid::resetDriveEncoders();
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
    void redLeft()
    {
        pid::resetDriveEncoders();

        claw.moveVoltage(-12000);
        pid::forwardPD(1980);
        claw.moveVoltage(12000);
        lift.moveRelative(900, 127);
        pid::delaySeconds(0.3);
        pid::forwardPD(-1000);
        pid::stop();
    }

}

namespace pid
{

    void delaySeconds(double seconds) { pros::delay(seconds * 1000); }

    void calibrate()
    {
        pros::lcd::initialize();
        pros::lcd::print(0, "calibrate : initialize (done)  ");

        int err = inertial.reset();
        pros::lcd::print(0, "calibrate : reset (done)  %d", err);
        int n = 0;
        while (inertial.isCalibrating())
        {
            pros::lcd::print(0, "calibrate : is_calibrating : %d", n++);

            pros::delay(20);
        }
        pros::lcd::print(0, "calibrate : is_calibrating (done)  ");

        delaySeconds(0.5);
    }

    void inertialReset()
    {
        inertial.reset();
    }

    void stop()
    {
        left.moveVelocity(0);
        right.moveVelocity(0);
    }

    void stop(double seconds)
    {
        left.moveVelocity(0);
        right.moveVelocity(0);
        pros::delay(seconds * 1000);
    }

    void resetDriveEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        middleEncoder.reset();
    }

    double avgDriveEncoders()
    {
        return (fabs(leftEncoder.get()) + fabs(rightEncoder.get())) / 2;
    }

    void turn(double power)
    {
        left.moveVelocity(power);
        right.moveVelocity(-power);
    }

    void move(int power)
    {
        left.moveVelocity(power);
        right.moveVelocity(power);
    }

    void forwardPD(int units)
    { // power in positive, units in positive or negative
        resetDriveEncoders();
        int direction = abs(units) / units;
        double rotation = inertial.get();
        int power = 0;
        int setPoint = abs(units);

        double kP = 0.5;
        double kD = 0.2;

        while (avgDriveEncoders() < abs(units))
        {
            int tune = 5;
            double tolerance = 0.3;

            double error = setPoint - avgDriveEncoders();
            double prevError = 0;
            double derivative;

            power = direction * (error * kP + derivative * kD);

            // pros::lcd::print(0, "Get encoder  >> %f\n",
            // fabs(driveLF.get_position()));
            pros::lcd::print(0, "rotation  >> %5.2f", inertial.get());
            pros::lcd::print(1, "encoder value  >> %5.2f", avgDriveEncoders());
            pros::lcd::print(2, "error   >> %5.2f", error);

            derivative = error - prevError;
            prevError = error;

            if (inertial.get() > rotation + tolerance)
            {
                left.moveVelocity(power - tune);
                right.moveVelocity(power + tune);
            }
            else if (inertial.get() < rotation - tolerance)
            {
                left.moveVelocity(power + tune);
                right.moveVelocity(power - tune);
            }
            else
            {
                left.moveVelocity(power);
                right.moveVelocity(power);
            }

            pros::delay(10);
        }
        stop(0);
    }

    void rotateDegreesPD(double deg)
    {
        double tolerance = 4;
        double bias = 0;

        double error = 0;
        double derivative;
        double prevError = 0;

        double kP = 3.2;
        double kD = 2;

        while (true)
        {
            double heading = inertial.get();

            error = heading - deg;

            if (heading < -360 || heading > 360)
            {
                continue;
            }
            bool turnRight = false;
            double error = heading - deg;
            // double error = 60;
            if (error > 0)
            {
                if (error > 180)
                {
                    // right
                    error = 360 - error;
                    turnRight = true;
                }
                else
                {
                    // left
                    turnRight = false;
                }
            }
            else
            {
                if (error < -180)
                {
                    // left
                    error = 360 + error;
                    turnRight = false;
                }
                else
                {
                    // right
                    error = error * -1;
                    turnRight = true;
                }
            }

            // error += bias;
            if (turnRight)
                pros::lcd::print(3, "error  >> %5.2f   RIGHT", error);
            else
                pros::lcd::print(3, "error  >> %5.2f   LEFT", error);

            pros::lcd::print(4, "tole  >> %5.2f", tolerance);

            if (error > tolerance)
            {
                double pow = 0;
                if (turnRight == false)
                {
                    // turn A(error) Left
                    pow = -error * kP - derivative * kD;
                    // pow = -20 + error / -2;
                }
                else
                {
                    // turn B(error) right
                    // pow = 20 + error / 2;
                    pow = error * kP + derivative * kD;
                }
                turn(pow);
                pros::lcd::print(2, "TURN  >> %5.2f", pow);
            }
            else
            {
                turn(0);
                break;
            }
            derivative = error - prevError;
            prevError = error;
            pros::delay(15);
            pros::lcd::print(0, "heading  >> %5.2f", heading);
            pros::lcd::print(1, "target   >> %5.2f", deg);
            move(0);
        }
    }
    
}