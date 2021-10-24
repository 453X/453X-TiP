#include "main.h"

Motor LF(-1), LB(2), RF(3), RB(-4), liftBack(5), roller(-6), lift(7), claw(-8);

ADIButton frontLimit('G');
ADIButton backLimit('H');

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

        claw.setBrakeMode(AbstractMotor::brakeMode::hold);
        claw.tarePosition();
        claw.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

        roller.setBrakeMode(AbstractMotor::brakeMode::coast);

        liftBack.setBrakeMode(AbstractMotor::brakeMode::brake);

        pid::resetDriveEncoders();
    }

    void opcontrol()
    {

        Controller master;

        // Arcade drive with the left stick.
        driveOdom->getModel()->arcade(master.getAnalog(ControllerAnalog::rightY),
                                      master.getAnalog(ControllerAnalog::leftX));
    }

    void turn(double power)
    {
        left.moveVelocity(power);
        right.moveVelocity(-power);
    }

    void turn (int degrees, int power)
    {
        left.moveRelative(degrees, power);
        right.moveRelative(degrees, power);
    }

    void drive(int power)
    {
        left.moveVelocity(power);
        right.moveVelocity(power);
    }

    void drive(int degrees, int power)
    {
        left.moveRelative(degrees, power);
        right.moveRelative(degrees, power);
    }

}

namespace auton
{
    void redRight()
    {
        // debug
        pros::lcd::initialize();

        pid::resetDriveEncoders();

        // open
        claw_open(true);
        pid::drivePID(1980);
        claw_open(false);

        pid::delaySeconds(10);
        pid::delaySeconds(0.1);

        // lift yellow goal
        lift.moveRelative(900, 127);
        pid::delaySeconds(0.3);

        // back
        pid::drivePID(-950);

        // release yellow goal
        pid::turnPID(-90);
        lift.moveRelative(-900, 127);
        claw_open(true);
        pid::delaySeconds(0.1);

        // back and lift red goal
        backLift_down();
        pid::drivePID(-350);
        backLift_up();
        lift.moveRelative(900, 127);
        pid::delaySeconds(1.2);

        //
        pid::turnPID(0);

        // roller
        pid::delaySeconds(3);
        roller_on();
        // to do : foward speed should be slower when rollering
        // pid::forwardPD(1500);
        drive::drive(8000, 200);
        pid::delaySeconds(2);
        roller_off();

        pid::stop();
    }

    void claw_open(bool open)
    {
        if (open)
        {
            int err = claw.moveAbsolute(-250, 100);
            pros::lcd::print(6, "claw  open>> %5.2f  err:%d", claw.getPosition(), err);
        }
        else
        {
            int err = claw.moveVoltage(12000);
            pros::lcd::print(7, "claw close>> %5.2f  err:%d", claw.getPosition(), err);
        }
    }

    void backLift_down()
    {
        while (!frontLimit.isPressed())
        {
            liftBack.moveVoltage(12000);
        }
        liftBack.moveVelocity(0);
    }
    void backLift_up()
    {
        liftBack.moveRelative(-2500, 100);
    }

    void roller_on()
    {
        roller.moveVoltage(12000);
    }

    void roller_off()
    {
        roller.moveVelocity(0);
    }
}

namespace pid
{
    void testRotate()
    {
        pros::lcd::initialize();

        // test right / left

        // for (size_t i = 0; i < 10; i++)
        // {
        //     rotateDegreesPD(60);
        //     delaySeconds(5);
        //     rotateDegreesPD(-60);
        //     delaySeconds(5);
        // }

        for (int i = 45; i <= 180; i += 45)
        {
            turnPID(i);
            delaySeconds(5);
            turnPID(-1 * i);
            delaySeconds(5);
        }

        // test right

        // for (size_t i = 0; i <= 10; i++)
        // {
        //     rotateDegreesPD(100);
        //     delaySeconds(3);
        // }

        // turn(50);
        // delaySeconds(1);
        // turn(-50);
        // delaySeconds(1);

        // turn(50);
        // delaySeconds(1);
        // turn(-50);
        // delaySeconds(1);

        // turn(50);
        // delaySeconds(1);
        // turn(-50);
        // delaySeconds(1);
    }

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

    

    void drivePID(int units)
    { // power in positive, units in positive or negative
        resetDriveEncoders();
        int direction = abs(units) / units;
        double rotation = inertial.get();
        int power = 0;
        int setPoint = abs(units);

        double kP = 0.65;
        double kD = 0.2;
        double kI = 0.03;

        double errorSum = 0;

        while (avgDriveEncoders() < abs(units))
        {
            int tune = 5;
            double tolerance = 0.3;

            double error = setPoint - avgDriveEncoders();
            if (error < 100)
                errorSum += error;
            double prevError = 0;
            double derivative;

            power = direction * (error * kP + derivative * kD + errorSum * kI);

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

    void turnPID(double deg)
    {
        double tolerance = 1;
        double bias = 0;

        double derivative;
        double prevError = 0;

        double kP = 3.2;
        double kD = 2;

        while (true)
        {
            double heading = inertial.get();
            if (heading < -360 || heading > 360)
            {
                continue;
            }
            bool turnRight = false;
            double error = deg - heading;
            pros::lcd::print(0, "heading  >> %5.2f", heading);
            pros::lcd::print(1, "target   >> %5.2f", deg);
            pros::lcd::print(2, "orignal error  >> %5.2f", error);

            if (error > 0)
            {
                if (error > 180)
                {
                    // left
                    error = 360 - error;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (error < -180)
                {
                    // right
                    error = 360 + error;
                    turnRight = true;
                }
                else
                {
                    // left
                    error = error * -1;
                    turnRight = false;
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
                double pow = error * kP + derivative * kD;
                if (turnRight == false)
                {
                    // turn A(error) Left
                    drive::turn(pow * -1);
                }
                else
                {
                    // turn B(error) right
                    drive::turn(pow);
                }
                pros::lcd::print(5, "TURN POW >> %5.2f", pow);
            }
            else
            {
                drive::turn(0);
                break;
            }
            derivative = error - prevError;
            prevError = error;
            pros::delay(15);

            drive::drive(0);
        }
    }

}