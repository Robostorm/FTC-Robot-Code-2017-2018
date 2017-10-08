package org.firstinspires.ftc.teamcode;

/**
 * Created by andrew on 10/8/17.
 */

public class RRBotMecanumDrive
{
    RRBotHardware2 robot;

    //constructor gets hardware object from teleop class when it is constructed
    public RRBotMecanumDrive(RRBotHardware2 robot)
    {
        this.robot = robot;
    }

    public void setMotorPower(double joyLX, double joyLY, double joyRX, double joyRY)
    {
        //remap input values using a function
        double leftX = inputFunction(joyLX);
        double leftY = inputFunction(-joyLY);
        double rightX = inputFunction(joyRX);
        double rightY = inputFunction(-joyRY);

        //calculate the magnitude of the vector
        double r = Math.hypot(leftX, leftY);

        //calculate the angle of the vector
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        //calculate the motor vectors and add rotation and right stick turbo
        final double v1 = r * Math.cos(robotAngle) + rightX + rightY;
        final double v2 = r * Math.sin(robotAngle) - rightX + rightY;
        final double v3 = r * Math.sin(robotAngle) + rightX + rightY;
        final double v4 = r * Math.cos(robotAngle) - rightX + rightY;

        //set the motor power
        robot.frontLeftMotor.setPower(v1);
        robot.frontRightMotor.setPower(v2);
        robot.rearLeftMotor.setPower(v3);
        robot.rearRightMotor.setPower(v4);
    }

    public double inputFunction(double input)
    {
        //f(x) = x^3
        return (input * input * input);
    }
}
