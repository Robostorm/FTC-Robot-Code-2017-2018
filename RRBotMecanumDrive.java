package org.firstinspires.ftc.teamcode;

/**
 * Created by andrew on 10/8/17.
 */

public class RRBotMecanumDrive
{
    RRBotHardware robot;

    //constructor gets hardware object from teleop class when it is constructed
    public RRBotMecanumDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    public double[] calcVelocities(double moveX, double moveY, double turn, double highSpeed, boolean doFunction)
    {
        double leftX = moveX;
        double leftY = moveY;
        double rightX = turn;
        double rightY = highSpeed;

        //remap input values using a function
        if(doFunction)
        {
            leftX = inputFunction(moveX);
            leftY = inputFunction(moveY);
            rightX = inputFunction(turn);
            rightY = inputFunction(highSpeed);
        }

        //calculate the magnitude of the vector
        double r = Math.hypot(leftX, leftY);

        //calculate the angle of the vector
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        //calculate the motor vectors and add rotation and right stick turbo
        final double v1 = r * Math.cos(robotAngle) + rightX + rightY;
        final double v2 = r * Math.sin(robotAngle) - rightX + rightY;
        final double v3 = r * Math.sin(robotAngle) + rightX + rightY;
        final double v4 = r * Math.cos(robotAngle) - rightX + rightY;

        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    public void setMotorPower(double moveX, double moveY, double turn, double highSpeed, boolean doFunction)
    {
        double[] velocities = calcVelocities(moveX, moveY, turn, highSpeed, doFunction);

        //set the motor power
        robot.frontLeftMotor.setPower(velocities[0]);
        robot.frontRightMotor.setPower(velocities[1]);
        robot.rearLeftMotor.setPower(velocities[2]);
        robot.rearRightMotor.setPower(velocities[3]);
    }

    public double inputFunction(double input)
    {
        //f(x) = x^3
        return (input * input * input);

        //f(x) = x^2
        /*double value = input * input;
        if(input < 0)
        {
            return value * -1;
        }
        else
        {
            return value;
        }*/
    }
}
