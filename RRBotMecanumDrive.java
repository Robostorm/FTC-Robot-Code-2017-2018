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

        /*
        //calculate the magnitude of the vector
        double r = Math.hypot(leftX, leftY);

        //calculate the angle of the vector
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        //calculate the motor vectors and add rotation and right stick turbo
        double v1 = r * Math.cos(robotAngle) + rightX + rightY;
        double v2 = r * Math.sin(robotAngle) - rightX + rightY;
        double v3 = r * Math.sin(robotAngle) + rightX + rightY;
        double v4 = r * Math.cos(robotAngle) - rightX + rightY;*/

        //experimental
        /*v1 *= Math.sqrt(2);
        v2 *= Math.sqrt(2);
        v3 *= Math.sqrt(2);
        v4 *= Math.sqrt(2);
        if(v1 > 1)
            v1 = 1;
        if(v2 > 1)
            v2 = 1;
        if(v3 > 1)
            v3 = 1;
        if(v4 > 1)
            v4 = 1;*/

        double v1 = leftY + rightX + leftX + rightY;
        double v2 = leftY - rightX - leftX + rightY;
        double v3 = leftY + rightX - leftX + rightY;
        double v4 = leftY - rightX + leftX + rightY;

        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1)
        {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }


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
