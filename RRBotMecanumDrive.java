package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by andrew on 10/8/17.
 */

public class RRBotMecanumDrive
{
    RRBotHardware robot;

    private ElapsedTime autoMoveTime = new ElapsedTime();

    private final double driveDeadBand = 0.6;
    private boolean isAutoMove = false;
    private double autoTime;

    //constructor gets hardware object from teleop class when it is constructed
    public RRBotMecanumDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    public double[] calcVelocities(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        double moveX = rightX;
        double moveY1 = leftY;
        double turn = leftX;
        double moveY2 = rightY;

        //remap input values using a function
        if(doFunction)
        {
            moveX = inputFunction(moveX);
            moveY1 = inputFunction(moveY1);
            turn = inputFunction(turn);
            moveY2 = inputFunction(moveY2);
        }

        double v1 = moveY1 + moveX + turn + moveY2;
        double v2 = moveY1 - moveX - turn + moveY2;
        double v3 = moveY1 + moveX - turn + moveY2;
        double v4 = moveY1 - moveX + turn + moveY2;

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

    public void setMotorPower(double leftX, double leftY, double rightX, double rightY, boolean doFunction)
    {
        double[] velocities = calcVelocities(leftX, leftY, rightX, rightY, doFunction);

        //set the motor power
        robot.frontLeftMotor.setPower(velocities[0]);
        robot.frontRightMotor.setPower(velocities[1]);
        robot.rearLeftMotor.setPower(velocities[2]);
        robot.rearRightMotor.setPower(velocities[3]);
    }

    public double inputFunction(double input)
    {
        double value = input;

        //f(x) = x^3
        value = value * value * value;

        /*if(value > 0)
        {
            Range.clip(value, driveDeadBand, 1);
        }
        else if(value < 0)
        {
            Range.clip(value, -1, -driveDeadBand);
        }*/
        /*if(Math.abs(value - 0) < 0.01)
        {
            value = 0;
        }*/

        return value;

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

    public void AutoMove(double speed, double time)
    {
        isAutoMove = true;
        autoTime = time;

        autoMoveTime.reset();

        robot.rearRightMotor.setPower(speed);
        robot.rearLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.frontLeftMotor.setPower(speed);
    }

    public void AutoMoveEndCheck()
    {
        if(autoMoveTime.milliseconds() >= autoTime)
        {
            isAutoMove = false;
        }
    }

    public boolean getIsAutoMove()
    {
        return isAutoMove;
    }
}
