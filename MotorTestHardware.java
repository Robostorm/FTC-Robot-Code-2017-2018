package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andrew on 9/10/17.
 */

//Hardware class for Team 12601's Relic Recovery robot (2017-2018)
//The purpose of this class is to define the hardware used by the robot so it can be referenced in the OpMode classes
public class MotorTestHardware
{
    public DcMotor motor = null;

    //DcMotorEx exMotor = (DcMotorEx) motor;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    //Constructor
    public MotorTestHardware(){}

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        //Initialize hardware
        motor = hwMap.dcMotor.get("motor");
        //DcMotorEx exMotor = (DcMotorEx) motor;
        //set motors to drive forwards
        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sets motors to brake mode
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
        {
            try
            {
                Thread.sleep(remaining);
            }
            catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
