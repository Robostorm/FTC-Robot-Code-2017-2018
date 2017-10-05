package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andrew on 9/30/17.
 */

@TeleOp(name = "MotorTestTeleop")
public class MotorTestTeleop extends OpMode
{
    MotorTestHardware robot = new MotorTestHardware();
    private ElapsedTime runtime1 = new ElapsedTime();
    private ElapsedTime accelTime = new ElapsedTime();

    private final int GLYPH_ARM_SPEED_UPDATE_MILLIS = 100;
    private final double GLYPH_ARM_MAX_SPEED = 1.0;
    private final int ACCEL_TIME = 2500;
    private final double SPEED_INCREMENT = GLYPH_ARM_MAX_SPEED / (ACCEL_TIME / GLYPH_ARM_SPEED_UPDATE_MILLIS);
    private GlyphArmState prevArmState = GlyphArmState.START;
    private GlyphArmState currentArmState = GlyphArmState.FRONT2;
    private double armMotorSpeed = 0;
    int deccelStartPos;
    boolean hasCalcDeccelStartPos = false;

    public void init()
    {
        robot.init(hardwareMap);
    }

    public void loop()
    {
        /*if(robot.motor.getCurrentPosition() < 5000 && motorSpeed < 1)
        {
            motorSpeed += 0.05;
        }
        if(robot.motor.getCurrentPosition() >= 5000)
        {
            motorSpeed -= 0.05;
        }
        robot.motor.setPower(motorSpeed);
        telemetry.addData("motorSpeed", motorSpeed);
        telemetry.addData("encoder", robot.motor.getCurrentPosition());
        telemetry.update();
        runtime.reset();
        while(runtime.milliseconds() < 200){}*/

        if(runtime1.milliseconds() >= GLYPH_ARM_SPEED_UPDATE_MILLIS)
        {
            GlyphArmSpeedUpdate();
            runtime1.reset();
        }

        if(!(robot.motor.isBusy()))
        {
            accelTime.reset();
        }

        if(!(robot.motor.isBusy()))
        {
            robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor.setTargetPosition(currentArmState.getArmEncoderPos());
            //robot.motor.setPower(0);
            hasCalcDeccelStartPos = false;
        }



        telemetry.addData("encoder", robot.motor.getCurrentPosition());
        telemetry.addData("speed", armMotorSpeed);
        telemetry.addData("acceltime", accelTime.milliseconds());
        telemetry.addData("deccelstartpos", deccelStartPos);
        telemetry.update();
    }

    public void GlyphArmSpeedUpdate()
    {
        int midPos = (currentArmState.getArmEncoderPos() + prevArmState.getArmEncoderPos()) / 2;

        if(accelTime.milliseconds() < ACCEL_TIME && robot.motor.getCurrentPosition() < midPos)
        {
            armMotorSpeed += SPEED_INCREMENT;
        }
        else
        {
            if(!(hasCalcDeccelStartPos))
            {
                int afterAccelPos = robot.motor.getCurrentPosition();
                deccelStartPos = currentArmState.getArmEncoderPos() - afterAccelPos;
                hasCalcDeccelStartPos = true;
            }

            if(robot.motor.getCurrentPosition() >= deccelStartPos && armMotorSpeed > SPEED_INCREMENT)
            {
                armMotorSpeed -= SPEED_INCREMENT;
            }
        }

        robot.motor.setPower(armMotorSpeed);
    }
}
