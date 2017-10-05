package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andrew on 9/10/17.
 */

//Teleop Opmode class for Team 12601's Relic Recovery Robot (2017-2018)

@TeleOp(name="RRBotTeleop2")
public class RRBotTeleop2 extends OpMode
{
    //construct an RRBotHardware object to reference its stuff
    RRBotHardware2 robot = new RRBotHardware2();
    private ElapsedTime runtime1 = new ElapsedTime();
    private ElapsedTime accelTime = new ElapsedTime();

    private final double GLYPH_WRIST_SPEED = 0.3;
    private final double GRABBER_OPEN_POS = 0;
    private final double GRABBER_CLOSE_POS = 1;
    private final double GAMEPAD_TRIGGER_THRESHOLD = 0.3;
    private final int GLYPH_ARM_SPEED_UPDATE_MILLIS = 100;
    private final double GLYPH_ARM_MAX_SPEED = 0.5;
    private final double GLYPH_ARM_SLOW_SPEED = 0.2;
    private final double GLYPH_ARM_SLOW_DIST = 400;
    private final int ACCEL_TIME = 2500;
    private final double SPEED_INCREMENT = GLYPH_ARM_MAX_SPEED / (ACCEL_TIME / GLYPH_ARM_SPEED_UPDATE_MILLIS);
    private final double HOME_ARM_SPEED = 0.2;
    private final double HOME_ARM_SPEED_SLOW = 0.1;
    private final int HOME_ARM_UP_POS = 1000;
    private final int HOME_ARM_POS_THRESHOLD = 50;
    private final double HOME_WRIST_SPEED = 0.1;
    private final double ARM_OVERRIDE_STICK_THRESHOLD = 0.1;
    private final int ARM_POS_THRESHOLD = 100;
    private final int WRIST_POS_THRESHOLD = 100;
    private GlyphArmState prevArmState = GlyphArmState.START;
    private GlyphArmState currentArmState = GlyphArmState.START;
    private boolean prevStartLimitState = false;
    private boolean prevEndLimitState = false;
    private double armMotorSpeed = 0;
    private boolean hasHomed = false;
    private boolean hasHomed1 = false;
    private boolean hasHomedUp = false;
    private boolean startLimitState;
    private boolean endLimitState;
    private int deccelStartPos;
    private boolean hasCalcDeccelStartPos = false;
    private boolean hasOverridden = false;
    private GlyphArmState beforeEndLimitState = currentArmState;

    @Override
    public void init()
    {
        //initialize hardware variables by calling the init function of the RRBotHardware class via the robot object
        robot.init(hardwareMap);

        telemetry.addData("Robot","is init");

        runtime1.reset();
    }

    @Override
    public void loop()
    {
        //MecanumDrive();

        startLimitState = !(robot.glyphStartLimit.getState());
        endLimitState = !(robot.glyphEndLimit.getState());

        if(!(robot.glyphArmMotor1.isBusy()))
        {
            accelTime.reset();
        }

        if(hasHomed)
        {
            GlyphArmUpdate();
            GrabberUpdate();

            if(runtime1.milliseconds() >= GLYPH_ARM_SPEED_UPDATE_MILLIS)
            {
                GlyphArmSpeedUpdate();
                runtime1.reset();
            }
        }
        else
        {
            HomeArm();
        }

        //GlyphArmUpdate();

        //robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.glyphArmMotor1.setPower(inputFunction(-gamepad2.left_stick_y));

        getEncoderPositions();
    }

    public void MecanumDrive()
    {
        //remap input values using a function
        double leftX = inputFunction(gamepad1.left_stick_x);
        double leftY = inputFunction(-gamepad1.left_stick_y);
        double rightX = inputFunction(gamepad1.right_stick_x);
        double rightY = inputFunction(-gamepad1.right_stick_y);

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

        telemetry.addData("v1", v1);
        telemetry.addData("v2", v2);
        telemetry.addData("v3", v3);
        telemetry.addData("v4", v4);
        telemetry.addData("magnitude", r);
        telemetry.addData("angle", robotAngle);
        telemetry.addData("rotation", rightX);
    }

    public double inputFunction(double input)
    {
        return (input * input * input);

        /*if(input < 0)
        {
            return (input * input) * -1;
        }
        else
            return (input * input);*/
    }

    public void JewelArmUpdate()
    {
        if(gamepad1.a)
        {
            robot.jewelArmServo.setPosition(robot.JEWEL_ARM_SERVO_START_POS);
        }
    }

    public void GrabberUpdate()
    {
        if(Math.abs(robot.glyphWristMotor.getCurrentPosition() - GlyphWristState.FRONT.getWristEncoderPos()) < WRIST_POS_THRESHOLD)
        {
            if (gamepad1.left_bumper)
            {
                robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
            }
            else if (gamepad1.right_bumper)
            {
                robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
            }
        }
        else if(Math.abs(robot.glyphWristMotor.getCurrentPosition() - GlyphWristState.BACK.getWristEncoderPos()) < WRIST_POS_THRESHOLD)
        {
            if (gamepad1.left_bumper)
            {
                robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
            }
            else if (gamepad1.right_bumper)
            {
                robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
            }
        }
    }

    public void HomeArm()
    {
        if(!hasHomed1)
        {
            if(startLimitState)
            {
                robot.glyphArmMotor1.setPower(0);
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //hasHomed1 = true;
                hasHomed = true;
            }
            else
            {
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.glyphArmMotor1.setPower(-HOME_ARM_SPEED);
            }
        }
        /*else
        {
            if(Math.abs(robot.glyphArmMotor1.getCurrentPosition() - HOME_ARM_UP_POS) < HOME_ARM_POS_THRESHOLD)
            {
                hasHomedUp = true;
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.glyphArmMotor1.setPower(-HOME_ARM_SPEED_SLOW);
            }
            else
            {
                if(!hasHomedUp)
                {
                    robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.glyphArmMotor1.setPower(HOME_ARM_SPEED);
                    robot.glyphArmMotor1.setTargetPosition(HOME_ARM_UP_POS);
                }
            }
        }

        if(hasHomedUp)
        {
            if(startLimitState)
            {
                robot.glyphArmMotor1.setPower(0);
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hasHomed = true;
            }
        }*/
    }

    public void GlyphArmUpdate()
    {
        if(gamepad2.start)
        {
            MoveGlyphArmToState(GlyphArmState.START);
            MoveGlyphWristToState(GlyphWristState.START);
        }
        else if(gamepad2.a)
        {
            MoveGlyphArmToState(GlyphArmState.FRONT1);
        }
        else if(gamepad2.b)
        {
            MoveGlyphArmToState(GlyphArmState.FRONT2);
        }
        else if(gamepad2.x)
        {
            MoveGlyphArmToState(GlyphArmState.FRONT3);
        }
        else if(gamepad2.y)
        {
            MoveGlyphArmToState(GlyphArmState.FRONT4);
        }
        else if(gamepad2.dpad_down)
        {
            MoveGlyphArmToState(GlyphArmState.BACK1);
        }
        else if(gamepad2.dpad_right)
        {
            MoveGlyphArmToState(GlyphArmState.BACK2);
        }
        else if(gamepad2.dpad_left)
        {
            MoveGlyphArmToState(GlyphArmState.BACK3);
        }
        else if(gamepad2.dpad_up)
        {
            MoveGlyphArmToState(GlyphArmState.BACK4);
        }
        else if(gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD)
        {
            MoveGlyphWristToState(GlyphWristState.FRONT);
        }
        else if(gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD)
        {
            MoveGlyphWristToState(GlyphWristState.BACK);
        }

        if(Math.abs(robot.glyphArmMotor1.getCurrentPosition() - currentArmState.getArmEncoderPos()) < ARM_POS_THRESHOLD)
        {
            prevArmState = currentArmState;
        }

        /*if(Math.abs(gamepad2.left_stick_y) > ARM_OVERRIDE_STICK_THRESHOLD)
        {
            if(!(hasOverridden))
            {
                robot.glyphArmMotor1.setPower(0);
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hasOverridden = true;
            }
        }*/
    }

    public void GlyphArmSpeedUpdate()
    {/*
        if(prevArmState != currentArmState)
        {
            if(prevArmState.isFrontPos() != currentArmState.isFrontPos())
            {
                int midPos = (currentArmState.getArmEncoderPos() + prevArmState.getArmEncoderPos()) / 2;

                if(currentArmState.getArmEncoderPos() > prevArmState.getArmEncoderPos())
                {
                    if (accelTime.milliseconds() < ACCEL_TIME && robot.glyphArmMotor1.getCurrentPosition() < midPos)
                    {
                        armMotorSpeed += SPEED_INCREMENT;
                    }
                    else
                    {
                        if (!(hasCalcDeccelStartPos))
                        {
                            int afterAccelPos = robot.glyphArmMotor1.getCurrentPosition();
                            deccelStartPos = currentArmState.getArmEncoderPos() - afterAccelPos;
                            hasCalcDeccelStartPos = true;
                        }

                        if (robot.glyphArmMotor1.getCurrentPosition() >= deccelStartPos && armMotorSpeed > SPEED_INCREMENT)
                        {
                            armMotorSpeed -= SPEED_INCREMENT;
                        }
                    }
                }
                else
                {
                    if (accelTime.milliseconds() < ACCEL_TIME && robot.glyphArmMotor1.getCurrentPosition() > midPos)
                    {
                        armMotorSpeed += SPEED_INCREMENT;
                    }
                    else
                    {
                        if (!(hasCalcDeccelStartPos))
                        {
                            int afterAccelPos = robot.glyphArmMotor1.getCurrentPosition();
                            deccelStartPos = currentArmState.getArmEncoderPos() - afterAccelPos;
                            hasCalcDeccelStartPos = true;
                        }

                        if (robot.glyphArmMotor1.getCurrentPosition() >= deccelStartPos && armMotorSpeed > SPEED_INCREMENT)
                        {
                            armMotorSpeed -= SPEED_INCREMENT;
                        }
                    }
                }
            }
            else
            {
                armMotorSpeed = GLYPH_ARM_MAX_SPEED;
            }
        }
        */
        armMotorSpeed = GLYPH_ARM_MAX_SPEED;

        if(currentArmState == GlyphArmState.START || currentArmState == GlyphArmState.FRONT1)
        {
            if(robot.glyphArmMotor1.getCurrentPosition() <= GLYPH_ARM_SLOW_DIST)
            {
                armMotorSpeed = GLYPH_ARM_SLOW_SPEED;
            }
        }
        else if(currentArmState == GlyphArmState.BACK1)
        {
            if(robot.glyphArmMotor1.getCurrentPosition() >= GlyphArmState.BACK1.getArmEncoderPos() - GLYPH_ARM_SLOW_DIST)
            {
                armMotorSpeed = GLYPH_ARM_SLOW_SPEED;
            }
        }

        if(!prevStartLimitState && startLimitState)
        {
            armMotorSpeed = 0;
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            prevStartLimitState = true;
        }
        else if(prevStartLimitState && !(startLimitState))
        {
            prevStartLimitState = false;
        }



        if(!(prevEndLimitState) && endLimitState)
        {
            //armMotorSpeed = 0;
            beforeEndLimitState = currentArmState;
            prevEndLimitState = true;
        }
        else if(prevEndLimitState && !(endLimitState))
        {
            prevEndLimitState = false;
        }

        if(endLimitState && currentArmState == beforeEndLimitState)
        {
            armMotorSpeed = 0;
        }

        robot.glyphArmMotor1.setPower(armMotorSpeed);
    }

    public void MoveGlyphArmToState(GlyphArmState state)
    {
        currentArmState = state;
        hasOverridden = false;

        if(prevArmState == GlyphArmState.START)
        {
            if(state != GlyphArmState.START && state.isFrontPos())
            {
                MoveGlyphWristToState(GlyphWristState.FRONT);
            }

            if(!state.isFrontPos())
            {
                MoveGlyphWristToState(GlyphWristState.BACK);
            }
        }

        robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.glyphArmMotor1.setTargetPosition(state.getArmEncoderPos());
        //robot.glyphArmMotor1.setPower(GLYPH_ARM_MAX_SPEED);
        hasCalcDeccelStartPos = false;
    }

    public void MoveGlyphWristToState(GlyphWristState state)
    {
        robot.glyphWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.glyphWristMotor.setTargetPosition(state.getWristEncoderPos());
        robot.glyphWristMotor.setPower(GLYPH_WRIST_SPEED);
    }

    public void getEncoderPositions()
    {
        telemetry.addData("ArmMotorPosition", robot.glyphArmMotor1.getCurrentPosition());
        telemetry.addData("WristMotorPosition", robot.glyphWristMotor.getCurrentPosition());
        telemetry.addData("switch", startLimitState);
        telemetry.update();
    }
}

