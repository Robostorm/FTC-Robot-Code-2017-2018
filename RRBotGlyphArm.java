package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andrew on 10/8/17.
 */

//class that controls the glyph arm
public class RRBotGlyphArm
{
    RRBotHardware2 robot;
    private ElapsedTime runtime1 = new ElapsedTime();
    private ElapsedTime accelTime = new ElapsedTime(); //not currently used

    //final variables for values that will not change
    private final double GLYPH_WRIST_SPEED = 1;
    private final double GRABBER_OPEN_POS = 0.8;
    private final double GRABBER_CLOSE_POS = 0.15;
    private final int GLYPH_ARM_SPEED_UPDATE_MILLIS = 100;
    private final double GLYPH_ARM_MAX_SPEED = 0.75;
    private final double GLYPH_ARM_SLOW_SPEED = 0.2;
    private final double GLYPH_ARM_SLOW_DIST = 300;
    private final int ACCEL_TIME = 2500; //not currently used
    private final double SPEED_INCREMENT = GLYPH_ARM_MAX_SPEED / (ACCEL_TIME / GLYPH_ARM_SPEED_UPDATE_MILLIS); //not currently used
    private final double HOME_ARM_SPEED = 0.2;
    private final double HOME_ARM_SPEED_SLOW = 0.075; //not currently used
    private final int HOME_ARM_UP_POS = 1000; //not currently used
    private final int HOME_ARM_POS_THRESHOLD = 50; //not currently used
    private final int ARM_POS_THRESHOLD = 100;
    private final int WRIST_POS_THRESHOLD = 100;

    private GlyphArmState prevArmState = GlyphArmState.START;
    private GlyphArmState currentArmState = GlyphArmState.START;
    private GlyphWristState prevWristState = GlyphWristState.START;
    private GlyphWristState currentWristState = GlyphWristState.START;
    private boolean prevStartLimitState = false;
    private boolean prevEndLimitState = false;
    private double armMotorSpeed = 0;
    private boolean hasHomed = false;
    private boolean hasHomed1 = false; //not currently used
    private boolean hasHomedUp = false; //not currently used
    private int deccelStartPos; //not currently used
    private boolean hasCalcDeccelStartPos = false; //not currently used
    private boolean hasOverridden = false; //not currently used
    private GlyphArmState beforeEndLimitState = currentArmState;

    //constructor gets hardware object from teleop class when it is constructed
    public RRBotGlyphArm(RRBotHardware2 robot)
    {
        this.robot = robot;
    }

    public void UpdateValues()
    {
        //set previous arm state equal to current arm state once the arm gets near its target
        if(Math.abs(robot.glyphArmMotor1.getCurrentPosition() - currentArmState.getArmEncoderPos()) < ARM_POS_THRESHOLD)
        {
            prevArmState = currentArmState;
        }

        //set previous wrist state equal to current wrist state once the arm gets wrist its target
        if(Math.abs(robot.glyphWristMotor.getCurrentPosition() - currentWristState.getWristEncoderPos()) < WRIST_POS_THRESHOLD)
        {
            prevWristState = currentWristState;
        }

        //call GlyphArmSpeedUpdate every GLYPH_ARM_SPEED_UPDATE_MILLIS milliseconds. this is used so motor accel can occur based on time
        if(runtime1.milliseconds() >= GLYPH_ARM_SPEED_UPDATE_MILLIS)
        {
            GlyphArmSpeedUpdate();
            runtime1.reset();
        }
    }

    public void MoveGlyphArmToState(GlyphArmState state)
    {
        //make sure neither grabber is open
        if(!isGrabberServo1Open() && !isGrabberServo2Open())
        {
            currentArmState = state;

            //use RUN_TO_POSITION mode to move the motor to desired position using PID
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.glyphArmMotor1.setTargetPosition(state.getArmEncoderPos());
        }
    }

    public void MoveGlyphWristToState(GlyphWristState state)
    {
        //make sure neither grabber is open
        if(!isGrabberServo1Open() && !isGrabberServo2Open())
        {
            currentWristState = state;

            //use RUN_TO_POSITION mode to move the motor to desired position using PID
            robot.glyphWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.glyphWristMotor.setTargetPosition(state.getWristEncoderPos());
            robot.glyphWristMotor.setPower(GLYPH_WRIST_SPEED);
        }
    }

    public void FlipWrist()
    {
        //if wrist is in the start or back positions, move it to the front position
        if(currentWristState == GlyphWristState.START || currentWristState == GlyphWristState.BACK)
        {
            MoveGlyphWristToState(GlyphWristState.FRONT);
        }
        //if wrist is in the front position, move it to the back position
        else if(currentWristState == GlyphWristState.FRONT)
        {
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
    }

    public void MoveGrabber()
    {
        //make sure the arm is not currently moving
        if(prevArmState == currentArmState)
        {
            //if wrist in in front position and arm is in a front position or wrist is in back position and arm is in a back position
            if((currentWristState == GlyphWristState.FRONT && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.BACK && !currentArmState.isFrontPos()))
            {
                if(isGrabberServo1Open())
                {
                    robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else
                {
                    robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
            //if wrist in in back position and arm is in a front position or wrist is in front position and arm is in a back position
            else if((currentWristState == GlyphWristState.BACK && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.FRONT && !currentArmState.isFrontPos()))
            {
                if(isGrabberServo2Open())
                {
                    robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else
                {
                    robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
        }
    }

    public void GlyphArmSpeedUpdate()
    {
        //set armMotorSpeed to max/normal speed, other stuff will change it to something else if necessary
        armMotorSpeed = GLYPH_ARM_MAX_SPEED;

        //if the arm is moving to the start or front_pickup positions, and in the encoder distance GLYPH_ARM_SLOW_DISTANCE before the target position, slow the arm down to GLYPH_ARM_SLOW_SPEED
        if(currentArmState == GlyphArmState.START || currentArmState == GlyphArmState.FRONT_PICKUP)
        {
            if(robot.glyphArmMotor1.getCurrentPosition() <= GLYPH_ARM_SLOW_DIST)
            {
                armMotorSpeed = GLYPH_ARM_SLOW_SPEED;
            }
        }
        //same but for back_pickup position
        else if(currentArmState == GlyphArmState.BACK_PICKUP)
        {
            if(robot.glyphArmMotor1.getCurrentPosition() >= GlyphArmState.BACK_PICKUP.getArmEncoderPos() - GLYPH_ARM_SLOW_DIST)
            {
                armMotorSpeed = GLYPH_ARM_SLOW_SPEED;
            }
        }

        //stop arm and reset encoder if start limit switch is pressed, only detect rising edge
        if(!prevStartLimitState && getStartLimitState())
        {
            armMotorSpeed = 0;
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            prevStartLimitState = true;
        }
        else if(prevStartLimitState && !(getStartLimitState()))
        {
            prevStartLimitState = false;
        }

        //stop arm if end limit switch is pressed, only detect rising edge
        if(!(prevEndLimitState) && getEndLimitState())
        {
            beforeEndLimitState = currentArmState;
            prevEndLimitState = true;
        }
        else if(prevEndLimitState && !(getEndLimitState()))
        {
            prevEndLimitState = false;
        }
        if(getEndLimitState() && currentArmState == beforeEndLimitState)
        {
            armMotorSpeed = 0;
        }

        //set arm motor to run at the speed specified in this method
        robot.glyphArmMotor1.setPower(armMotorSpeed);
    }

    //slowly move the arm to the start position, hit limit switch, and reset encoder
    public void HomeArm()
    {
        //if limit switch is pressed, turn off arm motor and reset encoder
        if(getStartLimitState())
        {
            robot.glyphArmMotor1.setPower(0);
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hasHomed = true;
        }
        //if limit switch is not pressed, slowly move arm towards start position
        else
        {
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.glyphArmMotor1.setPower(-HOME_ARM_SPEED);
        }
    }

    public boolean getStartLimitState()
    {
        return !(robot.glyphStartLimit.getState());
    }

    public boolean getEndLimitState()
    {
        return !(robot.glyphEndLimit.getState());
    }

    public GlyphArmState getCurrentArmState()
    {
        return currentArmState;
    }

    public GlyphArmState getPrevArmState()
    {
        return prevArmState;
    }

    public GlyphWristState getCurrentWristState()
    {
        return currentWristState;
    }

    public GlyphWristState getPrevWristState()
    {
        return prevWristState;
    }

    public boolean isGrabberServo1Open()
    {
        //grabber is open if the difference between the current grabber position and the open poisition is very small; we cannot equate doubles because of rounding errors
        return Math.abs(robot.grabber1Servo.getPosition() - GRABBER_OPEN_POS) < 0.01;
    }

    public boolean isGrabberServo2Open()
    {
        //grabber is open if the difference between the current grabber position and the open poisition is very small; we cannot equate doubles because of rounding errors
        return Math.abs(robot.grabber2Servo.getPosition() - GRABBER_OPEN_POS) < 0.01;
    }

    public int getArmPos()
    {
        return robot.glyphArmMotor1.getCurrentPosition();
    }

    public int getWristPos()
    {
        return robot.glyphWristMotor.getCurrentPosition();
    }

    public boolean hasHomed()
    {
        return hasHomed;
    }
}
