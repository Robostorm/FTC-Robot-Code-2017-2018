package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by andrew on 10/8/17.
 */

//class that controls the glyph/relic arm
public class RRBotGlyphArm
{
    RRBotHardware robot;
    RRBotMecanumDrive drive;
    private ElapsedTime joystickTime = new ElapsedTime();
    //private ElapsedTime accelTime = new ElapsedTime();
    //private ElapsedTime accelUpdateTime = new ElapsedTime();

    //final variables for values that will not change
    private final double GLYPH_WRIST_SPEED = 1;
    private final double GRABBER_OPEN_POS = 0.25; //was .8
    private final double GRABBER_CLOSE_POS = 0.8; //was .75 waswas .15
    private final double GRABBER_RELEASE_POS = 0.5; //was .4
    private final double GLYPH_ARM_MAX_SPEED = 1; //was .8
    private final double GLYPH_ARM_SLOW_SPEED = 0.2;
    private final double GLYPH_ARM_SLOW_DIST = 300;
    private final double HOME_ARM_SPEED = 0.2;
    //private final double GLYPH_ARM_FAST_SPEED = 0.8;
    private final int ARM_POS_THRESHOLD = 80;
    private final int WRIST_POS_THRESHOLD = 50;
    private final double RELIC_INIT_SERVO_GLYPHMODE_POS = 0.2;
    private final double RELIC_INIT_SERVO_RELICMODE_POS = 0.6;
    private final double RELIC_GRABBER_OPEN_POS = 0;
    private final double RELIC_GRABBER_CLOSE_POS = 0.85;
    private final double RELIC_MODE_ARM_SPEED = 0.8; //was .7
    private final double RELIC_MODE_WRIST_SPEED = 0.25; //was .5
    private final double JEWEL_ARM_POS1 = 0.1;
    private final double JEWEL_ARM_POS2 = 0.5;
    private final double JEWEL_ARM_POS3 = 0.8;
    private final int ARM_JOYSTICK_INCREMENT = 40;
    private final int WRIST_JOYSTICK_INCREMENT = 70;
    private final int JOYSTICK_UPDATE_MILLIS = 5;
    //private final int GLYPH_ARM_SPEED_UPDATE_MILLIS = 10;
    //private final int ACCEL_TIME = 1000;
    //private final double SPEED_INCREMENT = GLYPH_ARM_FAST_SPEED / (ACCEL_TIME / GLYPH_ARM_SPEED_UPDATE_MILLIS);
    private final double AUTO_GLYPH_PLACE_DRIVE_TIME_1 = 60;
    private final double AUTO_GLYPH_PLACE_DRIVE_TIME_2_LOW = 200;
    private final double AUTO_GLYPH_PLACE_DRIVE_TIME_3_LOW = 140;
    private final double AUTO_GLYPH_PLACE_DRIVE_TIME_2_HIGH = 320;
    private final double AUTO_GLYPH_PLACE_DRIVE_SPEED = 0.6;

    private GlyphArmState prevArmState = GlyphArmState.FRONT_PICKUP;
    private GlyphArmState currentArmState = GlyphArmState.FRONT_PICKUP;
    private GlyphWristState prevWristState = GlyphWristState.START;
    private GlyphWristState currentWristState = GlyphWristState.START;
    private boolean prevStartLimitState = false;
    private boolean prevEndLimitState = false;
    private double armMotorSpeed = 0;
    private double wristMotorSpeed = 0;
    private boolean hasHomed = false;
    private GlyphArmState beforeEndLimitState = currentArmState;
    private boolean relicMode = false;
    private double joystickValue;
    private boolean isAutoGlyphPlace = false;
    private int autoGlyphPlaceState = 0;
    private GlyphArmState autoGlyphPlaceArmInitState = null;
    private boolean hasGrabberClosed = false;

    //constructor gets hardware object and drive object from teleop class when it is constructed
    public RRBotGlyphArm(RRBotHardware robot, RRBotMecanumDrive drive)
    {
        this.robot = robot;
        this.drive = drive;
    }

    public void UpdateValues()
    {
        //set previous arm state equal to current arm state once the arm gets near its target
        if(Math.abs(robot.glyphArmMotor1.getCurrentPosition() - robot.glyphArmMotor1.getTargetPosition()) < ARM_POS_THRESHOLD)
        {
            prevArmState = currentArmState;
        }

        //set previous wrist state equal to current wrist state once the arm gets wrist its target
        if(Math.abs(robot.glyphWristMotor.getCurrentPosition() - currentWristState.getWristEncoderPos()) < WRIST_POS_THRESHOLD)
        {
            prevWristState = currentWristState;
        }
        
        GlyphArmSpeedUpdate();
        GlyphWristSpeedUpdate();

        /*if(accelUpdateTime.milliseconds() >= GLYPH_ARM_SPEED_UPDATE_MILLIS)
        {
            if(prevArmState.isFrontPos() != prevArmState.isFrontPos())
            {
                ArmAccelUpdate();
            }
            accelUpdateTime.reset();
        }*/

        if(isAutoGlyphPlace)
        {
            AutoGlyphPlaceRoutine();
        }

        GrabberSignal();
    }

    public void MoveGlyphArmToState(GlyphArmState state)
    {
        //make sure neither grabber is open if arm is going between front and back positions
        if(prevArmState.isFrontPos() == state.isFrontPos() || !getGrabber1Pos().equals("open") && !getGrabber2Pos().equals("open"))
        {
            if(currentArmState != state)
            {
                currentArmState = state;

                //use RUN_TO_POSITION mode to move the motor to desired position using PID
                robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.glyphArmMotor1.setTargetPosition(state.getArmEncoderPos());
            }
        }
    }

    public void MoveGlyphWristToState(GlyphWristState state)
    {
        //make sure neither grabber is open
        if(isAutoGlyphPlace || (!getGrabber1Pos().equals("open") && !getGrabber2Pos().equals("open")))
        {
            currentWristState = state;

            //use RUN_TO_POSITION mode to move the motor to desired position using PID
            robot.glyphWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.glyphWristMotor.setTargetPosition(state.getWristEncoderPos());
        }
    }

    public void FlipWrist()
    {
        robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
        robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

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

    public void MoveGrabber(String button)
    {
        //make sure the arm and wrist are not currently moving
        if(prevArmState == currentArmState && prevWristState == currentWristState)
        {
            /*//if wrist in in front position and arm is in a front position or wrist is in back position and arm is in a back position
            if((currentWristState == GlyphWristState.FRONT && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.BACK && !currentArmState.isFrontPos()))
            {
                if(getGrabber1Pos().equals("open") || getGrabber1Pos().equals("release"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(button.equals("open"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                }
                else if(button.equals("release"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                }
            }
            //if wrist in in back position and arm is in a front position or wrist is in front position and arm is in a back position
            else if((currentWristState == GlyphWristState.BACK && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.FRONT && !currentArmState.isFrontPos()))
            {
                if(getGrabber2Pos().equals("open") || getGrabber2Pos().equals("release"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(button.equals("open"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                }
                else if(button.equals("release"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_RELEASE_POS);
                }
            }*/

            if(getActiveGrabber() == 1)
            {
                if(getGrabber1Pos().equals("open") || getGrabber1Pos().equals("release"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(button.equals("open"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                }
                else if(button.equals("release"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                }
            }
            else if(getActiveGrabber() == 2)
            {
                if(getGrabber2Pos().equals("open") || getGrabber2Pos().equals("release"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(button.equals("open"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                }
                else if(button.equals("release"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_RELEASE_POS);
                }
            }
        }
    }

    /*public void MoveGrabberAuto(String position)
    {
        //make sure the arm and wrist are not currently moving
        if(prevArmState == currentArmState && prevWristState == currentWristState)
        {
            //if wrist in in front position and arm is in a front position or wrist is in back position and arm is in a back position
            if((currentWristState == GlyphWristState.FRONT && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.BACK && !currentArmState.isFrontPos()))
            {
                if(position.equals("close"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(position.equals("open"))
                {
                    robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
            //if wrist in in back position and arm is in a front position or wrist is in front position and arm is in a back position
            else if((currentWristState == GlyphWristState.BACK && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.FRONT && !currentArmState.isFrontPos()))
            {
                if(position.equals("close"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                }
                else if(position.equals("open"))
                {
                    robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
        }
    }*/

    public void AutoCloseGrabber()
    {
        if(prevArmState == currentArmState && prevWristState == currentWristState)
        {
            if(getActiveGrabber() == 1)
            {
                if(getGrabber1SwitchState())
                {
                    robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                    hasGrabberClosed = true;
                }
                else if(!hasGrabberClosed)
                {
                    robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
            else if(getActiveGrabber() == 2)
            {
                if(getGrabber2SwitchState())
                {
                    robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                    hasGrabberClosed = true;
                }
                else if(!hasGrabberClosed)
                {
                    robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                }
            }
        }
    }

    public void GrabberSignal()
    {
        if(getActiveGrabber() == 1)
        {
            if(getGrabber1Pos().equals("open"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS1);
            }
            else if(getGrabber1Pos().equals("release"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS2);
            }
            else if(getGrabber1Pos().equals("close"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS3);
            }
        }
        else if(getActiveGrabber() == 2)
        {
            if(getGrabber2Pos().equals("open"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS1);
            }
            else if(getGrabber2Pos().equals("release"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS2);
            }
            else if(getGrabber2Pos().equals("close"))
            {
                robot.jewelArmServo2.setPosition(JEWEL_ARM_POS3);
            }
        }
        else
        {
            robot.jewelArmServo2.setPosition(JEWEL_ARM_POS3);
        }
    }

    public void GlyphArmSpeedUpdate()
    {
        //set armMotorSpeed to max/normal speed, other stuff will change it to something else if necessary
        armMotorSpeed = GLYPH_ARM_MAX_SPEED;

        //if the arm is moving to the start or front_pickup positions, and in the encoder distance GLYPH_ARM_SLOW_DISTANCE before the target position, slow the arm down to GLYPH_ARM_SLOW_SPEED
        if(currentArmState == GlyphArmState.FRONT_PICKUP)
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

        if(relicMode)
        {
            armMotorSpeed = RELIC_MODE_ARM_SPEED;
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
    
    public void GlyphWristSpeedUpdate()
    {
        wristMotorSpeed = GLYPH_WRIST_SPEED;
        
        if(relicMode)
        {
            wristMotorSpeed = RELIC_MODE_WRIST_SPEED;
        }
        
        robot.glyphWristMotor.setPower(wristMotorSpeed);
    }

    /*public void ArmAccelUpdate()
    {
        if(prevArmState != currentArmState)
        {
            if(prevArmState.isFrontPos() != currentArmState.isFrontPos())
            {
                int midPos = (currentArmState.getArmEncoderPos() + prevArmState.getArmEncoderPos()) / 2;

                if(currentArmState.getArmEncoderPos() > prevArmState.getArmEncoderPos())
                {
                    if(accelTime.milliseconds() < ACCEL_TIME && robot.glyphArmMotor1.getCurrentPosition() < midPos)
                    {
                        armMotorSpeed += SPEED_INCREMENT;
                    }
                    else
                    {
                        if(!(hasCalcDeccelStartPos))
                        {
                            int afterAccelPos = robot.glyphArmMotor1.getCurrentPosition();
                            deccelStartPos = currentArmState.getArmEncoderPos() - afterAccelPos;
                            hasCalcDeccelStartPos = true;
                        }

                        if(robot.glyphArmMotor1.getCurrentPosition() >= deccelStartPos && armMotorSpeed > SPEED_INCREMENT)
                        {
                            armMotorSpeed -= SPEED_INCREMENT;
                        }
                    }
                }
                else
                {
                    if(accelTime.milliseconds() < ACCEL_TIME && robot.glyphArmMotor1.getCurrentPosition() > midPos)
                    {
                        armMotorSpeed += SPEED_INCREMENT;
                    }
                    else
                    {
                        if(!(hasCalcDeccelStartPos))
                        {
                            int afterAccelPos = robot.glyphArmMotor1.getCurrentPosition();
                            deccelStartPos = currentArmState.getArmEncoderPos() - afterAccelPos;
                            hasCalcDeccelStartPos = true;
                        }

                        if(robot.glyphArmMotor1.getCurrentPosition() >= deccelStartPos && armMotorSpeed > SPEED_INCREMENT)
                        {
                            armMotorSpeed -= SPEED_INCREMENT;
                        }
                    }
                }
            }
        }
    }*/

    //slowly move the arm to the start position, hit limit switch, and reset encoder
    public void HomeArm()
    {
        MoveGlyphWristToState(GlyphWristState.START);

        //if limit switch is pressed, turn off arm motor and reset encoder
        if(getStartLimitState())
        {
            robot.glyphArmMotor1.setPower(0);
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hasHomed = true;
        }
        //if limit switch is not pressed, slowly move arm towards start position
        else
        {
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.glyphArmMotor1.setPower(-HOME_ARM_SPEED);
        }
    }

    public void MoveToStartPos()
    {
        robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
        robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

        MoveGlyphArmToState(GlyphArmState.FRONT_PICKUP);
        MoveGlyphWristToState(GlyphWristState.START);
    }

    public void ToggleRelicMode()
    {
        if(relicMode)
        {
            relicMode = false;
        }
        else
        {
            relicMode = true;
        }

        MoveRelicInitServo();
    }

    public void MoveRelicInitServo()
    {
        if(relicMode)
        {
            robot.relicInitServo.setPosition(RELIC_INIT_SERVO_RELICMODE_POS);
        }
        else
        {
            robot.relicInitServo.setPosition(RELIC_INIT_SERVO_GLYPHMODE_POS);
        }
    }

    public void MoveRelicGrabber()
    {
        if(relicMode)
        {
            if(isRelicGrabberOpen())
            {
                robot.relicGrabberServo.setPosition(RELIC_GRABBER_CLOSE_POS);
            }
            else
            {
                robot.relicGrabberServo.setPosition(RELIC_GRABBER_OPEN_POS);
            }
        }
    }

    public void MoveToRelicPickupPos(int pickupPos)
    {
        if(pickupPos == 1)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PICKUP);

            if(prevArmState == currentArmState)
            {
                MoveGlyphWristToState(GlyphWristState.RELIC_PICKUP);
            }
        }
        else if(pickupPos == 2)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PICKUP2);

            if(prevArmState == currentArmState)
            {
                MoveGlyphWristToState(GlyphWristState.RELIC_PICKUP2);
            }
        }
    }

    public void MoveToRelicPlacePos()
    {
        MoveGlyphWristToState(GlyphWristState.RELIC_PLACE);

        if(prevWristState == currentWristState)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PLACE);
        }
    }

    public void MoveToRelicDonePos()
    {
        MoveGlyphArmToState(GlyphArmState.RELIC_DONE);
        MoveGlyphWristToState(GlyphWristState.RELIC_DONE);
    }

    public void EnableArmJoystick(double joystickValue)
    {
        //calls ArmJoystickUpdate every JOYSTICK_UPDATE_MILLIS milliseconds
        if(joystickTime.milliseconds() >= JOYSTICK_UPDATE_MILLIS)
        {
            ArmJoystickUpdate(joystickValue);
            joystickTime.reset();
        }
    }

    public void ArmJoystickUpdate(double joystickValue)
    {
        //make sure that arm is not currently going to a set position
        if(prevArmState == currentArmState && robot.glyphArmMotor1.getCurrentPosition() > 0)
        {
            robot.glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.joystickValue = joystickValue;

            //if joystickValue is positive, set motor target position to the motor's current position plus ARM_JOYSTICK_INCREMENT
            if(joystickValue > 0)
            {
                robot.glyphArmMotor1.setTargetPosition(robot.glyphArmMotor1.getCurrentPosition() + ARM_JOYSTICK_INCREMENT);
            }
            //if joystickValue is negative, set motor target position to the motor's current position plus ARM_JOYSTICK_INCREMENT
            else
            {
                robot.glyphArmMotor1.setTargetPosition(robot.glyphArmMotor1.getCurrentPosition() - ARM_JOYSTICK_INCREMENT);
            }

        }
    }

    public void EnableWristJoystick(double joystickValue)
    {
        //calls WristJoystickUpdate every JOYSTICK_UPDATE_MILLIS milliseconds
        if(joystickTime.milliseconds() >= JOYSTICK_UPDATE_MILLIS)
        {
            WristJoystickUpdate(joystickValue);
            joystickTime.reset();
        }
    }

    public void WristJoystickUpdate(double joystickValue)
    {
        //make sure that wrist is not currently going to a set position
        if(prevWristState == currentWristState)
        {
            robot.glyphWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.joystickValue = joystickValue;

            //if joystickValue is positive, set motor target position to the motor's current position plus ARM_JOYSTICK_INCREMENT
            if(joystickValue > 0)
            {
                robot.glyphWristMotor.setTargetPosition(robot.glyphWristMotor.getCurrentPosition() + WRIST_JOYSTICK_INCREMENT);
            }
            //if joystickValue is negative, set motor target position to the motor's current position plus ARM_JOYSTICK_INCREMENT
            else
            {
                robot.glyphWristMotor.setTargetPosition(robot.glyphWristMotor.getCurrentPosition() - WRIST_JOYSTICK_INCREMENT);
            }

        }
    }

    public void setIsAutoGlyphPlace(boolean isButtonPressed)
    {
        if(isButtonPressed &&
                prevArmState == currentArmState &&
                prevWristState == currentWristState &&
                currentWristState == GlyphWristState.BACK &&
                (currentArmState == GlyphArmState.FRONT1 ||
                currentArmState == GlyphArmState.FRONT2 ||
                currentArmState == GlyphArmState.FRONT3))
        {
            isAutoGlyphPlace = true;
        }
    }

    //automatically readjusts/flips wrist and places 2nd glyph
    public void AutoGlyphPlaceRoutine()
    {
        //make sure arm, wrist, and drive motors are not moving
        if(prevArmState == currentArmState && prevWristState == currentWristState && !drive.getIsAutoMove())
        {
            //do a linear routine with a number of steps; autoGlyphPlaceState holds the current step

            //step 1: open grabber 2
            if(autoGlyphPlaceState == 0)
            {
                robot.grabber2Servo.setPosition(GRABBER_RELEASE_POS);
                autoGlyphPlaceState = 1;
                return;
            }

            //step 2: drive robot backwards
            if(autoGlyphPlaceState == 1)
            {
                autoGlyphPlaceArmInitState = currentArmState;
                drive.AutoMove(-AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_1);
                autoGlyphPlaceState = 2;
                return;
            }

            //step 3: move wrist to start position
            if(autoGlyphPlaceState == 2)
            {
                MoveGlyphWristToState(GlyphWristState.START);
                autoGlyphPlaceState = 3;
                return;
            }

            //step 4: close grabber 2
            if(autoGlyphPlaceState == 3)
            {
                robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                autoGlyphPlaceState = 4;
                return;
            }

            //rest of steps when arm started in FRONT1
            if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT1)
            {
                if(autoGlyphPlaceState == 4)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT2);
                    autoGlyphPlaceState = 5;
                    return;
                }

                if(autoGlyphPlaceState == 5)
                {
                    drive.AutoMove(-AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_2_LOW);
                    autoGlyphPlaceState = 6;
                    return;
                }

                if(autoGlyphPlaceState == 6)
                {
                    MoveGlyphWristToState(GlyphWristState.FRONT);
                    autoGlyphPlaceState = 7;
                    return;
                }

                if(autoGlyphPlaceState == 7)
                {
                    drive.AutoMove(AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_3_LOW);
                    autoGlyphPlaceState = 8;
                    return;
                }

                if(autoGlyphPlaceState == 8)
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                    autoGlyphPlaceState = 9;
                    return;
                }

                if(autoGlyphPlaceState == 9)
                {
                    isAutoGlyphPlace = false;
                    autoGlyphPlaceState = 0;
                }
            }
            else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT2)
            {
                if(autoGlyphPlaceState == 4)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT3);
                    autoGlyphPlaceState = 5;
                    return;
                }

                if(autoGlyphPlaceState == 5)
                {
                    MoveGlyphWristToState(GlyphWristState.FRONT);
                    autoGlyphPlaceState = 6;
                    return;
                }

                if(autoGlyphPlaceState == 6)
                {
                    drive.AutoMove(AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_1);
                    autoGlyphPlaceState = 7;
                    return;
                }

                if(autoGlyphPlaceState == 7)
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                    autoGlyphPlaceState = 8;
                    return;
                }

                if(autoGlyphPlaceState == 8)
                {
                    isAutoGlyphPlace = false;
                    autoGlyphPlaceState = 0;
                }
            }
            else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT3)
            {
                if(autoGlyphPlaceState == 4)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT4);
                    autoGlyphPlaceState = 5;
                    return;
                }

                if(autoGlyphPlaceState == 5)
                {
                    MoveGlyphWristToState(GlyphWristState.FRONT);
                    autoGlyphPlaceState = 6;
                    return;
                }

                if(autoGlyphPlaceState == 6)
                {
                    drive.AutoMove(AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_2_HIGH);
                    autoGlyphPlaceState = 7;
                    return;
                }

                if(autoGlyphPlaceState == 7)
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                    autoGlyphPlaceState = 8;
                    return;
                }

                if(autoGlyphPlaceState == 8)
                {
                    isAutoGlyphPlace = false;
                    autoGlyphPlaceState = 0;
                }
            }





            /*if(autoGlyphPlaceState == 3)
            {
                if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT1)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT2);
                }
                else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT2)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT3);
                }
                else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT3)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT4);
                }
                autoGlyphPlaceState = 4;
                return;
            }

            if(autoGlyphPlaceState == 4)
            {
                if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT1)
                {
                    drive.AutoMove(-AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_2);
                }
                else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT2)
                {

                }
                else if(autoGlyphPlaceArmInitState == GlyphArmState.FRONT3)
                {
                    drive.AutoMove(AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_2);
                }
                autoGlyphPlaceState = 5;
                return;
            }

            if(autoGlyphPlaceState == 5)
            {
                MoveGlyphWristToState(GlyphWristState.FRONT);
                autoGlyphPlaceState = 6;
                return;
            }

            if(autoGlyphPlaceState == 6)
            {
                isAutoGlyphPlace = false;
                autoGlyphPlaceState = 0;
            }*/
        }
    }

    //accessor methods
    
    public boolean getStartLimitState()
    {
        return !(robot.glyphStartLimit.getState());
    }

    public boolean getEndLimitState()
    {
        return !(robot.glyphEndLimit.getState());
    }

    public boolean getGrabber1SwitchState()
    {
        return !(robot.glyphGrabberSwitch1.getState());
    }
    public boolean getGrabber2SwitchState()
    {
        return !(robot.glyphGrabberSwitch2.getState());
    }

    public int getActiveGrabber()
    {
        //if wrist in in front position and arm is in a front position or wrist is in back position and arm is in a back position
        if((currentWristState == GlyphWristState.FRONT && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.BACK && !currentArmState.isFrontPos()))
        {
            return 1;
        }
        //if wrist in in back position and arm is in a front position or wrist is in front position and arm is in a back position
        else if((currentWristState == GlyphWristState.BACK && currentArmState.isFrontPos()) || (currentWristState == GlyphWristState.FRONT && !currentArmState.isFrontPos()))
        {
            return 2;
        }
        else
        {
            return 0;
        }
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

    public String getGrabber1Pos()
    {
        String returnString = "";

        //grabber1 is in a specific state if the difference between the current grabber position and the state poisition is very small; we cannot equate doubles because of rounding errors
        if(Math.abs(robot.grabber1Servo.getPosition() - GRABBER_OPEN_POS) < 0.01)
        {
            returnString = "open";
        }
        else if(Math.abs(robot.grabber1Servo.getPosition() - GRABBER_CLOSE_POS) < 0.01)
        {
            returnString = "close";
        }
        else if(Math.abs(robot.grabber1Servo.getPosition() - GRABBER_RELEASE_POS) < 0.01)
        {
            returnString = "release";
        }
        return returnString;
    }

    public String getGrabber2Pos()
    {
        String returnString = "";

        //grabber2 is in a specific state if the difference between the current grabber position and the state poisition is very small; we cannot equate doubles because of rounding errors
        if(Math.abs(robot.grabber2Servo.getPosition() - GRABBER_OPEN_POS) < 0.01)
        {
            returnString = "open";
        }
        else if(Math.abs(robot.grabber2Servo.getPosition() - GRABBER_CLOSE_POS) < 0.01)
        {
            returnString = "close";
        }
        else if(Math.abs(robot.grabber2Servo.getPosition() - GRABBER_RELEASE_POS) < 0.01)
        {
            returnString = "release";
        }
        return returnString;
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

    public boolean isInRelicMode()
    {
        return relicMode;
    }

    public boolean isRelicGrabberOpen()
    {
        return Math.abs(robot.relicGrabberServo.getPosition() - RELIC_GRABBER_OPEN_POS) < 0.01;
    }

    public double getJoystickValue()
    {
        return joystickValue;
    }

    public void setHasGrabberClosed(boolean hasGrabberClosed)
    {
        this.hasGrabberClosed = hasGrabberClosed;
    }
}
