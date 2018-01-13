package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Controls the robot's main arm, wrist, and grabbers.
 * Used to manipulate glyphs and relics.
 * @author Andrew Hollabaugh
 * @since 2017-10-08
 */
public class RRBotGlyphArm
{
    RRBotHardware robot;
    RRBotMecanumDrive drive;

    private ElapsedTime joystickTime = new ElapsedTime();
    private ElapsedTime beltTime1 = new ElapsedTime();
    private ElapsedTime beltTime2 = new ElapsedTime();

    //final variables for values that will not change
    private final double GRABBER_OPEN_POS = 0.25; //was .8
    protected static final double GRABBER_CLOSE_POS = 0.8; //was .75 waswas .15
    protected static final double GRABBER_RELEASE_POS = 0.5; //was .4
    protected static final double GRABBER_ROTATE_POS1 = 0;
    private final double GRABBER_ROTATE_POS2 = 1;
    private final double GRABBER_BELT_SPEED = -1;
    private final double GLYPH_ARM_MAX_SPEED = 1; //was .8
    private final double GLYPH_ARM_SLOW_SPEED = 0.2;
    private final double GLYPH_ARM_SLOW_DIST = 300;
    private final double HOME_ARM_SPEED = 0.2;
    private final double GLYPH_WRIST_SPEED = 1;
    private final int ARM_POS_THRESHOLD = 80;
    private final int WRIST_POS_THRESHOLD = 50;
    protected static final double RELIC_INIT_SERVO_GLYPHMODE_POS = 0.2;
    private final double RELIC_INIT_SERVO_RELICMODE_POS = 0.6;
    private final double RELIC_GRABBER_OPEN_POS = 0;
    protected static final double RELIC_GRABBER_CLOSE_POS = 0.85;
    private final double RELIC_MODE_ARM_SPEED = 0.7; //was .8
    private final double RELIC_MODE_WRIST_SPEED = 0.25; //was .5
    private final double JEWEL_ARM_POS1 = 0.1;
    private final double JEWEL_ARM_POS2 = 0.5;
    private final double JEWEL_ARM_POS3 = 0.8;
    private final int ARM_JOYSTICK_INCREMENT = 40;
    private final int WRIST_JOYSTICK_INCREMENT = 20; //was 70
    private final int JOYSTICK_UPDATE_MILLIS = 5;
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
    private boolean isGrabber1Belt = false;
    private boolean isGrabber2Belt = false;
    private double beltTimeout = 5;

    /**
     * Constructor gets hardware object and drive object from teleop class when it is constructed
     * @param robot Hardware class for the robot
     * @param drive Drive class
     */
    public RRBotGlyphArm(RRBotHardware robot, RRBotMecanumDrive drive)
    {
        this.robot = robot;
        this.drive = drive;
    }

    /**
     * Updates variables that store the positions of the arm and wrist. Calls miscellaneous methods. Runs every loop() in the teleop class.
     */
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

        if(isAutoGlyphPlace)
        {
            AutoGlyphPlaceRoutine();
        }

        SetGrabberBelt();

        GrabberSignal();
    }

    /**
     * Moves the main arm to the specified state. Uses the RUN_TO_POSITION runMode to ensure the arm gets to the desired position and stays there using PID.
     * @param state The state the arm will move to; specified as the enum GlyphArmState
     */
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

    /**
     * Moves the wrist to the specified state. Uses the RUN_TO_POSITION runMode to ensure the wrist gets to the desired position and stays there using PID.
     * @param state The state the wrist will move to; specified as the enum GlyphWristState
     */
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

    /**
     * Flips the wrist
     */
    public void FlipWrist()
    {
        if(getGrabberRotatePos() == 1)
        {
            //close both grabbers so they don't hit other parts of the robot
            robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
            robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

            //if wrist is in the start or back positions, move it to the front position
            if (currentWristState == GlyphWristState.START || currentWristState == GlyphWristState.BACK)
            {
                MoveGlyphWristToState(GlyphWristState.FRONT);
            }
            //if wrist is in the front position, move it to the back position
            else if (currentWristState == GlyphWristState.FRONT)
            {
                MoveGlyphWristToState(GlyphWristState.BACK);
            }
        }
    }

    public void RotateGrabber()
    {
        if(currentWristState == GlyphWristState.BACK)
        {
            //close both grabbers so they don't hit other parts of the robot
            robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
            robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

            //MoveGlyphWristToState(GlyphWristState.BACK);

            if (getGrabberRotatePos() == 1)
            {
                robot.grabberRotateServo.setPosition(GRABBER_ROTATE_POS2);
            }
            else if (getGrabberRotatePos() == 2)
            {
                robot.grabberRotateServo.setPosition(GRABBER_ROTATE_POS1);
            }
        }
    }

    /**
     * Moves the front glyph grabber to one of three positions: open, release, or close
     * open: fully opened
     * release: partially opened, used for releasing glyphs into cryptobox without disturbing adjacent stacks
     * close: fully closed
     * @param button - Which button is pressed, the open/close button or the release/close button
     */
    public void MoveGrabber(String button)
    {
        //make sure the arm and wrist are not currently moving
        if(prevArmState == currentArmState && prevWristState == currentWristState)
        {
            if(getGrabberRotatePos() == 2)
            {
                if(button.equals("open"))
                {
                    if(getGrabber1Pos().equals("open"))
                    {
                        robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                        robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                        isGrabber1Belt = true;
                        isGrabber2Belt = true;
                    }
                    else if(getGrabber1Pos().equals("close") || getGrabber1Pos().equals("release"))
                    {
                        robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                        robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                        isGrabber1Belt = false;
                        isGrabber1Belt = false;
                    }
                }
                else if(button.equals("release"))
                {
                    if(getGrabber1Pos().equals("open") || getGrabber1Pos().equals("release"))
                    {
                        robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                        robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                        isGrabber1Belt = true;
                        isGrabber1Belt = true;
                    }
                    else if(getGrabber1Pos().equals("close"))
                    {
                        robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                        robot.grabber2Servo.setPosition(GRABBER_RELEASE_POS);
                        isGrabber1Belt = false;
                        isGrabber1Belt = false;
                    }
                }
            }
            else
            {
                //if the grabber in front is grabber 1
                if(getActiveGrabber() == 1)
                {
                    if(button.equals("open"))
                    {
                        if(getGrabber1Pos().equals("open"))
                        {
                            robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                            isGrabber1Belt = true;
                        }
                        else if(getGrabber1Pos().equals("close") || getGrabber1Pos().equals("release"))
                        {
                            robot.grabber1Servo.setPosition(GRABBER_OPEN_POS);
                            isGrabber1Belt = false;
                        }
                    }
                    else if(button.equals("release"))
                    {
                        if(getGrabber1Pos().equals("open") || getGrabber1Pos().equals("release"))
                        {
                            robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
                            isGrabber1Belt = true;
                        }
                        else if(getGrabber1Pos().equals("close"))
                        {
                            robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                            isGrabber1Belt = false;
                        }
                    }
                }

                //if the grabber in front is grabber 2
                if(getActiveGrabber() == 2)
                {
                    if(button.equals("open"))
                    {
                        if(getGrabber2Pos().equals("open"))
                        {
                            robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                            isGrabber2Belt = true;
                        }
                        else if(getGrabber2Pos().equals("close") || getGrabber2Pos().equals("release"))
                        {
                            robot.grabber2Servo.setPosition(GRABBER_OPEN_POS);
                            isGrabber2Belt = false;
                        }
                    }
                    else if(button.equals("release"))
                    {
                        if(getGrabber2Pos().equals("open") || getGrabber2Pos().equals("release"))
                        {
                            robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);
                            isGrabber2Belt = true;
                        }
                        else if(getGrabber2Pos().equals("close"))
                        {
                            robot.grabber2Servo.setPosition(GRABBER_RELEASE_POS);
                            isGrabber2Belt = false;
                        }
                    }
                }
            }
        }
    }

    /**
     * Automatically closes the front grabber when limit switches are pressed, indicating the glyph is in position to be picked up.
     * Grabber closing is "latching," after it closes it will not open again automatically
     */
    public void AutoCloseGrabber()
    {
        //make sure the arm and wrist are not moving
        if(prevArmState == currentArmState && prevWristState == currentWristState)
        {
            if(getActiveGrabber() == 1)
            {
                //use hasGrabberClosed for latching function
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

    public void SetGrabberBelt()
    {
        if(isGrabber1Belt)
        {
            //if the current power of grabber1Belt is 0
            if(Math.abs(robot.grabber1Belt.getPower()) < 0.01)
            {
                robot.grabber1Belt.setPower(GRABBER_BELT_SPEED);
                beltTime1.reset();
            }

            if(getGrabber1SwitchState() || beltTime1.seconds() > beltTimeout)
            {
                isGrabber1Belt = false;
            }
        }
        else
        {
            //if the current power of grabber1Belt is 1
            if(Math.abs(robot.grabber1Belt.getPower() - GRABBER_BELT_SPEED) < 0.01)
            {
                robot.grabber1Belt.setPower(0);
            }
        }


        if(isGrabber2Belt)
        {
            //if the current power of grabber2Belt is 0
            if(Math.abs(robot.grabber2Belt.getPower()) < 0.01)
            {
                robot.grabber2Belt.setPower(GRABBER_BELT_SPEED);
                beltTime2.reset();
            }

            if(getGrabber2SwitchState() || beltTime2.seconds() > beltTimeout)
            {
                isGrabber2Belt = false;
            }
        }
        else
        {
            //if the current power of grabber2Belt is 1
            if(Math.abs(robot.grabber2Belt.getPower() - GRABBER_BELT_SPEED) < 0.01)
            {
                robot.grabber2Belt.setPower(0);
            }
        }
    }

    /**
     * Moves jewel flipper according to position of the front grabber as a signal to the drivers
     */
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
        //fall back to position 3 if none of the previous conditions are true
        else
        {
            robot.jewelArmServo2.setPosition(JEWEL_ARM_POS3);
        }
    }

    /**
     * Updates and sets the speed of the main arm. Runs every UpdateValues() loop
     * The speed variable is set to the normal speed, then various conditions can change the speed variable, then the motors' speed is set to the variable.
     */
    public void GlyphArmSpeedUpdate()
    {
        //set armMotorSpeed to normal speed
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

        //set arm speed to a separate relic mode speed if robot is in relic mode
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

    /**
     * Updates and sets the speed of the wrist. Runs every UpdateValues() loop
     * The speed variable is set to the normal speed, then various conditions can change the speed variable, then the motor speed is set to the variable.
     */
    public void GlyphWristSpeedUpdate()
    {
        //set wristMotorSpeed to the default speed
        wristMotorSpeed = GLYPH_WRIST_SPEED;

        //change speed to a different relic mode speed if in relic mode
        if(relicMode)
        {
            wristMotorSpeed = RELIC_MODE_WRIST_SPEED;
        }

        //set wrist motor to run at the specified speed
        robot.glyphWristMotor.setPower(wristMotorSpeed);
    }

    /**
     * Slowly move the arm to the start position until it hits the limit switch, and reset encoder
     * Run at the beginning of each opMode
     */
    public void HomeArm()
    {
        //make sure wrist is in start (vertical) position at start of opMode
        MoveGlyphWristToState(GlyphWristState.START);

        //make sure grabbers are closed
        robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
        robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

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

    /**
     * Moves main arm, wrist, and grabbers to the start/neutral position
     * Run when "start" button is pressed
     */
    public void MoveToStartPos()
    {
        if(getGrabberRotatePos() == 1)
        {
            //move grabber servos to close position
            robot.grabber1Servo.setPosition(GRABBER_CLOSE_POS);
            robot.grabber2Servo.setPosition(GRABBER_CLOSE_POS);

            //move arm and wrist to start positions
            MoveGlyphArmToState(GlyphArmState.FRONT_PICKUP);
            MoveGlyphWristToState(GlyphWristState.START);
        }
    }

    /**
     * Toggles the relicMode variable, which stores a boolean of whether the robot is in glyph mode (false) or relic mode (true).
     */
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

    /**
     * Moves the relic init servo, which latches the wrist motor to the arm extension.
     */
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

    /**
     * Moves the relic grabber if in relic mode
     * Toggles between open and close positions
     */
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

    /**
     * Moves the arm or wrist to one of the two relic pickup positions (one for each relic starting position)
     * The first call moves the arm; the second call moves the wrist
     * @param pickupPos the pickup position to move to (1 for side farther from recovery zone, or 2 for side closer to recovery zone)
     */
    public void MoveToRelicPickupPos(int pickupPos)
    {
        if(pickupPos == 1)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PICKUP);

            //move the wrist if the arm got to its position
            if(prevArmState == currentArmState)
            {
                MoveGlyphWristToState(GlyphWristState.RELIC_PICKUP);
            }
        }
        else if(pickupPos == 2)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PICKUP2);

            //move the wrist if the arm got to its position
            if(prevArmState == currentArmState)
            {
                MoveGlyphWristToState(GlyphWristState.RELIC_PICKUP2);
            }
        }
    }

    /**
     * Moves the arm or wrist to the relic place position
     * The first call moves the wrist (transfer position); the second call moves the arm
     */
    public void MoveToRelicPlacePos()
    {
        MoveGlyphWristToState(GlyphWristState.RELIC_PLACE);

        //move the arm if the wrist got to its position
        if(prevWristState == currentWristState)
        {
            MoveGlyphArmToState(GlyphArmState.RELIC_PLACE);
        }
    }

    /**
     * Moves the arm and wrist to the relic done position at the same time
     */
    public void MoveToRelicDonePos()
    {
        MoveGlyphArmToState(GlyphArmState.RELIC_DONE);
        MoveGlyphWristToState(GlyphWristState.RELIC_DONE);
    }

    /**
     * Enables manual joystick control of the main arm by calling ArmJoystickUpdate every time interval
     * @param joystickValue current value of the joystick
     */
    public void EnableArmJoystick(double joystickValue)
    {
        //calls ArmJoystickUpdate every JOYSTICK_UPDATE_MILLIS milliseconds
        if(joystickTime.milliseconds() >= JOYSTICK_UPDATE_MILLIS)
        {
            ArmJoystickUpdate(joystickValue);
            joystickTime.reset();
        }
    }

    /**
     * Controls the main arm using manual joystick input, single speed in two directions. Used for fine control of arm and as a fallback.
     * Uses RUN_TO_POSITION; sets target position to an increment added to its current position
     * @param joystickValue current value of the joystick
     */
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

    /**
     * Enables manual joystick control of the wrist by calling WristJoystickUpdate every time interval
     * @param joystickValue current value of the joystick
     */
    public void EnableWristJoystick(double joystickValue)
    {
        //calls WristJoystickUpdate every JOYSTICK_UPDATE_MILLIS milliseconds
        if(joystickTime.milliseconds() >= JOYSTICK_UPDATE_MILLIS)
        {
            WristJoystickUpdate(joystickValue);
            joystickTime.reset();
        }
    }

    /**
     * Controls the wrist using manual joystick input, single speed in two directions. Used for fine control of arm and as a fallback.
     * Uses RUN_TO_POSITION; sets target position to an increment added to its current position
     * @param joystickValue current value of the joystick
     */
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

    /**
     * Turns on auto glyph place routine if the arm and wrist are not moving and in the correct positions.
     * Only enables the routine, does not disable. It will stop when it is done.
     * @param isButtonPressed is the enable button pressed
     */
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

    /**
     * Runs the auto glyph place routine: automatically places 1st glpyh, readjusts robot and flips wrist, and places 2nd glyph
     * autoGlyphPlaceState keeps track of the state of the routine. When the state is a certain value, a certain step will be executed and the state is incremented.
     */
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
                //step 5: move arm to FRONT2 position
                if(autoGlyphPlaceState == 4)
                {
                    MoveGlyphArmToState(GlyphArmState.FRONT2);
                    autoGlyphPlaceState = 5;
                    return;
                }

                //step 6: drive backwards
                if(autoGlyphPlaceState == 5)
                {
                    drive.AutoMove(-AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_2_LOW);
                    autoGlyphPlaceState = 6;
                    return;
                }

                //step 7: rotate wrist to front position
                if(autoGlyphPlaceState == 6)
                {
                    MoveGlyphWristToState(GlyphWristState.FRONT);
                    autoGlyphPlaceState = 7;
                    return;
                }

                //step 8: drive forwards
                if(autoGlyphPlaceState == 7)
                {
                    drive.AutoMove(AUTO_GLYPH_PLACE_DRIVE_SPEED, AUTO_GLYPH_PLACE_DRIVE_TIME_3_LOW);
                    autoGlyphPlaceState = 8;
                    return;
                }

                //step 9: set grabber 1 to release position
                if(autoGlyphPlaceState == 8)
                {
                    robot.grabber1Servo.setPosition(GRABBER_RELEASE_POS);
                    autoGlyphPlaceState = 9;
                    return;
                }

                //set isAutoGlyphPlace to false to stop this method from being called and reset the position
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
        }
    }

    //accessor methods

    /**
     * Get the front limit switch state, negated
     * @return the limit switch state
     */
    public boolean getStartLimitState()
    {
        return !(robot.glyphStartLimit.getState());
    }

    /**
     * Get the back limit switch state, negated
     * @return the limit switch state
     */
    public boolean getEndLimitState()
    {
        return !(robot.glyphEndLimit.getState());
    }

    /**
     * Get the grabber 1 limit switch state, negated
     * @return the limit switch state
     */
    public boolean getGrabber1SwitchState()
    {
        return !(robot.glyphGrabberSwitch1.getState());
    }

    /**
     * Get the grabber 2 limit switch state, negated
     * @return the limit switch state
     */
    public boolean getGrabber2SwitchState()
    {
        return !(robot.glyphGrabberSwitch2.getState());
    }

    /**
     * Get the current grabber in front
     * @return the number of the active grabber: 1 or 2, 0 for fallback
     */
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

    /**
     * Get the current position of glyph grabber 1.
     * @return position of grabber 1; a string containing "open," "close," or "release"
     */
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

    /**
     * Get the current position of glyph grabber 2.
     * @return position of grabber 2; a string containing "open," "close," or "release"
     */
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

    public int getGrabberRotatePos()
    {
        //grabberRotation is in a specific state if the difference between the current grabber position and the state poisition is very small; we cannot equate doubles because of rounding errors
        if(Math.abs(robot.grabberRotateServo.getPosition() - GRABBER_ROTATE_POS1) < 0.01)
        {
            return 1;
        }
        else if(Math.abs(robot.grabberRotateServo.getPosition() - GRABBER_ROTATE_POS2) < 0.01)
        {
            return 2;
        }
        else
        {
            return 0;
        }
    }

    public int getArmPos()
    {
        return robot.glyphArmMotor1.getCurrentPosition();
    }

    public int getWristPos()
    {
        return robot.glyphWristMotor.getCurrentPosition();
    }

    /**
     * Returns whether the main arm has homed
     * @return hasHomed - if the arm has homed
     */
    public boolean hasHomed()
    {
        return hasHomed;
    }

    /**
     * Returns whether the robot is in relic mode
     * @return relicMode - if the robot is in relic mode
     */
    public boolean isInRelicMode()
    {
        return relicMode;
    }

    /**
     * Returns whether the relic grabber is open
     * @return false if it is closed; true if it is open
     */
    public boolean isRelicGrabberOpen()
    {
        return Math.abs(robot.relicGrabberServo.getPosition() - RELIC_GRABBER_OPEN_POS) < 0.01;
    }

    public double getJoystickValue()
    {
        return joystickValue;
    }

    /**
     * Set if the grabber has closed
     * @param hasGrabberClosed has the grabber closed
     */
    public void setHasGrabberClosed(boolean hasGrabberClosed)
    {
        this.hasGrabberClosed = hasGrabberClosed;
    }
}
