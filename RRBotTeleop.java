package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by andrew on 9/10/17.
 */

//Teleop Opmode class for Team 12601's Relic Recovery Robot (2017-2018)

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    //construct an RRBotHardware object to reference its stuff
    RRBotHardware robot = new RRBotHardware();
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);
    RRBotGlyphArm glyphArm = new RRBotGlyphArm(robot, drive);
    
    private final double GAMEPAD_TRIGGER_THRESHOLD = 0.2;
    private final double JOYSTICK_DEADZONE = 0.3;
    private boolean prevGrabberOpenButton = false;
    private boolean prevGrabberReleaseButton = false;
    private boolean prevWristButton = false;
    private boolean prevRelicModeButton = false;
    private boolean prevRelicGrabberButton = false;
    private boolean prevAutoGrabberButton = false;

    @Override
    public void init()
    {
        //initialize hardware variables by calling the init function of the RRBotHardware class via the robot object
        robot.init(hardwareMap);

        telemetry.addData("Robot","is init");
    }

    @Override
    public void loop()
    {
        //multiple update methods that are called to update different robot mechanisms

        DriveUpdate();
        JewelArmUpdate();

        //make sure glyph arm is homed before allowing it to move
        if(glyphArm.hasHomed())
        {
            GlyphArmUpdate();
        }
        else
        {
            glyphArm.HomeArm();
        }

        Telemetry();
    }

    @Override
    public void stop()
    {
        robot.servoPowerModule.setPower(0);
    }

    public void DriveUpdate()
    {
        if(!drive.getIsAutoMove())
        {
            drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
        }
        else
        {
            drive.AutoMoveEndCheck();
        }
    }

    public void JewelArmUpdate()
    {
        //press the a button to reset the jewel arm, just in case it goes wild during teleop
        if(gamepad1.a)
        {
            robot.jewelArmServo1.setPosition(robot.JEWEL_ARM_SERVO_1_START_POS);
        }
    }
    
    public void GlyphArmUpdate()
    {
        glyphArm.UpdateValues();

        //toggle relic mode when left trigger is pressed, only detects rising edge of button press
        if(gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD && !prevRelicModeButton)
        {
            prevRelicModeButton = true;
            glyphArm.ToggleRelicMode();
        }
        if(!(gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD))
        {
            prevRelicModeButton = false;
        }

        if(!glyphArm.isInRelicMode()) //not relic mode
        {
            //move the glyph arm to a certain state when a button is pressed
            if(gamepad2.start)
            {
                glyphArm.MoveToStartPos();
            }
            else if(gamepad2.right_bumper)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT_PICKUP);
            }
            else if(gamepad2.dpad_down)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT_PICKUP_2);
            }
            else if(gamepad2.a)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT1);
            }
            else if(gamepad2.b)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT2);
            }
            else if(gamepad2.x)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT3);
            }
            else if(gamepad2.y)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.FRONT4);
            }

            //Run the auto glyph place routine when gamepad 1 right trigger is pressed
            if(gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD)
            {
                glyphArm.setIsAutoGlyphPlace(true);
            }

            /*else if(gamepad2.dpad_down)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.BACK1);
            }
            else if(gamepad2.dpad_right)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.BACK2);
            }
            else if(gamepad2.dpad_left)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.BACK3);
            }
            else if(gamepad2.dpad_up)
            {
                glyphArm.MoveGlyphArmToState(GlyphArmState.BACK4);
            }*/

            //flip the wrist when right trigger is pressed, only detects rising edge of button press
            if(gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD && !prevWristButton)
            {
                prevWristButton = true;
                glyphArm.FlipWrist();
            }
            if(!(gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD))
            {
                prevWristButton = false;
            }

            //open/close grabber when right bumper is pressed, only detects rising edge of button press
            if(gamepad1.right_bumper && !prevGrabberOpenButton)
            {
                prevGrabberOpenButton = true;
                glyphArm.MoveGrabber("open");
            }
            if(!gamepad1.right_bumper)
            {
                prevGrabberOpenButton = false;
            }

            //release glyph/close grabber when left bumper is pressed, only detects rising edge of button press
            if(gamepad1.left_bumper && !prevGrabberReleaseButton)
            {
                prevGrabberReleaseButton = true;
                glyphArm.MoveGrabber("release");
            }
            if(!gamepad1.left_bumper)
            {
                prevGrabberReleaseButton = false;
            }

            //auto grabber close button code
            if(gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD)
            {
                glyphArm.AutoCloseGrabber();
            }
            if(gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD && !prevAutoGrabberButton)
            {
                prevAutoGrabberButton = true;
                glyphArm.setHasGrabberClosed(false);
            }
            if(!(gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD))
            {
                prevAutoGrabberButton = false;
            }

        }
        else //relic mode
        {
            if(gamepad2.a)
            {
                glyphArm.MoveToRelicPickupPos(1);
            }
            else if(gamepad2.dpad_down)
            {
                glyphArm.MoveToRelicPickupPos(2);
            }
            else if(gamepad2.b)
            {
                glyphArm.MoveToRelicPlacePos();
            }
            else if(gamepad2.x)
            {
                glyphArm.MoveToRelicDonePos();
            }

            //open/close relic grabber when right bumper is pressed, only detects rising edge of button press
            if((gamepad1.right_bumper || gamepad2.right_bumper) && !prevRelicGrabberButton)
            {
                prevRelicGrabberButton = true;
                glyphArm.MoveRelicGrabber();
            }
            if(!(gamepad1.right_bumper || gamepad2.right_bumper))
            {
                prevRelicGrabberButton = false;
            }
        }

        //if y axis of left joystick is outside the deadzone, EnableArmJoystick is called, which enables the arm to be moved via joystick
        if(Math.abs(gamepad2.left_stick_y) > JOYSTICK_DEADZONE)
        {
            glyphArm.EnableArmJoystick(gamepad2.left_stick_y);
        }

        //if y axis of right joystick is outside deadzone, EnableWristJoystick is called, which enables the arm to be moved via joystick
        if(Math.abs(gamepad2.right_stick_y) > JOYSTICK_DEADZONE)
        {
            glyphArm.EnableWristJoystick(gamepad2.right_stick_y);
        }
    }

    //debug values to be shown on driver station
    public void Telemetry()
    {
        telemetry.addData("grabber1SwitchState", glyphArm.getGrabber1SwitchState());
        telemetry.addData("grabber2SwitchState", glyphArm.getGrabber2SwitchState());
        telemetry.addData("ArmMotorPosition", glyphArm.getArmPos());
        telemetry.addData("WristMotorPosition", glyphArm.getWristPos());
        telemetry.addData("startLimit", glyphArm.getStartLimitState());
        telemetry.addData("endLimit", glyphArm.getEndLimitState());
        telemetry.addData("relicMode", glyphArm.isInRelicMode());
        telemetry.addData("ArmCurrentState", glyphArm.getCurrentArmState());
        telemetry.addData("ArmPrevState", glyphArm.getPrevArmState());
        telemetry.addData("WristCurrentState", glyphArm.getCurrentWristState());
        telemetry.addData("WristPrevState", glyphArm.getPrevWristState());
        telemetry.addData("joystickValue", glyphArm.getJoystickValue());
        telemetry.update();
    }
}