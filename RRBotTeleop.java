package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by andrew on 9/10/17.
 */

//Teleop Opmode class for Team 12601's Relic Recovery Robot (2017-2018)

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    //construct an RRBotHardware object to reference its stuff
    RRBotHardware robot = new RRBotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    private final double GLYPH_ARM_SPEED = 0.3;
    private final double GLYPH_WRIST_SPEED = 0.3;
    private final double GRABBER_OPEN_POS = 0;
    private final double GRABBER_CLOSE_POS = 1;
    private final double GAMEPAD_TRIGGER_THRESHOLD = 0.3;

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
        MecanumDrive();
        //GlyphArmUpdate();
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
        if(gamepad1.left_bumper)
        {
            robot.jewelArmServo.setPosition(robot.JEWEL_ARM_SERVO_START_POS);
        }
    }
    
    /*public void GrabberUpdate()
    {
        
    }

    public void GlyphArmUpdate()
    {
        //robot.glyphArmMotor1.setPower(-gamepad2.left_stick_y);
        
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
            MoveGlyphWristToState(GlyphWristState.FRONT);
        }
        else if(gamepad2.y)
        {
            MoveGlyphArmToState(GlyphArmState.FRONT4);
            MoveGlyphWristToState(GlyphWristState.FRONT);
        }
        else if(gamepad2.dpad_down)
        {
            MoveGlyphArmToState(GlyphArmState.BACK1);
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
        else if(gamepad2.dpad_right)
        {
            MoveGlyphArmToState(GlyphArmState.BACK2);
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
        else if(gamepad2.dpad_left)
        {
            MoveGlyphArmToState(GlyphArmState.BACK3);
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
        else if(gamepad2.dpad_up)
        {
            MoveGlyphArmToState(GlyphArmState.BACK4);
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
        else if(gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD)
        {
            MoveGlyphWristToState(GlyphWristState.FRONT);
        }
        else if(gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD)
        {
            MoveGlyphWristToState(GlyphWristState.BACK);
        }
        
        if(robot.glyphStartLimit.getState())
        {
            glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void MoveGlyphArmToState(GlyphArmState state)
    {
        if(!(glyphArmMotor1.isBusy))
        {
            glyphArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION;
            robot.glyphArmMotor1.setTargetPosition(state.getArmEncoderPos);
            robot.glyphArmMotor1.setPower(GLYPH_ARM_SPEED);
        }
    }
    
    public void MoveGlyphWristToState(GlyphWristState state)
    {
        if(!(glyphWristMotor.isBusy))
        {
            glyphWristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.glyphWristMotor.setTargetPosition(state.getWristEncoderPos);
            robot.glyphWristMotor.setPower(GLYPH_WRIST_SPEED);
        }
    }
    
    public void getEncoderPositions()
    {
        telemetry.addData("ArmMotorPosition", robot.glyphArmMotor1.getCurrentPosition);
        telemtery.addData("WristMotorPosition", robot.glyphWristMotor.getCurrentPosition);
        telemetry.update();
    }*/
}