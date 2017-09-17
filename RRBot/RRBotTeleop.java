package org.firstinspires.ftc.teamcode.RRBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by andrew on 9/10/17.
 */

//Teleop Opmode class for Team 12601's Relic Recovery Robot (2017-2018)

@TeleOp(name="RRBotTeleop")
public class RRBotTeleop extends OpMode
{
    //construct an RRBotHardware object to reference its stuff
    RRBotHardware robot = new RRBotHardware();

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
        GlyphArm();
    }

    public void MecanumDrive()
    {
        //calculate the magnitude of the vector
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        //calculate the angle of the vector
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

        double rightX = gamepad1.right_stick_x;

        //calculate the motor vectors and add rotation and right stick turbo
        final double v1 = r * Math.cos(robotAngle) + rightX + -gamepad1.right_stick_y;
        final double v2 = r * Math.sin(robotAngle) - rightX + -gamepad1.right_stick_y;
        final double v3 = r * Math.sin(robotAngle) + rightX + -gamepad1.right_stick_y;
        final double v4 = r * Math.cos(robotAngle) - rightX + -gamepad1.right_stick_y;

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

    public void GlyphArm()
    {
        robot.glyphArmMotor.setPower(-gamepad2.left_stick_y);
    }
}