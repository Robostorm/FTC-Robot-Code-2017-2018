package org.firstinspires.ftc.teamcode.FourWheelBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This TeleOp Opmode is for Mecanum drive teleop
 * Uses the FourWheelBot Hardware class for a robot with four drive wheels, one motor per wheel
 */

@TeleOp(name="4WheelBot: Teleop Mecanum")
//@Disabled
public class FourWheelBotTeleopMecanum extends OpMode
{
    //define hardware class
    Hardware4WheelBot robot = new Hardware4WheelBot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
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
}
