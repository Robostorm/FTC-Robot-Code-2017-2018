package org.firstinspires.ftc.teamcode.FourWheelBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This TeleOp Opmode is for a simple Tank drive teleop
 * Uses the FourWheelBot Hardware class for a robot with four drive wheels, one motor per wheel
 */

@TeleOp(name="4WheelBot: Teleop Tank")
//@Disabled
public class FourWheelBotTeleopTank extends OpMode
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
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.rearLeftMotor.setPower(left);
        robot.rearRightMotor.setPower(right);
        robot.frontLeftMotor.setPower(left);
        robot.frontRightMotor.setPower(right);

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }
}
