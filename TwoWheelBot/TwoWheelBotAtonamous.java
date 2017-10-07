package org.firstinspires.ftc.teamcode.TwoWheelBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.TwoWheelBot.Hardware2WheelBot;

/**
 * This Atonamous Opmode is for a simple Tank drive Atonamous
 * Uses the TwoWheelBot Hardware class for a robot with two drive wheels, one motor per wheel
 */

@Autonomous(name="2WheelBot:Atonamous")
//@Disabled
public class TwoWheelBotAtonamous extends OpMode
{
    Hardware2WheelBot robot = new Hardware2WheelBot();

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

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);

        if (robot.distanceSensor.equals(5)){
            telemetry.addData("Say", "Distance 5");
        }
        else{
            telemetry.addData("Say", "Not 5");
        }
    }
}
