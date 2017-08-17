package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This TeleOp Opmode is for Holonomic drive teleop
 * Uses the FourWheelBot Hardware class for a robot with four drive wheels, one motor per wheel
 */

@TeleOp(name="4WheelBot: Teleop Holonomic")
//@Disabled
public class FourWheelBotTeleopHolonomic extends OpMode
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
        double speed;
        double angle;
        double rotation;
        double deadZone = 0.05;
        double axisY = -gamepad1.left_stick_y;
        double axisX = -gamepad1.left_stick_x;
        double axisSpin = -gamepad1.right_stick_x;
        double flOut;
        double frOut;
        double rrOut;
        double rlOut;

        if (Math.abs(axisY) < deadZone && Math.abs(axisX) < deadZone && Math.abs(axisSpin) < deadZone) {
            speed = 0;
            angle = 0;
            rotation = 0;
        } else {
            angle = Math.atan2(axisY, axisX) - Math.PI / 2;
            speed = Math.sqrt((axisY * axisY) + (axisX * axisX));
            rotation = axisSpin;
            if (speed > 1)
                speed = 1;

        }

        if (speed > 0) {
            flOut = speed * Math.cos(Math.PI / 4 - angle);
            frOut = speed * Math.cos(Math.PI / 4 + angle);
            rrOut = speed * Math.cos(Math.PI / 4 - angle);
            rlOut = speed * Math.cos(Math.PI / 4 + angle);

            flOut -= rotation;
            frOut += rotation;
            rrOut += rotation;
            rlOut -= rotation;
        } else if (rotation > deadZone || rotation < -deadZone) {
            flOut = -rotation;
            frOut = rotation;
            rrOut = rotation;
            rlOut = -rotation;
        } else {
            flOut = 0;
            frOut = 0;
            rrOut = 0;
            rlOut = 0;
        }

        robot.frontLeftMotor.setPower(flOut);
        robot.frontRightMotor.setPower(frOut);
        robot.rearRightMotor.setPower(rrOut);
        robot.rearLeftMotor.setPower(rlOut);

        telemetry.addData("speed", speed);
        telemetry.addData("angle", angle);
        telemetry.addData("rotation", rotation);
    }
}