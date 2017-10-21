package org.firstinspires.ftc.teamcode.TetrixBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

/**
 * This Auto Opmode is for a simple Atonamous Tank drive
 * Uses the TwoWheelBot Hardware class for a robot with two drive wheels, one motor per wheel
 */

@Autonomous(name="TetrixBot: Autonomous")
//@Disabled
public class TetrixBotAuto extends OpMode
{
    HardwareTetrixBot robot = new HardwareTetrixBot();

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

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }
}
