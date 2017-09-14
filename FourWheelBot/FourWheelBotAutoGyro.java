package org.firstinspires.ftc.teamcode.FourWheelBot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by andrew on 8/24/17.
 */

@Autonomous(name="4WheelBotAutoGyro")
//@Disabled
public class FourWheelBotAutoGyro extends LinearOpMode
{
    Hardware4WheelBot robot = new Hardware4WheelBot();
    public BNO055IMU imu;
    Orientation angles;

    final double TURN_SPEED = 0.3;

    public void runOpMode()
    {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        //test heading values
        /*while(opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }*/

        //test turn robot 90 degrees right
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;
        robot.rearRightMotor.setPower(-TURN_SPEED);
        robot.rearLeftMotor.setPower(TURN_SPEED);
        robot.frontRightMotor.setPower(-TURN_SPEED);
        robot.frontLeftMotor.setPower(TURN_SPEED);
        while(opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < 90)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }
        turnOffMotors();
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void turnOffMotors()
    {
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }
}