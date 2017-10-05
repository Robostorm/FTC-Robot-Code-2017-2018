package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by andrew on 9/10/17.
 */

//Autonomous OpMode for Relic Recovery robot (there will probably be more OpModes)
//Currently uses vuforia to determine which pictograph image is displayed

@Autonomous(name = "RRBotAuto")
public class RRBotAuto extends LinearOpMode
{
    RRBotHardware robot = new RRBotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    //constructs vuforia object
    //RRBotVuforiaClass vuforia = new RRBotVuforiaClass();

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    private final double JEWEL_ARM_SERVO_END_POS = 0.35;
    private final String JEWEL_ARM_COLOR_SENSOR_DIRECTION = "forwards";
    private String allianceColor;
    private final double JEWEL_TURN_SPEED = 0.3;
    private final int TURN_ANGLE = 25;
    private String ballColor = "unknown";
    private String origTurnDirection = null;
    
    private int fieldPos = 0; //position 0 has a cryoptobox on one side, position 1 has cyroptoboxes on both sides
    private final double DISTANCE_TO_SAFE_ZONE_1 = 34;
    private final double DISTANCE_TO_SAFE_ZONE_2_1 = 28; //was 29
    private final double DISTANCE_TO_SAFE_ZONE_2_2 = 13; //was 11
    private final double DISTANCE_TO_SAFE_ZONE_2_3 = 3;
    private final int TURN_90 = 83;
    private final double COUNTS_PER_MOTOR_REV = 560 ;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private final double SAFE_ZONE_DRIVE_SPEED = 0.5;
    private final double SAFE_ZONE_TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //vuforia.Init_Vuforia();

        //gyro initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        //VMark pictograph detection test
        /*while(opModeIsActive())
        {
            vuforia.Track_Target();
            if(vuforia.isVisible)
                telemetry.addData("Pictograph", vuforia.target_name);
            else
                telemetry.addData("Pictograph", "not visible");
            telemetry.update();
        }*/

        if(robot.allianceColorSwitch.getState())
            allianceColor = "red";
        else
            allianceColor = "blue";

        if(robot.fieldPosSwitch.getState())
            fieldPos = 0;
        else
            fieldPos = 1;

        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Field Position", fieldPos);
        telemetry.update();

        MoveJewelArm("down");

        sleep(1000);

        //color sensor determines color of ball
        if (robot.jewelArmColor.blue() > robot.jewelArmColor.red())
                ballColor = "blue";
            else if (robot.jewelArmColor.blue() < robot.jewelArmColor.red())
                ballColor = "red";

        /*while(opModeIsActive())
        {
            if (robot.jewelArmColor.blue() > robot.jewelArmColor.red())
                ballColor = "blue";
            else if (robot.jewelArmColor.blue() < robot.jewelArmColor.red())
                ballColor = "red";

            telemetry.addData("ballColor", ballColor);
            telemetry.update();
        }*/

        //logic to determine whether to knock the ball infront or behind
        if(!(ballColor.equals("unknown")))
        {
            if(JEWEL_ARM_COLOR_SENSOR_DIRECTION.equals("forwards"))
            {
                if(allianceColor.equals("red"))
                {
                    if(ballColor.equals("red"))
                    {
                        KnockBallBehind();
                    }
                    else if(ballColor.equals("blue"))
                    {
                        KnockBallInfront();
                    }
                }
                else if(allianceColor.equals("blue"))
                {
                    if(ballColor.equals("blue"))
                    {
                        KnockBallBehind();
                    }
                    else if(ballColor.equals("red"))
                    {
                        KnockBallInfront();
                    }
                }
            }
            else if(JEWEL_ARM_COLOR_SENSOR_DIRECTION.equals("backwards"))
            {
                if(allianceColor.equals("red"))
                {
                    if(ballColor.equals("blue"))
                    {
                        KnockBallBehind();
                    }
                    else if(ballColor.equals("red"))
                    {
                        KnockBallInfront();
                    }
                }
                else if(allianceColor.equals("blue"))
                {
                    if(ballColor.equals("red"))
                    {
                        KnockBallBehind();
                    }
                    else if(ballColor.equals("blue"))
                    {
                        KnockBallInfront();
                    }
                }
            }
        }
        
        MoveJewelArm("up");

        //sleep(3000);

        if(!ballColor.equals("unknown"))
        {
            if(origTurnDirection.equals("left"))
            {
                TurnByGyro("right", TURN_ANGLE, JEWEL_TURN_SPEED);
            }
            else if(origTurnDirection.equals("right"))
            {
                TurnByGyro("left", TURN_ANGLE, JEWEL_TURN_SPEED);
            }
        }

        sleep(250);
        
        //Move to safe zone using encoders and gyro
        if(allianceColor.equals("red"))
        {
            if (fieldPos == 0)
            {
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_1, DISTANCE_TO_SAFE_ZONE_1, 5);
                sleep(250);
                TurnByGyro("right", TURN_90, SAFE_ZONE_TURN_SPEED);
            }
            else if (fieldPos == 1)
            {
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_2_1, DISTANCE_TO_SAFE_ZONE_2_1, 5);
                sleep(250);
                TurnByGyro("left", TURN_90, SAFE_ZONE_TURN_SPEED);
                sleep(250);
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_2_2, DISTANCE_TO_SAFE_ZONE_2_2, 5);
                TurnByGyro("right", TURN_90, SAFE_ZONE_TURN_SPEED);
                sleep(250);
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_2_3, DISTANCE_TO_SAFE_ZONE_2_3, 3);
            }
        }
        else if(allianceColor.equals("blue"))
        {
            if (fieldPos == 0)
            {
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, -DISTANCE_TO_SAFE_ZONE_1, -DISTANCE_TO_SAFE_ZONE_1, 5);
                sleep(250);
                TurnByGyro("right", TURN_90, SAFE_ZONE_TURN_SPEED);
            }
            else if (fieldPos == 1)
            {
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, -DISTANCE_TO_SAFE_ZONE_2_1, -DISTANCE_TO_SAFE_ZONE_2_1, 5);
                sleep(250);
                TurnByGyro("left", TURN_90, SAFE_ZONE_TURN_SPEED);
                sleep(250);
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_2_2, DISTANCE_TO_SAFE_ZONE_2_2, 5);
                TurnByGyro("left", TURN_90, SAFE_ZONE_TURN_SPEED);
                sleep(250);
                EncoderDrive(SAFE_ZONE_DRIVE_SPEED, DISTANCE_TO_SAFE_ZONE_2_3, DISTANCE_TO_SAFE_ZONE_2_3, 3);
            }
        }
    }
    
    public void MoveJewelArm(String direction)
    {
        if(direction.equals("down"))
        {
            double servoPos = robot.JEWEL_ARM_SERVO_START_POS;
            while(opModeIsActive() && servoPos > JEWEL_ARM_SERVO_END_POS)
            {
                servoPos -= 0.01;
                robot.jewelArmServo.setPosition(servoPos);
                sleep(50);
            }
        }
        else if(direction.equals("up"))
        {
            double servoPos = JEWEL_ARM_SERVO_END_POS;
            while(opModeIsActive() && servoPos < robot.JEWEL_ARM_SERVO_START_POS)
            {
                servoPos += 0.01;
                robot.jewelArmServo.setPosition(servoPos);
                sleep(50);
            }
        }
    }

    public void KnockBallInfront()
    {
        origTurnDirection = "left";
        TurnByGyro("left", TURN_ANGLE, JEWEL_TURN_SPEED);

        telemetry.addData("knock ball", "infront");
        telemetry.update();
    }
    
    public void KnockBallBehind()
    {
        origTurnDirection = "right";
        TurnByGyro("right", TURN_ANGLE, JEWEL_TURN_SPEED);

        telemetry.addData("knock ball", "behind");
        telemetry.update();
    }
    
    public void TurnByGyro(String direction, int angle, double speed)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle;
        
        if(direction.equals("left"))
        {
            robot.rearRightMotor.setPower(speed);
            robot.rearLeftMotor.setPower(-speed);
            robot.frontRightMotor.setPower(speed);
            robot.frontLeftMotor.setPower(-speed);
        }
        else if(direction.equals("right"))
        {
            robot.rearRightMotor.setPower(-speed);
            robot.rearLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(-speed);
            robot.frontLeftMotor.setPower(speed);
        }

        while(opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }
        
        TurnOffMotors();
    }
    
    public void EncoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.rearLeftMotor.setTargetPosition(newRearLeftTarget);
            robot.rearRightMotor.setTargetPosition(newRearRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void TurnOffMotors()
    {
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
