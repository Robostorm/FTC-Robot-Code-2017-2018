package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;

/**
 * Created by andrew on 12/22/17.
 */

public class RRBotDriveAuto
{
    RRBotHardware robot;
    LinearOpMode opMode;
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);
    RRBotGlyphArm glyphArm = new RRBotGlyphArm(robot, drive);
    private ElapsedTime runtime = new ElapsedTime();

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    private final int TURN_90 = 83;
    private final double COUNTS_PER_MOTOR_REV = 560 ;    // Andymark 20:1 gearmotor
    private final double DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private final double GLYPH_PLACE_DRIVE_SPEED_SLOW = 0.25;
    private final double GLYPH_PLACE_DRIVE_SPEED = 0.5;
    private final double GLYPH_PLACE_DRIVE_SPEED_FAST = 1;
    private final double GLYPH_PLACE_TURN_SPEED = 0.5;
    private final int TURN_45_0 = 45;
    private final int TURN_45_1 = TURN_90 / 2 - 15;
    private final int TURN_135 = 135 - 15;
    private final int TURN_135_B = 135;

    private final double GLYPH_PLACE_DISTANCE_0_1_LEFT = 27.5;
    private final double GLYPH_PLACE_DISTANCE_0_1_CENTER = 45; //was 30
    private final double GLYPH_PLACE_DISTANCE_0_1_RIGHT = 39; //28
    private final double GLYPH_PLACE_DISTANCE_0_2 = 9;
    private final double GLYPH_PLACE_DISTANCE_0_2_alt = 9;
    private final double GLYPH_PLACE_DISTANCE_0_3 = 5; //was 3

    private final double GLYPH_PLACE_DISTANCE_1_1 = 24;
    private final double GLYPH_PLACE_DISTANCE_1_2_LEFT = -10.75;
    private final double GLYPH_PLACE_DISTANCE_1_2_CENTER = -3;
    private final double GLYPH_PLACE_DISTANCE_1_2_RIGHT = 3.75;
    private final double GLYPH_PLACE_DISTANCE_1_3 = 6;
    private final double GLYPH_PLACE_DISTANCE_1_4 = 5; //was 3

    private final double GLYPH_PLACE_DISTANCE_B_0_1_RIGHT = 30;
    private final double GLYPH_PLACE_DISTANCE_B_0_1_CENTER = 46; //was 30
    private final double GLYPH_PLACE_DISTANCE_B_0_1_LEFT = 40; //28
    private final double GLYPH_PLACE_DISTANCE_B_0_2 = 9;
    private final double GLYPH_PLACE_DISTANCE_B_0_3 = 5; //was 3

    private final double GLYPH_PLACE_DISTANCE_B_1_1 = 27; //was 24
    private final double GLYPH_PLACE_DISTANCE_B_1_2_RIGHT = -5;
    private final double GLYPH_PLACE_DISTANCE_B_1_2_CENTER = 0;
    private final double GLYPH_PLACE_DISTANCE_B_1_2_LEFT = 5.5;
    private final double GLYPH_PLACE_DISTANCE_B_1_3 = 6;
    private final double GLYPH_PLACE_DISTANCE_B_1_4 = 5; //was 3

    public RRBotDriveAuto(RRBotHardware robot, LinearOpMode opMode)
    {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void runOpMode() {}

    public void AutoPlaceGlyph(String allianceColor, int fieldPos, RelicRecoveryVuMark pictograph)
    {
        initGyro();

        while(!glyphArm.hasHomed())
        {
            glyphArm.HomeArm();
        }

        opMode.sleep(250);

        if(allianceColor.equals("red"))
        {
            if(fieldPos == 0)
            {
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_CENTER, GLYPH_PLACE_DISTANCE_0_1_CENTER, 30);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_RIGHT, GLYPH_PLACE_DISTANCE_0_1_RIGHT, 30);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_LEFT, GLYPH_PLACE_DISTANCE_0_1_LEFT, 30);
                }

                if(pictograph == RelicRecoveryVuMark.RIGHT || pictograph == RelicRecoveryVuMark.CENTER)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                }
                else
                {
                    TurnByGyro("right", TURN_45_0, GLYPH_PLACE_TURN_SPEED);
                }

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK);

                opMode.sleep(500);

                if(pictograph == RelicRecoveryVuMark.LEFT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2, GLYPH_PLACE_DISTANCE_0_2, 5);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2_alt, GLYPH_PLACE_DISTANCE_0_2_alt, 5);
                }

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.5);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_0_3, -GLYPH_PLACE_DISTANCE_0_3, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
            else if(fieldPos == 1)
            {
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_1_1, GLYPH_PLACE_DISTANCE_1_1, 7);

                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_CENTER, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_RIGHT, 5);
                }
                else
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_LEFT, 5);
                }

                TurnByGyro("left", TURN_45_1, GLYPH_PLACE_TURN_SPEED);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK);

                opMode.sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_3, GLYPH_PLACE_DISTANCE_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.5);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_4, -GLYPH_PLACE_DISTANCE_1_4, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
        }
        else if(allianceColor.equals("blue"))
        {
            if(fieldPos == 0)
            {
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_CENTER, -GLYPH_PLACE_DISTANCE_B_0_1_CENTER, 7);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_RIGHT, -GLYPH_PLACE_DISTANCE_B_0_1_RIGHT, 10);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_LEFT, -GLYPH_PLACE_DISTANCE_B_0_1_LEFT, 7);
                }

                if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                }
                else
                {
                    TurnByGyro("right", TURN_45_0, GLYPH_PLACE_TURN_SPEED);
                }

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK);

                opMode.sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_0_2, GLYPH_PLACE_DISTANCE_B_0_2, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.5);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_B_0_3, -GLYPH_PLACE_DISTANCE_B_0_3, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
            else if(fieldPos == 1)
            {
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_1_1, -GLYPH_PLACE_DISTANCE_B_1_1, 7);

                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_CENTER, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_RIGHT, 5);
                }
                else
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_LEFT, 5);
                }

                TurnByGyro("left", TURN_135_B, GLYPH_PLACE_TURN_SPEED);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK);

                opMode.sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_3, GLYPH_PLACE_DISTANCE_B_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.5);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_B_1_4, -GLYPH_PLACE_DISTANCE_B_1_4, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
        }
    }

    public void MultiGlyphInit(String allianceColor, int fieldPos, RelicRecoveryVuMark pictograph)
    {
        if(allianceColor.equals("red"))
        {
            if(fieldPos == 0)
            {
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, 3, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, 6, 5);
                }
                else
                {
                    TurnByGyro("left", TURN_135, GLYPH_PLACE_TURN_SPEED);
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, -3, 5);
                }
            }

            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, 10, 10, 10);
            TurnByGyro("right", TURN_90, GLYPH_PLACE_TURN_SPEED);
        }
        else if(allianceColor.equals("blue"))
        {
            if(fieldPos == 0)
            {

            }
        }
    }

    public void MultiGlyphAlignPickup(double alignDistance)
    {
        EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, alignDistance, alignDistance, 2);

        glyphArm.UpdateValues();
        glyphArm.MoveGlyphWristToState(GlyphWristState.BACK);

        glyphArm.UpdateValues();
        robot.grabber1Servo.setPosition(0.25);

        TurnByGyro("left", TURN_90, GLYPH_PLACE_DRIVE_SPEED);

        EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, 5, 5, 5);

        glyphArm.UpdateValues();
        robot.grabber1Servo.setPosition(0.8);

        EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -10, -10, 10);
    }

    public void initGyro()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

        while(opMode.opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            opMode.telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            opMode.telemetry.update();
        }

        TurnOffMotors();
    }

    public void EncoderDriveTank(double speed, double leftInches, double rightInches, double timeoutS)
    {
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if(opMode.opModeIsActive())
        {
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
            while(opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void EncoderDriveSideways(double speed, double distance, double timeoutS)
    {
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newRearLeftTarget;
        int newRearRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if(opMode.opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
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
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void EncoderDriveDiagonal(DcMotor motor1, DcMotor motor2, double speed, double distance, double timeoutS)
    {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newMotor1Target;
        int newMotor2Target;

        // Ensure that the opmode is still active
        if(opMode.opModeIsActive())
        {
            newMotor1Target = motor1.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newMotor2Target = motor2.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            motor1.setTargetPosition(newMotor1Target);
            motor2.setTargetPosition(newMotor2Target);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motor1.setPower(Math.abs(speed));
            motor2.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while(opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor1.isBusy() && motor2.isBusy()))
            {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newMotor1Target, newMotor2Target);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        opMode.telemetry.update());
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void TurnOffMotors()
    {
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.frontLeftMotor.setPower(0);
    }

    //gyro angle formatting methods
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
