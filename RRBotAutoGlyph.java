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
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;

/**
 * Created by andrew on 9/10/17.
 */

//Autonomous OpMode for Relic Recovery robot that runs the jewel arm and places the glyph in the correct column

@Autonomous(name = "RRBotAutoGlyph")
public class RRBotAutoGlyph extends LinearOpMode
{
    RRBotHardware robot = new RRBotHardware();
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);
    RRBotGlyphArm glyphArm = new RRBotGlyphArm(robot);
    private ElapsedTime runtime = new ElapsedTime();
    
    //constructs vuforia object
    RRBotVuforiaClass vuforia = new RRBotVuforiaClass();

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    private final double JEWEL_ARM_SERVO_1_END_POS = 0;
    private final double JEWEL_ARM_SERVO_2_MID_POS = 0.5;
    private final double JEWEL_ARM_SERVO_2_LEFT_POS = 0.15;
    private final double JEWEL_ARM_SERVO_2_RIGHT_POS = 0.9;
    private final String JEWEL_ARM_COLOR_SENSOR_DIRECTION = "forwards";
    private String allianceColor;
    private String ballColor = "unknown";
    
    private int fieldPos = 0; //position 0 has a cryoptobox on one side, position 1 has cyroptoboxes on both sides
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
    private final int TURN_135 = 135;

    private final double GLYPH_PLACE_DISTANCE_0_1_LEFT = 27.5;
    private final double GLYPH_PLACE_DISTANCE_0_1_CENTER = 30; //was 19.5
    private final double GLYPH_PLACE_DISTANCE_0_1_RIGHT = 50; //28
    private final double GLYPH_PLACE_DISTANCE_0_2 = 9;
    private final double GLYPH_PLACE_DISTANCE_0_3 = 3;

    private final double GLYPH_PLACE_DISTANCE_1_1 = 24;
    private final double GLYPH_PLACE_DISTANCE_1_2_LEFT = -10.75;
    private final double GLYPH_PLACE_DISTANCE_1_2_CENTER = -3;
    private final double GLYPH_PLACE_DISTANCE_1_2_RIGHT = 3.75;
    private final double GLYPH_PLACE_DISTANCE_1_3 = 6;
    private final double GLYPH_PLACE_DISTANCE_1_4 = 3;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        vuforia.Init_Vuforia();

        //gyro initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

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

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_MID_POS);

        MoveJewelArm("down");

        sleep(1000);

        //color sensor determines color of ball
        if (robot.jewelArmColor.blue() > robot.jewelArmColor.red())
                ballColor = "blue";
            else if (robot.jewelArmColor.blue() < robot.jewelArmColor.red())
                ballColor = "red";

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

        sleep(750); //wait for servo to move

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_MID_POS);

        MoveJewelArm("up");

        //sleep(3000);

        sleep(250);

        while(!glyphArm.hasHomed())
        {
            glyphArm.HomeArm();
        }

        sleep(250);

        RelicRecoveryVuMark pictograph = ScanPictograph(500);

        if(allianceColor.equals("red"))
        {
            if(fieldPos == 0)
            {
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_CENTER, GLYPH_PLACE_DISTANCE_0_1_CENTER, 7);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_RIGHT, GLYPH_PLACE_DISTANCE_0_1_RIGHT, 10);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_LEFT, GLYPH_PLACE_DISTANCE_0_1_LEFT, 7);
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
                glyphArm.FlipWrist();

                sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2, GLYPH_PLACE_DISTANCE_0_2, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.3);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_0_3, -GLYPH_PLACE_DISTANCE_0_3, 2);

                sleep(300);

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
                glyphArm.FlipWrist();

                sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_3, GLYPH_PLACE_DISTANCE_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.3);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_4, -GLYPH_PLACE_DISTANCE_1_4, 2);

                sleep(300);

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
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_0_1_CENTER, -GLYPH_PLACE_DISTANCE_0_1_CENTER, 7);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_0_1_RIGHT, -GLYPH_PLACE_DISTANCE_0_1_LEFT, 10);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_0_1_LEFT, -GLYPH_PLACE_DISTANCE_0_1_RIGHT, 7);
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
                glyphArm.FlipWrist();

                sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2, GLYPH_PLACE_DISTANCE_0_2, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.3);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_0_3, -GLYPH_PLACE_DISTANCE_0_3, 2);

                sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
            else if(fieldPos == 1)
            {
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_1_1, -GLYPH_PLACE_DISTANCE_1_1, 7);

                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_2_CENTER, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_2_LEFT, 5);
                }
                else
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_2_RIGHT, 5);
                }

                TurnByGyro("left", TURN_135, GLYPH_PLACE_TURN_SPEED);

                glyphArm.UpdateValues();
                glyphArm.FlipWrist();

                sleep(500);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_3, GLYPH_PLACE_DISTANCE_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber1Servo.setPosition(0.3);

                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_4, -GLYPH_PLACE_DISTANCE_1_4, 2);

                sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START);
            }
        }

        sleep(1000);
        robot.servoPowerModule.setPower(0);
    }
    
    public void MoveJewelArm(String direction)
    {
        if(direction.equals("down"))
        {
            double servoPos = robot.JEWEL_ARM_SERVO_1_START_POS;
            while(opModeIsActive() && servoPos > JEWEL_ARM_SERVO_1_END_POS)
            {
                servoPos -= 0.05; //was 0.01
                robot.jewelArmServo1.setPosition(servoPos);
                sleep(50);
            }
        }
        else if(direction.equals("up"))
        {
            double servoPos = JEWEL_ARM_SERVO_1_END_POS;
            while(opModeIsActive() && servoPos < robot.JEWEL_ARM_SERVO_1_START_POS)
            {
                servoPos += 0.05; //was 0.01
                robot.jewelArmServo1.setPosition(servoPos);
                sleep(50);
            }
        }
    }

    public void KnockBallInfront()
    {
        //origTurnDirection = "left";
        //TurnByGyro("left", TURN_ANGLE, JEWEL_TURN_SPEED);

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_LEFT_POS);

        telemetry.addData("knock ball", "infront");
        telemetry.update();
    }
    
    public void KnockBallBehind()
    {
        //origTurnDirection = "right";
        //TurnByGyro("right", TURN_ANGLE, JEWEL_TURN_SPEED);

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_RIGHT_POS);

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
    
    public void EncoderDriveTank(double speed, double leftInches, double rightInches, double timeoutS)
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
        if (opModeIsActive())
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
            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {

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

    //depreciated
    /*public void EncoderDriveMecanum(double speedX, double speedY, double distance, double timeoutS)
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

        double[] velocities = drive.calcVelocities(speedX, speedY, 0, 0, false);

        // Ensure that the opmode is still active
        if(opModeIsActive())
        {
            // Determine new target position, and pass to motor controller
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (velocities[2] * distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (velocities[3] * distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (velocities[0] * distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (velocities[1] * distance * Math.sqrt(2) * COUNTS_PER_INCH);
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
            robot.rearLeftMotor.setPower(Math.abs(velocities[2]));
            robot.rearRightMotor.setPower(Math.abs(velocities[3]));
            robot.frontLeftMotor.setPower(Math.abs(velocities[0]));
            robot.frontRightMotor.setPower(Math.abs(velocities[1]));


            //while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
            //        ((Math.abs(robot.rearLeftMotor.getCurrentPosition() - newRearLeftTarget) > POSITION_THRESHOLD) ||
            //        (Math.abs(robot.rearRightMotor.getCurrentPosition() - newRearRightTarget) > POSITION_THRESHOLD) ||
            //        (Math.abs(robot.frontLeftMotor.getCurrentPosition() - newFrontLeftTarget) > POSITION_THRESHOLD) ||
            //        (Math.abs(robot.frontRightMotor.getCurrentPosition() - newFrontRightTarget) > POSITION_THRESHOLD)))
            //{

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
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
    }*/

    public void EncoderDriveSideways(double speed, double distance, double timeoutS)
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
        if(opModeIsActive())
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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
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

    public void EncoderDriveDiagonal(DcMotor motor1, DcMotor motor2, double speed, double distance, double timeoutS)
    {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newMotor1Target;
        int newMotor2Target;

        // Ensure that the opmode is still active
        if(opModeIsActive())
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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor1.isBusy() && motor2.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newMotor1Target, newMotor2Target);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                telemetry.update());
            }

            TurnOffMotors();

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //scans pictograph for time seconds
    public RelicRecoveryVuMark ScanPictograph(int time)
    {
        RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds() < time)
        {
            vuforia.Track_Target();

            if(vuforia.isVisible)
            {
                pictograph = vuforia.target_name;
                telemetry.addData("Pictograph", vuforia.target_name);
            }
            else
            {
                pictograph = RelicRecoveryVuMark.UNKNOWN;
                telemetry.addData("Pictograph", "not visible");
            }
            telemetry.update();
        }

        return pictograph;
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
