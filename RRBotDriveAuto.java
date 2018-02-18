package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
 * Functionality to move automatically move robot and use glyph arm during autonomous mode.
 * @author Andrew Hollabaugh
 * @since 2017-12-22
 */
public class RRBotDriveAuto
{
    RRBotHardware robot;
    RRBotGlyphArm glyphArm;
    LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;


    private final double COUNTS_PER_MOTOR_REV = 560; //andymark 20:1 gearbox
    private final double DRIVE_GEAR_REDUCTION = 2.0; //drive train geared down 1:2
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private final double GLYPH_PLACE_DRIVE_SPEED_SLOW = 0.25;
    private final double GLYPH_PLACE_DRIVE_SPEED = 0.7;
    private final double GLYPH_PLACE_TURN_SPEED = 0.6;
    private final int TURN_90 = 83;
    private final int TURN_45_0 = 45;
    private final int TURN_45_1 = TURN_90 / 2 - 15;
    private final int TURN_135 = 135 - 15;
    private final int TURN_135_B = 135;

    private final double GLYPH_PLACE_DISTANCE_0_1_LEFT = 27.5;
    private final double GLYPH_PLACE_DISTANCE_0_1_CENTER = 45;
    private final double GLYPH_PLACE_DISTANCE_0_1_RIGHT = 39;
    private final double GLYPH_PLACE_DISTANCE_0_2 = 9;
    private final double GLYPH_PLACE_DISTANCE_0_2_alt = 9;
    private final double GLYPH_PLACE_DISTANCE_0_3 = 5;

    private final double GLYPH_PLACE_DISTANCE_1_1 = 24;
    private final double GLYPH_PLACE_DISTANCE_1_2_LEFT = -10.75;
    private final double GLYPH_PLACE_DISTANCE_1_2_CENTER = -3;
    private final double GLYPH_PLACE_DISTANCE_1_2_RIGHT = 3.75;
    private final double GLYPH_PLACE_DISTANCE_1_3 = 6;
    private final double GLYPH_PLACE_DISTANCE_1_4 = 5;

    private final double GLYPH_PLACE_DISTANCE_B_0_1_RIGHT = 30;
    private final double GLYPH_PLACE_DISTANCE_B_0_1_CENTER = 46;
    private final double GLYPH_PLACE_DISTANCE_B_0_1_LEFT = 40;
    private final double GLYPH_PLACE_DISTANCE_B_0_2 = 9;
    private final double GLYPH_PLACE_DISTANCE_B_0_3 = 5;

    private final double GLYPH_PLACE_DISTANCE_B_1_1 = 27;
    private final double GLYPH_PLACE_DISTANCE_B_1_2_RIGHT = -5;
    private final double GLYPH_PLACE_DISTANCE_B_1_2_CENTER = 0;
    private final double GLYPH_PLACE_DISTANCE_B_1_2_LEFT = 5.5;
    private final double GLYPH_PLACE_DISTANCE_B_1_3 = 6;
    private final double GLYPH_PLACE_DISTANCE_B_1_4 = 5;

    /**
     * Constructor gets hardware, glyph arm, and opmode objects from the opmode when it is constructed
     * @param robot Hardware class for the robot
     * @param glyphArm For controlling the glyph arm
     * @param opMode Opmode object, used for things like sleep and whileOpModeIsActive
     */
    public RRBotDriveAuto(RRBotHardware robot, RRBotGlyphArm glyphArm, LinearOpMode opMode)
    {
        this.robot = robot;
        this.glyphArm = glyphArm;
        this.opMode = opMode;
    }

    public void runOpMode() {}

    /**
     * Places the starting glyph in autonomous mode in the correct column
     * @param allianceColor "red" or "blue"
     * @param fieldPos the balancing stone the robot is on (0 for towards audience, 1 for away from audience)
     * @param pictograph pictograph object scanned using vuforia
     */
    public void AutoPlaceGlyph(String allianceColor, int fieldPos, RelicRecoveryVuMark pictograph)
    {
        initGyro();

        //home the glyph arm if not already homed
        while(!glyphArm.hasHomed())
        {
            glyphArm.HomeArm();
        }

        opMode.sleep(250);

        if(allianceColor.equals("red"))
        {
            if(fieldPos == 0)
            {
                //red 0 start position

                //drive forwards a differing amount depending on pictograph
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_CENTER, GLYPH_PLACE_DISTANCE_0_1_CENTER, 30);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_RIGHT, GLYPH_PLACE_DISTANCE_0_1_RIGHT, 30);
                }
                else //left column is the fallback if vuforia fails
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_0_1_LEFT, GLYPH_PLACE_DISTANCE_0_1_LEFT, 30);
                }

                //turn 135 degrees or 45 degrees depending on pictograph
                if(pictograph == RelicRecoveryVuMark.RIGHT || pictograph == RelicRecoveryVuMark.CENTER)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                }
                else
                {
                    TurnByGyro("right", TURN_45_0, GLYPH_PLACE_TURN_SPEED);
                }

                glyphArm.UpdateValues(); //must be run before every call from GlyphArm class to update arm/wrist positions and motor speed
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK); //move the wrist to the BACK position

                opMode.sleep(500);

                //move forwards a differing amount depending on the pictograph
                if(pictograph == RelicRecoveryVuMark.LEFT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2, GLYPH_PLACE_DISTANCE_0_2, 5);
                }
                else
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_0_2_alt, GLYPH_PLACE_DISTANCE_0_2_alt, 5);
                }

                glyphArm.UpdateValues();
                robot.grabber2Servo.setPosition(glyphArm.GRABBER_RELEASE_POS); //release the glyph

                //back up so robot is not touching the glyph
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_0_3, -GLYPH_PLACE_DISTANCE_0_3, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START); //move wrist back to vertical START position so it is zeroed for teleop
            }
            else if(fieldPos == 1)
            {
                //red 1 start position

                //drive forwards
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, GLYPH_PLACE_DISTANCE_1_1, GLYPH_PLACE_DISTANCE_1_1, 7);

                //strafe left or right a differing amount depending on the pictograph
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_CENTER, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_RIGHT, 5);
                }
                else //left column is the fallback if vuforia fails
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_2_LEFT, 5);
                }

                //turn 45 degrees
                TurnByGyro("left", TURN_45_1, GLYPH_PLACE_TURN_SPEED);

                glyphArm.UpdateValues(); //must be run before every call from GlyphArm class to update arm/wrist positions and motor speed
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK); //move wrist to BACK position

                opMode.sleep(500);

                //drive forwards
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_1_3, GLYPH_PLACE_DISTANCE_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber2Servo.setPosition(glyphArm.GRABBER_RELEASE_POS); //release the glyph

                //move backwards so robot is not touching the glyph
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_1_4, -GLYPH_PLACE_DISTANCE_1_4, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START); //move wrist back to vertical START position so it is zeroed for teleop
            }
        }
        else if(allianceColor.equals("blue"))
        {
            if(fieldPos == 0)
            {
                //blue 0 start position

                //drive backwards a differing amount depending on the pictograph
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_CENTER, -GLYPH_PLACE_DISTANCE_B_0_1_CENTER, 7);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_RIGHT, -GLYPH_PLACE_DISTANCE_B_0_1_RIGHT, 10);
                }
                else //left column is the fallback if vuforia fails
                {
                    EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_0_1_LEFT, -GLYPH_PLACE_DISTANCE_B_0_1_LEFT, 7);
                }

                //turn a differing amount depending on the pictograph
                if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    TurnByGyro("right", TURN_135, GLYPH_PLACE_TURN_SPEED);
                }
                else
                {
                    TurnByGyro("right", TURN_45_0, GLYPH_PLACE_TURN_SPEED);
                }

                glyphArm.UpdateValues(); //must be run before every call from GlyphArm class to update arm/wrist positions and motor speed
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK); //move wrist to BACK position

                opMode.sleep(500);

                //move forwards
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_0_2, GLYPH_PLACE_DISTANCE_B_0_2, 5);

                glyphArm.UpdateValues();
                robot.grabber2Servo.setPosition(glyphArm.GRABBER_RELEASE_POS); //release the glyph

                //move backwards so robot is not touching the glyph
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_B_0_3, -GLYPH_PLACE_DISTANCE_B_0_3, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START); //move wrist back to vertical START position so it is zeroed for teleop
            }
            else if(fieldPos == 1)
            {
                //blue 1 start position

                //drive backwards
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED_SLOW, -GLYPH_PLACE_DISTANCE_B_1_1, -GLYPH_PLACE_DISTANCE_B_1_1, 7);

                //strafe left or right a differing amount depending on pictograph
                if(pictograph == RelicRecoveryVuMark.CENTER)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_CENTER, 5);
                }
                else if(pictograph == RelicRecoveryVuMark.RIGHT)
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_RIGHT, 5);
                }
                else //left column is the fallback if vuforia fails
                {
                    EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_2_LEFT, 5);
                }

                //turn 135 degrees
                TurnByGyro("left", TURN_135_B, GLYPH_PLACE_TURN_SPEED);

                glyphArm.UpdateValues(); //must be run before every call from GlyphArm class to update arm/wrist positions and motor speed
                glyphArm.MoveGlyphWristToState(GlyphWristState.BACK); //move wrist to BACK position

                opMode.sleep(500);

                //drive forwards
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, GLYPH_PLACE_DISTANCE_B_1_3, GLYPH_PLACE_DISTANCE_B_1_3, 5);

                glyphArm.UpdateValues();
                robot.grabber2Servo.setPosition(glyphArm.GRABBER_RELEASE_POS); //release the glyph

                //drive backwards so robot is not touching the glyph
                EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -GLYPH_PLACE_DISTANCE_B_1_4, -GLYPH_PLACE_DISTANCE_B_1_4, 2);

                opMode.sleep(300);

                glyphArm.UpdateValues();
                glyphArm.MoveGlyphWristToState(GlyphWristState.START); //move wrist back to vertical START position so it is zeroed for teleop
            }
        }
    }

    /**
     * UNFINISHED Gets a glyph from the glyph pit during autonomous using dead reckoning
     * @param allianceColor "red" or "blue"
     * @param fieldPos starting balancing stone
     * @param pictograph pictograph object read using vuforia
     */
    public void MultiGlyphGet(String allianceColor, int fieldPos, RelicRecoveryVuMark pictograph)
    {
        if(fieldPos == 0 && allianceColor.equals("red"))
        {
            //move backwards
            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -2, -2, 5);

            //turn and strafe depending on the column the first glyph was place so the robot is facing the glyph pile and in the center
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
                TurnByGyro("left", TURN_135_B, GLYPH_PLACE_TURN_SPEED);
                EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, 4.25, 5);
            }

            glyphArm.UpdateValues();
            glyphArm.MoveGlyphWristToState(GlyphWristState.BACK); //move wrist to BACK position

            opMode.sleep(500);

            glyphArm.UpdateValues();
            glyphArm.RotateGrabber(); //move the second grabber up so two glyphs can be picked up at once

            opMode.sleep(500);

            glyphArm.UpdateValues();
            //open grabbers
            robot.grabber1Servo.setPosition(glyphArm.GRABBER_OPEN_POS);
            robot.grabber2Servo.setPosition(glyphArm.GRABBER_OPEN_POS);

            //move forwards into glyph pile
            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, 26, 26, 10);

            glyphArm.UpdateValues();
            //close grabbers and turn on belts
            robot.grabber1Servo.setPosition(glyphArm.GRABBER_CLOSE_POS);
            robot.grabber1Belt.setPower(glyphArm.GRABBER_BELT_SPEED);
            robot.grabber2Servo.setPosition(glyphArm.GRABBER_CLOSE_POS);
            robot.grabber2Belt.setPower(glyphArm.GRABBER_BELT_SPEED);

            //move backwards back to cryptobox
            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -20, -20, 10);

            //turn 135 degrees
            TurnByGyro("left", TURN_135_B, GLYPH_PLACE_TURN_SPEED);

            //turn off grabber belts
            robot.grabber1Belt.setPower(0);
            robot.grabber2Belt.setPower(0);

            //strafe to align with column
            EncoderDriveSideways(GLYPH_PLACE_DRIVE_SPEED, -2, 5);

            //drive forwards into cryptobox
            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, 5, 5, 10);

            glyphArm.UpdateValues();
            //release glyph(s)
            robot.grabber1Servo.setPosition(glyphArm.GRABBER_RELEASE_POS);
            robot.grabber2Servo.setPosition(glyphArm.GRABBER_RELEASE_POS);

            //move backwards so robot is not touching glyph
            EncoderDriveTank(GLYPH_PLACE_DRIVE_SPEED, -5, -5, 10);

            glyphArm.UpdateValues();
            glyphArm.RotateGrabber();

            opMode.sleep(500);

            glyphArm.UpdateValues();
            glyphArm.MoveGlyphWristToState(GlyphWristState.START);
        }
    }

    /**
     * Initialize the BNO055IMU gyro
     */
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

    /**
     * Turn the robot a specified angle using the gyro
     * @param direction "left" or "right"
     * @param angle turn angle in degrees
     * @param speed turn speed
     */
    public void TurnByGyro(String direction, int angle, double speed)
    {
        //get angle values from the gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle; //define the starting angle

        //turn on motors to turn the robot in the specified direction
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

        //keep looping until the difference between the current heading and the start heading equals the specified angle
        while(opMode.opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            //get values from the gyro
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //report the current heading to the driver station
            opMode.telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            opMode.telemetry.update();
        }

        TurnOffMotors();
    }

    /**
     * Moves the robot in the tank drive style
     * @param speed motor speed
     * @param leftInches distance for left side of robot in inches
     * @param rightInches distance for right side of robot in inches
     * @param timeoutS timeout in seconds
     */
    public void EncoderDriveTank(double speed, double leftInches, double rightInches, double timeoutS)
    {
        //reset encoders
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

        //ensure that the opmode is still active
        if(opMode.opModeIsActive())
        {
            //determine target positions
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.rearLeftMotor.setTargetPosition(newRearLeftTarget);
            robot.rearRightMotor.setTargetPosition(newRearRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            //turn on motors to specified speed
            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            //keep looping until one of the motors finished its movement
            while(opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {

                //report target and current positions to driver station
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            TurnOffMotors();

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Moves the robot side to side (strafing) a specified distance
     * @param speed drive motor speed
     * @param distance strafe distance in inches
     * @param timeoutS timeout in seconds
     */
    public void EncoderDriveSideways(double speed, double distance, double timeoutS)
    {
        //reset encoders
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

        //ensure that the opmode is still active
        if(opMode.opModeIsActive())
        {
            //calculate target positions, negative for two motors so the robot strafes
            newRearLeftTarget = robot.rearLeftMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newRearRightTarget = robot.rearRightMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeftMotor.getCurrentPosition() + (int) (distance * Math.sqrt(2) * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightMotor.getCurrentPosition() + (int) (-distance * Math.sqrt(2) * COUNTS_PER_INCH);
            robot.rearLeftMotor.setTargetPosition(newRearLeftTarget);
            robot.rearRightMotor.setTargetPosition(newRearRightTarget);
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            robot.rearLeftMotor.setPower(Math.abs(speed));
            robot.rearRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.frontRightMotor.setPower(Math.abs(speed));

            //keep looping until one of the motors finished its movement
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy()))
            {
                //report current and target positions to driver station
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newRearLeftTarget, newRearRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.rearLeftMotor.getCurrentPosition(),
                        robot.rearRightMotor.getCurrentPosition(),
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            TurnOffMotors();

            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns off all drive motors
     */
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
