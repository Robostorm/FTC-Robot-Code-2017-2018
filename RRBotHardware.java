package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Hardware class, defines the robot's hardware and initializes it
 * @author Andrew Hollabaugh
 * @since 2017-09-10
 */
public class RRBotHardware
{
    public DcMotor rearRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor glyphArmMotor1 = null;
    public DcMotor glyphWristMotor = null;
    public DcMotor servoPowerModule = null;
    public Servo jewelArmServo1 = null;
    public Servo jewelArmServo2 = null;
    public Servo grabber1Servo = null;
    public Servo grabber2Servo = null;
    public Servo grabberRotateServo = null;
    public Servo relicInitServo = null;
    public Servo relicGrabberServo = null;
    public CRServo grabber1Belt = null;
    public CRServo grabber2Belt = null;
    public ColorSensor jewelArmColor = null;
    public DigitalChannel allianceColorSwitch = null;
    public DigitalChannel fieldPosSwitch = null;
    public DigitalChannel glyphStartLimit = null;
    public DigitalChannel glyphEndLimit = null;
    public DigitalChannel glyphGrabberSwitch1 = null;
    public DigitalChannel glyphGrabberSwitch2 = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public static final double JEWEL_ARM_SERVO_1_START_POS = 1;
    public static final double JEWEL_ARM_SERVO_2_START_POS = 0.1;
    //public static final double GRABBER_START_POS = 0.8;
    //public static final double RELIC_INIT_SERVO_START_POS = 0.2;
    //public static final double RELIC_GRABBER_SERVO_START_POS = 0.8;

    /**
     * Default constructor
     */
    public RRBotHardware(){}

    /**
     * Defines the robot's hardware elements using hardwareMap.
     * Initializes motors with runMode and direction, certain servos to init positions, and sets mode of digitalInputs
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        //Initialize hardware
        rearRightMotor = hwMap.dcMotor.get("rear_right");
        rearLeftMotor = hwMap.dcMotor.get("rear_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        servoPowerModule = hwMap.dcMotor.get("servo_power");
        glyphArmMotor1 = hwMap.dcMotor.get("glyph_arm_1");
        glyphWristMotor = hwMap.dcMotor.get("glyph_wrist");
        jewelArmServo1 = hwMap.servo.get("jewel_arm_1");
        jewelArmServo2 = hwMap.servo.get("jewel_arm_2");
        grabber1Servo = hwMap.servo.get("grabber_1");
        grabber2Servo = hwMap.servo.get("grabber_2");
        grabberRotateServo = hwMap.servo.get("grabber_rotate");
        relicInitServo = hwMap.servo.get("relic_init");
        relicGrabberServo = hwMap.servo.get("relic_grabber");
        grabber1Belt = hwMap.crservo.get("grabber_1_belt");
        grabber2Belt = hwMap.crservo.get("grabber_2_belt");
        jewelArmColor = hwMap.colorSensor.get("arm_color");
        allianceColorSwitch = hwMap.digitalChannel.get("alliance_color");
        fieldPosSwitch = hwMap.digitalChannel.get("field_pos");
        glyphStartLimit = hwMap.digitalChannel.get("glyph_start_limit");
        glyphEndLimit = hwMap.digitalChannel.get("glyph_end_limit");
        glyphGrabberSwitch1 = hwMap.digitalChannel.get("grabber_switch_1");
        glyphGrabberSwitch2 = hwMap.digitalChannel.get("grabber_switch_2");

        //set motors to drive forwards
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        glyphArmMotor1.setDirection(DcMotor.Direction.FORWARD);
        glyphWristMotor.setDirection(DcMotor.Direction.FORWARD);
        servoPowerModule.setDirection(DcMotor.Direction.FORWARD);

        //set motors to zero power
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        glyphArmMotor1.setPower(0);
        glyphWristMotor.setPower(0);

        //servo power module is plugged into motor port, turn it on
        servoPowerModule.setPower(1);

        //set drive motors to run using encoder guidance and glyphArm motors to stop and reset encoders
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphWristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servoPowerModule.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //sets motors to brake mode and servo power module "motor" to coast mode
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphWristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoPowerModule.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //set servos to start positions
        jewelArmServo1.setPosition(JEWEL_ARM_SERVO_1_START_POS);
        jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_START_POS);
        //grabber1Servo.setPosition(GRABBER_START_POS);
        //grabber2Servo.setPosition(GRABBER_START_POS);
        grabberRotateServo.setPosition(RRBotGlyphArm.GRABBER_ROTATE_POS1);
        relicInitServo.setPosition(RRBotGlyphArm.RELIC_INIT_SERVO_GLYPHMODE_POS);
        relicGrabberServo.setPosition(RRBotGlyphArm.RELIC_GRABBER_CLOSE_POS);

        //set continuous rotation servos to run forwards
        grabber1Belt.setDirection(DcMotorSimple.Direction.FORWARD);
        grabber2Belt.setDirection(DcMotorSimple.Direction.FORWARD);

        //set cr servos to zero power
        grabber1Belt.setPower(0);
        grabber1Belt.setPower(0);

        //set digital channels to input mode
        allianceColorSwitch.setMode(DigitalChannel.Mode.INPUT);
        fieldPosSwitch.setMode(DigitalChannel.Mode.INPUT);
        glyphStartLimit.setMode(DigitalChannel.Mode.INPUT);
        glyphEndLimit.setMode(DigitalChannel.Mode.INPUT);
        glyphGrabberSwitch1.setMode(DigitalChannel.Mode.INPUT);
        glyphGrabberSwitch2.setMode(DigitalChannel.Mode.INPUT);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
        {
            try
            {
                Thread.sleep(remaining);
            }
            catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
