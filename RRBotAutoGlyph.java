package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous opmode that knocks the jewel and places glyph in correct column
 * @author Andrew Hollabaugh
 * @since 2017-9-10
 */
@Autonomous(name = "RRBotAutoGlyph")
//@Disabled
public class RRBotAutoGlyph extends LinearOpMode
{
    //construct other RRBot objects
    RRBotHardware robot = new RRBotHardware();
    RRBotJewelArm jewelArm = new RRBotJewelArm(robot, this);
    RRBotMecanumDrive drive = new RRBotMecanumDrive(robot);
    RRBotGlyphArm glyphArm = new RRBotGlyphArm(robot, drive);
    RRBotDriveAuto driveAuto = new RRBotDriveAuto(robot, glyphArm, this);

    private ElapsedTime runtime = new ElapsedTime();

    //constructs vuforia object
    RRBotVuforiaClass vuforia = new RRBotVuforiaClass();

    private String allianceColor;
    private int fieldPos = 0; //position 0: towards the audience, position 1: away from the audience

    /**
     * Runs the opmode
     */
    @Override
    public void runOpMode()
    {
        //initialize the hardware
        robot.init(hardwareMap);

        //initialize vuforia
        vuforia.Init_Vuforia();

        waitForStart();

        //set the alliance color depending on the state of a physical switch
        if(robot.allianceColorSwitch.getState())
            allianceColor = "red";
        else
            allianceColor = "blue";

        //set the field position depending on the state of a physical switch
        if(robot.fieldPosSwitch.getState())
            fieldPos = 0;
        else
            fieldPos = 1;

        //report the alliance color and field position to the driver station
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Field Position", fieldPos);
        telemetry.update();

        //run the jewel routine
        jewelArm.RunRoutine(allianceColor);

        sleep(250);

        //scan the pictograph for 1/2 second
        RelicRecoveryVuMark pictograph = ScanPictograph(500);

        //place the glyph based on pictograph
        driveAuto.AutoPlaceGlyph(allianceColor, fieldPos, pictograph);

        sleep(500);

        //turn off servo power module (connected to a motor output port)
        robot.servoPowerModule.setPower(0);
    }

    /**
     * Scans the pictograph using vuforia for a specified time
     * @param time scanning time
     * @return the pictograph object
     */
    public RelicRecoveryVuMark ScanPictograph(int time)
    {
        //initialize the pictograph as unknown
        RelicRecoveryVuMark pictograph = RelicRecoveryVuMark.UNKNOWN;

        runtime.reset();

        //keep looping until the specified time has passed
        while(opModeIsActive() && runtime.milliseconds() < time)
        {
            //scan the VuMark and update its values accordingly
            vuforia.Track_Target();

            //if the pictograph is visible, get its name and report to driver station
            if(vuforia.isVisible)
            {
                pictograph = vuforia.target_name;
                telemetry.addData("Pictograph", vuforia.target_name);
            }
            else //if no pictograph is found, set to unknown, report to driver station
            {
                pictograph = RelicRecoveryVuMark.UNKNOWN;
                telemetry.addData("Pictograph", "not visible");
            }
            telemetry.update();
        }

        return pictograph;
    }
}
