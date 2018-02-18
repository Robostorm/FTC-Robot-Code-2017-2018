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
 * UNFINISHED Autonomous opmode that gets glyphs from the glyph pit and places them
 * @author Andrew Hollabaugh
 * @since 2018-2-8
 */
@Autonomous(name = "RRBotAutoMultiGlyph")
@Disabled
public class RRBotAutoMultiGlyph extends LinearOpMode
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
        //vuforia.close();

        //place the first glyph based on pictograph
        driveAuto.AutoPlaceGlyph(allianceColor, fieldPos, pictograph);

        sleep(500);

        //get the glyph
        driveAuto.MultiGlyphGet(allianceColor, fieldPos, pictograph);

        sleep(500);

        //turn off servo power module (connected to a motor output port)
        robot.servoPowerModule.setPower(0);
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
}
