package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by andrew on 9/10/17.
 */

//Autonomous OpMode for Relic Recovery robot that runs the jewel arm and places the glyph in the correct column

@Autonomous(name = "RRBotAutoMultiGlyph")
public class RRBotAutoMultiGlyph extends LinearOpMode
{
    RRBotHardware robot = new RRBotHardware();
    RRBotJewelArm jewelArm = new RRBotJewelArm(robot, this);
    RRBotDriveAuto driveAuto = new RRBotDriveAuto(robot, this);
    private ElapsedTime runtime = new ElapsedTime();
    
    //constructs vuforia object
    RRBotVuforiaClass vuforia = new RRBotVuforiaClass();

    private String allianceColor;
    private int fieldPos = 0; //position 0 has a cryoptobox on one side, position 1 has cyroptoboxes on both sides
    private final double CAMERA_OFFSET = 5;

    private GlyphDetector glyphDetector = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        vuforia.Init_Vuforia();

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

        jewelArm.RunRoutine(allianceColor);

        sleep(250);

        RelicRecoveryVuMark pictograph = ScanPictograph(500);
        vuforia.close();

        driveAuto.AutoPlaceGlyph(allianceColor, fieldPos, pictograph);

        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.FAST;
        glyphDetector.enable();

        /*while(opModeIsActive())
        {
            telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
            telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition().toString());
        }*/

        driveAuto.MultiGlyphInit(allianceColor, fieldPos, pictograph);

        sleep(1000);

        double alignDistance = glyphDetector.getChosenGlyphOffset() - CAMERA_OFFSET;

        telemetry.addData("isFoundRect", glyphDetector.isFoundRect());
        telemetry.addData("alignDistance", alignDistance);

        if(glyphDetector.isFoundRect() && glyphDetector.getChosenGlyphOffset() < 5)
        {

            driveAuto.MultiGlyphAlignPickup(alignDistance);
        }
        else
        {
            driveAuto.MultiGlyphAlignPickup(0);
        }

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
