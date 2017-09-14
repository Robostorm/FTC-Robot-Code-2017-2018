package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by andrew on 9/10/17.
 */

//Autonomous OpMode for Relic Recovery robot (there will probably be more OpModes)d
//Currently uses vuforia to determine which pictograph image is displayed

@Autonomous(name = "RRBotAuto")
public class RRBotAuto extends LinearOpMode
{
    RRBotHardware robot = new RRBotHardware();
    RRBotVuforiaClass vuforia = new RRBotVuforiaClass();
    private ElapsedTime runtime = new ElapsedTime();
    enum GlyphPos {LEFT, CENTER, RIGHT}
    GlyphPos glypPos;

    String targetName = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        vuforia.Init_Vuforia();

        waitForStart();

        //image detection test
        while(opModeIsActive())
        {
            vuforia.Track_Target();
            if(vuforia.isVisible)
                telemetry.addData("Pictograph", vuforia.target_name);
            else
                telemetry.addData("Pictograph", "not visible");
            telemetry.update();
        }

        //red right
        /*vuforia.Track_Target();
        while(opModeIsActive() && !vuforia.isVisible)
        {
            vuforia.Track_Target();
        }
        //if(vuforia.target_name.equals(RelicRecoveryVuMark.LEFT))
            //glypPos = LEFT;*/
    }
}
