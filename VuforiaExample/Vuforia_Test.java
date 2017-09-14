package org.firstinspires.ftc.teamcode.VuforiaExample;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Team 6934 on 6/3/2017.
 */

@TeleOp(name = "Vuforia_Test")

public class Vuforia_Test extends OpMode {

    Vuforia_Class_2 vuforia = new Vuforia_Class_2();

    @Override
    public void init() {
        vuforia.Init_Vuforia();
    }

    @Override
    public void loop() {
        vuforia.Track_Target();
        Telemetry();
    }

    public void Telemetry(){
        telemetry.addData( "TX_" , vuforia.Tx );
        telemetry.addData( "TY_" , vuforia.Ty );
        telemetry.addData( "TZ_" , vuforia.Tz );
        telemetry.addData( "Deg_" , vuforia.Deg );
        telemetry.addData( "Name_" , vuforia.target_name );

        telemetry.addData( "RX_" , vuforia.Rx );
        telemetry.addData( "RY_" , vuforia.Ry );
        //telemetry.addData( "RZ_" , vuforia.Rz ); // not needed
    }
}
