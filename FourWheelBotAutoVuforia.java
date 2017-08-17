package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FourWheelBotAutoVuforia")

public class FourWheelBotAutoVuforia extends LinearOpMode
{
    FourWheelBotVuforiaClass vuforia = new FourWheelBotVuforiaClass();
    Hardware4WheelBot robot = new Hardware4WheelBot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.3;
    static final double threshold = 3;
    static final double targetZ = 100;

    @Override
    public void runOpMode()
    {
        vuforia.Init_Vuforia();
        robot.init(hardwareMap);

        waitForStart();

        vuforia.Track_Target();
        Telemetry();

        robot.frontLeftMotor.setPower(-TURN_SPEED);
        robot.frontRightMotor.setPower(TURN_SPEED);
        robot.rearLeftMotor.setPower(-TURN_SPEED);
        robot.rearRightMotor.setPower(TURN_SPEED);

        while(opModeIsActive() && (vuforia.Tx > threshold || vuforia.Tx < -threshold))
        {
            vuforia.Track_Target();
            Telemetry();
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);

        /*if(!(vuforia.Tx == Float.NaN))
        {
            if(vuforia.Tx - threshold > 0) //robot is right of center
            {
                Turn(1, "left");
            }

            else if(vuforia.Tx + threshold < 0) //robot is left of center
            {
                Turn(1, "right");
            }

            vuforia.Track_Target();

            //go straight to fix Tx incorrectness
            robot.frontLeftMotor.setPower(FORWARD_SPEED);
            robot.frontRightMotor.setPower(FORWARD_SPEED);
            robot.rearLeftMotor.setPower(FORWARD_SPEED);
            robot.rearRightMotor.setPower(FORWARD_SPEED);

            while(opModeIsActive() && (vuforia.Tx > threshold || vuforia.Tx < -threshold))
            {
                vuforia.Track_Target();
                Telemetry();
            }

            vuforia.Track_Target();

            if(vuforia.Deg - threshold > 0) //robot is turned right
            {
                robot.frontLeftMotor.setPower(-TURN_SPEED);
                robot.frontRightMotor.setPower(TURN_SPEED);
                robot.rearLeftMotor.setPower(-TURN_SPEED);
                robot.rearRightMotor.setPower(TURN_SPEED);
            }

            else if(vuforia.Deg + threshold < 0) //robot is turned left
            {
                robot.frontLeftMotor.setPower(TURN_SPEED);
                robot.frontRightMotor.setPower(-TURN_SPEED);
                robot.rearLeftMotor.setPower(TURN_SPEED);
                robot.rearRightMotor.setPower(-TURN_SPEED);
            }

            while(opModeIsActive() && (vuforia.Deg > threshold || vuforia.Deg < threshold))
            {
                vuforia.Track_Target();
                Telemetry();
            }

            vuforia.Track_Target();

            //go straight until target z is reached
            robot.frontLeftMotor.setPower(FORWARD_SPEED);
            robot.frontRightMotor.setPower(FORWARD_SPEED);
            robot.rearLeftMotor.setPower(FORWARD_SPEED);
            robot.rearRightMotor.setPower(FORWARD_SPEED);

            while(opModeIsActive() && vuforia.Tz > targetZ)
            {
                vuforia.Track_Target();
                Telemetry();
            }

            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.rearLeftMotor.setPower(0);
            robot.rearRightMotor.setPower(0);

            sleep(1000);
        }*/
    }

    public void Telemetry()
    {
        telemetry.addData( "TX_" , vuforia.Tx );
        telemetry.addData( "TZ_" , vuforia.Tz );
        telemetry.addData( "Deg_" , vuforia.Deg );
        telemetry.update();
    }

    public void Move(double seconds)
    {
        robot.frontLeftMotor.setPower(FORWARD_SPEED);
        robot.frontRightMotor.setPower(FORWARD_SPEED);
        robot.rearLeftMotor.setPower(FORWARD_SPEED);
        robot.rearRightMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < seconds)
        {
            vuforia.Track_Target();
            Telemetry();
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    public void Turn(double seconds, String direction)
    {
        if(direction.equals("left"))
        {
            robot.frontLeftMotor.setPower(-TURN_SPEED);
            robot.frontRightMotor.setPower(TURN_SPEED);
            robot.rearLeftMotor.setPower(-TURN_SPEED);
            robot.rearRightMotor.setPower(TURN_SPEED);
        }

        else if(direction.equals("right"))
        {
            robot.frontLeftMotor.setPower(TURN_SPEED);
            robot.frontRightMotor.setPower(-TURN_SPEED);
            robot.rearLeftMotor.setPower(TURN_SPEED);
            robot.rearRightMotor.setPower(-TURN_SPEED);
        }

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < seconds)
        {
            vuforia.Track_Target();
            Telemetry();
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }
}
