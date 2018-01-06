package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by andrew on 12/19/17.
 */

public class RRBotJewelArm
{
    RRBotHardware robot;
    LinearOpMode opMode;

    private final double JEWEL_ARM_SERVO_1_END_POS = 0;
    private final double JEWEL_ARM_SERVO_2_MID_POS = 0.5;
    private final double JEWEL_ARM_SERVO_2_LEFT_POS = 0.15;
    private final double JEWEL_ARM_SERVO_2_RIGHT_POS = 0.9;
    private final String JEWEL_ARM_COLOR_SENSOR_DIRECTION = "forwards";
    private String ballColor = "unknown";

    public RRBotJewelArm(RRBotHardware robot, LinearOpMode opMode)
    {
        this.robot = robot;
        this.opMode = opMode;
    }

    public void runOpMode() {}

    public void RunRoutine(String allianceColor)
    {
        robot.grabber1Servo.setPosition(0.8);
        robot.grabber2Servo.setPosition(0.8);

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_MID_POS);

        MoveJewelArm("down");

        opMode.sleep(1000);

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

        opMode.sleep(750); //wait for servo to move

        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_MID_POS);

        MoveJewelArm("up");
    }

    public void MoveJewelArm(String direction)
    {
        if(direction.equals("down"))
        {
            double servoPos = robot.JEWEL_ARM_SERVO_1_START_POS;
            while(opMode.opModeIsActive() && servoPos > JEWEL_ARM_SERVO_1_END_POS)
            {
                servoPos -= 0.05;
                robot.jewelArmServo1.setPosition(servoPos);
                opMode.sleep(50);
            }
        }
        else if(direction.equals("up"))
        {
            double servoPos = JEWEL_ARM_SERVO_1_END_POS;
            while(opMode.opModeIsActive() && servoPos < robot.JEWEL_ARM_SERVO_1_START_POS)
            {
                servoPos += 0.05;
                robot.jewelArmServo1.setPosition(servoPos);
                opMode.sleep(50);
            }
        }
    }

    public void KnockBallInfront()
    {
        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_LEFT_POS);

        opMode.telemetry.addData("knock ball", "infront");
        opMode.telemetry.update();
    }

    public void KnockBallBehind()
    {
        robot.jewelArmServo2.setPosition(JEWEL_ARM_SERVO_2_RIGHT_POS);

        opMode.telemetry.addData("knock ball", "behind");
        opMode.telemetry.update();
    }
}
