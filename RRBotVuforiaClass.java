package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by andrew on 9/10/17.
 */

public class RRBotVuforiaClass
{
    //Constructor
    public RRBotVuforiaClass(){}

    //VuforiaTrackables targets;
    VuforiaTrackableDefaultListener listener;
    VuforiaLocalizer vuforia_localizer;
    float Tx,Tz,Ty,Deg;
    float Rx,Ry;
    RelicRecoveryVuMark target_name;
    private float offset = 0;
    private float mm_to_Inch = 25.4f;
    boolean isVisible;
    VuforiaTrackable relicTemplate;

    public void Init_Vuforia()
    {
        //vuforia setup parameters
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AfUq6CD/////AAAAGchbRX+6y0LIt3pkxNeimLND8I1F1fye+F1YTEjcdwC0eVfpSIJDtEfT7bHRcGVkhpVJycLCv4uSzAdEoYSbY/hEK26PZs366KHoA904FbCYRGFHZMnyrhIoFDyKl4oHvjYKEm3W" +
                "M0vImHgOtNMikqetXakzjp+O1q/ZitBR1U3SK+LVGrmdWn7l7+js0m29d6DY4GUiKOPXU1szaEBcV8QmSo0UgE17DhZ3DHX6E+41TueXyJuTgaapnnim0TWSO1u+e8e/VvFoalTgozkwdfAcXpNQ488eIOJ3fJq6ChPe04s3Nb3" +
                "GCIcDBhsFae0W+AbfXyR3ohhuEa9+La+zjEtkLUUOCxAicpAG9YIPF80B";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia_localizer = ClassFactory.createVuforiaLocalizer(params);


        /*//load targets
        targets = vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");
        targets.get(0).setName("left_pictograph");
        //targets.get(1).setName("center_pictograph");
        //targets.get(2).setName("right_pictograph");*/

        VuforiaTrackables relicTrackables = this.vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    public void Track_Target()
    {
        /*for(VuforiaTrackable targ : targets)
        {
            listener = (VuforiaTrackableDefaultListener) targ.getListener();
            OpenGLMatrix pose = listener.getPose();

            if(pose != null)
            {
                VectorF Tdata = pose.getTranslation();
                Tx = Tdata.get(0);
                Ty = Tdata.get(1);
                Tz = Tdata.get(2);

                Tx += offset;

                Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                Deg = orientation.secondAngle;

                target_name = targ.getName();
            }

            //Robot position on field
            OpenGLMatrix robotLoc = listener.getUpdatedRobotLocation();
            if (robotLoc != null) {

                VectorF Rdata = robotLoc.getTranslation();
                Rx = Rdata.get(0) / mm_to_Inch;
                Ry = Rdata.get(1) / mm_to_Inch;
            }
        }*/

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if(vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            isVisible = true;

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            //telemetry.addData("VuMark", "%s visible", vuMark);
            target_name = vuMark;

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            //telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                /*double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;*/

                Tx = trans.get(0);
                Ty = trans.get(1);
                Tz = trans.get(2);

                Rx = rot.firstAngle;
                Ry = rot.secondAngle;
            }
        }
        else
        {
            isVisible = false;
        }
    }

    // this is a helper function to set targets field positions.
    /*private void target_field_position( int target_num , float tx_position , float ty_position , int ty_angle )
    {
        tx_position = (tx_position * mm_to_Inch);
        ty_position = (ty_position * mm_to_Inch);
        OpenGLMatrix TargetLocationOnField = OpenGLMatrix
                .translation(tx_position , ty_position , 0 )
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90 , ty_angle , 0));
        targets.get(target_num).setLocation(TargetLocationOnField);
    }*/

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
