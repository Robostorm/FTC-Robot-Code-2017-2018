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
 * Uses vuforia to scan pictograph
 * @author Andrew Hollabaugh
 * @since 2017-9-10
 */
public class RRBotVuforiaClass
{
    public RRBotVuforiaClass(){}

    ClosableVuforiaLocalizer vuforia_localizer;
    RelicRecoveryVuMark target_name;
    boolean isVisible;
    VuforiaTrackable relicTemplate;

    /**
     * Initializes vuforia by setting parameters and making a vuMark template
     */
    public void Init_Vuforia()
    {
        //vuforia setup parameters
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AfUq6CD/////AAAAGchbRX+6y0LIt3pkxNeimLND8I1F1fye+F1YTEjcdwC0eVfpSIJDtEfT7bHRcGVkhpVJycLCv4uSzAdEoYSbY/hEK26PZs366KHoA904FbCYRGFHZMnyrhIoFDyKl4oHvjYKEm3W" +
                "M0vImHgOtNMikqetXakzjp+O1q/ZitBR1U3SK+LVGrmdWn7l7+js0m29d6DY4GUiKOPXU1szaEBcV8QmSo0UgE17DhZ3DHX6E+41TueXyJuTgaapnnim0TWSO1u+e8e/VvFoalTgozkwdfAcXpNQ488eIOJ3fJq6ChPe04s3Nb3" +
                "GCIcDBhsFae0W+AbfXyR3ohhuEa9+La+zjEtkLUUOCxAicpAG9YIPF80B";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia_localizer = new ClosableVuforiaLocalizer(params);

        //load the trackable VuMarks from the asset to a variable
        VuforiaTrackables relicTrackables = this.vuforia_localizer.loadTrackablesFromAsset("RelicVuMark");

        //store the vumark template in a variable and name it
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    /**
     * Tracks the vuMark and sets the name accordingly
     */
    public void Track_Target()
    {
        //initialize the vuMark variable by setting it to the template vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //if the vuMark is something other than unknown
        if(vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            isVisible = true;

            //set the target name
            target_name = vuMark;
        }
        else
        {
            isVisible = false;
        }
    }

    //closes the vuforia localizer
    public void close()
    {
        vuforia_localizer.close();
    }
}
