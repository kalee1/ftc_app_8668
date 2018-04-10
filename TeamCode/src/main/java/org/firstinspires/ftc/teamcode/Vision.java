package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Vision
{
    /** Part of the vuforia system. */
    OpenGLMatrix lastLocation = null;
    /** The Vuforia system uses this. */
    VuforiaLocalizer vuforia;

    /** The Vuforia system uses this. */
    VuforiaTrackables relicTrackables;
    /** The Vuforia system uses this. */
    VuforiaTrackable relicTemplate;
    /** The Vuforia system uses this. */
    int cameraMonitorViewId;
    /** The Vuforia system uses this. */
    VuforiaLocalizer.Parameters parameters;
    /** The Vuforia system uses this. */
    RelicRecoveryVuMark vuMark;

    /** The camera is used to differentiate colors during the jewel mission. */
    protected AnalogInput camera;

    public void init(HardwareMap hardwareMap, Telemetry telemetry )
    {
        try {
            camera = hardwareMap.get(AnalogInput.class, "camera");
        } catch (Exception p_exeception) {
            telemetry.addData("camera not found in config file", 0);
            camera = null;
        }

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXqihmr/////AAAAGfBzFvn3mUp2pArXwOs50RaJ5JQdpAr4rsjsH+U8jWqZz9IZH657T+p7j4SgiRhOxlbMsoXP43dcRWb953uxv1Pd9ykpvITS8R0LGB8w8DIEYElzCWAvx0qxFO/6mUq2nuWvAhSyGbVsQk3IgjC17DwijqO1i21E7bZtAp3LRfUaNjvwh38Q0EZkIY0ulaUChjb/sep2XzJ8/yoOxq3deuAVx6pSPcQwaLpdV7vSvLr7rDr1OIOZeb5DGjAEA4QLiV/t8/daIVi3AAWTpCi0kskgtT/KZMzzok8ACYE96pDMKn7Z5epuguKyZ4/6w9Mc7oMF68XMbtf60AhZvgUApJCakYrDT9MwT7IpGa03e+HC";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

    }


    /**
     * Detects cryptograph pattern.
     *
     * @return  a String denoting left, center, right, or blank if it can't determine the cryptograph.
     */
    public String readCryptograph(){
        String dejavu="";

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            dejavu = vuMark.toString();
        }
        return dejavu;
    }

    public String redJewelLocation()
    {
        if (camera.getVoltage() < 2.1 )
        {
            return "LEFT";
        }
        else
        {
            return "RIGHT";
        }

    }


}
