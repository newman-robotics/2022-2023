package org.firstinspires.ftc.teamcode;

/*
    @author Declan J. Scott
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.ConeColorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CameraTest_Color extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Get camera view monitor on Driver Hub screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        ConeColorPipeline pipeline = new ConeColorPipeline();

        // Create camera instance
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Handle cam opening
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Set up processing pipeline and start streaming
                cam.setPipeline(pipeline);
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                // Stop
                telemetry.addData("ERROR: ", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */

            if (gamepad1.a)
            {
                telemetry.addData("Detected Color", pipeline.GetFace());
                telemetry.addData("RED: ", pipeline.colorTotals.get("RED"));
                telemetry.addData("GREEN: ", pipeline.colorTotals.get("GREEN"));
                telemetry.addData("BLUE: ", pipeline.colorTotals.get("BLUE"));
            }


            telemetry.update();
        }
    }
}
