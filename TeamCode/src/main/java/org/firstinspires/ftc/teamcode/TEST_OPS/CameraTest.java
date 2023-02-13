package org.firstinspires.ftc.teamcode.TEST_OPS;

/*
    @author Declan J. Scott
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.GrayStreamPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Get camera view monitor on Driver Hub screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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
                GrayStreamPipeline pipeline = new GrayStreamPipeline();
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
            telemetry.addData("Frame Count", cam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", cam.getFps()));
            telemetry.addData("Total frame time ms", cam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", cam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", cam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", cam.getCurrentPipelineMaxFps());
            telemetry.update();

            // Handle stopping
            if (gamepad1.a)
            {
                cam.stopStreaming();
            }
        }
    }
}
