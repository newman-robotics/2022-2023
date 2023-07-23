package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;

@TeleOp
public class libcardinal extends LinearOpMode {
    static {
        System.loadLibrary("libcardinal");
    }

    public native void runOpMode();

    public static class CaptureCallback implements CameraCaptureSession.CaptureCallback {
        public native void onNewFrame(CameraCaptureSession session, CameraCaptureRequest request, CameraFrame frame);
    }

    //not implemented in native: nothing to implement
    public static class StatusCallback implements CameraCaptureSession.StatusCallback {
        public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {};
    }

    public static class StateCallback implements CameraCaptureSession.StateCallback {
        public native void onConfigured(CameraCaptureSession session);
        public void onClosed(CameraCaptureSession session) {};
    }
}
