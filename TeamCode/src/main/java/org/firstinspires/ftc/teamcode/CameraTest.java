package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.videoio.VideoCapture;

@TeleOp(name="CameraTest",group="Tests")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutonomousImageProcessor processor = new AutonomousImageProcessor(new VideoCapture(0));
        Byte zone = processor.getSignalSleeveOrientation();
        telemetry.addData("zone: ",zone);
        return;
    }
}