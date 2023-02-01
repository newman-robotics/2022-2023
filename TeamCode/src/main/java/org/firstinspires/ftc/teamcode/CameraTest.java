package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutonomousDriver;

import org.opencv.videoio.VideoCapture;

@TeleOp(name="CameraTest",group="Tests")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutonomousDriver.AutonomousImageProcessor processor = new AutonomousDriver.AutonomousImageProcessor(new VideoCapture(0));
        Byte zone = processor.getSignalSleeveOrientation();
        telemetry.addData("zone: ",zone);
        return;
    }
}
