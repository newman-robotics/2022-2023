package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ThreadingTest extends LinearOpMode {
    static {
        System.loadLibrary("ThreadingTest");
    }
    public native int foot();

    public void runOpMode() {
        waitForStart();
        telemetry.addData("Threaded output from native:", foot());
        telemetry.update();
    }
}
