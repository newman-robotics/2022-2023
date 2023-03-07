package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CTest")
public class CTest extends LinearOpMode {
    public native int foot();

    @Override
    public void runOpMode() {
        telemetry.addData("Output from native", foot());
        telemetry.update();
    }
}
