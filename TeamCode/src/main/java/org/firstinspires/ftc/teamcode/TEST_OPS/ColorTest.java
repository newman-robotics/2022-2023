package org.firstinspires.ftc.teamcode.TEST_OPS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class ColorTest extends LinearOpMode {
    public ColorSensor colorSense;
    boolean LEDEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSense = hardwareMap.get(ColorSensor.class, "color");
        colorSense.enableLed(LEDEnabled);
        waitForStart();
        while (opModeIsActive())
        {
            String colorString = String.format("%s, %s, %s", colorSense.red(), colorSense.green(), colorSense.blue());
            telemetry.addData("COLOR: ", colorString);
            telemetry.update();
        }
    }
}
