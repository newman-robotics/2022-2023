package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ControllerTest extends LinearOpMode {

    private boolean testing1 = true;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            // Update controller name
            String activeControllerName = testing1 ? "CONTROLLER 1" : "CONTROLLER 2";
            telemetry.addData("ACTIVE", activeControllerName);

            // Get active controller instance
            Gamepad activeController = testing1 ? gamepad1 : gamepad2;

            telemetry.addData("Left Stick", String.format("(%f, %f)", activeController.left_stick_x, activeController.left_stick_y));
            telemetry.addData("Right Stick", String.format("(%f, %f)", activeController.right_stick_x, activeController.right_stick_y));
            telemetry.addData("Left Trigger", activeController.left_trigger);
            telemetry.addData("Right Trigger", activeController.right_trigger);
            telemetry.update();

            if (activeController.left_trigger > 0.9f && activeController.right_trigger > 0.9f)
                testing1 = !testing1;
        }
    }
}
