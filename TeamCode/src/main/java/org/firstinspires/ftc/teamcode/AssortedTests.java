package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="MotorTest",group="Tests")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor ul = hardwareMap.get(DcMotor.class, "ltMotor");
        DcMotor ur  = hardwareMap.get(DcMotor.class, "rtMotor");
        DcMotor bl = hardwareMap.get(DcMotor.class, "lbMotor");
		DcMotor br = hardwareMap.get(DcMotor.class, "rbMotor");
        ul.setPower(0.6);
        ur.setPower(0.6);
        bl.setPower(0.6);
        br.setPower(0.6);
        Thread.currentThread().sleep(1000);
        ul.setPower(0);
        ur.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        return;
    }
}