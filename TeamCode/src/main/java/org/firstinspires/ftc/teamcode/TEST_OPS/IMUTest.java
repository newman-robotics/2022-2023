package org.firstinspires.ftc.teamcode.TEST_OPS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotController;
import org.opencv.core.Mat;

import java.util.PriorityQueue;

@Autonomous
public class IMUTest extends RobotController {

    private BNO055IMU imu;

    public double getBearing()
    {
        double radBearing = imu.getAngularOrientation().firstAngle + (Math.PI);
        return (radBearing * 180) / Math.PI;
    }
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Status: ", "Received IMU");
        telemetry.update();
        BNO055IMU.Parameters initParam = new BNO055IMU.Parameters();
        initParam.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(initParam);
        telemetry.addData("Status: ", "Initialized IMU");
        telemetry.update();
        getSubComponents();
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("ANGULAR VELOCITY: ", imu.getAngularVelocity());
            telemetry.addData("ACCELERATION: ", imu.getAcceleration());
            telemetry.addData("LINEAR ACCELERATION: ", imu.getLinearAcceleration());
            telemetry.addData("VELOCITY: ", imu.getVelocity());
            telemetry.addData("RELATIVE BEARING: ", getBearing());
            telemetry.addData("ORIENTATION: ", imu.getAngularOrientation().toString());
            telemetry.update();
        }
    }
}
