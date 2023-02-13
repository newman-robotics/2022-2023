package org.firstinspires.ftc.teamcode.TEST_OPS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;

@Autonomous
public class IMUTest extends RobotController {

    private BNO055IMU imu;

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
            telemetry.update();
        }
    }
}
