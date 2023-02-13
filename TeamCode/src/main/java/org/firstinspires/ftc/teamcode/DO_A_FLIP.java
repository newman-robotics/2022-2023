package org.firstinspires.ftc.teamcode;

/*
@author Declan J. Scott
Encoder based steering test (robot should pivot 90 degrees)
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RobotController;
import org.opencv.core.Mat;

@Autonomous
public class DO_A_FLIP extends RobotController {
    private BNO055IMU imu;
    private boolean pivoting = false;
    private double startTime = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters initParam = new BNO055IMU.Parameters();
        initParam.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(initParam);

        getSubComponents();
        waitForStart();

        while (opModeIsActive())
        {
            if (!pivoting && gamepad1.dpad_down)
            {
                Pivot90();
            }

            telemetry.addData("Time: ", super.time);
            telemetry.addData("Pivoting: ", pivoting);
            telemetry.update();
        }
    }

    public void Pivot90()
    {
        startTime = super.time;
        double angVel = imu.getAngularVelocity().xRotationRate;
        pivoting = true;
        while (Math.abs(angleTravelledRad(angVel, super.time - startTime)) < Math.PI / 2)
        {
            angVel = imu.getAngularVelocity().xRotationRate;
            drivetrain.Steer(90);

            telemetry.addData("AngVel: ", angVel);
            telemetry.addData("Time Diff: ", super.time - startTime);
            telemetry.update();
        }
        drivetrain.Reset();
        pivoting = false;
    }

    public double angleTravelledRad(double angVel, double t)
    {
        return angVel * t;
    }
}
