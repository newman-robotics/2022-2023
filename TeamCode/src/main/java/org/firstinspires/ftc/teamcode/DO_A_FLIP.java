package org.firstinspires.ftc.teamcode;

/*
@author Declan J. Scott
Encoder based steering test (robot should pivot 90 degrees)
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DO_A_FLIP extends RobotController {
    private double robotCircumference = 1.28;
    private double countsPerRev = 537.7;
    private double wheelCircumference = 0.301593;
    private boolean pivoting = false;

    @Override
    public void runOpMode() {
        getSubComponents();
        waitForStart();

        while (opModeIsActive())
        {
            if (!pivoting && gamepad1.dpad_down)
            {
                Pivot90();
            }
        }
    }

    public void Pivot90()
    {
        pivoting = true;
        double startingDistance = distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference);
        while (distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference) - startingDistance < (robotCircumference / 4))
        {
            drivetrain.Steer(90);
        }
        pivoting = false;
    }

    public static double distanceTravelled(int counts, double countsPerRev, double circumference)
    {
        return countsToMeters(counts, countsPerRev, circumference) * 4;
    }
}
