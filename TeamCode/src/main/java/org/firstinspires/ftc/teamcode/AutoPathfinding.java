package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoPathfinding extends RobotController {

    private char[][] field = {
            {'0','0','0','4'},
            {'3','0','0','0'},
            {'0','0','2','0'},
            {'0','0','0','1'},
    };
    private double distanceBetweenPoints = 1; // in meters

    @Override
    public void runOpMode()
    {
        getSubComponents();

        waitForStart();
        while (opModeIsActive())
        {

        }
    }
}
