package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
@author Declan J. Scott
 */

@Autonomous
public class AutoPathfinding extends RobotController {

    // (y, x) coordinate system
    private char[][] field = {
            {'_','_','_','_'},
            {'_','_','_','_'},
            {'_','_','_','_'},
            {'_','_','_','_'},
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
