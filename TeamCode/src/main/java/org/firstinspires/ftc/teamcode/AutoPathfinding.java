package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DSYS.*;

/*
@author Declan J. Scott
 */

@Autonomous
public class AutoPathfinding extends RobotController {

    private double distanceBetweenPoints = 0.3048f; // in meters
    private double countsPerRev = 537.7;
    private double wheelCircumference = 0.301593;

    // (y, x) coordinate system
    public char[][] loadField(String path)
    {
        String rawFieldContent = IO.read(path);
        String[] yLines = rawFieldContent.split("\n");
        char[][] field = new char[yLines.length][yLines[0].length()];
        for (int y = 0; y < yLines.length; y++)
        {
            char[] yLineChars = yLines[y].toCharArray();
            for (int x = 0; x < yLineChars.length; x++)
            {
                field[y][x] = yLineChars[x];
            }
        }
        return field;
    }

    public static double distanceTravelled(int counts, double countsPerRev, double circumference)
    {
        return countsToMeters(counts, countsPerRev, circumference) * 4;
    }

    @Override
    public void runOpMode()
    {
        getSubComponents();
        char[][] field = loadField("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Pathfinding/field.txt");

        waitForStart();
        while (opModeIsActive())
        {

        }
    }
}
