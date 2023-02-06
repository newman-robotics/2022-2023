package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DSYS.*;
import org.firstinspires.ftc.teamcode.Pathfinding.BFS;
import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;

/*
@author Declan J. Scott
 */

@Autonomous
public class AutoPathfinding extends RobotController {

    private double distanceBetweenPoints = 0.3048f; // in meters
    private double countsPerRev = 537.7;
    private double wheelCircumference = 0.301593;
    private double robotCircumference = 1.28;

    private enum DIRECTIONS{
        UP,
        DOWN,
        LEFT,
        RIGHT
    }
    private char[][] field;
    private DIRECTIONS facingDirection;
    private Coordinate currentPos;

    public static float Direction2Theta(DIRECTIONS dir)
    {
        switch (dir)
        {
            case UP:
                return 0f;
            case DOWN:
                return 180f;
            case LEFT:
                return 270f;
            case RIGHT:
                return 90f;
        }
        return -1f; // error case
    }

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

    // To be overridden
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.RIGHT;
        currentPos = new Coordinate(2, 0);
    }

    public static double distanceTravelled(int counts, double countsPerRev, double circumference)
    {
        return countsToMeters(counts, countsPerRev, circumference) * 4;
    }

    @Override
    public void runOpMode()
    {
        field = loadField("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Pathfinding/field.txt");
        SetUpOrigin();
        getSubComponents();

        waitForStart();
        while (opModeIsActive())
        {
            Coordinate[] path = BFS.FindPath(field, currentPos, new Coordinate(0, 4));
            MovePath(path);
        }
    }

    void MovePath(Coordinate[] path)
    {
        for (int i = 1; i < path.length; i++)
        {
            // Identify required coordinates and directions
            Coordinate nextCoordinate = path[i];
            DIRECTIONS nextDirection = facingDirection;
            if (currentPos.y < nextCoordinate.y)
            {
                nextDirection = DIRECTIONS.UP;
            }else if (currentPos.y > nextCoordinate.y)
            {
                nextDirection = DIRECTIONS.DOWN;
            }else if (currentPos.x < nextCoordinate.x)
            {
                nextDirection = DIRECTIONS.RIGHT;
            }else if (currentPos.x > nextCoordinate.x)
            {
                nextDirection = DIRECTIONS.LEFT;
            }

            // Pivot direction
            float thetaToTravel = Direction2Theta(nextDirection) - Direction2Theta(facingDirection);
            Pivot(thetaToTravel);
            drivetrain.Reset();

            // Move a node's distance for 1 node
            double startingDistance = distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference);
            while (distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference) - startingDistance <= distanceBetweenPoints)
            {
                drivetrain.Drive(90);
            }

            // Stop moving
            drivetrain.Reset();
        }
    }

    public void Pivot(float theta)
    {
        double startingDistance = distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference);
        while (distanceTravelled(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference) - startingDistance < (robotCircumference / 360) * theta)
        {
            drivetrain.Steer(90);
        }
    }
}
