package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DSYS.*;
import org.firstinspires.ftc.teamcode.Pathfinding.BFS;
import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;
import org.firstinspires.ftc.teamcode.Pathfinding.FieldContainer;

/*
@author Declan J. Scott
 */

@Autonomous
public class AutoPathfinding extends RobotController {

    private double distanceBetweenPoints = 0.3048f; // in meters
    private double countsPerRev = 537.7;
    private double wheelCircumference = 0.301593;
    private double robotCircumference = 1.28;
    private BNO055IMU gyro;

    private enum DIRECTIONS{
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    // (y, x) coordinate system
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


    // To be overridden with different children classes for starting positions
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.RIGHT;
        currentPos = new Coordinate(2, 0);
    }

    // Distance travelled in straight line
    public static double distanceTravelled(int counts, double countsPerRev, double circumference)
    {
        return countsToMeters(counts, countsPerRev, circumference) * 4;
    }

    // Radian angle travelled when pivoting
    public double angleTravelledRad(double t)
    {
        float angVel = gyro.getAngularVelocity().xRotationRate;
        return angVel * t;
    }

    @Override
    public void runOpMode()
    {
        field = FieldContainer.field;
        SetUpOrigin();
        getSubComponents();

        // Access IMU
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters initParam = new BNO055IMU.Parameters();
        initParam.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        gyro.initialize(initParam);

        waitForStart();

        Coordinate[] path = BFS.FindPath(field, currentPos, new Coordinate(0, 4));
        MovePath(path);
    }

    void MovePath(Coordinate[] path)
    {
        for (int i = 1; i < path.length; i++)
        {
            String pathReport = "";
            for (Coordinate c : path)
            {
                pathReport += c.toString() + ", ";
            }
            telemetry.addData("PATH: ", pathReport);

            // Identify required coordinates and directions
            Coordinate nextCoordinate = path[i];
            DIRECTIONS nextDirection = facingDirection;

            telemetry.addData("CURRENT POSITION: ", currentPos.toString());
            telemetry.addData("NEXT POSITION: ", nextCoordinate.toString());

            if (currentPos.y > nextCoordinate.y)
            {
                nextDirection = DIRECTIONS.UP;
            }else if (currentPos.y < nextCoordinate.y)
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
            telemetry.addData("Pivot Direction: ", thetaToTravel);
            telemetry.update();
            Pivot(thetaToTravel);

            // Move a node's distance for 1 node
            double startingDistance = countsToMeters(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference);
            while (countsToMeters(drivetrain.topRight.getCurrentPosition(), countsPerRev, wheelCircumference) - startingDistance <= distanceBetweenPoints)
            {
                drivetrain.Drive(90);
            }

            // Stop moving and set new current position
            drivetrain.Reset();
            currentPos = nextCoordinate;
            facingDirection = nextDirection;
        }
    }

    public void Pivot(float theta)
    {
        // Configure variables needed
        double radTheta = theta * (Math.PI / 180);
        double startTime = super.time;
        float steerDirectionInput = theta > 0 ? 0 : 90;

        while (Math.abs(angleTravelledRad(super.time - startTime)) < radTheta)
        {
            drivetrain.Steer(steerDirectionInput);
        }
        drivetrain.Reset();
    }
}
