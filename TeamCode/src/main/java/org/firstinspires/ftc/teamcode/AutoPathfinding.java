package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Pathfinding.BFS;
import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;
import org.firstinspires.ftc.teamcode.Pathfinding.FieldContainer;
import org.opencv.core.Mat;

/*
@author Declan J. Scott
 */

class SpeedCalibration{
    private double startTime;
    private double speed;
    private boolean calibrated = false;

    public SpeedCalibration(double startTime) {
        this.startTime = startTime;
    }

    public void End(float time, float distance)
    {
        calibrated = true;
        speed = Math.abs(distance / (time - startTime));
    }

    public double getSpeed()
    {
        return speed;
    }

    public boolean isCalibrated()
    {
        return calibrated;
    }
}

@Autonomous
public class AutoPathfinding extends RobotController {

    // in meters
    private double distanceBetweenPoints = 0.3048f;
    public double getDistanceBetweenPoints(){ return distanceBetweenPoints; }
    private double countsPerRev = 537.7;
    private double wheelCircumference = 0.301593;
    private BNO055IMU gyro;

    // Speed calibrations for moving/pivoting
    private SpeedCalibration moveSpeed; // m/s
    private SpeedCalibration pivotSpeed; // rad/s

    private float angleOffset;
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

    // Degree bearing relative to initial bearing
    public float getBearing()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return angles.firstAngle + angleOffset;
    }

    // Radian angle travelled when pivoting
    public double angleTravelledRad(double t)
    {
        float angVel = gyro.getAngularVelocity().xRotationRate;
        return angVel * t;
    }

    double estimatedTime;
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

        // Set up angle offset
        angleOffset = Direction2Theta(facingDirection);

        waitForStart();

        // Calibration path
        Coordinate[] path = BFS.FindPath(field, currentPos, new Coordinate(0, 4));
        MovePath(path);

        Coordinate[] timedPath = BFS.FindPath(field, new Coordinate(0, 4), new Coordinate(10, 4));
        estimatedTime = EstimateTime(timedPath);
        MovePath(timedPath);
    }

    void MovePath(Coordinate[] path)
    {
        for (int i = 1; i < path.length; i++)
        {
            telemetry.addData("Estimated Time: ", estimatedTime);

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

            // Set new direction
            facingDirection = nextDirection;

            // Move
            MoveDist(distanceBetweenPoints);

            // Stop moving and set new current position
            currentPos = nextCoordinate;
        }
    }

    int countOffset = 0; // Counts utilized to reposition the robot and are not counted towards distance travelled.
    public void MoveDist(double dist)
    {
        countOffset = 0;

        // Check for calibration data
        if (moveSpeed == null)
        {
            moveSpeed = new SpeedCalibration((float) super.time);
        }

        // Move a node's distance for 1 node
        double startingDistance = FilteredDistanceTravelled();
        while (FilteredDistanceTravelled() - startingDistance <= Math.abs(dist))
        {
            // Drive
            drivetrain.Drive(dist > 0 ? 90 : 270);

            // Check for angle inconsistencies
            int startingCorrectionCount = drivetrain.topRight.getCurrentPosition();
            double bearingDifference = Direction2Theta(facingDirection) - getBearing();
            if (Math.abs(bearingDifference) > 0.5f)
            {
                // Stop drivetrain and correct angle
                drivetrain.Reset();
                Pivot((float) bearingDifference);

                // Calculate count offset
                countOffset += drivetrain.topRight.getCurrentPosition() - startingCorrectionCount;
            }
        }

        // Calibrate
        if (!moveSpeed.isCalibrated())
        {
            moveSpeed.End((float) super.time, (float) dist);
        }
    }

    public double FilteredDistanceTravelled()
    {
        double rawDist = countsToMeters(drivetrain.topRight.getCurrentPosition() - countOffset, countsPerRev, wheelCircumference);
        drivetrain.encoderFilter.AddReading(rawDist);
        return drivetrain.encoderFilter.GetAverage();
    }

    public void Pivot(float theta)
    {
        // Check for calibration data
        if (pivotSpeed == null)
        {
            pivotSpeed = new SpeedCalibration((float) super.time);
        }

        // Configure variables needed
        double radTheta = Math.abs(theta) * (Math.PI / 180);
        double startTime = super.time;
        float steerDirectionInput = theta > 0 ? 0 : 90;

        while (Math.abs(angleTravelledRad(super.time - startTime)) < radTheta)
        {
            drivetrain.Steer(steerDirectionInput);
        }

        // Calibrate
        if (!pivotSpeed.isCalibrated())
        {
            pivotSpeed.End((float) super.time, (float) radTheta);
        }
    }

    // Returns an approximation of how long a path will take assuming calibration data exists
    public double EstimateTime(Coordinate[] path)
    {
        if (moveSpeed == null || !moveSpeed.isCalibrated())
            return 0;
        if (pivotSpeed == null || !pivotSpeed.isCalibrated())
            return 0;

        // Find all nodes where pivoting is necessary
        int pivotCount = 0;
        DIRECTIONS currentDir = facingDirection;
        for (int i = 0; i < path.length-1; i++)
        {
            Coordinate current = path[i];
            Coordinate next = path[i+1];

            DIRECTIONS requiredDirection = currentDir;
            if (current.y > next.y)
            {
                requiredDirection = DIRECTIONS.UP;
            }else if (current.y < next.y)
            {
                requiredDirection = DIRECTIONS.DOWN;
            }else if (current.x < next.x)
            {
                requiredDirection = DIRECTIONS.RIGHT;
            }else if (current.x > next.x)
            {
                requiredDirection = DIRECTIONS.LEFT;
            }

            if (requiredDirection != currentDir)
            {
                pivotCount++;
            }

            currentDir = requiredDirection;
        }

        double singlePivotTime = (pivotSpeed.getSpeed() * (Math.PI / 2));
        double totalPivotTime = pivotCount * singlePivotTime;
        double singleMoveTime = moveSpeed.getSpeed() * distanceBetweenPoints;
        double totalMoveTime = singleMoveTime * (path.length - 1);

        return totalPivotTime + totalMoveTime;
    }
}
