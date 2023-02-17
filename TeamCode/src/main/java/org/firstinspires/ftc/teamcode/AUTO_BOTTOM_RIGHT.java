package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;

@Autonomous
public class AUTO_BOTTOM_RIGHT extends AutoPathfinding {
    @Override
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.LEFT;
        currentPos = new Coordinate(8, 10);
        parkZone = new Coordinate(10,10);
    }
}
