package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;

@Autonomous
public class AUTO_TOP_RIGHT extends AutoPathfinding {
    @Override
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.LEFT;
        currentPos = new Coordinate(2, 10);
        parkZone = new Coordinate(0,10);
    }
}
