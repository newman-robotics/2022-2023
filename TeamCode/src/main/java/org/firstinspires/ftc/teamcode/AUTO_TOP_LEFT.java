package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;

@Autonomous
public class AUTO_TOP_LEFT extends AutoPathfinding {
    @Override
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.RIGHT;
        currentPos = new Coordinate(2, 0);
        parkZone = new Coordinate(0,0);
    }
}
