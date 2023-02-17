package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pathfinding.Coordinate;

@Autonomous
public class AUTO_BOTTOM_LEFT extends AutoPathfinding {
    @Override
    public void SetUpOrigin()
    {
        facingDirection = DIRECTIONS.RIGHT;
        currentPos = new Coordinate(8, 0);
        parkZone = new Coordinate(10,0);
    }
}
