package org.firstinspires.ftc.teamcode.Pathfinding;
// Coordinate class for BFS

import java.util.ArrayList;

public class Coordinate {

    public final int y;
    public final int x;
    public Coordinate parent = null;

    public Coordinate(int y, int x) {
        this.y = y;
        this.x = x;
    }

    public Coordinate[] getNeighbors(char[][] field) {
        ArrayList<Coordinate> neighbors = new ArrayList<>();

        if (y > 0)
            neighbors.add(new Coordinate(y - 1, x));
        if (y < field.length - 1)
            neighbors.add(new Coordinate(y + 1, x));
        if (x > 0)
            neighbors.add(new Coordinate(y, x - 1));
        if (x < field[0].length - 1)
            neighbors.add(new Coordinate(y, x + 1));

        Coordinate[] output = new Coordinate[neighbors.size()];
        neighbors.toArray(output);
        return output;
    }

    public boolean Equals(Coordinate other)
    {
        return x == other.x && y == other.y;
    }

    public String toString()
    {
        return String.format("(%02d, %02d)", y, x);
    }
}
