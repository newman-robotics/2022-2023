package org.firstinspires.ftc.teamcode;

import java.util.PriorityQueue;

/*
    @author Declan J. Scott
 */
public class MovingAverageFilter {
    private final int windowSize;
    private PriorityQueue<Double> buffer;

    public MovingAverageFilter(int windowSize) {
        this.windowSize = windowSize;
        this.buffer = new PriorityQueue<>();
    }

    // Simple Queue stuff
    public void AddReading(double reading)
    {
        if (buffer.size() >= windowSize)
            buffer.remove();

        buffer.add(reading);
    }

    public double GetAverage()
    {
        double sum = 0;
        for (double reading : buffer) {
            sum += reading;
        }
        return sum / buffer.size();
    }
}