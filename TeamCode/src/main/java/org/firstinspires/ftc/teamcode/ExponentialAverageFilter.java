package org.firstinspires.ftc.teamcode;

/*
    @author Declan J. Scott
 */
public class ExponentialAverageFilter {
    private Double[] buffer;
    // buffer[1] is the old value. buffer[0] is the new value
    private float decayConstant;

    public ExponentialAverageFilter(float decayConstant) {
        this.buffer = new Double[2];
        this.decayConstant = decayConstant;
    }

    public void AddReading(double reading)
    {
        buffer[1] = buffer[0];
        buffer[0] = reading;
    }

    public double GetAverage()
    {
        return buffer[1] == null ? buffer[0] : (buffer[0] * decayConstant) + (buffer[1] * (1 - decayConstant));
    }
}