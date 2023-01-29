package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;

public class ConeColorPipeline extends OpenCvPipeline {

    private Hashtable<String, Double> colorTotals = new Hashtable<>();

    public String GetFace()
    {
        // Iterative search for highest value
        double highestValue = 0;
        String highestName = "";

        Enumeration<String> e = colorTotals.keys();
        while (e.hasMoreElements())
        {
            String key = e.nextElement();
            double value = colorTotals.get(key);

            if (value > highestValue)
            {
                highestName = key;
                highestValue = value;
            }
        }

        return highestName;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Reset sums
        colorTotals.clear();
        colorTotals.put("RED", (double) 0);
        colorTotals.put("GREEN", (double) 0);
        colorTotals.put("BLUE", (double) 0);

        // Get split channels
        ArrayList<Mat> RGBChannels = new ArrayList<>();
        Core.split(input, RGBChannels);
        Mat red = RGBChannels.get(0);
        Mat green = RGBChannels.get(1);
        Mat blue = RGBChannels.get(2);

        // Threshold each channel
        red = thresholdChannel(red);
        green = thresholdChannel(green);
        blue = thresholdChannel(blue);

        // Go through each pixel to get things
        for (int y = 0; y < input.height(); y++) {
            for (int x = 0; x < input.width(); x++) {
                // Get amounts
                double redAmount = red.get(y, x)[0];
                double greenAmount = green.get(y, x)[0];
                double blueAmount = blue.get(y, x)[0];

                // Add to totals
                colorTotals.put("RED", colorTotals.get("RED") + greenAmount);
                colorTotals.put("GREEN", colorTotals.get("GREEN") + greenAmount);
                colorTotals.put("BLUE", colorTotals.get("BLUE") + greenAmount);
            }
        }

        // Check which color prevails
        switch (GetFace())
        {
            case "RED":
                return red;
            case "GREEN":
                return green;
            case "BLUE":
                return blue;
        }

        // Unreachable technically
        return input;
    }

    public Mat thresholdChannel(Mat input)
    {
        Mat threshold = new Mat();
        Imgproc.threshold(input, threshold, 230, 255, Imgproc.THRESH_BINARY);
        return threshold;
    }
}
