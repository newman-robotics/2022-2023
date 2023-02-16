package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;

public class ConeColorPipeline extends OpenCvPipeline {

    public Hashtable<String, Double> colorTotals = new Hashtable<>();
    public native int findZone(int[] pixels);
    private Mat currentVisual;

    public String GetFace()
    {
        processCurrent();

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
        currentVisual = input;
        return input;
    }

    public void processCurrent()
    {
        Mat input = currentVisual;

        // Need to zoom in for better accuracy and speed (win win)
        Point corner1 = new Point();
        Point corner2 = new Point();
        Rect rectCrop = new Rect((int) corner1.x, (int) corner1.y, (int) (corner2.x-corner1.x+1), (int) (corner2.y-corner1.y+1));
        input = input.submat(rectCrop);

        // Reset sums
        colorTotals.clear();
        colorTotals.put("RED", (double) 0);
        colorTotals.put("GREEN", (double) 0);
        colorTotals.put("BLUE", (double) 0);

        // Get split channels
        /*Mat red = new Mat();
        Core.extractChannel(input, red, 0);
        Mat green = new Mat();
        Core.extractChannel(input, green, 1);
        Mat blue = new Mat();
        Core.extractChannel(input, blue, 2);

        // Threshold each channel
        red = thresholdChannel(red);
        green = thresholdChannel(green);
        blue = thresholdChannel(blue);*/

        // Go through each pixel to get things
        // Let's see if I can get this in C++...
        int[] pixels = new int[(int)input.total()*input.channels()];
        input.get(0, 0, pixels);
        int foundZone = findZone(pixels);
        if (foundZone == 0) {
            colorTotals.put("RED", 1.0);
        } else if (foundZone == 1) {
            colorTotals.put("GREEN", 1.0);
        } else if (foundZone == 2) {
            colorTotals.put("BLUE", 1.0);
        }
        /*
        for (int y = 0; y < input.height(); y++) {
            for (int x = 0; x < input.width(); x++) {
                // Get amounts
                double redAmount = red.get(y, x)[0];
                double greenAmount = green.get(y, x)[0];
                double blueAmount = blue.get(y, x)[0];

                // Add to totals
                colorTotals.put("RED", colorTotals.get("RED") + redAmount);
                colorTotals.put("GREEN", colorTotals.get("GREEN") + greenAmount);
                colorTotals.put("BLUE", colorTotals.get("BLUE") + blueAmount);
            }
        }*/
    }

    public Mat thresholdChannel(Mat input)
    {
        Mat threshold = new Mat();
        Imgproc.threshold(input, threshold, 130, 255, Imgproc.THRESH_BINARY);
        return threshold;
    }
}
