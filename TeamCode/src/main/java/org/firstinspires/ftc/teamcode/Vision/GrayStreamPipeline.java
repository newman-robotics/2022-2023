package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
    @author Declan J. Scott
*/

public class GrayStreamPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input)
    {
        Mat matYCrCb = new Mat();
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2GRAY);
        return matYCrCb;
    }
}
