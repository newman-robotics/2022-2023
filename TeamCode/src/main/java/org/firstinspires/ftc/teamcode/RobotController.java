package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.AbstractQueue;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Deprecated;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.CvType;
import org.opencv.core.Scalar;
import org.opencv.core.Point;

/*
    @author Declan J. Scott
*/

// Base class for all sub-components. Standardizes the way in which components report telemetry
class BaseComponent
{
    private String ComponentID;
    private Hashtable<String, String> telemetryDict;


    // All components take in an ID for telemetry
    public BaseComponent(String componentID) {
        ComponentID = componentID;
        telemetryDict = new Hashtable<>();
    }

    // Adds data to a components telemetry queue
    public void AddTelemetry(String key, String value)
    {
        telemetryDict.put(ComponentID + ": " + key, value);
    }

    // Returns all telemetry
    public Hashtable<String, String> GetTelemetry()
    {
        return telemetryDict;
    }
}

// Controls the Linear Slide
class LinearSlideController extends BaseComponent
{
    private DcMotor slideMotor;
    private float speed;

    // Constructor
    public LinearSlideController(DcMotor slideMotor, float speed) {
        super("LINEAR_SLIDE");
        this.slideMotor = slideMotor;
        this.speed = speed;
    }

    // Takes in Gamepad input
    public void Update(float input)
    {
        slideMotor.setPower(input * speed);
        AddTelemetry("Power", String.valueOf(slideMotor.getPower()));
    }
}

// Controls the mechanum wheels
class MechanumWheelController extends BaseComponent
{
    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor bottomLeft;
    private DcMotor bottomRight;
    private float initialSpeed;
    private float currentSpeed;

    public MechanumWheelController(DcMotor topLeft, DcMotor topRight, DcMotor bottomLeft, DcMotor bottomRight, float speed) {
        super("DRIVETRAIN");
        this.topLeft = topLeft;
        this.topRight = topRight;
        this.bottomLeft = bottomLeft;
        this.bottomRight = bottomRight;
        this.initialSpeed = speed;
    }

    // Motor states
    public enum MotorState{
        FORWARD,
        BACKWARD,
        DISABLED
    }
    private MotorState rightSlantState = MotorState.DISABLED;
    private MotorState leftSlantState = MotorState.DISABLED;

    // Calculates joystick angle
    double MovementTheta(Gamepad gamepad)
    {
        float leftJoystickX = gamepad.left_stick_x;
        float leftJoystickY = -gamepad.left_stick_y;
        AddTelemetry("RAW X", String.valueOf(leftJoystickX));
        AddTelemetry("RAW Y", String.valueOf(leftJoystickY));

        // Get the direction of the joystick (this has to be the worst code I've ever written)
        float theta = 0;
        float tangent = leftJoystickY / leftJoystickX;
        float additionalAngle = (float) Math.abs(Math.toDegrees(Math.atan(tangent)));
        if(leftJoystickX <= 0 && leftJoystickY > 0)
        {
            theta = 90;
            theta += 90 - additionalAngle;
        }
        else if(leftJoystickX < 0 && leftJoystickY <= 0)
        {
            theta = 180;
            theta += additionalAngle;
        }
        else if(leftJoystickX >= 0 && leftJoystickY < 0)
        {
            theta = 270;
            theta += 90 - additionalAngle;
        }else{
            theta = additionalAngle;
        }

        AddTelemetry("Theta", String.valueOf(theta));
	//theta = -theta //A possible fix to the mirroring problem
        return theta;
    }

    public void Update(Gamepad inputDevice)
    {
        // Get joystick direction
        double theta = MovementTheta(inputDevice);

        currentSpeed = inputDevice.right_bumper ? initialSpeed * 2 : initialSpeed;

        // control steering or driving
        if(!Double.isNaN(theta))
        {
            if(inputDevice.left_bumper)
                Steer((float) theta);
            else
                Drive((float) theta);
        }else{
            // Reset power to 0
            topRight.setPower(0);
            bottomLeft.setPower(0);
            topLeft.setPower(0);
            bottomRight.setPower(0);
        }
    }

    private void Drive(float theta)
    {
        // Configure slant states
        // Right slant
        if(theta >= 85 && theta <= 185)
            rightSlantState = MotorState.FORWARD;
        else if(theta >= 265 && theta <= 360)
            rightSlantState = MotorState.BACKWARD;
        else
            rightSlantState = MotorState.DISABLED;

        // Left slant
        if(theta >= 0 && theta <= 95)
            leftSlantState = MotorState.FORWARD;
        else if(theta >= 175 && theta <= 275)
            leftSlantState = MotorState.BACKWARD;
        else
            leftSlantState = MotorState.DISABLED;

        // Lazy, but I got to do it
        if(Math.round(theta / 10) == 0)
        {
            leftSlantState = MotorState.FORWARD;
            rightSlantState = MotorState.BACKWARD;
        }

        // Configure motor power
        float rightSlantPower = rightSlantState == MotorState.FORWARD ? currentSpeed : -currentSpeed;
        if(rightSlantState == MotorState.DISABLED)
            rightSlantPower = 0;

        float leftSlantPower = leftSlantState == MotorState.FORWARD ? currentSpeed : -currentSpeed;
        if(leftSlantState == MotorState.DISABLED)
            leftSlantPower = 0;

        AddTelemetry("Right Slant State", rightSlantState.toString());
        AddTelemetry("Left Slant State", leftSlantState.toString());

        // Set power
        topRight.setPower(rightSlantPower);
        bottomLeft.setPower(-rightSlantPower);
        topLeft.setPower(-leftSlantPower);
        bottomRight.setPower(leftSlantPower);
    }

    private void Steer(float theta)
    {
        float leftWheelPower = 0;
        float rightWheelPower = 0;
        if(theta >= 90 && theta <= 270)
        {
            leftWheelPower = -currentSpeed;
            rightWheelPower = currentSpeed;
        }else {
            leftWheelPower = currentSpeed;
            rightWheelPower = -currentSpeed;
        }

        // Set Power
        bottomLeft.setPower(-leftWheelPower);
        topRight.setPower(rightWheelPower);
        bottomRight.setPower(rightWheelPower);
        topLeft.setPower(-leftWheelPower);
    }
}

@TeleOp
public class RobotController extends LinearOpMode {

    // Components
    private LinearSlideController linearSlide;
    private MechanumWheelController drivetrain;
    @Override
    public void runOpMode() {
        // Set up linear slide
        linearSlide = new LinearSlideController(hardwareMap.get(DcMotor.class, "slider"), 0.5f);

        // Set up drivetrain
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "ltMotor");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "rtMotor");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "lbMotor");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "rbMotor");
        MechanumWheelController drivetrain = new MechanumWheelController(topLeft, topRight, bottomLeft, bottomRight, 0.4f);

        // Update initialization telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            linearSlide.Update(gamepad1.right_stick_y);
            drivetrain.Update(gamepad1);

            // Report Telemetry
            ReportTelemetry(linearSlide.GetTelemetry());
            ReportTelemetry(drivetrain.GetTelemetry());
            telemetry.update();
        }
    }

    // Takes a components telemetry and reports it to the OpMode telemetry
    public void ReportTelemetry(Hashtable<String, String> toReport)
    {
        Enumeration telemetryKeys = toReport.keys();
        while (telemetryKeys.hasMoreElements())
        {
            String key = (String) telemetryKeys.nextElement();
            String value = toReport.get(key);

            telemetry.addData(key, value);
        }
    }
}

//Everything past this point is pure theory
//None of it is actually implemented
//You tell me if it'll actually work

//Not that the contents of this class are
//particularly important, but I'll try
//to document them
public class AutonomousImageProcessing {
	 VideoCapture camera;
	 //Always call when creating a new AutonomousImageProcessing
	 //to establish a camera connection
	 public void bindCamera(VideoCapture incamera){
		 camera = incamera;
	 }
	 //If a camera is bound, returns the current frame 
	 //represented by the camera
	 //TODO: Calibrate the camera somehow
         //(if it's not already calibrated)
	 private Optional<Mat> getCameraImage() {
		 Mat image = new Mat();
		 if (camera.isOpened()) {
			 //Oops, someone forgot to bind the camera
			 return Optional.of(camera.read(image));
		 }
		 return Optional.empty();
	 }
	 //Returns a segmented version of inputImage
	 //that can be used as a mask over the original
         //image to divide it into objects
         //This should help you understand why we're
         //probably NOT supposed to be using OpenCV
	 private Mat segmentImage(Mat inputImage) {
		 Mat kernel = new Mat(3, 3, CvType.CV_32F);
		 float[] kernelData = new float[(int) (kernel.total() * kernel.channels())];
		 kernelData[0] = 1; kernelData[1] = 1; kernelData[2] = 1;
		 kernelData[3] = 1; kernelData[4] = -8; kernelData[5] = 1;
		 kernelData[6] = 1; kernelData[7] = 1; kernelData[8] = 1;
		 kernel.put(0, 0, kernelData);
		 //Laplacian filtering to make the edges acute
		 Mat laplacian = new Mat();
		 Imgproc.filter2D(inputImage, laplacian, 0, kernel);
		 Mat sharp = new Mat();
		 inputImage.convertTo(sharp, CvType.CV_32F);
		 Mat result = new Mat();
		 Core.subtract(sharp, laplacian, result);
		 result.convertTo(result, CvType.CV_8UC3);
		 laplacian.convertTo(laplacian, CvType.CV_8UC3);
		 //Create binary image
		 Mat bw = new Mat();
		 Imgproc.cvtColor(result, bw, Imgproc.COLOR_BGR2GRAY);
		 Imgproc.threshold(bw, bw, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
		 //Perform distance transform algorithm
		 Mat dist = new Mat();
		 Imgproc.distanceTransform(bw, dist, Imgproc.DIST_L2, 3);
		 //Normalize the distance image
		 Core.normalize(dist, dist, 0.0, 1.0, Core.NORM_MINMAX);
		 Mat distDisplayScaled = new Mat();
		 Core.multiply(dsit, new Scalar(255), distDisplayScaled);
		 Mat distDisplay = new Mat();
		 distDisplayScaled.convertTo(distDisplay, CvType.CV_8U);
		 //Threshold to obtain the peaks
		 Imgproc.threshold(dist, dist, 0.4, 1.0, Imgproc.THRESH_BINARY);
		 //Dilate the dist image a little
		 Mat kernel1 = Mat.onew(3, 3, CvType.CV_8U);
		 Imgproc.dilate(dist, dist, kernel1);
		 Mat distDisplay2 = new Mat();
		 dist.convertTo(distDisplay2, CvType.CV_8U);
		 Core.multiply(distDisplay2, new Scalar(255), distDisplay2);
		 //Create markers
		 Mat dist_8u = new Mat();
		 dist.convertTo(dist_8u, CvType.CV_8U);
		 List<MatOfPoint> contours = new ArrayList<>();
		 Mat hierarchy = new Mat();
		 Imgproc.findContours(dist_8u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		 Mat markers = Mat.zeros(dist.size(), CvType.CV_32S);
		 for (int i = 0; i < contours.size(); i++) {
			 Imgproc.drawContours(markers, contours, i, new Scalar(i+1), -1);
		 }
		 Mat markersScaled = new Mat();
		 markers.convertTo(markersScaled, CvType.CV_32F);
		 Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
		 Imgproc.circle(markersScaled, new Point(5, 5), 3, new Scalar(i + 1), -1);
		 Mat markersDisplay = new Mat();
		 markersScaled.convertTo(markersDisplay, CvType.CV_8U);
		 Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
		 Imgproc.watershed(imgResult, markers);
		 Mat mark = Mat.zeros(markers.sixe(), CvType.CV_8U);
		 markers.convertTo(mark,  CvType.CV_8UC1);
		 Core.bitwise_not(mark, mark);
		 //Fill everything in with random colours
		 //The random colours help the Canny edge
		 //detector do its job
		 Random rng = new Random(12345);
		 List<Scalar> colours = new ArrayList<>(contours.size());
		 for (int i = 0; i < contours.size(); i++) {
		     int b = rng.nextInt(256);
		     int g = rng.nextInt(256);
		     int r = rng.nextInt(256);
		     colours.add(new Scalar(b, g, r));
		 }
		 Mat dstColoured = Mat.zeros(markers.size(), CvType.CV_8UC3);
		 byte[] dstData = new byte[(int) (dstColoured.total() * dstColoured.channels())];
		 dstColoured.get(0, 0, dstData);
		 int[] markersData = new int[(int) (markers.total() * markers.channels())];
		 markers.get(0, 0, markersData);
		 //This part actually fills the image in
		 for (int i = 0; i < markers.rows(); i++) {
		     for (int j = 0; j < markers.cols(); j++) {
			 int index = markersData[i * markers.cols() + j];
			 if (index > 0 && index <= contours.size()) {
			     dstData[(i * dstColoured.cols() + j) * 3 + 0] = (byte) colours.get(index-1).val[0];
			     dstData[(i * dstColoured.cols() + j) * 3 + 1] = (byte) colours.get(index-1).val[1];
			     dstData[(i * dstColoured.cols() + j) * 3 + 2] = (byte) colours.get(index-1).val[2];
			 } else {
			     dstData[(i * dstColoured.cols() + j) * 3 + 0] = 0;
			     dstData[(i * dstColoured.cols() + j) * 3 + 1] = 0;
			     dstData[(i * dstColoured.cols() + j) * 3 + 2] = 0;
			 }
		     }
		 }
		 dstColoured.put(0, 0, dstData);
		 //Now we run a Canny edge detector
		 //on the coloured image
		 //By this point I'm starting to realise that
		 //this is probably the worst code from every
		 //FIRST competition carried out this year
		 Mat dstGray = new Mat;
		 Mat canny = new Mat;
		 Imgproc.cvtColor(dstColoured, dstGray, Imgproc.COLOR_BGR2GRAY);
		 Imgproc.blur(dstGray, canny, new Size(3, 3));
		 Imgproc.Canny(canny, canny, 100, 100*3);
		 Mat out = new Mat;
		 Core.add(out, Scalar.all(0), out);
		 frame.copyTo(out, canny);
		 //Finally we return out
		 //That was awful
		 return out;
	 }
	 //If a camera is bound, segments the current frame
	 //into its component parts.
         //Only use for testing purposes
         @Deprecated(forRemoval=true)
	 public Optional<Mat> getImageObjects() {
		 Optional<Mat> frame = this.getCameraImage();
		 if (!frame.isPresent()) {
			 //Oops, someone forgot to bind the camera
			 return Optional.empty();
		 }
		 return Optional.of(segmentImage(frame)));
         }
//By this point, Eclipse is broken, so I'm using EMACS
//Searches for the signal sleeve and returns the
//ID of the found signal area
//(1 for right, 2 for centre, 3 for left)
//TODO: Finish this
//(It should work by layering the mask over the current frame,
//and then removing everything outside of the mask before
//looking for the colours green, orange, or magenta, which
//will be the colours of the custom signal sleeve)
public byte getSignalSleeveOrientation() {
    Optional<Mat> frame = getImageObjects();
    if (!frame.isPresent) {
	//Oops, someone forgot to bind the camera
        return Optional.empty();
    }
    Mat mask = segmentImage(frame);
    Mat fg = new Mat;
    return Optional.empty();
}
}

//My best effort, but I don't really know what I'm doing
//I also have to completely redo everything, since your
//drivers were written for tele-op
public class AutonomousDriver {
    AutonomousImageProcessor sleeve;
    DcMotor ul;
    DcMotor ur;
    DcMotor bl;
    DcMotor br;
    private void init() {
        ul = hardwareMap.get(DcMotor.class, "ltMotor");
        ur  = hardwareMap.get(DcMotor.class, "rtMotor");
        bl = hardwareMap.get(DcMotor.class, "lbMotor");
	br = hardwareMap.get(DcMotor.class, "rbMotor");
	sleeve = new AutonomousImageProcessor();
	sleeve.bind(new VideoCapture(0));
	//TODO: I don't think 0 is the right camera
	//for the setup we're using
    }
    //I have no idea how to do thist, but I'm trying
    //This should only run once
    public void run() {
	byte zone = sleeve.getSignalSleeveOrientation();
	//Adjust for zone
	if (zone == 0) {
	    ul.setPower();
	    ur.setPower();
	    bl.setPower();
	    br.setPower();
	}
	if (zone == 2) {
	    ul.setPower();
	    ur.setPower();
	    bl.setPower();
	    br.setPower();
	}
	//Move the robot forward
	ul.setPower();
	ur.setPower();
	bl.setPower();
	br.setPower();
    }
}
