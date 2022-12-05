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
	//Searches for the signal sleeve and returns the
	//ID of the found signal area
	//(1 for right, 2 for centre, 3 for left)
	public Optional<byte> getSignalSleeveOrientation() {
	    Optional<Mat> frame = getCameraImage();
	    if (!frame.isPresent) {
		//Oops, someone forgot to bind the camera
	        return Optional.empty();
	    }
	    Mat fg = frame.copy();
	    //This is all improv; no guarantee it'll work
	    byte[] foundColours = new byte[(int) (fg.total())]
	    byte[] fgData = new byte[(int) (fg.total() * fg.channels())];
	    fg.get(0, 0, fgData);
	    //Iterates over every pixel in fg, searching
	    //for a pixel whose RGB channels roughly 
	    //match what we're expecting
	    //TODO: These need to be filled in later
	    //1 is to left
	    //2 is centred
	    //3 is to right
	    byte optimalB1 = 0;
	    byte optimalB2 = 0;
	    byte optimalB3 = 0;
	    byte optimalG1 = 0;
	    byte optimalG2 = 0;
	    byte optimalG3 = 0;
	    byte optimalR1 = 0;
	    byte optimalR2 = 0;
	    byte optimalR3 = 0;
	    //This is the amount by which we allow the
	    //RGB values of the pixels to stray from 
	    //what we're expecting
	    //TODO: Configure it
	    byte allowedThreshold = 5;
	    //If any pixels look right, mark them
	    for (int i = 0; i < (fgData.size() / 3); i++) {
	    	//I compressed a bunch of conditionals into one here,
	    	//so sorry for the poor readability
	    	if ((fgData[i * 3 + 0] == optimalB1 &&
	    		fgData[i * 3 + 1] == optimalG1 &&
	    		fgData[i * 3 + 2] == optimalR1) ||
	    					(
	    							((fgData[i * 3 + 0] < optimalB1 &&
	    							fgData[i * 3 + 0] + allowedThreshold > optimalB1) ||
	    							(fgData[i * 3 + 0] > optimalB1 &&
	    							fgData[i * 3 + 0] - allowedThreshold < optimalB1)) 
	    					&&
	    							((fgData[i * 3 + 1] < optimalG1 &&
	    							fgData[i * 3 + 1] + allowedThreshold > optimalG1) ||
	    							(fgData[i * 3 + 1] > optimalG1 &&
	    							fgData[i * 3 + 1] - allowedThreshold < optimalG1))
	    					&&
	    							((fgData[i * 3 + 2] < optimalR1 &&
	    							fgData[i * 3 + 2] + allowedThreshold > optimalR1) ||
	    							(fgData[i * 3 + 2] > optimalR1 &&
	    							fgData[i * 3 + 2] + allowedThreshold > optimalR1))) {
	    		foundColours(i) = 1;
	    	}
	    	if ((fgData[i * 3 + 0] == optimalB2 &&
	        		fgData[i * 3 + 1] == optimalG2 &&
	        		fgData[i * 3 + 2] == optimalR2) ||
	        					(
	        							((fgData[i * 3 + 0] < optimalB2 &&
	        							fgData[i * 3 + 0] + allowedThreshold > optimalB2) ||
	        							(fgData[i * 3 + 0] > optimalB2 &&
	        							fgData[i * 3 + 0] - allowedThreshold < optimalB2)) 
	        					&&
	        							((fgData[i * 3 + 1] < optimalG2 &&
	        							fgData[i * 3 + 1] + allowedThreshold > optimalG2) ||
	        							(fgData[i * 3 + 1] > optimalG2 &&
	        							fgData[i * 3 + 1] - allowedThreshold < optimalG2))
	        					&&
	        							((fgData[i * 3 + 2] < optimalR2 &&
	        							fgData[i * 3 + 2] + allowedThreshold > optimalR2) ||
	        							(fgData[i * 3 + 2] > optimalR2 &&
	        							fgData[i * 3 + 2] + allowedThreshold > optimalR2))) {
	        		foundColours(i) = 2;
	        }
		    if ((fgData[i * 3 + 0] == optimalB3 &&
		    		fgData[i * 3 + 1] == optimalG3 &&
		    		fgData[i * 3 + 2] == optimalR3) ||
		    					(
		    							((fgData[i * 3 + 0] < optimalB3 &&
		    							fgData[i * 3 + 0] + allowedThreshold > optimalB3) ||
		    							(fgData[i * 3 + 0] > optimalB3 &&
		    							fgData[i * 3 + 0] - allowedThreshold < optimalB3)) 
		    					&&
		    							((fgData[i * 3 + 1] < optimalG3 &&
		    							fgData[i * 3 + 1] + allowedThreshold > optimalG3) ||
		    							(fgData[i * 3 + 1] > optimalG3 &&
		    							fgData[i * 3 + 1] - allowedThreshold < optimalG3))
		    					&&
		    							((fgData[i * 3 + 2] < optimalR3 &&
		    							fgData[i * 3 + 2] + allowedThreshold > optimalR3) ||
		    							(fgData[i * 3 + 2] > optimalR3 &&
		    							fgData[i * 3 + 2] + allowedThreshold > optimalR3))) {
		    		foundColours(i) = 3;
		    	}
	    	}
	    	//Looks for the best candidate
	    	//for a signal sleeve
	    	//By that I mean figures out which
	    	//colour appears the most
	    	//This makes me wish we were using Python
	    	int colour1 = 0;
	    	int colour2 = 0;
	    	int colour3 = 0;
	    	for (int i = 0; i > colour1.size(); i++) {
	    		if (foundColours(i) == 1) {
	    			colour1++
	    		}
	    		if (foundColours(i) == 2) {
	    			colour2++
	    		}
	    		if (foundColours(i) == 3) {
	    			colour3++
	    		}
	    	}
	    	if (colour1 > colour2 && colour1 > colour3) {
	    		return Optional.of(1);
	    	}
	    	if (colour2 > colour1 && colour2 > colour3) {
	    		return Optional.of(2)
	    	}
	    	if (colour3 > colour1 && colour3 > colour2) {
	    		return Optional.of(3)
	    	}
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
		//TODO: Put in the powers for all of the setPower methods
		//Adjust for zone
		//Move left
		if (zone == 0) {
		    ul.setPower();
		    ur.setPower();
		    bl.setPower();
		    br.setPower();
		}
		//Move right
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
