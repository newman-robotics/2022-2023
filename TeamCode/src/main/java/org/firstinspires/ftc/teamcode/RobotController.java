package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.AbstractQueue;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.PriorityQueue;
import java.util.Queue;

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
        return theta;
    }

    public void Update(Gamepad inputDevice)
    {
        // Get joystick direction
        double theta = MovementTheta(inputDevice);

        currentSpeed = inputDevice.right_bumper ? initialSpeed * 0.5f : initialSpeed;

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

    public void Drive(float theta)
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

    public void Steer(float theta)
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

class Grabber extends BaseComponent
{
    private Servo lServo;
    private Servo rServo;
    private float maxServo = 0.5f;
    private float lOffset;
    private float rOffset;

    public Grabber(String componentID, Servo lServo, Servo rServo, float l, float r) {
        super(componentID);
        this.lServo = lServo;
        this.rServo = rServo;
        lOffset = l;
        rOffset = r;
        this.rServo.setDirection(Servo.Direction.REVERSE);
        this.lServo.setDirection(Servo.Direction.FORWARD);
    }

    public void Update(float input)
    {
        lServo.setPosition(Math.min(maxServo, input) + lOffset);
        rServo.setPosition(Math.min(maxServo, input) + rOffset);

        AddTelemetry("Left Grabber", String.valueOf(lServo.getPosition()));
        AddTelemetry("Right Grabber", String.valueOf(rServo.getPosition()));
        AddTelemetry("Raw Input", String.valueOf(input));
    }
}

@TeleOp
public class RobotController extends LinearOpMode {

    // Components
    protected LinearSlideController linearSlide;
    protected MechanumWheelController drivetrain;
    protected Grabber grabber;

    public void getSubComponents()
    {
        // Set up linear slide
        linearSlide = new LinearSlideController(hardwareMap.get(DcMotor.class, "slider"), 0.5f);

        // Set up drivetrain
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "ltMotor");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "rtMotor");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "lbMotor");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "rbMotor");
        drivetrain = new MechanumWheelController(topLeft, topRight, bottomLeft, bottomRight, 0.4f);

        // Set up grabber
        Servo rGrabber = hardwareMap.get(Servo.class, "rGrabber");
        Servo lGrabber = hardwareMap.get(Servo.class, "lGrabber");
        grabber = new Grabber("Grabber", lGrabber, rGrabber, 0.65f, 0.35f);
    }

    @Override
    public void runOpMode() {
        getSubComponents();

        // Update initialization telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            linearSlide.Update(gamepad1.right_stick_y);
            drivetrain.Update(gamepad1);
            grabber.Update(gamepad1.right_trigger);

            // Report Telemetry
            ReportTelemetry(linearSlide.GetTelemetry());
            ReportTelemetry(drivetrain.GetTelemetry());
            ReportTelemetry(grabber.GetTelemetry());
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
