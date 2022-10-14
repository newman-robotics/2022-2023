package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Queue;

/*
    @author Declan J. Scott
*/

// Base class for all sub-components. Standardizes the way in which components report telemetry
class BaseComponent
{
    private String ComponentID;
    private Queue<String[]> telemetryDict;

    // All components take in an ID for telemetry
    public BaseComponent(String componentID) {
        ComponentID = componentID;
    }

    // Adds data to a components telemetry queue
    public void AddTelemetry(String key, String value)
    {
        telemetryDict.add(new String[]{ComponentID + ": " + key, value});
    }

    // Returns all telemetry
    public Queue<String[]> GetTelemetry()
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

@TeleOp
public class RobotController extends LinearOpMode {

    // Components
    private LinearSlideController linearSlide;

    @Override
    public void runOpMode() {
        // Get instances of all components
        linearSlide = new LinearSlideController(hardwareMap.get(DcMotor.class, "slider"), 0.5f);

        // Update initialization telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            linearSlide.Update(-gamepad1.right_stick_y);

            // Report Telemetry
            ReportTelemetry(linearSlide.GetTelemetry());
            telemetry.update();
        }
    }

    // Takes a components telemetry and reports it to the OpMode telemetry
    public void ReportTelemetry(Queue<String[]> toReport)
    {
        for (String[] data : toReport)
        {
            telemetry.addData(data[0], data[1]);
        }
    }
}
