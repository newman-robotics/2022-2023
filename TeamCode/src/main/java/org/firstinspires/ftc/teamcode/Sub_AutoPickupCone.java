package org.firstinspires.ftc.teamcode;

/*
@author Declan J. Scott

A sub program for autonomous that picks up a cone from the stack
when the robot arrives at a cone node. This assumes that the robot
is facing in the direction of the cone stack.
 */

public class Sub_AutoPickupCone {
    private AutoPathfinding parentProcess;
    private static Sub_AutoPickupCone singleton; // Singleton instance for static calls

    public Sub_AutoPickupCone(AutoPathfinding parentProcess) {
        this.parentProcess = parentProcess;
        singleton = this;

        // Initialize slide
        while (!parentProcess.linearSlide.initializedAtBottom)
            parentProcess.linearSlide.AutoUpdate();
    }

    public static void PickupCone()
    {
        LinearSlideController slide = singleton.parentProcess.linearSlide;
        slide.StartMove(2); // Move slide to medium height (above the cones)

        // Wait for slide to reach height
        while (!slide.atTarget())
            slide.AutoUpdate();

        singleton.parentProcess.grabber.Update(0); // Open Grabber Fully

        // Move forward 1/2 a node
        singleton.parentProcess.MoveDist(singleton.parentProcess.getDistanceBetweenPoints() / 2);

        // Move downwards until touch sensor is hit
        while (!singleton.parentProcess.grabber.getTopTouch().isPressed())
        {
            slide.getSlideMotor().setPower(0.5f);
        }

        // Hit. Close grabber
        singleton.parentProcess.grabber.Update(1);

        // Move slide to max height
        slide.StartMove(3);
        while (!slide.atTarget())
            slide.AutoUpdate();

        // Move back 1/2 a node
        singleton.parentProcess.MoveDist(-singleton.parentProcess.getDistanceBetweenPoints() / 2);
    }
}
