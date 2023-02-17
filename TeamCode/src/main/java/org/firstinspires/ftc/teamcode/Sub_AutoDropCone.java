package org.firstinspires.ftc.teamcode;

/*
@author Declan J. Scott

A sub program for autonomous that drops a cone on the pole. This assumes that the robot
is facing in the direction of the cone stack and has the it's slide at highest height.
 */

public class Sub_AutoDropCone {
    private AutoPathfinding parentProcess;
    private static Sub_AutoDropCone singleton; // Singleton instance for static calls

    public Sub_AutoDropCone(AutoPathfinding parentProcess) {
        this.parentProcess = parentProcess;
        singleton = this;
    }

    public static void DropCone()
    {
        // Move back 1/2 a node
        singleton.parentProcess.MoveDist(-singleton.parentProcess.getDistanceBetweenPoints() / 2);

        singleton.parentProcess.grabber.Update(0); // Open Grabber Fully

        // Move forward 1/2 a node
        singleton.parentProcess.MoveDist(singleton.parentProcess.getDistanceBetweenPoints() / 2);

        // Move slide to max height
        singleton.parentProcess.linearSlide.StartMove(0);
        while (!singleton.parentProcess.linearSlide.atTarget())
            singleton.parentProcess.linearSlide.AutoUpdate();

    }
}
