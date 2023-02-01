package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Optional;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

/*Not that the contents of this class are
particularly important, but I'll try
to document them*/

/*My best effort, but I don't really know what I'm doing
I also have to completely redo everything, since your
drivers were written for tele-op*/
@Autonomous(name="Autonomous Alpha")
public class AutonomousDriver extends LinearOpMode{
    AutonomousImageProcessor sleeve;
    DcMotor ul;
    DcMotor ur;
    DcMotor bl;
    DcMotor br;
	@Override
	//Ummm... this is a problem...
    public void init() {
        ul = hardwareMap.get(DcMotor.class, "ltMotor");
        ur  = hardwareMap.get(DcMotor.class, "rtMotor");
        bl = hardwareMap.get(DcMotor.class, "lbMotor");
		br = hardwareMap.get(DcMotor.class, "rbMotor");
		sleeve = new AutonomousImageProcessor(new VideoCapture(0));
    }
    //I have no idea how to do this, but I'm trying
    public void runOpMode() throws java.lang.InterruptedException {
		byte zone = sleeve.getSignalSleeveOrientation();
		//TODO: Fix tileTime
		int tileTime = 1; //This is the time in seconds for the robot to traverse one tile
						  //using a speed of 0.6
		float speed = 0.6f;
		//Does nothing if we can't find
		//the sleeve
		if (zone == 0) {
		    return;
		}
		//Move left
		if (zone == 1) {
		    ul.setPower(-speed);
		    ur.setPower(speed);
		    bl.setPower(speed);
		    br.setPower(-speed);
		}
		//Move right
		if (zone == 3) {
		    ul.setPower(speed);
		    ur.setPower(-speed);
		    bl.setPower(-speed);
		    br.setPower(speed);
		}
		//It's Thread.sleep(), right?
		Thread.currentThread().sleep(tileTime * 1000);
		//Move the robot forward
		ul.setPower(speed);
		ur.setPower(speed);
		bl.setPower(speed);
		br.setPower(speed);
		Thread.currentThread().sleep(tileTime * 1000);
		//Park the robot
		ul.setPower(0);
		ur.setPower(0);
		bl.setPower(0);
		br.setPower(0);
	}
	public class AutonomousImageProcessor {
		VideoCapture camera;
		public AutonomousImageProcessor(VideoCapture camera){
			this.camera = camera;
		}
		//If a camera is bound, returns the current frame
		//represented by the camera
		private Mat getCameraImage() {
			if (this.camera.isOpened()) {
				//Oops, someone forgot to bind the camera
				Mat out = new Mat();
				this.camera.read(out);
				return out;
			}
			return Mat.zeros(0, 0, 0);
		}
		//Searches for the signal sleeve and returns the
		//ID of the found signal area
		//(1 for left, 2 for centre, 3 for right)
		public Byte getSignalSleeveOrientation() {
			Mat frame = getCameraImage();
			if (frame.empty()) {
				//Oops, someone forgot to bind the camera
				return 0;
			}
			Mat fg = frame.clone();
			//This is all improv; no guarantee it'll work
			byte[] foundColours = new byte[(int) (fg.total())];
			byte[] fgData = new byte[(int) (fg.total() * fg.channels())];
			fg.get(0, 0, fgData);
			//Iterates over every pixel in fg, searching
			//for a pixel whose RGB channels roughly
			//match what we're expecting
			//1 is to left
			//2 is centred
			//3 is to right
			short optimalR1 = 61;
			short optimalG1 = 26;
			short optimalB1 = 127;
			short optimalR2 = 140;
			short optimalG2 = 35;
			short optimalB2 = 130;
			short optimalR3 = 45;
			short optimalG3 = 55;
			short optimalB3 = 145;
			//This is the amount by which we allow the
			//RGB values of the pixels to stray from
			//what we're expecting
			byte allowedThreshold = 10;
			//If any pixels look right, mark them
			for (int i = 0; i < (fgData.length / 3); i++) {
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
														fgData[i * 3 + 2] + allowedThreshold > optimalR1)))) {
					foundColours[i] = 1;
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
														fgData[i * 3 + 2] + allowedThreshold > optimalR2)))) {
					foundColours[i] = 2;
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
														fgData[i * 3 + 2] + allowedThreshold > optimalR3)))) {
					foundColours[i] = 3;
				}
			}
			//Looks for the best candidate for a signal sleeve
			//By that I mean figures out which colour appears the most
			int colour1 = 0;
			int colour2 = 0;
			int colour3 = 0;
			for (int i = 0; i < foundColours.length; i++) {
				if (foundColours[i] == 1) {
					colour1++;
				}
				if (foundColours[i] == 2) {
					colour2++;
				}
				if (foundColours[i] == 3) {
					colour3++;
				}
			}
			//Again, bad, but it should work
			if (colour1 > colour2 && colour1 > colour3) {
				return 1;
			}
			if (colour2 > colour1 && colour2 > colour3) {
				return 2;
			}
			if (colour3 > colour1 && colour3 > colour2) {
				return 3;
			}
			//If they were all the same, it'll sit there and do nothing
			return 0;
		}
	}
}

//VSCode is SO refreshing compared to Eclipse!
