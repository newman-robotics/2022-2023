package org.firstinspires.ftc.teamcode.TEST_OPS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
    @author Declan J. Scott
*/

@TeleOp(group="Test")
public class EncoderTest extends LinearOpMode {

    DcMotor encodedMotor1;
    DcMotor encodedMotor2;
    DcMotor encodedMotor3;
    DcMotor encodedMotor4;
    int startCounts1 = 0;
    int startCounts2 = 0;
    int startCounts3 = 0;
    int startCounts4 = 0;

    // Conversion counts to meters
    double countsPerRev = 537.7; //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    double wheelCircumference = 0.301593; // Diameter is 96mm. https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/
    double pulleyCircumference = 0.112; // https://www.gobilda.com/4-stage-viper-slide-kit/
    double countsPerMeters = countsPerRev / pulleyCircumference;

    @Override
    public void runOpMode() {
        waitForStart();
        encodedMotor1 = hardwareMap.get(DcMotor.class, "ENCODED1");
//        encodedMotor2 = hardwareMap.get(DcMotor.class, "ENCODED2");
//        encodedMotor3 = hardwareMap.get(DcMotor.class, "ENCODED3");
//        encodedMotor4 = hardwareMap.get(DcMotor.class, "ENCODED4");

        startCounts1 = encodedMotor1.getCurrentPosition();
//        startCounts2 = encodedMotor2.getCurrentPosition();
//        startCounts3 = encodedMotor3.getCurrentPosition();
//        startCounts4 = encodedMotor4.getCurrentPosition();

        while (opModeIsActive()) {
            int position1 = encodedMotor1.getCurrentPosition();
//            int position2 = encodedMotor2.getCurrentPosition();
//            int position3 = encodedMotor3.getCurrentPosition();
//            int position4 = encodedMotor4.getCurrentPosition();

            encodedMotor1.setPower(gamepad1.right_stick_y);
//            encodedMotor2.setPower(gamepad1.right_trigger);
//            encodedMotor3.setPower(gamepad1.right_trigger);
//            encodedMotor4.setPower(gamepad1.right_trigger);

//            int countsTravelled = ((position1 - startCounts1) + (position2 + startCounts2) + (position3 + startCounts3) + (position4 + startCounts4)) / 4;
            int countsTravelled = position1 - startCounts1;
            double metersTravelled = countsTravelled / countsPerMeters;

            telemetry.addData("METERS TRAVELLED: ",  String.valueOf(metersTravelled));
            telemetry.addData("INPUTTED POWER: ", gamepad1.right_stick_y);
            telemetry.addData("MOTOR POSITION (1): ", position1);
//            telemetry.addData("MOTOR POSITION (2): ", position2);
//            telemetry.addData("MOTOR POSITION (3): ", position3);
//            telemetry.addData("MOTOR POSITION (4): ", position4);
            telemetry.update();
        }
    }
}
