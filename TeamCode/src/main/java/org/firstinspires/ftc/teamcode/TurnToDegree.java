package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.FreightFrenzyConceptCompassCalibration;


@TeleOp(name = "TurnToDegree", group = "concept")
public class TurnToDegree extends FreightFrenzyConceptCompassCalibration {
    private double Degrees;
    private double DegreesGoal;
    CompassSensor compass;


    public void runOpMode() {

        leftDriveFront = hardwareMap.get(DcMotor.class, "lf");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        /* Initialize the org.firstinspires.ftc.teamcode.drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        init(hardwareMap);

        double StartingDegrees = 0;
        compass = hardwareMap.get(CompassSensor.class, "compass");

        StartingDegrees = compass.getDirection();


        DegreesGoal = 90;//(StartingDegrees + 90) % 360;

        leftDriveFront.setPower(.5);
        rightDriveFront.setPower(-.5);
        leftDriveBack.setPower(.5);
        rightDriveBack.setPower(-.5);
        while (Degrees < DegreesGoal) {
            Degrees = compass.getDirection();
        }
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        telemetry.addData("Degrees", Degrees);
        telemetry.addData("StartingDegrees", StartingDegrees);
    }
}

