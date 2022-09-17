package org.firstinspires.ftc.teamcode.source;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the setup and control class for the arm motors and servos, and the duck spinner motor
 */


public class Arm {
    public DcMotor shoulder = null;
    public DcMotor elbow = null;
    public DcMotor spinner = null;
    public DcMotor intake = null;

    public Servo claw = null;
    public Servo magnet = null;

    private int shoulderTarget = 0;
    private double shoulderGravity;
    public boolean shoulderIsBusy = false;

    private int elbowTarget = 0;
    private double elbowGravity;
    public boolean elbowIsBusy = false;

    private int intakeTarget = 0;
    public boolean intakeIsBusy = false;
    private int intakeMultiplier = -1;

    public double shoulderErr;
    public double elbowErr;

    private int i = 0;

    //both shoulder and elbow encoders have 8192 ticks per revolution

    PID shoulderPID = new PID();
    PID elbowPID = new PID();

    ElapsedTime timer = new ElapsedTime();

    private int encoderMultiplier = -1;

    public Arm(HardwareMap hardwareMap) {
        //initializing all hardware
        shoulderPID.setCoefficients(0.05, 0.01, 0.0006,-1, 1, 5);
        elbowPID.setCoefficients(0.035, 0.007, 0.00015, -1, 1, 5);

        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        intake = hardwareMap.get(DcMotor.class, "intake");

        claw  = hardwareMap.get(Servo.class, "claw");
        magnet = hardwareMap.get(Servo.class, "mag");

        shoulder.setDirection(DcMotor.Direction.FORWARD);
        elbow.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * Resetting the encoders.  It has to be set to run without encoder mode or else a qualcomm class appears to put a PID controller on the motor velocity, which will
         * mess with any of your own code.
         */
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateShoulder() {
        shoulderPID.update(shoulderTarget, encoderMultiplier * shoulder.getCurrentPosition() / 22.76);
        shoulderGravity =  0.05 * Math.cos( Math.toRadians(encoderMultiplier * shoulder.getCurrentPosition() / 22.76 + 150) );
        shoulder.setPower(shoulderPID.output + shoulderGravity); //forward is towards the back of the robot
        if(Math.abs(shoulderPID.Err) < 3) {
            shoulderIsBusy = false;
        }
        shoulderErr = shoulderPID.Err;
    }
    public void updateElbow() {
        elbowPID.update(elbowTarget, encoderMultiplier * elbow.getCurrentPosition() / 22.76);
        elbowGravity =  0.02 * Math.cos(
                Math.toRadians(encoderMultiplier * shoulder.getCurrentPosition() / 22.76 + encoderMultiplier * elbow.getCurrentPosition() / 22.76 + 35 )
        );
        elbow.setPower(elbowPID.output + elbowGravity);
        if(Math.abs(elbowPID.Err) < 3) {
            elbowIsBusy = false;
        }
        elbowErr = elbowPID.Err;
    }
    public void updateIntake() {
        if (timer.milliseconds() > 1250) {
            intake.setPower(0);
            intakeIsBusy = false;
        }
    }
    public void openIntake() {
        intakeIsBusy = true;
        intake.setPower(1);
        timer.reset();
    }
    public void closeIntake() {
        intakeIsBusy = true;
        intake.setPower(-1);
        timer.reset();
    }
    public void runShoulderTo(int shoulderTarget) {
        shoulderIsBusy = true;
        this.shoulderTarget = shoulderTarget - 150;
    }
    public void runElbowTo(int elbowTarget) {
        elbowIsBusy = true;
        this.elbowTarget = elbowTarget - 35;
    }
    public void storeArmPose() {
        PoseStorage.shoulderTicks = shoulder.getCurrentPosition();
        PoseStorage.elbowTicks = elbow.getCurrentPosition();
    }
}
