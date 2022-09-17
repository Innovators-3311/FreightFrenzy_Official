package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDControl;


public class ArmPID_Control extends Arm2_Control {

    public PIDControl shoulderPID = null;
    public PIDControl elbowPID = null;
    public boolean shoulderInitialized = false;
    public int armState = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        // Save reference to Hardware map
        hwMap = ahwMap;
        cl = hwMap.get(Servo.class, "claw");
        mag = hwMap.get(Servo.class, "mag");

        //declare motors so that they can be used
        elbow = hwMap.get(DcMotorEx.class, "elbow");
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");

        shoulderPID = new PIDControl(shoulder,  -1e-1, -1e-3, -2e-5);
        elbowPID = new PIDControl(elbow, -4e-2, -1e-3, -5e-5);

        // Set the direction that the motors will turn
        elbow.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        // Note: you can not set direction on a servo

        //make sure motors are at zero power
        elbow.setPower(0);
        shoulder.setPower(0);
//        shoulder.setVelocity(0);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake the motors
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void arm_reset(){
        //shoulder
        switch (armState) {
            case 0: // Drive Shoulder to back
                shoulder.setPower(0.4);
                elbow.setPower(0.05);

                if (shoulderLimitSwitch.isPressed()) {
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armState = 1;
                }
                break;
            case 1: // Drive elbow to init position.
                elbow.setPower(-0.3);
                if (elbowLimitSwitch.isPressed()) {
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armState = 2;
                }
                break;
            case 2: // Move arm to init position.
                elbowDriveAbsolute(.7, 6);
                shoulderDriveAbsolute(.7, 42);
                armInitalized = true;
                armState = 3;
                break;
            case 3: // Continue to update arm.
                shoulderPID.update(getShoulderGravityVector() * SHOULDER_GRAVITY_FACTOR);
                elbowPID.update(0.0);

                break;
        }
    }

    /**
     * Drives the elbow joint to an absolute angle position.
     *
     * @param speed the maximum power to apply.
     * @param elbowAngle The target angle.
     */
    @Override
    public void elbowDriveAbsolute(double speed,
                                   double elbowAngle) {
        elbowPID.setTargetAngle(elbowAngle);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbowPID.maxPower = speed;
    }

    /**
     * Drives the shoulder joint to an absolute angle position.
     *
     * @param speed the maximum power to apply.
     * @param shoulderAngle The target angle.
     */
    @Override
    public void shoulderDriveAbsolute(double speed,
                                      double shoulderAngle) {
        shoulderPID.setTargetAngle(shoulderAngle);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulderPID.maxPower = speed;
    }

    /**
     * Gets the current elbow joint target angle.
     *
      * @return angle in degrees
     */
    @Override
    public double getElbowTargetAngle() {
        return elbowPID.getTargetAngle();
    }

    /**
     * Gets the current shoulder joint target angle.
     *
     * @return angle in degrees
     */
    @Override
    public double getShoulderTargetAngle() {
        return shoulderPID.getTargetAngle();
    }

    /**
     * This function updates the PID control based on the new encoder values for each joint by
     * calling the joint's update() command.
     *
     */
    @Override
    public void update() {

      //  Graph:https://www.desmos.com/calculator/k8serz0f8g
        double chill = Math.pow((shoulderPID.getTargetAngle()-100)/190, 2) +.2; // .75 + .5*Math.cos(2*(shoulderPID.getTargetAngle())*(Math.PI/180));
        chill = PIDControl.clamp(chill, .2, 1);
        shoulderPID.pChill = chill;

//        if (Math.abs(shoulderPID.getAngle()-90)< 30 &&
//                Math.abs(shoulderPID.getTargetAngle() - 90) < 30){
//          shoulderPID.pChill=.2;
//        }
//        else {
//            shoulderPID.pChill = 1;
//        }
//        if (Math.abs(elbowPID.getAngle()-100)< 30 &&
//                Math.abs(elbowPID.getTargetAngle() - 100) < 30){
//            elbowPID.pChill=.2;
//        }
//        else {
//            elbowPID.pChill = 1;
//        }
        shoulderPID.update(getShoulderGravityVector() * SHOULDER_GRAVITY_FACTOR);
        elbowPID.update(0);
    }

    @Override
    public boolean isBusy() {
        // Might want to have this check the shoulderPID and elbowPID's isBusy functions.
        return true; // shoulder.isBusy() || elbow.isBusy();
    }

    @Override
    public void scaleShoulderPid(double v) {
        shoulderPID.p *= v;
        shoulderPID.i *= v;
        shoulderPID.d *= v;
    }

    @Override
    public void showPIDs(Telemetry telemetry) {
        telemetry.addData("shoulderPID motor:", shoulderPID.motor.toString());

        telemetry.addData("shoulder p:", String.format("%.4e", shoulderPID.p));
        telemetry.addData("shoulder i:", String.format("%.4e", shoulderPID.i));
        telemetry.addData("shoulder d:", String.format("%.4e", shoulderPID.d));
        telemetry.addData("shoulder pos p:", String.format("%.4e", shoulderPID.p));
        telemetry.addData("shoulder pos i:", String.format("%.4e", shoulderPID.i));
        telemetry.addData("shoulder pos d:", String.format("%.4e", shoulderPID.d));
        telemetry.addData("shoulder vel:", String.format("%.4e", shoulder.getVelocity()));
    }
}
                                                                                                                                                                                                                             //Hi. You found me. -SECRET COMMENT