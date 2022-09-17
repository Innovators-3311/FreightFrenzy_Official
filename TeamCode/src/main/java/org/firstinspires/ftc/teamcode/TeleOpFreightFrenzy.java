/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DriveDirection;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOpFreightFrenzy", group = "3311")
public class TeleOpFreightFrenzy extends OpMode {
    // Declare OpMode members.
    protected final ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDriveFront        = null;
    public DcMotor rightDriveFront       = null;
    public DcMotor leftDriveBack         = null;
    public DcMotor rightDriveBack        = null;
    public DcMotor spinner               = null;
    public DcMotor intake                = null;
    public ColorSensor colorSensor2      = null;
    public DistanceSensor distanceSensor = null;
    public double speedFactor            = 1.0;
    public double driveAngleOffSet = 0.0;
    /* Setup a variable for each drive wheel to save power level for telemetry */
    public double leftPowerFront  = 1.0;
    public double rightPowerFront = 1.0;
    public double rightPowerBack  = 1.0;
    public double leftPowerBack   = 1.0;
    public double drive           = 0.0;
    public double turn            = 0.0;
    public double strafe          = 0.0;
    public double strafeValue     = 0.0;
    public double backup          = 0;
    public int armLevel = 1;
    //Bring in code to setup arm.
    protected Arm2_Control arm = new Arm2_Control();
    // debounce A & X
    protected boolean debounceA = false;
    protected boolean debounceX = false;
    private boolean distanceSlow= false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        this.arm.init(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDriveFront = hardwareMap.get(DcMotor.class, "lf");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rf");
        leftDriveBack = hardwareMap.get(DcMotor.class, "lb");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rb");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        colorSensor2 = hardwareMap.colorSensor.get("colorSensor3");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor3");
        intake = hardwareMap.get(DcMotor.class, "intake");
        // Set Motor Direction
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);


        // Run Without Encoders
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake when power set to Zero
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    protected void handleIntake() {
        if (gamepad2.a) {
            intake.setPower(1);
        }else{
            if(gamepad2.x){
                intake.setPower(-1);
            }else{
                intake.setPower(0);
            }
        }

    }
// Daniel
    protected void handleArm() {
        double elbowPower    = 0.0;
        double shoulderPower = 0.0;
        // If driver moves the stick more than 10% or we're already in run without encoder mode ...
        elbowPower += gamepad2.left_stick_y * 0.7;
        shoulderPower += gamepad2.right_stick_y * 0.7;
        elbowPower -= -gamepad2.right_stick_x * 0.7;
        shoulderPower += -gamepad2.right_stick_x * 0.7;

        arm.shoulderMovePower(shoulderPower);
        arm.elbowMovePower(elbowPower);

        telemetry.addData("shoulder current angle:", arm.getShoulderAngle());
        telemetry.addData("shoulder angle target:", arm.getShoulderTargetAngle());
        telemetry.addData("gravity vector", arm.getShoulderGravityVector());
        // Publish Elbow values
        telemetry.addData("elbow Current angle:", arm.getElbowAngle());
        telemetry.addData("elbow angle target:", arm.getElbowTargetAngle());
        telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.CM));
        //if(distanceSensor.getDistance(DistanceUnit.CM) < 2){
       //     spinner.setPower(1);
       // }
        //else spinner.setPower(0);




        while (gamepad2.right_stick_button || gamepad2.left_stick_button) {
            arm.emergencyStop();
        }

        if (arm.elbow.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            arm.elbow.setTargetPosition(arm.elbow.getTargetPosition());
        }
        if (arm.shoulder.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            arm.shoulder.setTargetPosition(arm.shoulder.getTargetPosition());
        }

    }

    public void handleSpinner() {
//        double spin = distanceSensor.getDistance(DistanceUnit.CM) / 5 - 0.2;
//        spinner.setPower(spin);
//        telemetry.addData("distance", spin * 5)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            if (gamepad1.right_bumper) {
                spinner.setPower(1);
            }
            if (gamepad1.left_bumper) {
                spinner.setPower(-1);
            }
        } else {
            spinner.setPower(0);
        }
    }

    public void handleDriving() {
        handleDriveControls();
        handleDriveMotors();
    }

    public void handleDriveControls() {
        // Get the game pad control values for this loop iteration
        drive = -gamepad1.left_stick_y - gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = 0;
        if (gamepad1.dpad_left)
            strafe = -1;
        else if (gamepad1.dpad_right)
            strafe = 1;
        else {
            strafe = gamepad1.left_stick_x;
        }
        speedFactor = 1.0 - (gamepad1.left_trigger * .5);
    }

    public void handleDriveMotors() {
        leftPowerFront = (drive + turn + strafe) * speedFactor;
        rightPowerFront = (drive - turn - strafe) * speedFactor;
        leftPowerBack = (drive + turn - strafe) * speedFactor;
        rightPowerBack = (drive - turn +strafe) * speedFactor;

        leftDriveFront.setPower(leftPowerFront);
        rightDriveFront.setPower(rightPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveBack.setPower(rightPowerBack);

        telemetry.addData("Motors", "lf(%.2f), rf(%.2f), lb(%.2f), rb(%.2f)", leftPowerFront, rightPowerFront, leftPowerBack, rightPowerBack);
        telemetry.addData("Speed control", speedFactor);

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        this.handleArm();
        this.handleIntake();
        this.handleSpinner();
        this.handleDriving();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

}
