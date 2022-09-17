package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.source.Arm;

@Disabled
@Autonomous(name = "Warehouse_v1", group = "Warehouse")
public class Warehouse_v1 extends LinearOpMode {

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "ATCNswP/////AAABmboo62E3M0RLvUoBrala8GQowW4hvn2lz0v4xIUqDcerBojdZbFDT7KxueF7R6JgJY9tQ+gV9sHXv6aOcnznTsupwlzsqujeV1pIN0j5/uZNqLkxZCORToVMVD/kd8XY5y58Pnml+lS3pqkZee6pSUTNWfmWgJAu/oKPGVrOm5GwCPObOM9Mx3NSbWeRVSiKcaN9o6QyqV+Knuf2xYpF87rKiH0pbWGRIFSy8JgVQ6dabuIoDCKbXpDeTwK3PJ2VtgON+8PA2TIIn95Yq8UmBYJRJc6kDyvCDyCnKJ63oPRfzth3P8DM4IchQd69ccU6vqeto4JNQbPZh5JB5KRXFS8CcmQJLkSRcHDIP92eIhv/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //defining program states
    enum mainState {
        TRAJECTORY_1,   // Head to the duck carousel
        DROP,           // Drop block
        TRAJECTORY_2,
        TRAJECTORY_3,   // Go to the shipping hub
        WAIT_1,
        TRAJECTORY_4,   // Drive forward slowly a few inches to the hub
        TRAJECTORY_5,   // Park in the storage unit
        IDLE            // Enter IDLE state when complete
    }
    enum armState {
        TIER_1,
        TIER_2,
        TIER_3,
        PICKUP_1,
        IDLE
    }

    //defining the main state
    mainState currentState = mainState.IDLE;

    //defining the arm state
    armState currentArmState = armState.IDLE;

    //defining start pose
    Pose2d startPose = new Pose2d(-12, -64, Math.toRadians(180));

    //initializing timer
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        int Duck = 0;
        timer.reset();
        while(Duck == 0 && timer.seconds() < 0) {
            if (tfod != null) {
                List<Recognition> recognitions = tfod.getRecognitions();
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    if(recognition.getLabel() == "Duck") {
                        Duck = DuckSpot(recognition.getRight()-((recognition.getRight()-recognition.getLeft())/2));
                    }
                    i++;
                }
            }
        }
        if(Duck == 0) { //If it took too long to find the duck then it defaults to spot 3
            Duck = 3;
        }
        telemetry.addData("Duck Spot", Duck);
        telemetry.update();

        //Initializing our other robot hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap); //Shoulder starts at 150 degrees, elbow at 20 degrees relative to the shoulder
        arm.runShoulderTo(60);
        arm.runElbowTo(90);

        //Setting initial position estimate
        drive.setPoseEstimate(startPose);

        //Defining trajectories.  Yes, I need to comment but it's 9:45 and I'm going to bed.
        Trajectory trajectory1_1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, -35), Math.toRadians(-135))
                .build();
        Trajectory trajectory1_2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, -35), Math.toRadians(-135))
                .build();
        Trajectory trajectory1_3 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, -50), Math.toRadians(45))
                .build();
        Trajectory trajectory2_1 = drive.trajectoryBuilder(trajectory1_1.end())
                .splineTo(new Vector2d(-12, -64), Math.toRadians(180))
                .build();
        Trajectory trajectory2_2 = drive.trajectoryBuilder(trajectory1_2.end())
                .splineTo(new Vector2d(-12, -64), Math.toRadians(180))
                .build();
        Trajectory trajectory2_3 = drive.trajectoryBuilder(trajectory1_3.end())
                .splineTo(new Vector2d(-12, -64), Math.toRadians(180))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2_1.end())
                .forward(20)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .back(20)
                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4.end())
                .splineTo(new Vector2d(-12, -64), Math.toRadians(45))
                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .splineTo(new Vector2d(-12, -35), Math.toRadians(-135))
                .build();
        waitForStart();

        if (isStopRequested()) return;

        currentState = mainState.TRAJECTORY_1;
        switch(Duck) {
            case 1:
                drive.followTrajectoryAsync(trajectory1_1);
                currentArmState = armState.TIER_1;
            case 2:
                drive.followTrajectoryAsync(trajectory1_2);
                currentArmState = armState.TIER_2;
            case 3:
                drive.followTrajectoryAsync(trajectory1_3);
                currentArmState = armState.TIER_3;
        }
        while (opModeIsActive() && !isStopRequested()) {
            //You can have multiple switch statements running together for multiple state machines
            //in parallel. This is the basic idea for subsystems and commands.

            //Essentially defining the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1: //driving to the shipping hub
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        timer.reset();
                        currentState = mainState.DROP;
                    }
                    break;
                case DROP:
                    //arm.runClaw();
                    //arm.retractMagnet();
                    if(timer.milliseconds() > 2000) {
                        switch(Duck) {
                            case 1:
                                drive.followTrajectoryAsync(trajectory2_1);
                            case 2:
                                drive.followTrajectoryAsync(trajectory2_2);
                            case 3:
                                drive.followTrajectoryAsync(trajectory2_3);
                        }
                    }
                case TRAJECTORY_2: //going back against wall
                    if (!drive.isBusy()) {
                        currentState = mainState.TRAJECTORY_3;
                        currentArmState = armState.PICKUP_1;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3: //going to pick up freight from the warehouse
                    if (!drive.isBusy() && !arm.shoulderIsBusy && !arm.elbowIsBusy) {
                        timer.reset();
                        currentState = mainState.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case WAIT_1:
                    if(timer.milliseconds() > 250) {
                        currentState = mainState.TRAJECTORY_4;
                        currentArmState = armState.TIER_3;
                    }
                case TRAJECTORY_4: //positioning to go to shipping hub
                    if (!drive.isBusy()) {
                        currentState = mainState.TRAJECTORY_5;
                    }
                    break;
                case TRAJECTORY_5: //placing freight on top tier
                    if (!drive.isBusy()) {
                        currentState = mainState.IDLE;
                    }
                    break;
                case IDLE:
                    arm.storeArmPose();
                    break;
            }

            switch(currentArmState) {
                case TIER_1:
                    arm.runShoulderTo(153);
                    arm.runElbowTo(270);
                    currentArmState = Warehouse_v1.armState.IDLE;
                    break;
                case TIER_2:
                    arm.runShoulderTo(120);
                    arm.runElbowTo(285);
                    currentArmState = Warehouse_v1.armState.IDLE;
                    break;
                case TIER_3:
                    arm.runShoulderTo(50);
                    arm.runElbowTo(165);
                    currentArmState = Warehouse_v1.armState.IDLE;
                    break;
                case PICKUP_1:
                    arm.runShoulderTo(-10);
                    arm.runElbowTo(190);
                    break;
                case IDLE:
                    break;
            }

            //Updating the arm and elbow continuously in the background, regardless of state
            arm.updateShoulder();
            arm.updateElbow();

            //Updating drive continuously in the background, regardless of state
            drive.update();

            //Printing data to telemetry
            telemetry.addData("current state", currentState);
            telemetry.update();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam2);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    private int DuckSpot(double DuckX) {
        if(DuckX < 233) {
            return 1;
        } else if(DuckX < 467) {
            return 2;
        } else {
            return 3;
        }
    }
}