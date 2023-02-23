package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RRdrive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRtrajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(group = "advanced")
public class AutoFSMTest extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    private int autoStart = 0;
    private int autoMode = 0;
    private int initialDelay = 0;
    private int barcodeValue = 1;
    boolean buttonPressed = false;

    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double maxSlideVelocity = (wheelMotorRPM * wheelMotorTicks / 60) * 1.5;

    private double GROUNDJUNCTION = 0;
    private double LOWJUNCTION = 1749;
    private double MIDDLEJUNCTION = 2893;
    private double HIGHJUNCTION = 3969;

    private ElapsedTime trajectoryTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();

    private DcMotorEx slideMotor;

    private CRServo intakeLeft;
    private CRServo intakeRight;

    private ColorSensor color;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family


    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take

    /** State Machine for Pathing*/
    enum TrajectoryState {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TRAJECTORY_3, // Then, we follow another lineTo() trajectory
        TRAJECTORY_4,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_5,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_6,

        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    TrajectoryState currentTState = TrajectoryState.IDLE;

    /** State Machine for Slide*/
    enum SlideState {
        IDLE,
        DOWN,
        LOW,
        MIDDLE,
        HIGH,
    }

    SlideState currentSState = SlideState.IDLE;

    /**State Machine for Intake*/

    enum IntakeState {
        HIGHDROPFIRST,
        HIGHDROPCYCLE,
        INTAKE_DEFAULT,
        INTAKE_OUT,
        STACK5,
        STACK4,
        STACK3,
        STACK2,
        STACK1,
        IDLE,
    }

    IntakeState currentIState = IntakeState.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-33, -61, Math.toRadians(90));

    public void intakeIn(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void intakeOut(double power) {
        intakeLeft.setPower(-power);
        intakeRight.setPower(-power);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //HARDWARE MAPPING HERE etc.

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setVelocity(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);


        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(CRServo.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");


        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-15, -54))
                .lineToSplineHeading(new Pose2d(-10, -24, Math.toRadians(45)))
                .lineToSplineHeading(new Pose2d(-20, -5, Math.toRadians(180)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-64, -2))


                .build();


        // Third trajectory
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(54)))
//                .lineToConstantHeading(new Vector2d(-40, -8))
                .lineToSplineHeading(new Pose2d(-66, 0, Math.toRadians(180)))



                .build();

        TrajectorySequence trajectoryCycleHigh = drive.trajectorySequenceBuilder(new Pose2d(-66, 0, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(54)))

//                .lineToConstantHeading(new Vector2d(-40, -8))
                .lineToSplineHeading(new Pose2d(-68, 0, Math.toRadians(180)))


                .build();



        while (!isStarted() && !isStopRequested()) {
            //create variable to store visual information
            String[] display = {"Left", "Right", "2High", "2LowMidHighCycle"};

            //for loop to allow selection of autonomous situation


            //change starting line
            if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
                autoStart--;
                buttonPressed = true;
            } else if (gamepad2.dpad_right && autoStart < 1 && !buttonPressed) {
                autoStart++;
                buttonPressed = true;
                //change automode
            }

            //change initial delay
            else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
                initialDelay--;
                buttonPressed = true;
            } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
                initialDelay++;
                buttonPressed = true;
            }
            //choose which autonomous mode
            else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }
            //choose barcode position
            else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
                barcodeValue--;
                buttonPressed = true;
            } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
                barcodeValue++;
                buttonPressed = true;
            }

            //wait until buttons are not pressed
            else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
                    !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.square && !gamepad2.circle)
                buttonPressed = false;


            //output telemetry


            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        if (tag.id == LEFT)
                            barcodeValue = 1;
                        if (tag.id == MIDDLE)
                            barcodeValue = 2;
                        if (tag.id == RIGHT)
                            barcodeValue = 3;

                        tagOfInterest = tag;
                        tagFound = true;
                        break;


                    }
                }

                if (tagFound) {
                    telemetry.addLine("TARGET ACQUIRED\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("NO TARGET :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The TARGET has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the TARGET before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
            telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 2]);
            telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);
            telemetry.addData("BarcodeValue", barcodeValue);

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        waitForStart();


        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentTState = TrajectoryState.TRAJECTORY_1;
        drive.followTrajectorySequenceAsync(trajectory1);
        trajectoryTime.reset();
        slideMotor.setVelocity(maxSlideVelocity);


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentTState) {
                case TRAJECTORY_1:
                    if (trajectoryTime.seconds() > .5) {
                        currentIState = IntakeState.HIGHDROPFIRST;
                    }

                    if (trajectoryTime.seconds() > 5) {
                        currentIState = IntakeState.INTAKE_DEFAULT;
                    }

                    if (!drive.isBusy()) {
                        currentTState = TrajectoryState.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                        trajectoryTime.reset();
                    }
                    break;

                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && trajectoryTime.seconds() < 4) {
                        currentIState = IntakeState.STACK5;
                        autoIntakeTime.reset();
                    }
                    if (!drive.isBusy()) {
                        currentTState = TrajectoryState.TRAJECTORY_3;
                        drive.followTrajectorySequenceAsync(trajectory3);
                        trajectoryTime.reset();
                    }
                    break;

                case TRAJECTORY_3:

                    if (trajectoryTime.seconds() > 0 ) {
                        currentIState = IntakeState.HIGHDROPCYCLE;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 1.75 ) {
                        currentIState = IntakeState.INTAKE_OUT;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.INTAKE_DEFAULT;
                    }

                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.STACK4;
                        autoIntakeTime.reset();
                    }

                    if (!drive.isBusy()) {

                        currentTState = TrajectoryState.TRAJECTORY_4;
                        drive.followTrajectorySequenceAsync(trajectoryCycleHigh);
                        trajectoryTime.reset();
                    }
                    break;

                case TRAJECTORY_4:
                    if (trajectoryTime.seconds() > 0 ) {
                        currentIState = IntakeState.HIGHDROPCYCLE;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 1.75 ) {
                        currentIState = IntakeState.INTAKE_OUT;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.INTAKE_DEFAULT;
                    }

                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.STACK3;
                        autoIntakeTime.reset();
                    }

                    if (!drive.isBusy()) {

                        currentTState = TrajectoryState.TRAJECTORY_5;
                        drive.followTrajectorySequenceAsync(trajectoryCycleHigh);
                        trajectoryTime.reset();
                    }


                    break;

                case TRAJECTORY_5:
                    if (trajectoryTime.seconds() > 0 ) {
                        currentIState = IntakeState.HIGHDROPCYCLE;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 1.75 ) {
                        currentIState = IntakeState.INTAKE_OUT;
                        autoIntakeTime.reset();
                    }

                    if (trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.INTAKE_DEFAULT;
                    }

                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && trajectoryTime.seconds() > 3.5) {
                        currentIState = IntakeState.STACK2;
                        autoIntakeTime.reset();
                    }

                    if (!drive.isBusy()) {

                        currentTState = TrajectoryState.TRAJECTORY_6;
                        drive.followTrajectorySequenceAsync(trajectoryCycleHigh);
                        trajectoryTime.reset();
                    }
                    break;

                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            switch (currentIState) {
                case HIGHDROPFIRST:

                    slideMotor.setTargetPosition((int) HIGHJUNCTION);

                    if (color.alpha() > 60 && !slideMotor.isBusy()) {
                        intakeOut(1);
                        currentIState = IntakeState.IDLE;
                    }

                    break;

                case HIGHDROPCYCLE:
                    slideMotor.setTargetPosition((int) HIGHJUNCTION);

//                    if (color.alpha() < 75 && !slideMotor.isBusy()) {
//                        intakeOut(1);
//                        currentIState = IntakeState.IDLE;
//                    }

                    break;

                case INTAKE_DEFAULT:
                    intakeOut(0);
                    slideMotor.setTargetPosition(880);


                    break;

                case INTAKE_OUT:
                    intakeOut(1);
                    break;

                case STACK5:

                    slideMotor.setTargetPosition(640);
                    intakeIn(.5);
                    if (autoIntakeTime.seconds() > .1) {
                        slideMotor.setTargetPosition(1100);
                        currentIState = IntakeState.IDLE;
                    }
                        break;

                case STACK4:
                    slideMotor.setTargetPosition(530);
                    intakeIn(.5);
                    if (autoIntakeTime.seconds() > .3) {
                        slideMotor.setTargetPosition(1100);
                        currentIState = IntakeState.IDLE;
                    }
                    break;

                case STACK3:
                    slideMotor.setTargetPosition(400);
                    intakeIn(.5);
                    if (autoIntakeTime.seconds() > .3) {
                        slideMotor.setTargetPosition(1100);
                        currentIState = IntakeState.IDLE;
                    }
                    break;

                case STACK2:
                    slideMotor.setTargetPosition(280);
                    intakeIn(.5);
                    if (autoIntakeTime.seconds() > .3) {
                        slideMotor.setTargetPosition(1100);
                        currentIState = IntakeState.IDLE;
                    }
                    break;

                case IDLE:
                    //Idle
                    break;

            }



            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state


            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("IntakeState", currentIState);
            telemetry.addData("TrajectoryState", currentTState);
            telemetry.addLine()
                    .addData("Red", color.red())
                    .addData("Green", color.green())
                    .addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));


            telemetry.update();
        }




    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}



