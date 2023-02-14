package org.firstinspires.ftc.teamcode.Archive;//package org.firstinspires.ftc.teamcode.Archive;
//
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auto.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import java.util.ArrayList;
//
//@Autonomous
//public class AutoComp3 extends LinearOpMode
//{
//    //INTRODUCE VARIABLES HERE
//
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//
//    private int autoStart = 0;
//    private int autoMode = 0;
//    private int initialDelay = 0;
//    private int barcodeValue = 1;
//    boolean buttonPressed = false;
//
//    private double GROUNDJUNCTION = 0;
//    private double LOWJUNCTION = 1749;
//    private double MIDDLEJUNCTION = 2893;
//    private double HIGHJUNCTION = 3969;
//
//    private final double wheelMotorTicks = 384.5;
//    private final double wheelMotorRPM = 435;
//    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
//
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    // Tag ID 1,2,3 from the 36h11 family
//
//
//
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//
//    AprilTagDetection tagOfInterest = null;
//
//    private DcMotorEx slideMotor;
//
//    private CRServo intakeLeft;
//    private CRServo intakeRight;
//
//    public void intakeIn(double power) {
//        intakeLeft.setPower(power);
//        intakeRight.setPower(power);
//    }
//
//    public void intakeOut(double power) {
//        intakeLeft.setPower(-power);
//        intakeRight.setPower(-power);
//    }
//
//    @Override
//    public void runOpMode()
//    {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//
//
//
//        //HARDWARE MAPPING HERE etc.
//
//        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setTargetPosition(0);
//        slideMotor.setVelocity(0);
//        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
//
//
//        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
//        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
//        intakeRight.setDirection(CRServo.Direction.REVERSE);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        //initialize imu parameters
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.loggingEnabled = false;
//        //imu hardware map
//
////         * The INIT-loop:
////         * This REPLACES waitForStart!
//
//
//
//        TrajectorySequence Left2HighCycle = drive.trajectorySequenceBuilder(new Pose2d(-33, -61, Math.toRadians(90)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    intakeIn(.1);})
//                .lineToConstantHeading(new Vector2d(-12, -58))
//                .lineToConstantHeading(new Vector2d(-12, -20))
//                .lineToSplineHeading(new Pose2d(-18, -1.25, Math.toRadians(120)))
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(.3);})
//                .lineToSplineHeading(new Pose2d(-34, -6, Math.toRadians(180)))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(942);
//                    intakeOut(0);})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(672);
//                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);
//                    intakeIn(.1);})
//                .lineToConstantHeading(new Vector2d(-64, -3))
//
//                .lineToSplineHeading(new Pose2d(-31, 0, Math.toRadians(60)))
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(.3);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(0);
//                    intakeOut(0);})
//                .lineToConstantHeading(new Vector2d(-34, -9))
//
//                .build();
//
//        TrajectorySequence Right2HighCycle = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    intakeIn(.1);})
//                .lineToConstantHeading(new Vector2d(0, -58))
//                .lineToConstantHeading(new Vector2d(0, -20))
//                .lineToSplineHeading(new Pose2d(18, -6, Math.toRadians(85)))
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(.3);})
//                .lineToSplineHeading(new Pose2d(33, -10, Math.toRadians(0)))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(942);
//                    intakeOut(0);})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(672);
//                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);
//                    intakeIn(.1);})
//                .lineToConstantHeading(new Vector2d(57, -10))
//
//                .lineToSplineHeading(new Pose2d(25, 0, Math.toRadians(140)))
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(.3);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(0);
//                    intakeOut(0);})
//                .lineToConstantHeading(new Vector2d(34, -12))
//
//                .build();
//
//        TrajectorySequence LeftLowMidHigh = drive.trajectorySequenceBuilder(new Pose2d(-33, -61, Math.toRadians(90)))
////*Deliver First Cone to Low
//
//                .lineToSplineHeading(new Pose2d(-39, -43, Math.toRadians(0)))
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) LOWJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    intakeOut(.7);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    intakeOut(0);})
//
////*Line up to Stack
//
//
//                .lineToLinearHeading(new Pose2d(-34, -7, Math.toRadians(180)))
//
////*Intake Stack and Deliver 2nd to Low
//
//                .lineToConstantHeading(new Vector2d(-61.5, -5))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(950);})
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(672);
//                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) LOWJUNCTION);
//                    intakeIn(.1);})
//
//
//                .lineToSplineHeading(new Pose2d(-49, -9, Math.toRadians(270)))
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    intakeOut(.7);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    intakeOut(0);})
//
////*Intake Stack and Deliver 3rd to Mid
//
//                .lineToSplineHeading(new Pose2d(-63, -6 , Math.toRadians(180)))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(950);})
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(550);
//                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) MIDDLEJUNCTION);
//                    intakeIn(.1);})
//
//                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(315)))
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    intakeOut(.7);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    intakeOut(0);})
//
////*Intake Stack and Deliver 4th to High
//
//                .lineToSplineHeading(new Pose2d(-63, -8 , Math.toRadians(180))) //going to stack to intake
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(950);})
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(350);                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);
//                    intakeIn(.1);})
//
//                .lineToSplineHeading(new Pose2d(-34, -3, Math.toRadians(45))) // going to the high junction
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    intakeOut(.7);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    intakeOut(0);})
//
////*Intake Stack and Deliver 5th to High
//
//                .lineToSplineHeading(new Pose2d(-66, -4 , Math.toRadians(180))) //intake
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(400);})
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(0);
//                    intakeIn(.5);})
//                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);
//                    intakeIn(.1);})
//
//                .lineToSplineHeading(new Pose2d(-36, 0, Math.toRadians(45))) //high junction
//                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> {
//                    intakeOut(.7);})
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    intakeOut(0);})
//
//                .lineToSplineHeading(new Pose2d(-40, -5, Math.toRadians(90)))
//                .build();
//        //mkmk
//
//
//
//
//
//
//
//
//
//        while (!isStarted() && !isStopRequested())
//        {
//            //create variable to store visual information
//            String[] display = {"Left", "Right", "2LowMidHighCycle", "2High"};
//
//            //for loop to allow selection of autonomous situation
//
//
//            //change starting line
//            if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
//                autoStart--;
//                buttonPressed = true;
//            } else if (gamepad2.dpad_right && autoStart < 1 && !buttonPressed) {
//                autoStart++;
//                buttonPressed = true;
//                //change automode
//            }
//
//            //change initial delay
//            else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
//                initialDelay--;
//                buttonPressed = true;
//            } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
//                initialDelay++;
//                buttonPressed = true;
//            }
//            //choose which autonomous mode
//            else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
//                autoMode--;
//                buttonPressed = true;
//            } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
//                autoMode++;
//                buttonPressed = true;
//            }
//            //choose barcode position
//            else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
//                barcodeValue--;
//                buttonPressed = true;
//            } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
//                barcodeValue++;
//                buttonPressed = true;
//            }
//
//            //wait until buttons are not pressed
//            else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
//                    !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.square && !gamepad2.circle)
//                buttonPressed = false;
//
//
//
//
//
//            //output telemetry
//
//
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        if (tag.id == LEFT)
//                            barcodeValue = 1;
//                        if (tag.id == MIDDLE)
//                            barcodeValue = 2;
//                        if (tag.id == RIGHT)
//                            barcodeValue = 3;
//
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//
//
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("TARGET ACQUIRED\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("NO TARGET :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The TARGET has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the TARGET before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
//            telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 2]);
//            telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);
//            telemetry.addData("BarcodeValue", barcodeValue);
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        slideMotor.setVelocity(maxSlideVelocity);
//
//
//
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
//        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)
//
//        //TODO Cycle 2 High and Park
//
//        //REDLEFT
//        if((autoStart == 0) && tagOfInterest.id == LEFT && autoMode == 1){
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Left2HighCycle);
//
//            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
////                    .lineToLinearHeading(new Pose2d(-62, -9, Math.toRadians(90)))
//                    .lineToConstantHeading(new Vector2d(-62,-13))
//                    .build();
//
//            drive.followTrajectorySequence(parkLeft);
//        } else if ((autoStart == 0) && tagOfInterest.id == MIDDLE && autoMode == 1) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Left2HighCycle);
//
//            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
//                    .lineToConstantHeading(new Vector2d(-40,-11))
//
////                    .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(90)))
//                    .build();
//
//            drive.followTrajectorySequence(parkMiddle);
//        } else if ((autoStart == 0) && tagOfInterest.id == RIGHT && autoMode == 1) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Left2HighCycle);
//
//            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
//                    .lineToConstantHeading(new Vector2d(-7,-11))
//
////                    .lineToLinearHeading(new Pose2d(-5, -8, Math.toRadians(90)))
//                    .build();
//
//            drive.followTrajectorySequence(parkRight);
//        }
//        //REDRIGHT
//        else if ((autoStart == 1) && tagOfInterest.id == LEFT && autoMode == 1) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);
//
//            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
//                    .lineToLinearHeading(new Pose2d(5, -14, Math.toRadians(90)))
//                    .build();
//
//            drive.followTrajectorySequence(parkLeft);
//
//        } else if ((autoStart == 1) && tagOfInterest.id == MIDDLE && autoMode == 1) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);
//
//            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
//                    .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(90)))
//                    .build();
//
//            drive.followTrajectorySequence(parkMiddle);
//
//        } else if ((autoStart == 1) && tagOfInterest.id == RIGHT && autoMode == 1) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);
//
//            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
//                    .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(90)))
//                    .build();
//
//            drive.followTrajectorySequence(parkRight);
//        }
//
//        //TODO 2 Low, 1 Mid, Cycle High
//        //REDLEFT
//        if((autoStart == 0) && tagOfInterest.id == LEFT && autoMode == 0){
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);
//
//            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-67, -8, Math.toRadians(90)))
//                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                        slideMotor.setVelocity(maxSlideVelocity);
//                        slideMotor.setTargetPosition(0);})
//                    .build();
//            drive.followTrajectorySequence(parkLeft);
//        } else if ((autoStart == 0) && tagOfInterest.id == MIDDLE && autoMode == 0)  {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);
//
//            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-45, -8, Math.toRadians(90)))
//                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                        slideMotor.setVelocity(maxSlideVelocity);
//                        slideMotor.setTargetPosition(0);})
//                    .build();
//            drive.followTrajectorySequence(parkMiddle);
//        } else if ((autoStart == 0) && tagOfInterest.id == RIGHT && autoMode == 0) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);
//
//            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-14, -8, Math.toRadians(180)))
//                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                        slideMotor.setVelocity(maxSlideVelocity);
//                        slideMotor.setTargetPosition(0);})
//                    .build();
//            drive.followTrajectorySequence(parkRight);
//
//        }
//        //REDRIGHT
//        else if ((autoStart == 1) && tagOfInterest.id == LEFT && autoMode == 0) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))
//
//                    .build();
//            //drive.followTrajectorySequence(traj);
//
//        } else if ((autoStart == 1) && tagOfInterest.id == MIDDLE && autoMode == 0) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))
//
//
//                    .build();
//            //drive.followTrajectorySequence(traj);
//
//        } else if ((autoStart == 1) && tagOfInterest.id == RIGHT && autoMode == 0) {
//            sleep(initialDelay * 1000);
//            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))
//
//                    .build();
//            //drive.followTrajectorySequence(traj);
//        }
//
//
//
//
//
//    }
//
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}
