package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.HardwareMapMech.DEFAULTFOURBARPOS;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.FOURBARUSIDE;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.intakeLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.intakeRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.powerVariable;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorRight;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMapMech;
import org.firstinspires.ftc.teamcode.RRdrive.DriveConstants;
import org.firstinspires.ftc.teamcode.RRdrive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRtrajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoMainOdometry extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

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

    private double GROUNDJUNCTION = 0;
    private double LOWJUNCTION = 1211;
    private double MIDDLEJUNCTION = 2085;
    private double HIGHJUNCTION = 2893;

    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double maxSlideVelocity = (wheelMotorRPM * wheelMotorTicks / 60) * 1.5;

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



    public void intakeIn(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void intakeOut(double power) {
        intakeLeft.setPower(-power);
        intakeRight.setPower(-power);
    }

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);




        //HARDWARE MAPPING HERE etc.

        HardwareMapMech robot = new HardwareMapMech();

        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        //imu hardware map

//         * The INIT-loop:
//         * This REPLACES waitForStart!


        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setVelocity(maxSlideVelocity);
                    slideMotorLeft.setVelocity(maxSlideVelocity);

                    slideMotorLeft.setTargetPosition(420);
                    slideMotorRight.setTargetPosition(420);


                })
                .splineTo(new Vector2d(-36.05, -37.16), Math.toRadians(98.75))
                .splineTo(new Vector2d(-36, -10.15), Math.toRadians(77.47))
                .splineToConstantHeading(new Vector2d(-36, -8), Math.toRadians(14.68),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL/2,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .splineToConstantHeading(new Vector2d(-32, -20), Math.toRadians(-33.94),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL/2,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))

                /**Adjust Below */
                .lineToSplineHeading(new Pose2d(-27, -17, Math.toRadians(58)))

                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.HIGHJUNCTIONU();

                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    intakeOut(1);
                    slideMotorLeft.setTargetPosition((int) HIGHJUNCTION - 300);
                    slideMotorRight.setTargetPosition((int) HIGHJUNCTION - 300);

                })
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    intakeOut(1);
                    robot.HIGHJUNCTIONU();})
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                    slideMotorLeft.setTargetPosition(700);
                    slideMotorRight.setTargetPosition(700);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);

                    intakeOut(0);})
                .lineToSplineHeading(new Pose2d(-36.05, -28, Math.toRadians(120)))
                .lineToSplineHeading(new Pose2d(-67.34, -18, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/1.3))
                .UNSTABLE_addTemporalMarkerOffset(-.2, () -> {

                    slideMotorLeft.setTargetPosition(403);
                    slideMotorRight.setTargetPosition(403);

                    intakeIn(.8);})
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {

                    slideMotorLeft.setTargetPosition(900);
                    slideMotorRight.setTargetPosition(900);

                    intakeIn(.1);})
                /** **/
                .lineToSplineHeading(new Pose2d(-36.05, -20, Math.toRadians(60)))
                .lineToSplineHeading(new Pose2d(-30, -11, Math.toRadians(50)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.HIGHJUNCTIONU();

                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    intakeOut(1);
                    slideMotorLeft.setTargetPosition((int) HIGHJUNCTION - 300);
                    slideMotorRight.setTargetPosition((int) HIGHJUNCTION - 300);

                })
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    intakeOut(1);
                    robot.HIGHJUNCTIONU();})
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                    slideMotorLeft.setTargetPosition(700);
                    slideMotorRight.setTargetPosition(700);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);

                    intakeOut(0);})
                .lineToSplineHeading(new Pose2d(-36.05, -22, Math.toRadians(120)))



                .build();

//        TrajectorySequence test2 = drive.trajectorySequenceBuilder(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    intakeIn(.1);
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(600);
//                })
//                .splineTo(new Vector2d(-36.05, -37.16), Math.toRadians(98.75))
//                .splineTo(new Vector2d(-36, -10.15), Math.toRadians(77.47))
//                .splineToConstantHeading(new Vector2d(-36, -8), Math.toRadians(14.68))
//                .splineToConstantHeading(new Vector2d(-32, -20), Math.toRadians(-33.94))
//                .lineToSplineHeading(new Pose2d(-27, -16, Math.toRadians(65)))
//
//                .waitSeconds(.5)
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(1);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION - 300);})
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    intakeOut(1);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(600);
//                    intakeOut(0);})
//                .lineToSplineHeading(new Pose2d(-36.05, -12.20, Math.toRadians(180.00)))
//                .splineToConstantHeading(new Vector2d(-49.83, -13.32), Math.toRadians(168.31),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL/2,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//                .splineToConstantHeading(new Vector2d(-69, -12.02), Math.toRadians(179.66),
//                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL/2,DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//
//
//                .UNSTABLE_addTemporalMarkerOffset(-.7, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(350);
//                    intakeIn(.8);})
//                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(900);
//                    intakeIn(.1);})
//
//                .lineToSplineHeading(new Pose2d(-39.40, -13.69, Math.toRadians(65.00)))
//                .splineToConstantHeading(new Vector2d(-27.10, -6.99), Math.toRadians(25.59))
//
//                .waitSeconds(.5)
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
//                    intakeOut(1);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION - 300);})
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    intakeOut(1);
//                    slideMotor.setTargetPosition((int) HIGHJUNCTION);})
//                .UNSTABLE_addTemporalMarkerOffset(.7, () -> {
//                    slideMotor.setVelocity(maxSlideVelocity);
//                    slideMotor.setTargetPosition(600);
//                    intakeOut(0);})
//                .lineToSplineHeading(new Pose2d(-36.05, -12.20, Math.toRadians(180.00)))
//
//
//                .build();
//






        while (!isStarted() && !isStopRequested())
        {
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

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
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

                if(tagFound)
                {
                    telemetry.addLine("TARGET ACQUIRED\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("NO TARGET :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The TARGET has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the TARGET before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
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

        slideMotorLeft.setVelocity(maxSlideVelocity);
        slideMotorRight.setVelocity(maxSlideVelocity);
        fourBarLeft.setVelocity(maxSlideVelocity * powerVariable);
        fourBarRight.setVelocity(maxSlideVelocity * powerVariable);






        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        //TODO Cycle 2 High and Park

        //REDLEFT
        if((autoStart == 0) && tagOfInterest.id == LEFT && autoMode == 0){
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)));
//            drive.followTrajectorySequence(test);
            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-62, -9, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-65,-20))
                    .build();

            TrajectorySequence basicParkLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(30)
                    .strafeLeft(30)
                    .build();

            drive.followTrajectorySequence(basicParkLeft);

//            drive.followTrajectorySequence(parkLeft);
        } else if ((autoStart == 0) && tagOfInterest.id == MIDDLE && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)));
//            drive.followTrajectorySequence(test);

            /*TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-40,-5))

//                    .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(90)))
                    .build();


            drive.followTrajectorySequence(parkMiddle);*/

            TrajectorySequence basicParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(30)
                    .build();
            drive.followTrajectorySequence(basicParkMiddle);

        } else if ((autoStart == 0) && tagOfInterest.id == RIGHT && autoMode == 0) {
            sleep(initialDelay * 1000);
           // drive.setPoseEstimate(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)));
            //drive.followTrajectorySequence(test);

//            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
//                    .lineToConstantHeading(new Vector2d(-15 ,-20.))
//
////                    .lineToLinearHeading(new Pose2d(-5, -8, Math.toRadians(90)))
//                    .build();

//            drive.followTrajectorySequence(parkRight);
            TrajectorySequence basicParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(30)
                    .strafeRight(30)
                    .build();
            drive.followTrajectorySequence(basicParkMiddle);
        }
        //REDRIGHT
        else if ((autoStart == 1) && tagOfInterest.id == LEFT && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);

            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
                    .lineToLinearHeading(new Pose2d(5, -8, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(parkLeft);

        } else if ((autoStart == 1) && tagOfInterest.id == MIDDLE && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);

            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
                    .lineToLinearHeading(new Pose2d(30, -8, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(parkMiddle);

        } else if ((autoStart == 1) && tagOfInterest.id == RIGHT && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(Right2HighCycle);

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
                    .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(parkRight);
        }

        //TODO 2 Low, 1 Mid, Cycle High
        //REDLEFT
        if((autoStart == 0) && tagOfInterest.id == LEFT && autoMode == 1){
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);

            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-67, -8, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        slideMotorLeft.setVelocity(maxSlideVelocity);
                        slideMotorRight.setVelocity(maxSlideVelocity);

                        slideMotorLeft.setTargetPosition(0);
                        slideMotorRight.setTargetPosition(0);})
                    .build();
//            drive.followTrajectorySequence(parkLeft);
        } else if ((autoStart == 0) && tagOfInterest.id == MIDDLE && autoMode == 1)  {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);

            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-45, -8, Math.toRadians(90)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        slideMotorLeft.setVelocity(maxSlideVelocity);
                        slideMotorRight.setVelocity(maxSlideVelocity);

                        slideMotorLeft.setTargetPosition(0);
                        slideMotorRight.setTargetPosition(0);})
                    .build();
//            drive.followTrajectorySequence(parkMiddle);
        } else if ((autoStart == 0) && tagOfInterest.id == RIGHT && autoMode == 1) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-33, -61, Math.toRadians(90)));
//            drive.followTrajectorySequence(LeftLowMidHigh);

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-14, -8, Math.toRadians(180)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        slideMotorLeft.setVelocity(maxSlideVelocity);
                        slideMotorRight.setVelocity(maxSlideVelocity);

                        slideMotorLeft.setTargetPosition(0);
                        slideMotorRight.setTargetPosition(0);})
                    .build();
//            drive.followTrajectorySequence(parkRight);

        }
        //REDRIGHT
        else if ((autoStart == 1) && tagOfInterest.id == LEFT && autoMode == 1) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))

                    .build();
            //drive.followTrajectorySequence(traj);

        } else if ((autoStart == 1) && tagOfInterest.id == MIDDLE && autoMode == 1) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))


                    .build();
            //drive.followTrajectorySequence(traj);

        } else if ((autoStart == 1) && tagOfInterest.id == RIGHT && autoMode == 1) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33, -61, Math.toRadians(90)));
            TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(33, -61, Math.toRadians(90)))

                    .build();
            //drive.followTrajectorySequence(traj);
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
