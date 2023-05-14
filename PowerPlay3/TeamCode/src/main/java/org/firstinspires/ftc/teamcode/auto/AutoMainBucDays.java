package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.HardwareMapMech.DEFAULTFOURBARPOS;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.DEFAULTSLIDEPOS;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.FOURBARUSIDE;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.STACK;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMapMech;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Disabled
@Autonomous
public class AutoMainBucDays extends LinearOpMode
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

//        fourBarLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        fourBarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        //imu hardware map

//         * The INIT-loop:
//         * This REPLACES waitForStart!


        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(-33.26, -62.87, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setVelocity(maxSlideVelocity);
                    slideMotorLeft.setVelocity(maxSlideVelocity);

                    slideMotorLeft.setTargetPosition(DEFAULTSLIDEPOS);
                    slideMotorRight.setTargetPosition(DEFAULTSLIDEPOS);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);


                })

                .splineToConstantHeading(new Vector2d(-37.32, -9.13), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(-29.38, -4.85, Math.toRadians(46.33)))


                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    intakeIn(.1);
                    robot.HIGHJUNCTIONU();

                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(-37.32, -9.13), Math.toRadians(90.00))

                .lineToSplineHeading(new Pose2d(-42, -9.59, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-62.63, -9.59), Math.toRadians(180))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.5);
                    slideMotorLeft.setTargetPosition(STACK-400);
                    slideMotorRight.setTargetPosition(STACK-400);

                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setTargetPosition(STACK+100);
                    slideMotorRight.setTargetPosition(STACK+100);

                })
                .waitSeconds(.5)

                //deliver mid 1
                .lineToSplineHeading(new Pose2d(-33, -24, Math.toRadians(160)))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    intakeIn(.1);
                    robot.MIDDLEJUNCTIONOUT();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(-42, -11, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-64.63, -11), Math.toRadians(180))



                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.5);
                    slideMotorLeft.setTargetPosition(STACK-550);
                    slideMotorRight.setTargetPosition(STACK-550);

                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);

                })
                .waitSeconds(.5)

                //deliver mid 2


                .lineToSplineHeading(new Pose2d(-33, -24, Math.toRadians(160)))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    intakeIn(.1);
                    robot.MIDDLEJUNCTIONOUT();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(0);
                    slideMotorRight.setTargetPosition(0);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)


                //park
                .lineToSplineHeading(new Pose2d(-38, -11, Math.toRadians(90)))





                .build();

        TrajectorySequence test2 = drive.trajectorySequenceBuilder(new Pose2d(33.26, -62.87, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setVelocity(maxSlideVelocity);
                    slideMotorLeft.setVelocity(maxSlideVelocity);

                    slideMotorLeft.setTargetPosition(DEFAULTSLIDEPOS);
                    slideMotorRight.setTargetPosition(DEFAULTSLIDEPOS);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);


                })

                .splineToConstantHeading(new Vector2d(37.32, -9.13), Math.toRadians(-90.00))
                .lineToSplineHeading(new Pose2d(29.38, -4.85, Math.toRadians(46.33 + 90)))


                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    intakeIn(.1);
                    robot.HIGHJUNCTIONU();

                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(37.32, -9.13), Math.toRadians(-90.00))

                .lineToSplineHeading(new Pose2d(42, -9.59, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(62.63, -9.59), Math.toRadians(-180))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.5);
                    slideMotorLeft.setTargetPosition(STACK-400);
                    slideMotorRight.setTargetPosition(STACK-400);

                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setTargetPosition(STACK+100);
                    slideMotorRight.setTargetPosition(STACK+100);

                })
                .waitSeconds(.5)

                //deliver mid 1
                .lineToSplineHeading(new Pose2d(33, -24, Math.toRadians(160-90)))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    intakeIn(.1);
                    robot.MIDDLEJUNCTIONOUT();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(42, -11, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(64.63, -11), Math.toRadians(-180))



                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeIn(.5);
                    slideMotorLeft.setTargetPosition(STACK-550);
                    slideMotorRight.setTargetPosition(STACK-550);

                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeIn(.1);
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);

                })
                .waitSeconds(.5)

                //deliver mid 2


                .lineToSplineHeading(new Pose2d(33, -24, Math.toRadians(160-90)))

                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    intakeIn(.1);
                    robot.MIDDLEJUNCTIONOUT();
                })
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                    intakeOut(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intakeOut(0);
                    slideMotorLeft.setTargetPosition(0);
                    slideMotorRight.setTargetPosition(0);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                })
                .waitSeconds(.5)


                //park
                .lineToSplineHeading(new Pose2d(38, -11, Math.toRadians(90)))





                .build();



        while (!isStarted() && !isStopRequested())
        {
            //create variable to store visual information
            String[] display = {"Left", "Right", "CycleAuto", "Park"};

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
             drive.setPoseEstimate(new Pose2d(-33.26, -62.87, Math.toRadians(90.00)));

            drive.followTrajectorySequence(test);
            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-62, -9, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-70,-15))
                    .build();

//            TrajectorySequence basicParkLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
//                    .forward(30)
//                    .strafeLeft(30)
//                    .build();
//
//            drive.followTrajectorySequence(basicParkLeft);

            drive.followTrajectorySequence(parkLeft);
        } else if ((autoStart == 0) && tagOfInterest.id == MIDDLE && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-33.26, -62.87, Math.toRadians(90.00)));

            drive.followTrajectorySequence(test);

            /*TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-40, -5, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-40,-5))

//                    .lineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(90)))
                    .build();


            drive.followTrajectorySequence(parkMiddle);*/

            TrajectorySequence basicParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(30)
                    .build();
//            drive.followTrajectorySequence(basicParkMiddle);

        } else if ((autoStart == 0) && tagOfInterest.id == RIGHT && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(-33.26, -62.87, Math.toRadians(90.00)));


            // drive.setPoseEstimate(new Pose2d(-31.58, -66.60, Math.toRadians(90.00)));
            drive.followTrajectorySequence(test);

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-10 ,-15.))

//                    .lineToLinearHeading(new Pose2d(-5, -8, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(parkRight);
            TrajectorySequence basicParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(5)
                    .strafeRight(30)
                    .build();
//            drive.followTrajectorySequence(basicParkMiddle);
        }
        //REDRIGHT
        else if ((autoStart == 1) && tagOfInterest.id == LEFT && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33.26, -62.87, Math.toRadians(90.00)));

//            drive.followTrajectorySequence(Right2HighCycle);
            drive.followTrajectorySequence(test2);


            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
//                    .lineToLinearHeading(new Pose2d(-62, -9, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-70,-15))
                    .build();
            drive.followTrajectorySequence(parkLeft);

        } else if ((autoStart == 1) && tagOfInterest.id == MIDDLE && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33.26, -62.87, Math.toRadians(90.00)));

            drive.followTrajectorySequence(test2);

//            drive.followTrajectorySequence(Right2HighCycle);

            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(new Pose2d(34, -12, Math.toRadians(140)))
                    .lineToLinearHeading(new Pose2d(30, -8, Math.toRadians(90)))
                    .build();

//            drive.followTrajectorySequence(parkMiddle);

        } else if ((autoStart == 1) && tagOfInterest.id == RIGHT && autoMode == 0) {
            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(33.26, -62.87, Math.toRadians(90.00)));
            drive.followTrajectorySequence(test2);


//            drive.followTrajectorySequence(Right2HighCycle);

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(new Pose2d(-40, -20, Math.toRadians(90)))
                    .lineToConstantHeading(new Vector2d(-10 ,-15.))
//                    .lineToLinearHeading(new Pose2d(-5, -8, Math.toRadians(90)))
                    .build();

            drive.followTrajectorySequence(parkRight);
        }

        //TODO Park
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

            TrajectorySequence basicParkLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(5)
                    .waitSeconds(.5)
                    .strafeLeft(5)
                    .build();

            drive.followTrajectorySequence(basicParkLeft);
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


            TrajectorySequence basicParkMid = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(5)

                    .build();

            drive.followTrajectorySequence(basicParkMid);
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

            TrajectorySequence basicParkRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .forward(5)
                    .waitSeconds(.5)
                    .strafeRight(5)
                    .build();

            drive.followTrajectorySequence(basicParkRight);

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
