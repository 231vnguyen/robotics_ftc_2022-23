package org.firstinspires.ftc.teamcode.Archive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RRdrive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@Disabled
@TeleOp(group = "advanced")
public class TeleopFieldCentric extends LinearOpMode {


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = .9; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxSlideTicks = 3095;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double spinnyTicks = 537.7;
    private final double maxSpinnyVelocity = 312 * spinnyTicks / 60;


    //gear change variables
    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3


    //inverse drive variables
    private boolean inversePressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, .75, 1.0};

    private int horizontalPosition = 2;
    private final double[] horizontalValues = {
            -240,
            -120,
            0,

    };

    //dropdown servos position values
    private int dropdownPosition = 0;
    private final double[] dropdownValues = {
            0, .1, .2, .3, .4, .5, .6, .7, .85, .9, 1
    };
    boolean dropdownMoving = false;

    private int slidePosition = 10;
    private final double[] slideValues = {
            0, .1, .2, .3, .4, .5, .6, .7, .85, .9, 1
    };
    boolean slideMoving = false;


    private double GROUNDJUNCTION = 0;
    private double LOWJUNCTION = 1211;
    private double MIDDLEJUNCTION = 2085;
    private double HIGHJUNCTION = 2893;
    int slidePositionCurrent = 0;


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
//    private ElapsedTime spinnyTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();




    //create motor and servo objects

    private DcMotorEx slideMotor;

    private CRServo intakeLeft;
    private CRServo intakeRight;
//    private ColorSensor color;






    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        INITIAL_GRAB,
        INITIAL_GRAB_STACK,
        FINAL_GRAB,
        DROP,
        DROP_DEFAULT,
        DROP_STACKED,
        MOVEAWAY,
        HIGH_JUNCTION,
        MIDDLE_JUNCTION,
        LOW_JUNCTION,
        GROUND_JUNCTION,
        STACK_POSITION

    }

    AutoIntakeState autointakeState = AutoIntakeState.DEFAULT_POSITION;


    //toggle booleans

    private boolean slidePlayerControlled = true;
    private boolean slideSwitch = false;
    private boolean autoIntakeActive = false;

    private boolean intakeMoving = false;
    //private boolean stickMoving = false;


    //method to change gears
    private void changeGears() {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

    }

    public void dropdownControl() {
        if (gamepad1.dpad_up && dropdownPosition > 0 && !dropdownMoving) {
            //decrease position
            dropdownPosition--;
            dropdownMoving = true;
        } else if (gamepad1.dpad_down && dropdownPosition < (dropdownValues.length - 1) && !dropdownMoving) {
            //increase position
            dropdownPosition++;
            dropdownMoving = true;
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down && dropdownMoving)
            dropdownMoving = false;

//        flicky.setPosition(dropdownValues[dropdownPosition]);


    }
    
    public void slideControl() {
        if (gamepad2.cross && slidePosition > 0 && !slideMoving) {
            //decrease position
            slidePosition--;
            slideMoving = true;
        } else if (gamepad2.triangle && slidePosition < (slideValues.length - 1) && !slideMoving) {
            //increase position
            slidePosition++;
            slideMoving = true;
        } else if (!gamepad2.cross && !gamepad2.triangle && slideMoving)
            slideMoving = false;

        slideMotor.setTargetPosition((int) (slideValues[slidePosition] * maxSlideTicks));


    }

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
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        //TODO organize
        //setup driving motors


        //armMotor encoders
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setVelocity(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);


        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(CRServo.Direction.REVERSE);

//        color = hardwareMap.colorSensor.get("color");




        waitForStart();


        slideMotor.setVelocity(maxSlideVelocity);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * gearValues[activeGear],
                            input.getY() * gearValues[activeGear],
                            -gamepad1.right_stick_x * gearValues[activeGear] * botRotationSpeed
                    )
            );

            if (gamepad1.options) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            }


            //TODO Gamepad 1


            //update controller variables


            //event methods
            changeGears();


            //-----------------------------------------------------------------------------
            //Inverse drive with start
/*            if (gamepad1.options && !inversePressed)
                inverse *= -1;
            inversePressed = gamepad1.options;*/


            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control




            //TODO Gamepad 2


            switch (autointakeState) {
                case DEFAULT_POSITION:

                    slideMotor.setVelocity(maxSlideVelocity);
                    slideMotor.setTargetPosition(403);

                    if (gamepad2.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HIGH_JUNCTION;
                        slidePositionCurrent = (int) HIGHJUNCTION;
                    } else if (gamepad2.circle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                        slidePositionCurrent = (int) MIDDLEJUNCTION;
                    } else if (gamepad2.cross && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.LOW_JUNCTION;
                        slidePositionCurrent = (int) LOWJUNCTION;
                    } else if (gamepad2.right_bumper && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.INITIAL_GRAB;
                    } else if (gamepad2.square && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.STACK_POSITION;
                    } else if (gamepad1.left_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DROP;
                    }


                    break;

                case INITIAL_GRAB:

                    slideMotor.setTargetPosition(0);
                    intakeIn(1);
                    if (gamepad2.right_bumper) {

                        autoIntakeTime.reset();
                    } else if (!gamepad2.right_bumper) {

                        if (autoIntakeTime.seconds() < .1) {
                            intakeOut(.1);

                        } else if (autoIntakeTime.seconds() > .1) {
                            intakeIn(.05);
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        }
                    }

                    break;

                case INITIAL_GRAB_STACK:

                    slideMotor.setTargetPosition(0);
                    intakeIn(1);

                    if (gamepad2.right_bumper) {

                        autoIntakeTime.reset();
                    } else if (!gamepad2.right_bumper) {
                        slideMotor.setVelocity(maxSlideVelocity);

//                        if (autoIntakeTime.seconds() < .1) {
//                            intakeOut(.5);

//                        } else if (autoIntakeTime.seconds() > .1) {
                            intakeIn(0);
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION;
//                        }
                    }

                    break;

                case DROP:

                    if (gamepad1.left_bumper) {
                        intakeOut(.6);
                        slideMotor.setTargetPosition(slidePositionCurrent - 300);

                    } else if (!gamepad1.left_bumper) {
                        intakeOut(0);
                        slideMotor.setTargetPosition(slidePositionCurrent + 300);
                    }

                    if (gamepad1.ps | gamepad2.ps) {
                        intakeIn(0);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        slidePositionCurrent = 403;
                    }
                    else if (gamepad2.right_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.INITIAL_GRAB;
                    }


                    break;

                case DROP_DEFAULT:

                    if (gamepad1.left_bumper) {
                        intakeOut(.6);
                    } else if (!gamepad1.left_bumper) {
                        intakeOut(0.05);
                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        slidePositionCurrent = 403;
                    }

                case DROP_STACKED:

                    if (gamepad1.left_bumper) {
                        intakeOut(.6);
                    } else if (!gamepad1.left_bumper) {
                        intakeOut(0.05);
                        autointakeState = AutoIntakeState.STACK_POSITION;
                    }


                    break;


                case STACK_POSITION:
                    slideMotor.setTargetPosition(807);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                            slidePositionCurrent = (int) MIDDLEJUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                            slidePositionCurrent = (int) LOWJUNCTION;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_STACKED;
                        } else if (gamepad2.right_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.INITIAL_GRAB_STACK;
                            slideMotor.setVelocity(maxSlideVelocity * .5);
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                            slidePositionCurrent = (int) HIGHJUNCTION;
                        }
                    }


                    break;


                case HIGH_JUNCTION:
                    slideMotor.setTargetPosition((int) HIGHJUNCTION);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                            slidePositionCurrent = (int) MIDDLEJUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                            slidePositionCurrent = (int) LOWJUNCTION;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION;
                        }
                    }


                    break;

                case MIDDLE_JUNCTION:
                    slideMotor.setTargetPosition((int) MIDDLEJUNCTION);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                            slidePositionCurrent = (int) HIGHJUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                            slidePositionCurrent = (int) LOWJUNCTION;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION;
                        }
                    }


                    break;

                case LOW_JUNCTION:
                    slideMotor.setTargetPosition((int) LOWJUNCTION);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                            slidePositionCurrent = (int) MIDDLEJUNCTION;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                            slidePositionCurrent = (int) HIGHJUNCTION;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION;
                        }
                    }


                    break;

                case GROUND_JUNCTION:

                    slideMotor.setTargetPosition(134);
                    intakeIn(.1);

                    if (autoIntakeTime.seconds() > .1) {

                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = 403;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                            slidePositionCurrent = (int) MIDDLEJUNCTION;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                            slidePositionCurrent = (int) HIGHJUNCTION;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION;
                        }
                    }

                    break;

            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("State:", autointakeState);
            telemetry.addData("Slide Position:", slidePosition);
            telemetry.addData("Slide Ticks", slideMotor.getTargetPosition());
            telemetry.addData("Slide Value", slideValues[slidePosition]);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));
//            telemetry.addLine()
//                    .addData("Red", color.red())
//                    .addData("Green", color.green())
//                    .addData("Blue", color.blue());
//            telemetry.addData("Alpha", color.alpha());

            telemetry.update();
        }
    }
}