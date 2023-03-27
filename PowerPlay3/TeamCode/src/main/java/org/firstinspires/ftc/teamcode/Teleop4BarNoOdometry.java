package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareMapMech.DEFAULTFOURBARPOS;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.DEFAULTSLIDEPOS;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.FOURBAROUT;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.FOURBARUSIDE;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.HIGHJUNCTIONSLIDEOUT;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.HIGHJUNCTIONSLIDEU;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.LOWJUNCTIONSLIDEOUT;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.LOWJUNCTIONSLIDEU;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.MIDDLEJUNCTIONSLIDEOUT;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.MIDDLEJUNCTIONSLIDEU;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.STACK;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.intakeLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.intakeRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.powerVariable;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorRight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(group = "advanced")
public class Teleop4BarNoOdometry extends LinearOpMode {

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;


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
 
    private int slidePositionCurrent = 0;


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


 


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
//    private ElapsedTime spinnyTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();


    HardwareMapMech robot = new HardwareMapMech();

    






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
        HIGH_JUNCTION_INTAKE,
        MIDDLE_JUNCTION_INTAKE,
        LOW_JUNCTION_INTAKE,
        GROUND_JUNCTION_INTAKE,
        STACK_POSITION_INTAKE,
        HIGH_JUNCTION_OUTTAKE,
        MIDDLE_JUNCTION_OUTTAKE,
        LOW_JUNCTION_OUTTAKE,
        STACK_POSITION_OUTTAKE,


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

        slideMotorLeft.setTargetPosition((int) (slideValues[slidePosition] * maxSlideTicks));
        slideMotorRight.setTargetPosition((int) (slideValues[slidePosition] * maxSlideTicks));


    }

    public void intakeIn(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void intakeOut(double power) {
        intakeLeft.setPower(-power);
        intakeRight.setPower(-power);
    }

    public void slideMotorTargetPosition(double target) {
        slideMotorLeft.setTargetPosition((int) target);
        slideMotorRight.setTargetPosition((int) target);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        //motor direction
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.FORWARD);

        //zero power behavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        //TODO organize
        //setup driving motors

        robot.init(hardwareMap);



        waitForStart();


        slideMotorLeft.setVelocity(maxSlideVelocity);
        slideMotorRight.setVelocity(maxSlideVelocity);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {



            //TODO Gamepad 1


            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;



            //-----------------------------------------------------------------------------
            //Inverse drive with start
            if (gamepad1.options && !inversePressed)
                inverse *= -1;
            inversePressed = gamepad1.options;


            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setPower((gearValues[activeGear] * (inverse *(-gamepad1LY + gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setPower((gearValues[activeGear] * (inverse * (-gamepad1LY - gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setPower((gearValues[activeGear] * (inverse * (-gamepad1LY - gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setPower((gearValues[activeGear] * (inverse  * (-gamepad1LY + gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else {
                flMotor.setPower(0);
                frMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
            }

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

            //finite statemachines
            switch (autointakeState) {
                case DEFAULT_POSITION:

                    slideMotorLeft.setVelocity(maxSlideVelocity);
                    slideMotorRight.setVelocity(maxSlideVelocity);
                    slideMotorLeft.setTargetPosition(DEFAULTSLIDEPOS);
                    slideMotorRight.setTargetPosition(DEFAULTSLIDEPOS);


                    fourBarLeft.setVelocity(maxSlideVelocity * powerVariable);
                    fourBarRight.setVelocity(maxSlideVelocity * powerVariable);
                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);


                    //Fourbar for u side
                    if (gamepad2.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                        slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                    } else if (gamepad2.circle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                        slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                    } else if (gamepad2.cross && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                        slidePositionCurrent = LOWJUNCTIONSLIDEU;
                    } else if (gamepad2.right_bumper && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.INITIAL_GRAB;
                    } else if (gamepad2.square && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                    } else if (gamepad1.left_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DROP;
                    }
                    //Fourbar for other side
                    else if (gamepad2.dpad_up && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                        slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                    } else if (gamepad2.dpad_left && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                        slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                    } else if (gamepad2.dpad_down && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                        slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                    }


                    break;

                case INITIAL_GRAB:

                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                    slideMotorLeft.setTargetPosition(0);
                    slideMotorRight.setTargetPosition(0);
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
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        }
                    }

                    break;

                case INITIAL_GRAB_STACK:

                    fourBarLeft.setTargetPosition(DEFAULTFOURBARPOS);
                    fourBarRight.setTargetPosition(DEFAULTFOURBARPOS);
                    slideMotorLeft.setTargetPosition(0);
                    slideMotorRight.setTargetPosition(0);
                    intakeIn(1);

                    if (gamepad2.right_bumper) {

                        autoIntakeTime.reset();
                    } else if (!gamepad2.right_bumper) {
                        slideMotorLeft.setVelocity(maxSlideVelocity);
                        slideMotorRight.setVelocity(maxSlideVelocity);


//                        if (autoIntakeTime.seconds() < .1) {
//                            intakeOut(.5);

//                        } else if (autoIntakeTime.seconds() > .1) {
                            intakeIn(0);
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
//                        }
                    }

                    break;

                case DROP:

                    if (gamepad1.left_bumper) {
                        intakeOut(.6);
                        slideMotorLeft.setTargetPosition(slidePositionCurrent - 300);
                        slideMotorRight.setTargetPosition(slidePositionCurrent - 300);


                    } else if (!gamepad1.left_bumper) {
                        intakeOut(0);
                        slideMotorLeft.setTargetPosition(slidePositionCurrent + 300);
                        slideMotorRight.setTargetPosition(slidePositionCurrent + 300);

                    }

                    if (gamepad1.ps | gamepad2.ps) {
                        intakeIn(0);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        slidePositionCurrent = DEFAULTSLIDEPOS;
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
                        slidePositionCurrent = DEFAULTSLIDEPOS;
                    }

                case DROP_STACKED:

                    if (gamepad1.left_bumper) {
                        intakeOut(.6);
                    } else if (!gamepad1.left_bumper) {
                        intakeOut(0.05);
                        autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                    }


                    break;


                case STACK_POSITION_INTAKE:
                    slideMotorLeft.setTargetPosition(STACK);
                    slideMotorRight.setTargetPosition(STACK);
                    fourBarLeft.setTargetPosition(130);
                    fourBarRight.setTargetPosition(130);

                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_STACKED;
                        } else if (gamepad2.right_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.INITIAL_GRAB_STACK;
                            slideMotorLeft.setVelocity(maxSlideVelocity * .5);
                            slideMotorRight.setVelocity(maxSlideVelocity * .5);
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;


                case HIGH_JUNCTION_INTAKE:
                    slideMotorLeft.setTargetPosition(HIGHJUNCTIONSLIDEU);
                    slideMotorRight.setTargetPosition(HIGHJUNCTIONSLIDEU);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;

                case MIDDLE_JUNCTION_INTAKE:
                    slideMotorLeft.setTargetPosition(MIDDLEJUNCTIONSLIDEU);
                    slideMotorRight.setTargetPosition(MIDDLEJUNCTIONSLIDEU);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);

                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;

                case LOW_JUNCTION_INTAKE:
                    slideMotorLeft.setTargetPosition(LOWJUNCTIONSLIDEU);
                    slideMotorRight.setTargetPosition(LOWJUNCTIONSLIDEU);
                    fourBarLeft.setTargetPosition(FOURBARUSIDE);
                    fourBarRight.setTargetPosition(FOURBARUSIDE);

                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;
                case HIGH_JUNCTION_OUTTAKE:
                    slideMotorLeft.setTargetPosition(HIGHJUNCTIONSLIDEOUT);
                    slideMotorRight.setTargetPosition(HIGHJUNCTIONSLIDEOUT);
                    fourBarLeft.setTargetPosition(FOURBAROUT);
                    fourBarRight.setTargetPosition(FOURBAROUT);
                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {

                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;

                case MIDDLE_JUNCTION_OUTTAKE:
                    slideMotorLeft.setTargetPosition(MIDDLEJUNCTIONSLIDEOUT);
                    slideMotorRight.setTargetPosition(MIDDLEJUNCTIONSLIDEOUT);
                    fourBarLeft.setTargetPosition(FOURBAROUT);
                    fourBarRight.setTargetPosition(FOURBAROUT);

                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_down) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_OUTTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEOUT;
                        }
                    }


                    break;

                case LOW_JUNCTION_OUTTAKE:
                    slideMotorLeft.setTargetPosition(LOWJUNCTIONSLIDEOUT);
                    slideMotorRight.setTargetPosition(LOWJUNCTIONSLIDEOUT);
                    fourBarLeft.setTargetPosition(FOURBAROUT);
                    fourBarRight.setTargetPosition(FOURBAROUT);

                    intakeIn(.05);

                    if (autoIntakeTime.seconds() > .1) {


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                            slidePositionCurrent = DEFAULTSLIDEPOS;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_INTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEU;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_INTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEU;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION_INTAKE;
                            slidePositionCurrent = LOWJUNCTIONSLIDEU;
                        } else if (gamepad1.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP;
                        } else if (gamepad2.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.STACK_POSITION_INTAKE;
                        } else if (gamepad2.dpad_up) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION_OUTTAKE;
                            slidePositionCurrent = HIGHJUNCTIONSLIDEOUT;
                        } else if (gamepad2.dpad_left) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION_OUTTAKE;
                            slidePositionCurrent = MIDDLEJUNCTIONSLIDEOUT;
                        }
                    }


                    break;


            }

            // Update everything. Odometry. Etc.


            // Print pose to telemetry
            telemetry.addData("State:", autointakeState);
            telemetry.addData("Slide Position:", slidePosition);
            telemetry.addData("Left Slide Target ", slideMotorLeft.getTargetPosition());
            telemetry.addData("Left Slide Current", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Right Slide Target ", slideMotorRight.getTargetPosition());
            telemetry.addData("Right Slide Current", slideMotorRight.getCurrentPosition());
            telemetry.addData("Slide Value", slideValues[slidePosition]);
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