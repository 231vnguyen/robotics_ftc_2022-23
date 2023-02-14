package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "TeleOpFlicky", group = "Testing")


public class TeleopFlicky extends LinearOpMode {


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = .9; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxSlideTicks = 384.5 * 3.5;
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


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
//    private ElapsedTime spinnyTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();

    //create motor/servo objects
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;


    //    private CRServo leftCarousel;
//    private CRServo rightCarousel;


    private DcMotorEx slideMotor;

    private Servo flicky;
    private Servo leftSpoiler;
    private Servo rightSpoiler;


    //create arm objects


    //create arm variables


    //arm position values
    /*private int slidePosition = 0;
    private final double[] slideValues = {
            maxSlideTicks * 1, //top level
            maxSlideTicks * .55, //shared
            maxSlideTicks * .3, //low level
            maxSlideTicks * .6, //middle level

    };*/
    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        INITIAL_GRAB,
        INTIAL_GRAB_LOW,
        FINAL_GRAB,
        HIGH_JUNCTION,
        MIDDLE_JUNCTION,
        LOW_JUNCTION,

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

        flicky.setPosition(dropdownValues[dropdownPosition]);


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


    @Override
    public void runOpMode() throws InterruptedException {


        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");


        //motor direction
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        //zero power behavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //armMotor encoders
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setVelocity(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        flicky = hardwareMap.get(Servo.class, "flicky");
        leftSpoiler = hardwareMap.get(Servo.class, "leftSpoiler");
        rightSpoiler = hardwareMap.get(Servo.class, "rightSpoiler");
        rightSpoiler.setDirection(Servo.Direction.REVERSE);




        waitForStart();


        slideMotor.setVelocity(maxSlideVelocity);


        //while loop
        while (opModeIsActive() && !isStopRequested()) {


            //TODO Gamepad 1


            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;

            //event methods
            changeGears();
//            dropdownControl();
//            slideControl();


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
                flMotor.setPower((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setPower((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setPower((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setPower((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else {
                flMotor.setPower(0);
                frMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
            }
            //-----------------------------------------------------------------------------------

            if (gamepad1.left_stick_y < 0) { //joystick up
                leftSpoiler.setPosition(.85);
                rightSpoiler.setPosition(.85);
            } else {
                leftSpoiler.setPosition(1);
                rightSpoiler.setPosition(1);
            }



            //TODO Gamepad 2


            switch (autointakeState) {
                case DEFAULT_POSITION:

                    slideMotor.setVelocity(maxSlideVelocity);
                    slideMotor.setTargetPosition(0);

                    leftSpoiler.setPosition(1);
                    rightSpoiler.setPosition(1);

                    flicky.setPosition(1);


                    if (gamepad1.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.INITIAL_GRAB;
                    } else if (gamepad1.cross && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.FINAL_GRAB;
                    }

                    /* gp1 triangle => slide = 269, */

                    break;

                case INITIAL_GRAB:
                    if (autoIntakeTime.seconds() > 0) {
                        slideMotor.setTargetPosition(320);
                        flicky.setPosition(1);
                    }

                    if (autoIntakeTime.seconds() > .1) {

                        if (gamepad1.right_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.FINAL_GRAB;

                        } else if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        }
                    }

                    break;


                case FINAL_GRAB:
                    flicky.setPosition(.3);
                    dropdownPosition = 3;

                    if (autoIntakeTime.seconds() > 0) {

                        if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                        } else if (gamepad1.square) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.INITIAL_GRAB;
                        } else if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        }

                    }
                    break;

                case HIGH_JUNCTION:
                    slideMotor.setTargetPosition(4037);

                    if (autoIntakeTime.seconds() > .1) {
                        dropdownControl();


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                        }
                    }


                    break;

                case MIDDLE_JUNCTION:
                    slideMotor.setTargetPosition(2960);

                    if (autoIntakeTime.seconds() > .1) {
                        dropdownControl();


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                        } else if (gamepad2.cross) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.LOW_JUNCTION;
                        }
                    }


                    break;

                case LOW_JUNCTION:
                    slideMotor.setTargetPosition(1884);

                    if (autoIntakeTime.seconds() > .1) {
                        dropdownControl();


                        if (gamepad1.ps | gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;
                        } else if (gamepad2.circle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.MIDDLE_JUNCTION;
                        } else if (gamepad2.triangle) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HIGH_JUNCTION;
                        }
                    }


                    break;









            }

            telemetry.addData("Flicky Position:", flicky.getPosition());
            telemetry.addData("Slide Position:", slidePosition);
            telemetry.addData("Slide Ticks", slideMotor.getTargetPosition());
            telemetry.addData("Slide Value", slideValues[slidePosition]);


            telemetry.update();

        }


    }
}






