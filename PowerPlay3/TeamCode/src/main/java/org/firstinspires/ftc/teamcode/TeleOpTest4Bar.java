package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.powerVariable;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.slideMotorRight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//88@Disabled
@TeleOp(name = "Slide/Servo Position Test", group = "Testing")


public class TeleOpTest4Bar extends LinearOpMode {

    HardwareMapMech robot = new HardwareMapMech();


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = .9; //to match rotation with driving
    private final double wheelMaxVelocity = 1;

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

    private final double maxSlideTicks = 384.5 * 3.5;
    private int slidePosition = 0;
    private final double[] slideValues = {
            0, .1, .2, .3, .4,  .5, .6, .7, .85, .9, 1, 1.1, 1.2, 1.3, 1.4
            , 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7,
            2.8, 2.9, 3.0, 3.1
    };
    boolean slideMoving = false;


    private final double maxFourBarTicks = 900;
    private int fourBarPosition = 0;
    boolean fourBarMoving = false;
    private final double[] fourBarValues = {
            0, .1, .15, .2, .25, .3, .35, .4, .45, .5, .55, .6, .65, .7, .75, .8, .85, .9, .95,
            1, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.35, 1.4, 1.45, 1.5, 1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7,
            2.8, 2.9, 3.0, 3.1
    };




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
    private DcMotor testMotor;





    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        TOP_LEVEL,
        DROP_FREIGHT,
        SLIDE_DOWN,
        MIDDLE_LEVEL,
        SHARED,
        HIGHER_LEVEL,
        CAPPING,
        INTAKE_DOWN,
        DROP_FREIGHT_SHARED,
        INTAKE_DOWN_SHARED,
        SLIDE_HOME,
        INTAKE_HOME
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

    public void fourBarControl() {
        if (gamepad1.dpad_up && fourBarPosition > 0 && !fourBarMoving) {
            //decrease position
            fourBarPosition--;
            fourBarMoving = true;
        } else if (gamepad1.dpad_down && fourBarPosition < (fourBarValues.length - 1) && !fourBarMoving) {
            //increase position
            fourBarPosition++;
            fourBarMoving = true;
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down && fourBarMoving)
            fourBarMoving = false;

        fourBarLeft.setTargetPosition((int) (fourBarValues[fourBarPosition] * maxFourBarTicks));
        fourBarRight.setTargetPosition((int) (fourBarValues[fourBarPosition] * maxFourBarTicks));


    }




    @Override
    public void runOpMode() throws InterruptedException {


        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");



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


      robot.init(hardwareMap);




        waitForStart();


        slideMotorLeft.setVelocity(maxSlideVelocity);
        slideMotorRight.setVelocity(maxSlideVelocity);
        fourBarRight.setVelocity(maxSlideVelocity * powerVariable);
        fourBarLeft.setVelocity(maxSlideVelocity * powerVariable);







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
            slideControl();
            fourBarControl();


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
                flMotor.setPower((gearValues[activeGear] * ((-gamepad1LY + gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setPower((gearValues[activeGear] * ((-gamepad1LY - gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setPower((gearValues[activeGear] * ((-gamepad1LY - gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setPower((gearValues[activeGear] * ((-gamepad1LY + gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else {
                flMotor.setPower(0);
                frMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
            }

            testMotor.setPower(gamepad2.right_stick_y * .5);

            //-----------------------------------------------------------------------------------


            //TODO Gamepad 2


            /*switch (autointakeState) {
                case DEFAULT_POSITION:

                    slideMotor.setVelocity(maxSlideVelocity);
                    slideMotor.setTargetPosition(0);

                    flicky.setPosition(1);


                    if (gamepad2.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.TOP_LEVEL;
                    }

                    *//* gp1 triangle => slide = 269, *//*

                    break;


                case TOP_LEVEL:


                    if (autoIntakeTime.seconds() > 0) {
                        slideMotor.setPower(1);

                        slideMotor.setTargetPosition((int) (maxSlideTicks * 1.3));


                    }
                    if (autoIntakeTime.seconds() > .1) {

                        if (gamepad1.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DEFAULT_POSITION;


                        }
                    }


                    break;*/



                telemetry.addData("Slide Position:", slidePosition);
                telemetry.addData("Slide Ticks", slideMotorLeft.getTargetPosition());
                telemetry.addData("Slide Value", slideValues[slidePosition]);


                telemetry.update();



            }



        }






    }






