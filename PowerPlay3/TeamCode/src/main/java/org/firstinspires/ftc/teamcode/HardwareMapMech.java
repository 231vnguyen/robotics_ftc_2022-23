package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class HardwareMapMech {

    public static double p = 10, i = 10, d = 0;
    public static double f = 0;
    public static double positionP = 10, positionI = .049988, positionD = 0;
    public PIDFCoefficients controller = new PIDFCoefficients(positionP, positionI, positionD,0);


    public final static double wheelMotorTicks = 384.5;
    public final static double wheelMotorRPM = 435;
    public final static double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    public static double powerVariable = .25;

    public static int target = 0;

    public static int DEFAULTSLIDEPOS = 500;
    public static int DEFAULTFOURBARPOS = 0;
    public static int STACK = 1250; //U side



//    public static int GROUNDJUNCTIONSLIDEU = 0;
    public static int LOWJUNCTIONSLIDEU = 100;
    public static int MIDDLEJUNCTIONSLIDEU = 1450;
    public static int HIGHJUNCTIONSLIDEU = 2550;

//    public static int GROUNDJUNCTIONSLIDEO = 0;
    public static int LOWJUNCTIONSLIDEOUT = 200;
    public static int MIDDLEJUNCTIONSLIDEOUT = 1600;
    public static int HIGHJUNCTIONSLIDEOUT = 2700;



//    public static int GROUNDJUNCTION4U = 0;
    public static int FOURBARUSIDE = 280;

//    public static int GROUNDJUNCTION4O = 0;
    public static int FOURBAROUT = 525;



    public static int GROUNDJUNCTIONOUT = 0;
    public static int LOWJUNCTIONOUT = 0;
    public static int MIDDLEJUNCTIONOUT  = 0;
    public static int HIGHJUNCTIONOUT  = 0;




    public static DcMotorEx slideMotorLeft;
    public static DcMotorEx slideMotorRight;
    public static DcMotorEx fourBarLeft;
    public static DcMotorEx fourBarRight;
    public static CRServo intakeLeft;
    public static CRServo intakeRight;

    HardwareMap hwMap = null;

    //constructor
    public HardwareMapMech() {}

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        fourBarLeft = hwMap.get(DcMotorEx.class, "fourBarLeft");
        fourBarLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fourBarLeft.setTargetPosition(0);
        fourBarLeft.setVelocity(0);
        fourBarLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fourBarLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fourBarLeft.setVelocityPIDFCoefficients(p, i, d, f);
//        fourBarLeft.setPositionPIDFCoefficients(positionP);
//        fourBarLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, controller);



//        fourBarLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, controller);

        fourBarRight = hwMap.get(DcMotorEx.class, "fourBarRight");
        fourBarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarRight.setTargetPosition(0);
        fourBarRight.setVelocity(0);
        fourBarRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarRight.setDirection(DcMotorEx.Direction.FORWARD);
        fourBarRight.setVelocityPIDFCoefficients(p, i, d, f);
//        fourBarRight.setPositionPIDFCoefficients(positionP);

        slideMotorLeft = hwMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setTargetPosition(0);
        slideMotorLeft.setVelocity(0);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        slideMotorRight = hwMap.get(DcMotorEx.class, "slideMotorRight");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setTargetPosition(0);
        slideMotorRight.setVelocity(0);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLeft = hwMap.get(CRServo.class, "intakeLeft");
        intakeRight = hwMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(CRServo.Direction.REVERSE);
    }

    public void intakeIn(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }
    public void HIGHJUNCTIONU() {
        slideMotorLeft.setTargetPosition(HIGHJUNCTIONSLIDEU+150);
        slideMotorRight.setTargetPosition(HIGHJUNCTIONSLIDEU+150);
        fourBarLeft.setTargetPosition(FOURBARUSIDE);
        fourBarRight.setTargetPosition(FOURBARUSIDE);
        intakeIn(.05);
    }

    public void MIDDLEJUNCTIONOUT() {
        slideMotorLeft.setTargetPosition(MIDDLEJUNCTIONSLIDEOUT+300);
        slideMotorRight.setTargetPosition(MIDDLEJUNCTIONSLIDEOUT+300);
        fourBarLeft.setTargetPosition(FOURBAROUT+20);
        fourBarRight.setTargetPosition(FOURBAROUT+20);
        intakeIn(.05);
    }
}
