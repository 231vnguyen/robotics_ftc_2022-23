package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class HardwareMapMech {

    public static double p = 1.175, i = .1175, d = 0;
    public static double f = 11.75;
    public static double positionP = 5;

    public final static double wheelMotorTicks = 384.5;
    public final static double wheelMotorRPM = 435;
    public final static double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    public static double powerVariable = .25;

    public static int target = 0;

    public final static int DEFAULTSLIDEPOS = 403;
    public final static int GROUNDJUNCTION = 0;
    public final static int LOWJUNCTION = 1211;
    public final static int MIDDLEJUNCTION = 2085;
    public final static int HIGHJUNCTION = 2893;


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
        fourBarLeft.setDirection(DcMotorEx.Direction.FORWARD);
        fourBarLeft.setVelocityPIDFCoefficients(p, i, d, f);
        fourBarLeft.setPositionPIDFCoefficients(positionP);


//        fourBarLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, controller);

        fourBarRight = hwMap.get(DcMotorEx.class, "fourBarRight");
        fourBarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarRight.setTargetPosition(0);
        fourBarRight.setVelocity(0);
        fourBarRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarRight.setDirection(DcMotorEx.Direction.REVERSE);
        fourBarRight.setVelocityPIDFCoefficients(p, i, d, f);
        fourBarRight.setPositionPIDFCoefficients(positionP);

        slideMotorLeft = hwMap.get(DcMotorEx.class, "slideMotorLeft");
//        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setTargetPosition(0);
        slideMotorLeft.setVelocity(0);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        slideMotorRight = hwMap.get(DcMotorEx.class, "slideMotorRight");
//        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setTargetPosition(0);
        slideMotorRight.setVelocity(0);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setDirection(DcMotorEx.Direction.REVERSE);

        intakeLeft = hwMap.get(CRServo.class, "intakeLeft");
        intakeRight = hwMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(CRServo.Direction.REVERSE);
    }
}
