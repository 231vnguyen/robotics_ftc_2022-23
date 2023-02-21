package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDtuneUsingEncoders extends OpMode {


    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    PIDFCoefficients controller = new PIDFCoefficients(p, i, d, f);


    public static int target = 0;

    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    private DcMotorEx fourBarLeft;
    private DcMotorEx fourBarRight;

    @Override
    public void init() {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fourBarLeft = hardwareMap.get(DcMotorEx.class, "fourBarLeft");
        fourBarLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarLeft.setTargetPosition(0);
        fourBarLeft.setVelocity(0);
        fourBarLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fourBarLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, controller);

        fourBarRight = hardwareMap.get(DcMotorEx.class, "fourBarRight");
        fourBarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarRight.setTargetPosition(0);
        fourBarRight.setVelocity(0);
        fourBarRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarRight.setDirection(DcMotorEx.Direction.FORWARD);
        fourBarLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, controller);


    }

    @Override
    public void loop() {

        int leftBarPos = fourBarLeft.getCurrentPosition();
        int rightBarPos = fourBarRight.getCurrentPosition();

        fourBarLeft.setVelocity(maxSlideVelocity);
        fourBarLeft.setTargetPosition(target);

        fourBarRight.setVelocity(maxSlideVelocity);
        fourBarRight.setTargetPosition(target);

        telemetry.addData("leftBarPos", leftBarPos);
        telemetry.addData("rightBarPos", rightBarPos);

        telemetry.addData("target", target);

    }
}
