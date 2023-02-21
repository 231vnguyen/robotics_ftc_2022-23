package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class PIDTuneNoEncoder extends OpMode {

    private PIDController controller;


    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double LIFT_SYNC_KP = 0;

    public static int target = 0;

    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    private final double ticks_in_degrees = wheelMotorTicks / 180;


    private DcMotorEx fourBarLeft;
    private DcMotorEx fourBarRight;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fourBarLeft = hardwareMap.get(DcMotorEx.class, "fourBarLeft");
        fourBarLeft.setDirection(DcMotorEx.Direction.REVERSE);
        fourBarRight = hardwareMap.get(DcMotorEx.class, "fourBarRight");
        fourBarRight.setDirection(DcMotorEx.Direction.FORWARD);



    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int leftArmPos = fourBarLeft.getCurrentPosition();
        int rightArmPos = fourBarRight.getCurrentPosition();
        double pid = controller.calculate(leftArmPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;
        double differentiatePower = (rightArmPos - leftArmPos) * LIFT_SYNC_KP;
        fourBarLeft.setPower(Range.clip(power + differentiatePower, -1.0, 1.0));
        fourBarRight.setPower(Range.clip(power - differentiatePower, -1.0, 1.0));


        telemetry.addData("pos", leftArmPos);
        telemetry.addData("target", target);


    }
}
