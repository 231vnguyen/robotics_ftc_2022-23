package org.firstinspires.ftc.teamcode.PIDTuners;

import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarLeft;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.fourBarRight;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.maxSlideVelocity;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.powerVariable;
import static org.firstinspires.ftc.teamcode.HardwareMapMech.target;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareMapMech;

@TeleOp

public class PIDtuneUsingEncoders extends OpMode {


    HardwareMapMech robot = new HardwareMapMech();

    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void loop() {

        fourBarRight.setVelocity(maxSlideVelocity * powerVariable);
        fourBarLeft.setVelocity(maxSlideVelocity * powerVariable);

//        DcMotorControllerEx fourBarLeftController = (DcMotorControllerEx)fourBarLeft.getController();
//        int fourBarLeftIndex =((DcMotorEx)fourBarLeft).getPortNumber();
//        fourBarLeftController.setPIDFCoefficients(fourBarLeftIndex, DcMotor.RunMode.RUN_TO_POSITION, controller);
//
//        DcMotorControllerEx fourBarRightController = (DcMotorControllerEx)fourBarRight.getController();
//        int fourBarRightIndex =((DcMotorEx)fourBarRight).getPortNumber();
//        fourBarRightController.setPIDFCoefficients(fourBarRightIndex, DcMotor.RunMode.RUN_TO_POSITION, controller);



        int leftBarPos = fourBarLeft.getCurrentPosition();
        int rightBarPos = fourBarRight.getCurrentPosition();

        fourBarLeft.setTargetPosition(target);
        fourBarRight.setTargetPosition(target);


        telemetry.addData("leftBarPos", leftBarPos);
        telemetry.addData("rightBarPos", rightBarPos);
        telemetry.addData("PID", fourBarLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

        telemetry.addData("target", target);

    }
}
