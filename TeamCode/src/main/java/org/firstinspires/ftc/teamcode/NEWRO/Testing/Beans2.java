package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class Beans2 extends OpMode {

    public DcMotorEx Beans;
    public DcMotorEx Beans2;
    private Servo arm;

//    private Servo arm;
//    private double armup = 0.15;
//    private double armdown = 0;

    public double highVelocity =1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    public static double F = 18;

    public static double P = 2;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

//    public void initHardware() {
//        initarm();
//        TeleOpControls();
//    }

//    private void initarm() {
//        arm = hardwareMap.get(Servo.class, "arm");
//        arm.setDirection(Servo.Direction.FORWARD);
//    }

    @Override
    public void init() {
        Beans = hardwareMap.get(DcMotorEx.class,"shooterT");
        Beans.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Beans.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Beans.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");

        Beans2 = hardwareMap.get(DcMotorEx.class,"shooterB");
        Beans2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Beans2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P,0,0,F);
        Beans2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        telemetry.addLine("init complete");

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);
    }


    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            arm.setPosition(0.3);
        }

        if (gamepad2.dpadDownWasPressed()) {
            arm.setPosition(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P,0, 0, F);
        Beans.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Beans2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        Beans.setVelocity(curTargetVelocity);
        Beans2.setVelocity(curTargetVelocity);

        double curVelocity = Beans.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning f", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
