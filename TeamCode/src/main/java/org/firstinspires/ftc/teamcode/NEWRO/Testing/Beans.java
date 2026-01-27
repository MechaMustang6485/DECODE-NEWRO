package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class Beans extends OpMode {

    public DcMotorEx Beans;

//    private Servo arm;
//    private double armup = 0.15;
//    private double armdown = 0;

    public double highVelocity =1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0;

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
        Beans = hardwareMap.get(DcMotorEx.class,"shooter");
        Beans.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Beans.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Beans.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");
    }

//    private void TeleOpControls() {
//        if (gamepad1.xWasPressed()) {
//            arm.setPosition(armup);
//        }
//
//        if (gamepad1.aWasPressed()) {
//            arm.setPosition(armdown);
//        }
//    }

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


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Beans.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Beans.setVelocity(curTargetVelocity);

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
