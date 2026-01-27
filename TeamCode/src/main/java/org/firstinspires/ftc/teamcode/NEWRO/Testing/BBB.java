package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp
public class BBB extends OpMode {
    private DcMotorEx shooterT;
    private DcMotorEx shooterB;

    private DcMotor Fl, Fr, Bl, Br;

    private DcMotorEx intake;

    private PIDFController controllerrev;

    public static double prev = 0.1, irev = 0, drev = 0.001;
    public static double frev = 0.000001;

    // Revolver slot math
    public static int SLOT_TICKS = 96;
    public static int target = 96; // increment target; do not reset encoders mid-run

    private final double ticks_in_degree = 700 / 180.0;
    private DcMotorEx Revolver;

    private Servo arm;

    public static double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 10;

    public static double HighVelocityShot = 5000;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    private IMU imu;
    private double headingOffsetRad = 0.0;

    public static double DRIVE_DEADBAND = 0.05;
    public static double DRIVE_TURN_SCALE = 1.0;
    public static double DRIVE_SPEED_SCALE = 1.0;

    public boolean shootmode = false;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Fl = hardwareMap.get(DcMotorEx.class, "Fl");
        Fr = hardwareMap.get(DcMotorEx.class, "Fr");
        Bl = hardwareMap.get(DcMotorEx.class, "Bl");
        Br = hardwareMap.get(DcMotorEx.class, "Br");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);
        Fr.setDirection(DcMotorSimple.Direction.FORWARD);
        Br.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(0);

        imu = hardwareMap.get(IMU.class, "imu");
        headingOffsetRad = 0.0;

        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        controllerrev = new PIDController(prev, irev, drev);
        Revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // =========================
    // FIELD CENTRIC: heading (radians)
    // =========================
    private double getHeadingRad() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double yawRad = ypr.getYaw(AngleUnit.RADIANS);
        return yawRad - headingOffsetRad;
    }

    private static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = applyDeadband(y, DRIVE_DEADBAND);
        x = applyDeadband(x, DRIVE_DEADBAND);
        rx = applyDeadband(rx, DRIVE_DEADBAND);

        rx *= DRIVE_TURN_SCALE;

        double heading = getHeadingRad();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotX = x * cos - y * sin;
        double rotY = x * sin + y * cos;

        double flp = rotY + rotX + rx;
        double frp = rotY - rotX - rx;
        double blp = rotY - rotX + rx;
        double brp = rotY + rotX - rx;

        double max = Math.max(1.0, Math.max(Math.abs(flp),
                Math.max(Math.abs(frp), Math.max(Math.abs(blp), Math.abs(brp)))));

        flp = (flp / max) * DRIVE_SPEED_SCALE;
        frp = (frp / max) * DRIVE_SPEED_SCALE;
        blp = (blp / max) * DRIVE_SPEED_SCALE;
        brp = (brp / max) * DRIVE_SPEED_SCALE;

        Fl.setPower(flp);
        Fr.setPower(frp);
        Bl.setPower(blp);
        Br.setPower(brp);
    }

    @Override
    public void loop() {


        // Field centric reset
        if (gamepad1.back) {
            headingOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        fieldCentricDrive();

        // Revolver PID power
        controllerrev.setPIDF(prev, irev, drev, frev);
        int revpose = Revolver.getCurrentPosition();
        double pid = controllerrev.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * frev;
        controllerrev.setTolerance(0.5);
        controllerrev.atSetPoint();
        double power = pid + ff;
        Revolver.setPower(power);


        // Shooter PIDF
        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        if (gamepad1.right_bumper){
            shooterT.setVelocity(curTargetVelocity);
            shooterB.setVelocity(curTargetVelocity);
        }
        if (gamepad1.left_bumper){
            shooterT.setVelocity(0);
            shooterB.setVelocity(0);
        }
        if (gamepad1.dpadLeftWasPressed()) intake.setPower(1);
        if (gamepad1.dpadRightWasPressed()) intake.setPower(0);

        if (gamepad1.dpadUpWasPressed()) {
            arm.setPosition(-0.3);
        }

        if (gamepad1.dpadDownWasPressed()) {
            arm.setPosition(0);
        }
        if (gamepad1.b){
            target = 48;
            Revolver.setTargetPosition(target);
        }
        if (gamepad1.bWasPressed()) {
            target = 96;
            Revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.a) {
            Revolver.setTargetPosition(target);
        }

        if (gamepad1.aWasReleased()){
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

//        if (gamepad1.xWasPressed()) {
//            if (target == 96 || target == 192 || target == 288 || target == 96+288) {
//                target = 140;
//            } else if (target == 140) {
//                target = 240;
//            } else if (target == 240) {
//                target = 340;
//            } else if (target == 340) {
//                target = 140;
//            }
//        }

        telemetry.addData("target", target);
        telemetry.addData("shoot?", shootmode);
        telemetry.addData("CurrentPos", Revolver.getCurrentPosition());
        telemetry.update();

    }
}
