package org.firstinspires.ftc.teamcode.NewRo2.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Disabled
@Config
@TeleOp
public class Sequencer extends OpMode {

    // =========================
    // Drive
    // =========================
    private DcMotor Fl, Fr, Bl, Br;
    private IMU imu;
    private double headingOffsetRad = 0.0;

    public static double DRIVE_DEADBAND = 0.05;
    public static double DRIVE_TURN_SCALE = 1.0;
    public static double DRIVE_SPEED_SCALE = 1.0;

    // =========================
    // Shooter (NEW: two motors)
    // =========================
    private DcMotorEx shooterT;
    private DcMotorEx shooterB;

    public static double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 10;

    public static double HighVelocityShot = 5000;
    public static double LowVelocityShot = 900;
    public static double curTargetVelocity = HighVelocityShot;

    // =========================
    // Intake + arm + touch
    // =========================
    private DcMotorEx intake;
    private Servo arm;
    private TouchSensor touchSensor;

    // Arm positions (dashboard)
    public static double ARM_UP_POS = 0.30;
    public static double ARM_DOWN_POS = 0.00;

    // =========================
    // Revolver PID (hold target)
    // =========================
    private PIDFController controllerrev;

    public static double prev = 0.1, irev = 0, drev = 0.001;
    public static double frev = 0.000001;

    private final double ticks_in_degree = 700 / 180.0;
    private DcMotorEx Revolver;

    // =========================
    // NEW Revolver mechanism positions (dashboard)
    // intake cycle: 0 -> 96 -> 192 -> 0
    // shooter cycle: 0 -> 144 -> 240 -> shoot(final) then FULL=true
    // =========================
    public static int POS_HOME = 0;
    public static int POS_INTAKE_1 = 96;
    public static int POS_INTAKE_2 = 192;

    public static int POS_SHOOT_1 = 144;
    public static int POS_SHOOT_2 = 240;

    // "shoot" (final) position (set this to whatever your real shoot slot is)
    public static int POS_SHOOT_FINAL = 0;

    public static int target = 96;

    public boolean FULL = false;

    // Touch pressed edge detect
    private boolean wasPressed = false;

    // =========================
    // Limelight (pattern)
    // =========================
    private Limelight3A limelight;
    private int lastFiducialId = -1;

    public enum BallPattern { GPP, PGP, PPG }
    public static BallPattern selectedPattern = BallPattern.GPP;

    public static int TAG_ID_GPP = 21;
    public static int TAG_ID_PGP = 22;
    public static int TAG_ID_PPG = 23;

    // =========================
    // 3-shot sequencer (runs when Y pressed)
    // =========================
    private enum ShootState {
        IDLE,
        SPINUP,
        ARM_UP,
        ARM_DOWN,
        NEXT_SHOT,
        DONE,
        ABORT
    }

    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();
    private int shotsDone = 0;

    // Sequencer timings (dashboard)
    public static double SPINUP_SEC = 1.0;
    public static double ARM_HOLD_SEC = 1.0;
    public static double ARM_DOWN_SETTLE_SEC = 0.20;

    // Optional overall timeout so it can’t get stuck
    public static double SEQUENCE_TIMEOUT_SEC = 8.0;
    private final ElapsedTime sequenceTimeout = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Drive motors
        Fl = hardwareMap.get(DcMotorEx.class, "Fl");
        Fr = hardwareMap.get(DcMotorEx.class, "Fr");
        Bl = hardwareMap.get(DcMotorEx.class, "Bl");
        Br = hardwareMap.get(DcMotorEx.class, "Br");

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);
        Fr.setDirection(DcMotorSimple.Direction.FORWARD);
        Br.setDirection(DcMotorSimple.Direction.FORWARD);

        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        // Intake + arm + touch
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(ARM_DOWN_POS);

        // CHANGE NAME if needed
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Shooter motors (keeping your old mapping exactly)
        shooterB = hardwareMap.get(DcMotorEx.class, "shooterT");
        shooterB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterT = hardwareMap.get(DcMotorEx.class, "shooterB");
        shooterT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterT.setDirection(DcMotorSimple.Direction.REVERSE);

        // Revolver + PID
        controllerrev = new PIDController(prev, irev, drev);
        Revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(9);

        // Defaults
        headingOffsetRad = 0.0;
        FULL = false;
        shootState = ShootState.IDLE;
        shotsDone = 0;
        lastFiducialId = -1;
    }

    @Override
    public void start() {
        if (limelight != null) limelight.start();
    }

    // =========================
    // Field centric
    // =========================
    private double getHeadingRad() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.RADIANS) - headingOffsetRad;
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

    // =========================
    // Limelight pattern read (called on Y press)
    // =========================
    private void updatePatternFromLimelight() {
        lastFiducialId = -1;

        if (limelight == null) return;
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return;

        List<LLResultTypes.FiducialResult> fid = r.getFiducialResults();
        if (fid == null || fid.isEmpty()) return;

        int id = fid.get(0).getFiducialId();
        lastFiducialId = id;

        if (id == TAG_ID_GPP) selectedPattern = BallPattern.GPP;
        else if (id == TAG_ID_PGP) selectedPattern = BallPattern.PGP;
        else if (id == TAG_ID_PPG) selectedPattern = BallPattern.PPG;
    }

    // =========================
    // Revolver PID holding target
    // =========================
    private void updateRevolverPID() {
        controllerrev.setPIDF(prev, irev, drev, frev);
        int revpose = Revolver.getCurrentPosition();

        double pid = controllerrev.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * frev;

        controllerrev.setTolerance(0.5);
        controllerrev.atSetPoint();

        Revolver.setPower(pid + ff);
    }

    // =========================
    // Shooter helper
    // =========================
    private void setShooterVelocity(double v) {
        shooterT.setVelocity(v);
        shooterB.setVelocity(v);
    }

    // =========================
    // Sequence control
    // =========================
    private void startSequence() {
        shotsDone = 0;
        setShooterVelocity(curTargetVelocity);
        shootTimer.reset();
        sequenceTimeout.reset();
        shootState = ShootState.SPINUP;
    }

    private void abortSequence(String reason) {
        setShooterVelocity(0);
        arm.setPosition(ARM_DOWN_POS);
        shootState = ShootState.ABORT;
        telemetry.addLine("SEQUENCE ABORT: " + reason);
    }

    private void updateSequence() {
        if (shootState == ShootState.IDLE) return;

        if (sequenceTimeout.seconds() > SEQUENCE_TIMEOUT_SEC) {
            abortSequence("timeout " + SEQUENCE_TIMEOUT_SEC + "s");
            return;
        }

        switch (shootState) {
            case SPINUP:
                if (shootTimer.seconds() >= SPINUP_SEC) {
                    arm.setPosition(ARM_UP_POS);
                    shootTimer.reset();
                    shootState = ShootState.ARM_UP;
                }
                break;

            case ARM_UP:
                if (shootTimer.seconds() >= ARM_HOLD_SEC) {
                    arm.setPosition(ARM_DOWN_POS);
                    shootTimer.reset();
                    shootState = ShootState.ARM_DOWN;
                }
                break;

            case ARM_DOWN:
                if (shootTimer.seconds() >= ARM_DOWN_SETTLE_SEC) {
                    shotsDone++;
                    shootState = ShootState.NEXT_SHOT;
                }
                break;

            case NEXT_SHOT:
                if (shotsDone >= 3) {
                    shootState = ShootState.DONE;
                    break;
                }
                arm.setPosition(ARM_UP_POS);
                shootTimer.reset();
                shootState = ShootState.ARM_UP;
                break;

            case DONE:
                setShooterVelocity(0);
                arm.setPosition(ARM_DOWN_POS);
                shootState = ShootState.IDLE;
                break;

            case ABORT:
                // one loop of ABORT is enough; return to IDLE next frame
                shootState = ShootState.IDLE;
                break;

            default:
                shootState = ShootState.IDLE;
                break;
        }
    }

    // =========================
    // Your NEW revolver mechanism (exact behavior)
    // =========================
    private void updateNewRevolverMechanism(boolean pressedRising) {

        // FULL block (touch press to advance intake/home)
        if (FULL) {
            if (pressedRising) {
                if (target == POS_HOME) {
                    target = POS_INTAKE_1;
                } else if (target == POS_INTAKE_1) {
                    target = POS_INTAKE_2;
                } else {
                    target = POS_HOME;
                }
                FULL = false;
            }
        }

        // Y press block: intake -> shooter -> intake -> shooter ... (your mapping)
        if (gamepad1.yWasPressed()) {

            // Grab pattern when user chooses to shoot (limelight)
            updatePatternFromLimelight();

            if (target == POS_HOME) {
                target = POS_SHOOT_1;
            } else if (target == POS_SHOOT_1) {
                target = POS_SHOOT_2;
            } else {
                target = POS_SHOOT_FINAL;
                FULL = true;
            }

            // Start the 3-shot sequence on Y (only if not already running)
            if (shootState == ShootState.IDLE) {
                startSequence();
            }
        }
    }

    @Override
    public void loop() {

        // Field-centric reset
        if (gamepad1.back) {
            headingOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        fieldCentricDrive();

        // Touch rising edge
        boolean pressed = (touchSensor != null && touchSensor.getValue() >= 0.1);
        boolean pressedRising = pressed && !wasPressed;
        wasPressed = pressed;

        // Revolver mechanism updates target (your snippet)
        updateNewRevolverMechanism(pressedRising);

        // Revolver PID holds at current target
        updateRevolverPID();

        // Shooter PIDF
        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooterB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);
        shooterT.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        // Manual shooter
        if (gamepad1.right_bumper) setShooterVelocity(curTargetVelocity);
        if (gamepad1.left_bumper) setShooterVelocity(0);

        // Manual intake
        if (gamepad1.dpadLeftWasPressed()) intake.setPower(1);
        if (gamepad1.dpadRightWasPressed()) intake.setPower(0);

        // Manual arm (still works; sequencer will overwrite while running)
        if (gamepad1.dpadUpWasPressed()) arm.setPosition(ARM_UP_POS);
        if (gamepad1.dpadDownWasPressed()) arm.setPosition(ARM_DOWN_POS);

        // Optional: keep your old A/B stuff
        if (gamepad1.b) {
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
        if (gamepad1.aWasReleased()) {
            Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            Revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Sequencer update
        updateSequence();

        // Telemetry
        telemetry.addData("target", target);
        telemetry.addData("RevolverPos", Revolver.getCurrentPosition());
        telemetry.addData("FULL", FULL);
        telemetry.addData("TouchPressed", pressed);

        telemetry.addData("SeqState", shootState);
        telemetry.addData("ShotsDone", shotsDone);
        telemetry.addData("SeqT", "%.2f", shootTimer.seconds());
        telemetry.addData("SeqTimeoutT", "%.2f", sequenceTimeout.seconds());

        telemetry.addData("ShooterVelT", "%.0f", shooterT.getVelocity());
        telemetry.addData("ShooterVelB", "%.0f", shooterB.getVelocity());
        telemetry.addData("ShooterTarget", "%.0f", curTargetVelocity);

        telemetry.addData("Pattern", selectedPattern);
        telemetry.addData("LimelightTag", lastFiducialId);

        telemetry.update();
    }
}
