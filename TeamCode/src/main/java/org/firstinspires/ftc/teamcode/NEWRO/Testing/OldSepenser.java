package org.firstinspires.ftc.teamcode.NEWRO.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.teamcode.NEWRO.Processors.ColorSensorProcessor;

import java.util.List;



import java.util.List;
@Disabled
@Config
@TeleOp
public class OldSepenser extends OpMode {

    // =========================
    // FIELD CENTRIC DRIVE
    // =========================
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    private double headingOffsetRad = 0.0;

    public static double DRIVE_DEADBAND = 0.05;
    public static double DRIVE_TURN_SCALE = 1.0;
    public static double DRIVE_SPEED_SCALE = 1.0;

    // =========================
    // REV BLINKIN LEDS
    // =========================
    private RevBlinkinLedDriver lights;
    public static double SHOOTER_READY_TOLERANCE = 75.0;
    public static RevBlinkinLedDriver.BlinkinPattern LED_READY = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    public static RevBlinkinLedDriver.BlinkinPattern LED_NOT_READY = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern LED_IDLE = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    // =========================
    // REVOLVER PID
    // =========================
    private PIDFController controllerrev;

    public static double prev = 0.1, irev = 0, drev = 0.001;
    public static double frev = 0.000001;

    // Revolver slot math
    public static int SLOT_TICKS = 96;      // <--- one index step
    public static int target = 0;           // <--- IMPORTANT: start at 0 and increment, don't reset each time
    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx Revolver;

    // =========================
    // SHOOTER PIDF
    // =========================
    public static double pshot = 7.3013, ishot = 0, dshot = 0;
    public static double fshot = 25;

    public double HighVelocityShot = 2000;
    public double LowVelocityShot = 900;
    public double curTargetVelocity = HighVelocityShot;

    private DcMotorEx shooter;

    // =========================
    // MECH
    // =========================
    private TouchSensor touchSensor;
    private CRServo intake;
    private Servo stopper;
    public Servo arm;

    // =========================
    // BALL SENSING
    // =========================
    ColorSensorProcessor colorSensorProcessor;

    // =========================
    // BALL COUNT / FULL
    // =========================
    public boolean BallFull = false;
    public double BallCount = 0;

    // =========================
    // TOUCH TIMER
    // =========================
    private boolean wasPressed = false;
    private boolean waiting = false;
    private final ElapsedTime runtime = new ElapsedTime();

    // =========================
    // LIMELIGHT
    // =========================
    private Limelight3A limelight;
    private int lastFiducialId = -1;

    // =========================================================
    // DASHBOARD PATTERN + TIMEOUT + TELEMETRY
    // =========================================================
    public enum BallPattern {
        GPP, // Green Purple Purple
        PGP, // Purple Green Purple
        PPG  // Purple Purple Green
    }

    private enum AutoShootState {
        IDLE,
        SPINUP,
        FIND_CORRECT_BALL,
        INDEX_SETTLE,
        ARM_UP,
        ARM_DOWN
    }

    // ---- Dashboard knobs ----
    public static BallPattern selectedPattern = BallPattern.GPP;

    public static double SPINUP_SEC = 1.0;
    public static double ARM_HOLD_SEC = 1.0;
    public static double ARM_DOWN_SETTLE_SEC = 0.20;

    public static double FIND_READ_DELAY_SEC = 1.0;
    public static double INDEX_SETTLE_SEC = 1.0;

    public static double FIND_TIMEOUT_SEC = 6.0;
    public static int MAX_INDEX_STEPS = 12;

    public static double ARM_UP_POS = 0.15;
    public static double ARM_DOWN_POS = 0.0;
    public static double STOPPER_SHOOT_POS = 0.3;

    // FAILSAFE: ALWAYS shoot 3 even if color never matches
    public static boolean SHOOT_ANY_ON_TIMEOUT = true;
    public static boolean SHOOT_ANY_ON_MAX_STEPS = true;

    // Limelight -> Pattern mapping
    public static int TAG_ID_GPP = 21;
    public static int TAG_ID_PGP = 22;
    public static int TAG_ID_PPG = 23;

    // ---- State machine runtime ----
    private AutoShootState autoState = AutoShootState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime findTimer = new ElapsedTime();

    private boolean shootRequested = false; // only start when Y pressed
    private boolean ballFullLatched = false;
    private int shotIndex = 0;          // 0..2
    private int indexStepsThisShot = 0;

    private double lastFindReadSec = -999.0;

    // ---- Safe/park positions ----
    public static double SAFE_STOPPER_POS = -0.3;   // "ready for next sequence"
    public static double SAFE_ARM_POS = 0.0;        // usually down

    private void resetShooterMechanismToSafe() {
        try { arm.setPosition(SAFE_ARM_POS); } catch (Exception ignored) {}
        try { stopper.setPosition(SAFE_STOPPER_POS); } catch (Exception ignored) {}
    }

    private void resetSequenceState() {
        autoState = AutoShootState.IDLE;
        ballFullLatched = false;
        shotIndex = 0;
        indexStepsThisShot = 0;
        lastFindReadSec = -999.0;
        shootRequested = false;
        shooter.setVelocity(0);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "Fl");
        fr = hardwareMap.get(DcMotorEx.class, "Fr");
        bl = hardwareMap.get(DcMotorEx.class, "Bl");
        br = hardwareMap.get(DcMotorEx.class, "Br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        headingOffsetRad = 0.0;

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(LED_IDLE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(9);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);
        arm.setPosition(SAFE_ARM_POS);

        controllerrev = new PIDController(prev, irev, drev);
        Revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        Revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = 0;

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        intake = hardwareMap.get(CRServo.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        colorSensorProcessor = new ColorSensorProcessor(hardwareMap);

        stopper.setPosition(SAFE_STOPPER_POS);
    }

    @Override
    public void start() {
        limelight.start();
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

        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);
    }

    // =========================
    // Limelight helper: sets selectedPattern
    // =========================
    private void updatePatternFromLimelight() {
        lastFiducialId = -1;

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        LLResultTypes.FiducialResult fr = fiducials.get(0);
        int id = fr.getFiducialId();
        lastFiducialId = id;

        if (id == TAG_ID_PPG) selectedPattern = BallPattern.PPG;
        else if (id == TAG_ID_PGP) selectedPattern = BallPattern.PGP;
        else if (id == TAG_ID_GPP) selectedPattern = BallPattern.GPP;
    }

    // =========================
    // Pattern -> expected color for shot #
    // =========================
    private ColorSensorProcessor.BallSlot expectedColorForShot(int index) {
        switch (selectedPattern) {
            case GPP:
                return (index == 0) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
            case PGP:
                return (index == 1) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
            case PPG:
                return (index == 2) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
        }
        return ColorSensorProcessor.BallSlot.PURPLE;
    }

    private void advanceRevolverOneSlot() {
        // IMPORTANT: for PID setpoint control, increment the target
        target += SLOT_TICKS;
    }

    private void startShootMotion() {
        stopper.setPosition(STOPPER_SHOOT_POS);
        arm.setPosition(ARM_UP_POS);
        stateTimer.reset();
        autoState = AutoShootState.ARM_UP;
    }

    private void updateBlinkin() {
        double vel = shooter.getVelocity();
        boolean shooterCommanded = shootRequested || gamepad1.right_bumper || autoState != AutoShootState.IDLE;
        boolean atSpeed = Math.abs(vel - curTargetVelocity) <= SHOOTER_READY_TOLERANCE;

        if (!shooterCommanded || curTargetVelocity <= 0) lights.setPattern(LED_IDLE);
        else if (atSpeed) lights.setPattern(LED_READY);
        else lights.setPattern(LED_NOT_READY);
    }

    private void abortSequence(String reason) {
        shooter.setVelocity(0);
        resetShooterMechanismToSafe();
        resetSequenceState();

        BallFull = false;
        BallCount = 0;

        telemetry.addLine("AUTO SHOOT ABORT: " + reason);
    }

    @Override
    public void loop() {

        // Field centric reset
        if (gamepad1.back) {
            headingOffsetRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        fieldCentricDrive();

        // Only start shooting when Y pressed
        if (gamepad1.yWasPressed()) {
            updatePatternFromLimelight();
            shootRequested = true;
        }

        // Revolver PID power
        controllerrev.setPIDF(prev, irev, drev, frev);
        int revpose = Revolver.getCurrentPosition();
        double pid = controllerrev.calculate(revpose, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * frev;
        controllerrev.setTolerance(0.5);
        controllerrev.atSetPoint();
        Revolver.setPower(pid + ff);

        // Shooter PIDF
        PIDFCoefficients shotpidCoeff = new PIDFCoefficients(pshot, ishot, dshot, fshot);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shotpidCoeff);

        // Manual
        if (gamepad1.right_bumper) shooter.setVelocity(curTargetVelocity);
        if (gamepad1.dpadUpWasPressed()) intake.setPower(-1);

        // BallFull logic
        if (BallCount == 3) BallFull = true;

        // Touch sensor increments BallCount (intake indexing)
        boolean pressed = (touchSensor.getValue() >= 0.1);
        if (pressed && !wasPressed && !BallFull) {
            runtime.reset();
            waiting = true;

            BallCount = BallCount + 1;

            // Move one slot after delay so ball settles
            // (no encoder reset here; just add one slot)
        }
        if (waiting) {
            if (runtime.seconds() >= 1) {
                advanceRevolverOneSlot();
                waiting = false;
            }
        }
        wasPressed = pressed;

        // Auto shoot gating
        boolean canStartAuto = BallFull && shootRequested;

        if (canStartAuto && !ballFullLatched && autoState == AutoShootState.IDLE) {
            ballFullLatched = true;
            shotIndex = 0;
            indexStepsThisShot = 0;

            shooter.setVelocity(curTargetVelocity);

            stateTimer.reset();
            autoState = AutoShootState.SPINUP;

            findTimer.reset();
            lastFindReadSec = -999.0;
        }

        if (!BallFull) {
            resetShooterMechanismToSafe();
            resetSequenceState();
        }

        switch (autoState) {
            case IDLE:
                break;

            case SPINUP:
                if (stateTimer.seconds() >= SPINUP_SEC) {
                    findTimer.reset();
                    lastFindReadSec = -999.0;
                    indexStepsThisShot = 0;
                    autoState = AutoShootState.FIND_CORRECT_BALL;
                }
                break;

            case FIND_CORRECT_BALL: {
                double tFind = findTimer.seconds();

                // Read throttle
                if ((tFind - lastFindReadSec) < FIND_READ_DELAY_SEC) break;
                lastFindReadSec = tFind;

                ColorSensorProcessor.BallSlot seen = colorSensorProcessor.colorInSlot1();
                ColorSensorProcessor.BallSlot expected = expectedColorForShot(shotIndex);

                // FAILSAFE behavior so you ALWAYS get 3 shots
                if (tFind >= FIND_TIMEOUT_SEC) {
                    if (SHOOT_ANY_ON_TIMEOUT) {
                        startShootMotion();
                    } else {
                        abortSequence("Find timeout " + FIND_TIMEOUT_SEC + "s on shot " + (shotIndex + 1));
                    }
                    break;
                }
                if (indexStepsThisShot >= MAX_INDEX_STEPS) {
                    if (SHOOT_ANY_ON_MAX_STEPS) {
                        startShootMotion();
                    } else {
                        abortSequence("Max index steps " + MAX_INDEX_STEPS + " on shot " + (shotIndex + 1));
                    }
                    break;
                }

                if (seen == expected) {
                    startShootMotion();
                } else {
                    advanceRevolverOneSlot();
                    indexStepsThisShot++;
                    stateTimer.reset();
                    autoState = AutoShootState.INDEX_SETTLE;
                }
                break;
            }

            case INDEX_SETTLE:
                if (stateTimer.seconds() >= INDEX_SETTLE_SEC) {
                    autoState = AutoShootState.FIND_CORRECT_BALL;
                }
                break;

            case ARM_UP:
                if (stateTimer.seconds() >= ARM_HOLD_SEC) {
                    arm.setPosition(ARM_DOWN_POS);
                    stateTimer.reset();
                    autoState = AutoShootState.ARM_DOWN;
                }
                break;

            case ARM_DOWN:
                if (stateTimer.seconds() >= ARM_DOWN_SETTLE_SEC) {
                    shotIndex++;

                    if (shotIndex >= 3) {
                        // finished all 3
                        shooter.setVelocity(0);

                        BallFull = false;
                        BallCount = 0;

                        resetShooterMechanismToSafe();
                        resetSequenceState();
                    } else {
                        // IMPORTANT FIX: go back and find the next correct ball
                        stopper.setPosition(SAFE_STOPPER_POS);
                        arm.setPosition(ARM_DOWN_POS);

                        findTimer.reset();
                        lastFindReadSec = -999.0;
                        indexStepsThisShot = 0;

                        stateTimer.reset();
                        autoState = AutoShootState.FIND_CORRECT_BALL;
                    }
                }
                break;
        }

        updateBlinkin();

        telemetry.addData("HeadingDeg", Math.toDegrees(getHeadingRad()));
        telemetry.addData("AutoState", autoState);
        telemetry.addData("ShootRequested(Y)", shootRequested);
        telemetry.addData("Pattern", selectedPattern);
        telemetry.addData("Limelight Fiducial", lastFiducialId);

        telemetry.addData("BallFull", BallFull);
        telemetry.addData("BallCount", BallCount);
        telemetry.addData("ShotIndex", shotIndex);
        telemetry.addData("IndexStepsThisShot", indexStepsThisShot);

        telemetry.addData("SeenColor", colorSensorProcessor.colorInSlot1());
        telemetry.addData("ExpectedColor", (autoState == AutoShootState.IDLE) ? "N/A" : expectedColorForShot(Math.min(shotIndex, 2)));

        telemetry.addData("ShooterVel", "%.1f", shooter.getVelocity());
        telemetry.addData("ShooterTarget", "%.1f", curTargetVelocity);

        telemetry.addData("FindTimer", "%.2f / %.2f", findTimer.seconds(), FIND_TIMEOUT_SEC);
        telemetry.addData("StateTimer", "%.2f", stateTimer.seconds());

        telemetry.update();
    }
}

