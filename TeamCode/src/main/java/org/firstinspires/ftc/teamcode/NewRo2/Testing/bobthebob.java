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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEWRO.Processors.ColorSensorProcessor;

import java.util.List;
@Disabled
@Config
@TeleOp
public class bobthebob extends OpMode {

    // =========================
    // DASHBOARD / TUNING
    // =========================
    public static int STEP_TICKS = 96;                 // one slot step (96)
    public static double SPINUP_SEC = 1.0;

    public static double ARM_UP_POS = 0.15;
    public static double ARM_DOWN_POS = 0.0;
    public static double ARM_UP_HOLD_SEC = 1.0;
    public static double ARM_DOWN_SETTLE_SEC = 0.20;

    public static double STOPPER_SHOOT_POS = 0.3;
    public static double SAFE_STOPPER_POS = -0.3;
    public static double SAFE_ARM_POS = 0.0;

    public static double INDEX_SETTLE_SEC = 0.7;
    public static double TOTAL_TIMEOUT_SEC = 10.0;

    public static double SHOOTER_TARGET_VEL = 2000;
    public static double READY_TOL = 75;

    // Limelight
    public static int LIMELIGHT_PIPELINE = 9;
    public static int TAG_ID_GPP = 21;
    public static int TAG_ID_PGP = 22;
    public static int TAG_ID_PPG = 23;

    public enum BallPattern { GPP, PGP, PPG }
    public static BallPattern DEFAULT_PATTERN = BallPattern.GPP;

    // =========================
    // REVOLVER PID
    // =========================
    private PIDFController controllerrev;
    public static double prev = 0.1, irev = 0, drev = 0.001;
    public static double frev = 0.000001;
    private final double ticks_in_degree = 700 / 180.0;

    // =========================
    // HARDWARE
    // =========================
    private DcMotorEx revolver;
    private DcMotorEx shooter;
    private Servo arm;
    private Servo stopper;
    private Limelight3A limelight;
    private ColorSensorProcessor colorProc;

    // slotMem[0]=slot1(color1), slotMem[1]=slot2(color2), slotMem[2]=slot3(color3)
    private final ColorSensorProcessor.BallSlot[] slotMem = new ColorSensorProcessor.BallSlot[3];

    // limelight latch (until stop)
    private boolean patternLatched = false;
    private BallPattern latchedPattern = DEFAULT_PATTERN;
    private int latchedFid = -1;

    // =========================
    // STATE MACHINE
    // =========================
    private enum ShootState {
        IDLE,
        SPINUP,
        PLAN_POSITION,
        INDEX_STEP,
        INDEX_SETTLE,
        ARM_UP,
        ARM_DOWN,
        NEXT_SHOT,
        SHOOT_ALL,          // <--- fallback mode
        DONE,
        ABORT
    }

    private ShootState state = ShootState.IDLE;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    private int shotIndex = 0;                 // 0..2 for smart sequence
    private int remainingStepsToPosition = 0;  // clockwise steps to bring target color to slot1

    // for SHOOT_ALL fallback
    private int shootAllRemaining = 0;
    private boolean shootAllNeedIndex = false;

    // =========================
    // INIT
    // =========================
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(Servo.class, "arm");
        stopper = hardwareMap.get(Servo.class, "stopper");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        colorProc = new ColorSensorProcessor(hardwareMap);

        controllerrev = new PIDController(prev, irev, drev);

        parkSafe();
        refreshSlotMemory();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Revolver PID power (your style)
        controllerrev.setPIDF(prev, irev, drev, frev);
        int revpose = revolver.getCurrentPosition();
        double pid = controllerrev.calculate(revpose, STEP_TICKS);
        double ff = Math.cos(Math.toRadians(STEP_TICKS / ticks_in_degree)) * frev;
        revolver.setPower(pid + ff);

        // Start on Y
        if (gamepad1.yWasPressed() && state == ShootState.IDLE) {
            latchLimelightOnce();
            refreshSlotMemory();
            beginSequence();
        }

        // Manual cancel
        if (gamepad1.xWasPressed() && state != ShootState.IDLE) {
            abort("Manual cancel");
        }

        runStateMachine();

        // Telemetry
        telemetry.addData("State", state);
        telemetry.addData("ShotIndex", shotIndex);
        telemetry.addData("StepsRemaining", remainingStepsToPosition);
        telemetry.addData("TotalSec", "%.2f", totalTimer.seconds());

        telemetry.addData("LatchedPattern", latchedPattern);
        telemetry.addData("LatchedFid", latchedFid);

        telemetry.addData("Slot1", slotMem[0]);
        telemetry.addData("Slot2", slotMem[1]);
        telemetry.addData("Slot3", slotMem[2]);

        telemetry.addData("Raw1", "%.1f", colorProc.rawcolorslot1());
        telemetry.addData("Raw2", "%.1f", colorProc.rawcolorslot2());
        telemetry.addData("Raw3", "%.1f", colorProc.rawcolorslot3());

        telemetry.addData("ShooterVel", "%.1f", shooter.getVelocity());
        telemetry.addData("ShooterTarget", "%.1f", SHOOTER_TARGET_VEL);

        telemetry.update();
    }

    // =========================
    // BEGIN SEQUENCE
    // =========================
    private void beginSequence() {
        totalTimer.reset();
        stateTimer.reset();
        shotIndex = 0;
        remainingStepsToPosition = 0;

        // Re-read slots once at start
        refreshSlotMemory();

        int pCount = count(ColorSensorProcessor.BallSlot.PURPLE);
        int gCount = count(ColorSensorProcessor.BallSlot.GREEN);

        // rules:
        // - if only purples -> shoot all
        if (pCount == 3) {
            startShootAll("All purples");
            return;
        }

        // - if not 2P+1G -> shoot all
        if (!(pCount == 2 && gCount == 1)) {
            startShootAll("Not 2P+1G");
            return;
        }

        // - if 2P+1G but can't see a valid pattern -> fallback to GPP
        if (!matchesAnyValidPattern()) {
            latchedPattern = BallPattern.GPP;
        }

        shooter.setVelocity(SHOOTER_TARGET_VEL);
        parkSafe();
        state = ShootState.SPINUP;
    }

    // =========================
    // MAIN FSM
    // =========================
    private void runStateMachine() {

        // global timeout => shoot all
        if (state != ShootState.IDLE && state != ShootState.DONE && state != ShootState.ABORT) {
            if (totalTimer.seconds() >= TOTAL_TIMEOUT_SEC) {
                startShootAll("Total timeout");
            }
        }

        switch (state) {

            case IDLE:
                break;

            case SPINUP:
                if (stateTimer.seconds() >= SPINUP_SEC) {
                    stateTimer.reset();
                    state = ShootState.PLAN_POSITION;
                }
                break;

            case PLAN_POSITION: {
                refreshSlotMemory();

                ColorSensorProcessor.BallSlot expected = expectedForShot(latchedPattern, shotIndex);
                int steps = stepsToBringExpectedToSlot1Clockwise(expected);
                if (steps < 0) {
                    startShootAll("Expected not found");
                    break;
                }

                remainingStepsToPosition = steps;

                if (remainingStepsToPosition == 0) {
                    stopper.setPosition(STOPPER_SHOOT_POS);
                    arm.setPosition(ARM_UP_POS);
                    stateTimer.reset();
                    state = ShootState.ARM_UP;
                } else {
                    state = ShootState.INDEX_STEP;
                }
                break;
            }

            case INDEX_STEP:
                doOneClockwiseStep();
                remainingStepsToPosition--;
                stateTimer.reset();
                state = ShootState.INDEX_SETTLE;
                break;

            case INDEX_SETTLE:
                if (stateTimer.seconds() >= INDEX_SETTLE_SEC) {
                    state = ShootState.PLAN_POSITION;
                }
                break;

            case ARM_UP:
                if (stateTimer.seconds() >= ARM_UP_HOLD_SEC) {
                    arm.setPosition(ARM_DOWN_POS);
                    stateTimer.reset();
                    state = ShootState.ARM_DOWN;
                }
                break;

            case ARM_DOWN:
                if (stateTimer.seconds() >= ARM_DOWN_SETTLE_SEC) {
                    parkSafe();

                    if (state == ShootState.SHOOT_ALL) {
                        // not reachable here; SHOOT_ALL handles its own flow below
                    }

                    // If we're in shoot-all flow, we'll set a flag and handle in SHOOT_ALL
                    if (shootAllRemaining > 0) {
                        shootAllNeedIndex = true;
                        state = ShootState.SHOOT_ALL;
                    } else {
                        state = ShootState.NEXT_SHOT;
                    }
                }
                break;

            case NEXT_SHOT:
                shotIndex++;
                if (shotIndex >= 3) {
                    state = ShootState.DONE;
                } else {
                    stateTimer.reset();
                    state = ShootState.PLAN_POSITION;
                }
                break;

            case SHOOT_ALL:
                runShootAllFlow();
                break;

            case DONE:
                shooter.setVelocity(0);
                parkSafe();
                shootAllRemaining = 0;
                shootAllNeedIndex = false;
                state = ShootState.IDLE;
                break;

            case ABORT:
                shooter.setVelocity(0);
                parkSafe();
                shootAllRemaining = 0;
                shootAllNeedIndex = false;
                state = ShootState.IDLE;
                break;
        }
    }

    // =========================
    // SHOOT ALL FLOW:
    // shoot -> (index+settle) -> shoot -> (index+settle) -> shoot -> done
    // Uses ARM_UP/ARM_DOWN for timing
    // =========================
    private void runShootAllFlow() {

        // If we just finished a shot, we index once before the next shot (except after the last)
        if (shootAllNeedIndex) {
            if (shootAllRemaining <= 1) {
                // last shot already done, no need to index
                shootAllNeedIndex = false;
                state = ShootState.DONE;
                return;
            }

            doOneClockwiseStep();
            stateTimer.reset();
            shootAllNeedIndex = false;

            // wait settle before next shot
            state = ShootState.INDEX_SETTLE;
            return;
        }

        // If we are in INDEX_SETTLE due to shoot-all, when settle ends we trigger the next shot
        if (state == ShootState.INDEX_SETTLE) {
            if (stateTimer.seconds() >= INDEX_SETTLE_SEC) {
                // shoot next
                stopper.setPosition(STOPPER_SHOOT_POS);
                arm.setPosition(ARM_UP_POS);
                stateTimer.reset();
                state = ShootState.ARM_UP;
            }
            return;
        }

        // If we're here, we need to start or continue shoot-all shots
        if (shootAllRemaining <= 0) {
            state = ShootState.DONE;
            return;
        }

        // Start first shot immediately if not currently in ARM_* states
        if (state != ShootState.ARM_UP && state != ShootState.ARM_DOWN) {
            stopper.setPosition(STOPPER_SHOOT_POS);
            arm.setPosition(ARM_UP_POS);
            stateTimer.reset();
            state = ShootState.ARM_UP;
        }

        // When ARM_DOWN completes, we decrement shootAllRemaining in ARM_DOWN handler
        // So we must ensure that decrement happens:
        if (state == ShootState.ARM_DOWN && stateTimer.seconds() >= ARM_DOWN_SETTLE_SEC) {
            // handled in ARM_DOWN case
        }
    }

    // =========================
    // HELPERS
    // =========================
    private void startShootAll(String reason) {
        telemetry.addLine("Fallback: SHOOT ALL (" + reason + ")");
        shooter.setVelocity(SHOOTER_TARGET_VEL);
        parkSafe();

        // shoot 3 balls no matter what
        shootAllRemaining = 3;
        shotIndex = 0; // irrelevant, but keep clean

        // Start first shot immediately
        stopper.setPosition(STOPPER_SHOOT_POS);
        arm.setPosition(ARM_UP_POS);
        stateTimer.reset();

        state = ShootState.ARM_UP; // we reuse ARM_UP/ARM_DOWN timing
    }

    private void abort(String reason) {
        telemetry.addLine("ABORT: " + reason);
        state = ShootState.ABORT;
    }

    private void parkSafe() {
        stopper.setPosition(SAFE_STOPPER_POS);
        arm.setPosition(SAFE_ARM_POS);
    }

    private void refreshSlotMemory() {
        slotMem[0] = colorProc.colorInSlot1();
        slotMem[1] = colorProc.colorInSlot2();
        slotMem[2] = colorProc.colorInSlot3();
    }

    private int count(ColorSensorProcessor.BallSlot c) {
        int n = 0;
        for (int i = 0; i < 3; i++) if (slotMem[i] == c) n++;
        return n;
    }

    private boolean matchesAnyValidPattern() {
        boolean gpp = slotMem[0] == ColorSensorProcessor.BallSlot.GREEN &&
                slotMem[1] == ColorSensorProcessor.BallSlot.PURPLE &&
                slotMem[2] == ColorSensorProcessor.BallSlot.PURPLE;

        boolean pgp = slotMem[0] == ColorSensorProcessor.BallSlot.PURPLE &&
                slotMem[1] == ColorSensorProcessor.BallSlot.GREEN &&
                slotMem[2] == ColorSensorProcessor.BallSlot.PURPLE;

        boolean ppg = slotMem[0] == ColorSensorProcessor.BallSlot.PURPLE &&
                slotMem[1] == ColorSensorProcessor.BallSlot.PURPLE &&
                slotMem[2] == ColorSensorProcessor.BallSlot.GREEN;

        return gpp || pgp || ppg;
    }

    private ColorSensorProcessor.BallSlot expectedForShot(BallPattern p, int shotIndex) {
        switch (p) {
            case GPP:
                return (shotIndex == 0) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
            case PGP:
                return (shotIndex == 1) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
            case PPG:
                return (shotIndex == 2) ? ColorSensorProcessor.BallSlot.GREEN : ColorSensorProcessor.BallSlot.PURPLE;
        }
        return ColorSensorProcessor.BallSlot.PURPLE;
    }

    // nearest clockwise purple, otherwise green
    private int stepsToBringExpectedToSlot1Clockwise(ColorSensorProcessor.BallSlot expected) {
        if (expected == ColorSensorProcessor.BallSlot.PURPLE) {
            if (slotMem[0] == ColorSensorProcessor.BallSlot.PURPLE) return 0;
            if (slotMem[1] == ColorSensorProcessor.BallSlot.PURPLE) return 1;
            if (slotMem[2] == ColorSensorProcessor.BallSlot.PURPLE) return 2;
            return -1;
        }
        if (expected == ColorSensorProcessor.BallSlot.GREEN) {
            if (slotMem[0] == ColorSensorProcessor.BallSlot.GREEN) return 0;
            if (slotMem[1] == ColorSensorProcessor.BallSlot.GREEN) return 1;
            if (slotMem[2] == ColorSensorProcessor.BallSlot.GREEN) return 2;
            return -1;
        }
        return -1;
    }

    private void rotateSlotMemoryClockwise() {
        ColorSensorProcessor.BallSlot s1 = slotMem[0];
        ColorSensorProcessor.BallSlot s2 = slotMem[1];
        ColorSensorProcessor.BallSlot s3 = slotMem[2];

        slotMem[0] = s3;
        slotMem[1] = s1;
        slotMem[2] = s2;
    }

    private void doOneClockwiseStep() {
        // physical step
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setTargetPosition(STEP_TICKS);

        // remember rotation
        rotateSlotMemoryClockwise();
    }

    private void latchLimelightOnce() {
        if (patternLatched) return;

        int fid = -1;
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
            if (fids != null && !fids.isEmpty()) {
                fid = fids.get(0).getFiducialId();
            }
        }

        BallPattern chosen;
        if (fid == TAG_ID_GPP) chosen = BallPattern.GPP;
        else if (fid == TAG_ID_PGP) chosen = BallPattern.PGP;
        else if (fid == TAG_ID_PPG) chosen = BallPattern.PPG;
        else chosen = DEFAULT_PATTERN;

        patternLatched = true;
        latchedPattern = chosen;
        latchedFid = fid;
    }
}
