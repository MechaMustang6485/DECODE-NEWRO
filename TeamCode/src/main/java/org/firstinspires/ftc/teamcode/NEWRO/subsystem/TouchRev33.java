package org.firstinspires.ftc.teamcode.NEWRO.subsystem;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.NEWRO.Processors.PIDClassForAuto;

import java.util.List;

@Config
public class TouchRev33 {

    // =========================
    // REVOLVER CONFIG
    // =========================
    public static int TICKS_PER_SLOT = 96;
    public static int MAX_SLOTS = 3;
    public static int MAX_POSITION = MAX_SLOTS * TICKS_PER_SLOT; // 288

    public static int POS_INTAKE = 0;
    public static int POS_SHOOT0 = 48;
    public static int POS_SHOOT1 = 144;
    public static int POS_SHOOT2 = 240;

    public static double MOTOR_POWER_LIMIT = 1.0;
    public static int AT_TARGET_TOL = 8;

    // How long we wait to arrive (if not at target yet)
    public double MOVE_TIMEOUT_SEC = 0.45;

    // =========================
    // COLOR4 CONFIG
    // =========================
    public static float GREEN_H_MIN = 100;
    public static float GREEN_H_MAX = 180;
    public static float PURPLE_H_MIN = 181;
    public static float PURPLE_H_MAX = 255;

    public double VALIDATE_DELAY_SEC = 0.05;

    public double UNKNOWN_SCAN_SEC = 0.25;

    // Only validate first 2 balls (3rd assumed correct)
    public static boolean ONLY_VALIDATE_FIRST_TWO = true;

    // Arm timing
    public double ARM_UPDOWN_SEC = 0.12;

    // ADDED BACK: Wait time after a shot for the shooter to recover RPMs
    public static double REVUP_DELAY_SEC = 0.5;

    // =========================
    // LIMELIGHT CONFIG
    // =========================
    public static int LIMELIGHT_PIPELINE = 8;
    public static int POLL_HZ = 100;
    public double LIMELIGHT_SCAN_TIMEOUT_SEC = 1.0;


    // =========================
    // TURRET CONFIG (optional)
    // =========================
    public double turretP = 0.02;
    public double turretD = 0.002;
    public double turretMaxPower = 0.5;
    public double turretMinPower = 0.05;
    public double turretTolerance = 0.5;

    public boolean turretLimits = true;
    public int turretMinEnc = -3_000_000;
    public int turretMaxEnc = 3_000_000;

    // =========================
    // ARM CONFIG
    // =========================
    public static double ARM_DOWN = 0.0;
    public static double ARM_UP = 0.3;

    // =========================
    // HARDWARE
    // =========================
    private final DcMotorEx revolver;
    private final TouchSensor touchSensor;
    private final Limelight3A limelight;
    private final NormalizedColorSensor color4;
    private final Servo arm;

    // Turret (optional)
    public final CRServo turretServo;
    private final DcMotorEx turretEncoderMotor;
    private final IMU imu;

    // =========================
    // TOUCHREV2-STYLE STATE
    // =========================
    private int targetPosition = 0;
    private int currentSlotCount = 0;
    private boolean lastButtonState = false;
    private boolean isSensorEnabled = false;

    // =========================
    // LIMELIGHT STATE
    // =========================
    public enum Pattern { GPP, PGP, PPG }
    private volatile Pattern desiredPattern = Pattern.PGP;
    private volatile int lastSequenceId = -1;

    // =========================
    // SEQUENCE STATE (PERSISTENT)
    // =========================
    private boolean sequenceRunning = false;

    private enum SeqState {
        INIT,
        MOVE_TO_TARGET,
        WAIT_VALIDATE,
        READ_VALIDATE,
        ARM_UP_STATE,
        ARM_DOWN_STATE,
        POST_SHOT_SETTLE, // Re-activated
        NEXT_SHOT,
        RETURN_INTAKE,
        DONE
    }

    private SeqState seqState = SeqState.INIT;

    private int shotsDone = 0;
    private int currentIdx = 0;
    private int targetIdx = 0;
    private int targetPos = POS_SHOOT0;

    private final boolean[] shotMask = new boolean[]{false, false, false};

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime unknownScanTimer = new ElapsedTime();

    private BallColor lastNonUnknownSeen = BallColor.UNKNOWN;

    private final int[] SHOOT_POS = new int[]{POS_SHOOT0, POS_SHOOT1, POS_SHOOT2};

    public enum BallColor { GREEN, PURPLE, UNKNOWN }

    // =========================
    // MEMORY SCAN (NEW)
    // =========================
    private final BallColor[] slotColorMem = new BallColor[]{BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
    private boolean memValid = false;

    private enum MemScanState {
        INIT,
        MOVE_SLOT,
        WAIT_DELAY,
        SAMPLE,
        NEXT_SLOT,
        RETURN_INTAKE,
        DONE
    }

    private MemScanState memScanState = MemScanState.INIT;
    private int memScanIdx = 0;
    private final ElapsedTime memTimer = new ElapsedTime();
    private final ElapsedTime memUnknownTimer = new ElapsedTime();
    private BallColor memLastNonUnknown = BallColor.UNKNOWN;

    private enum MemShootState {
        INIT,
        MOVE_TO_SLOT,
        ARM_UP,
        ARM_DOWN,
        SETTLE, // Re-activated
        NEXT,
        RETURN_INTAKE,
        DONE
    }

    private MemShootState memShootState = MemShootState.INIT;
    private int memShotsDone = 0;
    private int memTargetIdx = 0;
    private int memTargetPos = POS_SHOOT0;
    private final boolean[] memShotMask = new boolean[]{false, false, false};
    private final ElapsedTime memShootTimer = new ElapsedTime();
    private final ElapsedTime memMoveTimer = new ElapsedTime();

    // Turret PID runtime
    private double turretLastError = 0;
    private final ElapsedTime turretPidTimer = new ElapsedTime();
    private boolean turretTrackingEnabled = true;

    // =========================
    // CONSTRUCTOR
    // =========================
    public TouchRev33(HardwareMap hardwareMap) {
        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        color4 = hardwareMap.get(NormalizedColorSensor.class, "color4");
        arm = hardwareMap.get(Servo.class, "arm");

        turretServo = hardwareMap.get(CRServo.class, "Turret");
        turretEncoderMotor = hardwareMap.get(DcMotorEx.class, "Fl");

        turretServo.setDirection(CRServo.Direction.REVERSE);
        turretPidTimer.reset();


        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setDirection(DcMotorEx.Direction.FORWARD);
        revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.setPollRateHz(POLL_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hub = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(hub));

        color4.setGain(12);
        arm.setPosition(ARM_DOWN);

        targetPosition = 0;
        currentSlotCount = 0;
        lastButtonState = false;
        isSensorEnabled = false;

        resetSequenceState();
        clearMemoryScan();
    }

    // =========================
    // BACKGROUND PID (RUN IN ParallelAction ALWAYS)
    // =========================
    public Action updatePID() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = PIDClassForAuto.returnRevPID(targetPosition, revolver.getCurrentPosition());
                power = Range.clip(power, -MOTOR_POWER_LIMIT, MOTOR_POWER_LIMIT);
                revolver.setPower(power);

                packet.put("Rev/SensorEnabled", isSensorEnabled);
                packet.put("Rev/SlotCount", currentSlotCount);
                packet.put("Rev/Target", targetPosition);
                packet.put("Rev/Actual", revolver.getCurrentPosition());

                packet.put("Seq/Running", sequenceRunning);
                packet.put("LL/LastID", lastSequenceId);
                packet.put("LL/Pattern", desiredPattern.toString());

                packet.put("Mem/Valid", memValid);
                packet.put("Mem/S0", slotColorMem[0].toString());
                packet.put("Mem/S1", slotColorMem[1].toString());
                packet.put("Mem/S2", slotColorMem[2].toString());

                return true;
            }
        };
    }

    // =========================
    // TOUCHREV2 ACTIONS (KEPT)
    // =========================
    public Action resetRevolver() {
        return packet -> {
            targetPosition = 0;
            currentSlotCount = 0;
            lastButtonState = false;
            return false;
        };
    }

    public Action enableSensor() {
        return packet -> {
            isSensorEnabled = true;
            lastButtonState = false;
            return false;
        };
    }

    public Action disableSensor() {
        return packet -> {
            isSensorEnabled = false;
            lastButtonState = false;
            return false;
        };
    }

    public Action resetHardwareEncoder() {
        return packet -> {
            revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            revolver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetPosition = 0;
            currentSlotCount = 0;
            lastButtonState = false;
            return false;
        };
    }

    public Action setTarget(int pos) {
        return packet -> {
            setTargetInternal(pos);
            currentSlotCount = Math.round((float) targetPosition / TICKS_PER_SLOT);
            return false;
        };
    }

    // =========================
    // LIMELIGHT: START/STOP
    // =========================
    public Action startLimelight() {
        return packet -> {
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.setPollRateHz(POLL_HZ);
            limelight.start();
            return false;
        };
    }

    public Action stopLimelight() {
        return packet -> {
            limelight.stop();
            return false;
        };
    }

    // =========================
    // LIMELIGHT: SCAN PATTERN (21/22/23)
    // =========================
    public Action scanLimelightPattern() {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    init = true;
                    limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
                    limelight.setPollRateHz(POLL_HZ);
                    limelight.start();
                    timer.reset();
                }

                YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(ypr.getYaw(AngleUnit.RADIANS));

                LLResult result = limelight.getLatestResult();

                int bestId = -1;
                double bestArea = -1;

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            int id = tag.getFiducialId();
                            if (id != 21 && id != 22 && id != 23) continue;

                            double area = tag.getTargetArea();
                            if (area > bestArea) {
                                bestArea = area;
                                bestId = id;
                            }
                        }
                    }
                }

                if (bestId != -1) {
                    lastSequenceId = bestId;
                    if (bestId == 21) desiredPattern = Pattern.GPP;
                    else if (bestId == 22) desiredPattern = Pattern.PGP;
                    else desiredPattern = Pattern.PPG;

                    packet.put("LL/FOUND_ID", bestId);
                    packet.put("LL/PATTERN", desiredPattern.toString());
                    return false;
                }

                if (timer.seconds() > LIMELIGHT_SCAN_TIMEOUT_SEC) {
                    packet.put("LL/TIMEOUT", true);
                    packet.put("LL/LAST_ID", lastSequenceId);
                    packet.put("LL/PATTERN", desiredPattern.toString());
                    return false;
                }

                packet.put("LL/SCANNING", true);
                packet.put("LL/T", timer.seconds());
                return true;
            }
        };
    }

    public Pattern getDesiredPattern() { return desiredPattern; }
    public int getLastSequenceId() { return lastSequenceId; }

    // =========================
    // TURRET ACTIONS
    // =========================
    public Action turretStop() {
        return packet -> {
            turretServo.setPower(0);
            return false;
        };
    }

    public Action turretTimedOutAndBack(double power, double seconds) {
        return new Action() {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean reversing = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    turretServo.setPower(power);
                    timer.reset();
                    initialized = true;
                }

                if (!reversing && timer.seconds() >= seconds) {
                    reversing = true;
                    timer.reset();
                    turretServo.setPower(-power);
                }

                if (reversing && timer.seconds() >= seconds) {
                    turretServo.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    public Action turretTrack() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    turretPidTimer.reset();
                    turretLastError = 0;
                    initialized = true;
                }

                if (turretTrackingEnabled) {
                    runTurretLogic(packet);
                } else {
                    turretServo.setPower(0);
                }

                packet.put("Turret/Enabled", turretTrackingEnabled);
                packet.put("Turret/Enc", turretEncoderMotor.getCurrentPosition());
                packet.put("Turret/Pwr", turretServo.getPower());
                return true;
            }
        };
    }

    public Action enableTurretTrack() {
        return p -> { turretTrackingEnabled = true; return false; };
    }

    public Action disableTurretTrack() {
        return p -> { turretTrackingEnabled = false; turretServo.setPower(0); return false; };
    }

    // =========================
    // NEW: scanBall() — rotate through shooter slots and remember colors
    // =========================
    public Action scanBall() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                sequenceRunning = true;

                p.put("MemScan/State", memScanState.toString());
                p.put("MemScan/Idx", memScanIdx);
                p.put("Mem/Valid", memValid);

                switch (memScanState) {
                    case INIT: {
                        clearMemoryScan();

                        memScanIdx = 0;
                        memScanState = MemScanState.MOVE_SLOT;

                        memTargetToIndex(memScanIdx);
                        memMoveTimer.reset();
                        memTimer.reset();
                        memUnknownTimer.reset();
                        memLastNonUnknown = BallColor.UNKNOWN;
                        break;
                    }

                    case MOVE_SLOT: {
                        if (atTarget(memTargetPos) || memMoveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            memTimer.reset();
                            memUnknownTimer.reset();
                            memLastNonUnknown = BallColor.UNKNOWN;
                            memScanState = MemScanState.WAIT_DELAY;
                        }
                        break;
                    }

                    case WAIT_DELAY: {
                        if (memTimer.seconds() >= VALIDATE_DELAY_SEC) {
                            memUnknownTimer.reset();
                            memLastNonUnknown = BallColor.UNKNOWN;
                            memScanState = MemScanState.SAMPLE;
                        }
                        break;
                    }

                    case SAMPLE: {
                        BallColor seen = readColor4();
                        if (seen != BallColor.UNKNOWN) memLastNonUnknown = seen;

                        if (seen == BallColor.UNKNOWN && memUnknownTimer.seconds() < UNKNOWN_SCAN_SEC) {
                            break;
                        }

                        if (seen == BallColor.UNKNOWN && memLastNonUnknown != BallColor.UNKNOWN) {
                            seen = memLastNonUnknown;
                        }

                        slotColorMem[memScanIdx] = seen;

                        p.put("MemScan/Seen", seen.toString());
                        p.put("MemScan/Hue", getHue(color4));

                        memScanState = MemScanState.NEXT_SLOT;
                        break;
                    }

                    case NEXT_SLOT: {
                        memScanIdx++;
                        if (memScanIdx >= 3) {
                            memValid = (slotColorMem[0] != BallColor.UNKNOWN
                                    && slotColorMem[1] != BallColor.UNKNOWN
                                    && slotColorMem[2] != BallColor.UNKNOWN);

                            setTargetInternal(POS_INTAKE);
                            memTargetPos = POS_INTAKE;
                            memMoveTimer.reset();
                            memScanState = MemScanState.RETURN_INTAKE;
                            break;
                        }

                        memTargetToIndex(memScanIdx);
                        memMoveTimer.reset();
                        memTimer.reset();
                        memUnknownTimer.reset();
                        memLastNonUnknown = BallColor.UNKNOWN;
                        memScanState = MemScanState.MOVE_SLOT;
                        break;
                    }

                    case RETURN_INTAKE: {
                        if (atTarget(POS_INTAKE) || memMoveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            memScanState = MemScanState.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        sequenceRunning = false;
                        memScanState = MemScanState.INIT;
                        return false;
                    }
                }

                return true;
            }
        };
    }

    // =========================
    // NEW: scanShot() — uses memory (from scanBall) and shoots immediately (no color read)
    // =========================
    public Action scanShot() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                sequenceRunning = true;

                p.put("MemShoot/State", memShootState.toString());
                p.put("MemShoot/ShotsDone", memShotsDone);
                p.put("Mem/Valid", memValid);
                p.put("Mem/S0", slotColorMem[0].toString());
                p.put("Mem/S1", slotColorMem[1].toString());
                p.put("Mem/S2", slotColorMem[2].toString());
                p.put("Pattern", desiredPattern.toString());

                switch (memShootState) {

                    case INIT: {
                        if (!memValid) {
                            sequenceRunning = false;
                            memShootState = MemShootState.INIT;
                            return false;
                        }

                        memShotMask[0] = memShotMask[1] = memShotMask[2] = false;
                        memShotsDone = 0;

                        memTargetIdx = chooseSlotForExpected(expectedForShot(desiredPattern, 0), memShotMask);
                        if (memTargetIdx == -1) memTargetIdx = firstUnshot(memShotMask);

                        memTargetPos = posFromIdx(memTargetIdx);
                        setTargetInternal(memTargetPos);

                        memMoveTimer.reset();
                        memShootTimer.reset();
                        memShootState = MemShootState.MOVE_TO_SLOT;
                        break;
                    }

                    case MOVE_TO_SLOT: {
                        if (atTarget(memTargetPos) || memMoveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            memShootTimer.reset();
                            memShootState = MemShootState.ARM_UP;
                        }
                        break;
                    }

                    case ARM_UP: {
                        arm.setPosition(ARM_UP);
                        if (memShootTimer.seconds() >= ARM_UPDOWN_SEC) {
                            memShootTimer.reset();
                            memShootState = MemShootState.ARM_DOWN;
                        }
                        break;
                    }

                    case ARM_DOWN: {
                        arm.setPosition(ARM_DOWN);
                        if (memShootTimer.seconds() >= ARM_UPDOWN_SEC) {
                            memShootTimer.reset();
                            memShootState = MemShootState.SETTLE; // Changed back to SETTLE
                        }
                        break;
                    }

                    case SETTLE: {
                        // RE-ACTIVATED: Waits for REVUP_DELAY_SEC
                        if (memShootTimer.seconds() >= REVUP_DELAY_SEC) {
                            memShootState = MemShootState.NEXT;
                        }
                        break;
                    }

                    case NEXT: {
                        memShotMask[memTargetIdx] = true;
                        memShotsDone++;

                        if (memShotsDone >= 3) {
                            setTargetInternal(POS_INTAKE);
                            memTargetPos = POS_INTAKE;
                            memMoveTimer.reset();
                            memShootState = MemShootState.RETURN_INTAKE;
                            break;
                        }

                        if (ONLY_VALIDATE_FIRST_TWO && memShotsDone >= 2) {
                            memTargetIdx = firstUnshot(memShotMask);
                        } else {
                            BallColor exp = expectedForShot(desiredPattern, memShotsDone);
                            int idx = chooseSlotForExpected(exp, memShotMask);
                            memTargetIdx = (idx != -1) ? idx : firstUnshot(memShotMask);
                        }

                        if (memTargetIdx == -1) {
                            setTargetInternal(POS_INTAKE);
                            memTargetPos = POS_INTAKE;
                            memMoveTimer.reset();
                            memShootState = MemShootState.RETURN_INTAKE;
                            break;
                        }

                        memTargetPos = posFromIdx(memTargetIdx);
                        setTargetInternal(memTargetPos);
                        memMoveTimer.reset();
                        memShootTimer.reset();
                        memShootState = MemShootState.MOVE_TO_SLOT;
                        break;
                    }

                    case RETURN_INTAKE: {
                        if (atTarget(POS_INTAKE) || memMoveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            memShootState = MemShootState.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        arm.setPosition(ARM_DOWN);
                        sequenceRunning = false;
                        memShootState = MemShootState.INIT;
                        return false;
                    }
                }

                return true;
            }
        };
    }

    // =========================
    // ORIGINAL: scan+shoot sequence
    // =========================
    public Action runSequence3ShotsNoShooter() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                sequenceRunning = true;

                p.put("Seq/State", seqState.toString());
                p.put("Seq/ShotsDone", shotsDone);
                p.put("Seq/Pattern", desiredPattern.toString());
                p.put("Seq/TargetIdx", targetIdx);
                p.put("Seq/TargetPos", targetPos);

                switch (seqState) {

                    case INIT: {
                        int nearest = nearestShootPos(revolver.getCurrentPosition());
                        currentIdx = idxFromPos(nearest);

                        shotsDone = 0;
                        shotMask[0] = shotMask[1] = shotMask[2] = false;

                        targetIdx = currentIdx;
                        targetPos = posFromIdx(targetIdx);
                        setTargetInternal(targetPos);

                        stateTimer.reset();
                        moveTimer.reset();
                        unknownScanTimer.reset();
                        lastNonUnknownSeen = BallColor.UNKNOWN;

                        seqState = SeqState.MOVE_TO_TARGET;
                        break;
                    }

                    case MOVE_TO_TARGET: {
                        if (atTarget(targetPos) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            stateTimer.reset();
                            unknownScanTimer.reset();
                            lastNonUnknownSeen = BallColor.UNKNOWN;
                            seqState = SeqState.WAIT_VALIDATE;
                        }
                        break;
                    }

                    case WAIT_VALIDATE: {
                        if (stateTimer.seconds() >= VALIDATE_DELAY_SEC) {
                            unknownScanTimer.reset();
                            lastNonUnknownSeen = BallColor.UNKNOWN;
                            seqState = SeqState.READ_VALIDATE;
                        }
                        break;
                    }

                    case READ_VALIDATE: {
                        BallColor expected = expectedForShot(desiredPattern, shotsDone);

                        BallColor seen = readColor4();
                        if (seen != BallColor.UNKNOWN) lastNonUnknownSeen = seen;

                        boolean doColorCheck = !(ONLY_VALIDATE_FIRST_TWO && shotsDone >= 2);

                        p.put("Seq/Expected", expected.toString());
                        p.put("Seq/Seen", seen.toString());
                        p.put("Seq/DoColorCheck", doColorCheck);
                        p.put("Seq/Hue", getHue(color4));

                        if (doColorCheck) {
                            if (seen == BallColor.UNKNOWN) {
                                if (unknownScanTimer.seconds() < UNKNOWN_SCAN_SEC) {
                                    break;
                                }
                                if (lastNonUnknownSeen != BallColor.UNKNOWN) {
                                    seen = lastNonUnknownSeen;
                                }
                            }

                            if (seen != expected) {
                                int nextIdx = nextUnshotIdxForwardOnly(targetIdx, shotMask);
                                if (nextIdx == -1) {
                                    setTargetInternal(POS_INTAKE);
                                    targetPos = POS_INTAKE;
                                    moveTimer.reset();
                                    seqState = SeqState.RETURN_INTAKE;
                                    break;
                                }
                                targetIdx = nextIdx;
                                targetPos = posFromIdx(targetIdx);
                                setTargetInternal(targetPos);
                                moveTimer.reset();
                                seqState = SeqState.MOVE_TO_TARGET;
                                break;
                            }
                        }

                        stateTimer.reset();
                        seqState = SeqState.ARM_UP_STATE;
                        break;
                    }

                    case ARM_UP_STATE: {
                        arm.setPosition(ARM_UP);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            stateTimer.reset();
                            seqState = SeqState.ARM_DOWN_STATE;
                        }
                        break;
                    }

                    case ARM_DOWN_STATE: {
                        arm.setPosition(ARM_DOWN);
                        if (stateTimer.seconds() >= ARM_UPDOWN_SEC) {
                            stateTimer.reset();
                            seqState = SeqState.POST_SHOT_SETTLE; // Changed back to POST_SHOT_SETTLE
                        }
                        break;
                    }

                    case POST_SHOT_SETTLE: {
                        // RE-ACTIVATED: Waits for REVUP_DELAY_SEC
                        if (stateTimer.seconds() >= REVUP_DELAY_SEC) {
                            stateTimer.reset();
                            seqState = SeqState.NEXT_SHOT;
                        }
                        break;
                    }

                    case NEXT_SHOT: {
                        shotMask[targetIdx] = true;
                        shotsDone++;

                        currentIdx = targetIdx;

                        if (shotsDone >= 3) {
                            setTargetInternal(POS_INTAKE);
                            targetPos = POS_INTAKE;
                            moveTimer.reset();
                            seqState = SeqState.RETURN_INTAKE;
                            break;
                        }

                        int nextIdx;
                        if (ONLY_VALIDATE_FIRST_TWO && shotsDone >= 2) {
                            nextIdx = findOnlyRemainingUnshot(shotMask);
                        } else {
                            nextIdx = preferBacktrackOne(currentIdx, shotMask);
                        }

                        if (nextIdx == -1) {
                            setTargetInternal(POS_INTAKE);
                            targetPos = POS_INTAKE;
                            moveTimer.reset();
                            seqState = SeqState.RETURN_INTAKE;
                            break;
                        }

                        targetIdx = nextIdx;
                        targetPos = posFromIdx(targetIdx);
                        setTargetInternal(targetPos);

                        moveTimer.reset();
                        stateTimer.reset();
                        unknownScanTimer.reset();
                        lastNonUnknownSeen = BallColor.UNKNOWN;

                        seqState = SeqState.MOVE_TO_TARGET;
                        break;
                    }

                    case RETURN_INTAKE: {
                        if (atTarget(POS_INTAKE) || moveTimer.seconds() >= MOVE_TIMEOUT_SEC) {
                            seqState = SeqState.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        arm.setPosition(ARM_DOWN);
                        sequenceRunning = false;
                        resetSequenceState();
                        return false;
                    }
                }

                return true;
            }
        };
    }

    // =========================
    // TOUCH ADVANCE
    // =========================
    private boolean touchAdvanceEnabled = true;
    private boolean touchLockedOut = false;
    private int touchBallCount = 3;
    private boolean touchLastPressed = false;

    public Action updateTouchAdvance() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (sequenceRunning) {
                    packet.put("Touch/BlockedBySeq", true);
                    return true;
                }

                boolean pressed = touchSensor.isPressed();

                if (touchAdvanceEnabled && !touchLockedOut) {
                    if (pressed && !touchLastPressed) {

                        if (touchBallCount < MAX_SLOTS) {
                            touchBallCount++;

                            targetPosition = clamp(touchBallCount * TICKS_PER_SLOT, 0, MAX_POSITION);

                            if (touchBallCount >= MAX_SLOTS) {
                                touchLockedOut = true;
                            }
                        } else {
                            touchLockedOut = true;
                        }
                    }
                }

                touchLastPressed = pressed;

                packet.put("Touch/Pressed", pressed);
                packet.put("Touch/Enabled", touchAdvanceEnabled);
                packet.put("Touch/Locked", touchLockedOut);
                packet.put("Touch/Balls", touchBallCount);
                packet.put("Rev/Target", targetPosition);
                packet.put("Rev/Actual", revolver.getCurrentPosition());

                return true;
            }
        };
    }

    public Action resetTouch() {
        return packet -> {
            touchBallCount = 0;
            touchLockedOut = false;
            touchLastPressed = false;
            return false;
        };
    }

    public Action disableTouchAdvance() { return p -> { touchAdvanceEnabled = false; return false; }; }
    public Action enableTouchAdvance()  { return p -> { touchAdvanceEnabled = true;  return false; }; }
    public int getTouchBallCount() { return touchBallCount; }
    public boolean isTouchLockedOut() { return touchLockedOut; }

    // =========================
    // INTERNAL HELPERS (REV)
    // =========================
    private void setTargetInternal(int pos) {
        targetPosition = clamp(pos, 0, MAX_POSITION);
    }

    private boolean atTarget(int target) {
        return Math.abs(revolver.getCurrentPosition() - target) <= AT_TARGET_TOL;
    }

    private int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private int idxFromPos(int pos) {
        for (int i = 0; i < 3; i++) if (SHOOT_POS[i] == pos) return i;
        return 0;
    }

    private int posFromIdx(int idx) {
        idx = wrap3(idx);
        return SHOOT_POS[idx];
    }

    private int wrap3(int i) {
        i %= 3;
        if (i < 0) i += 3;
        return i;
    }

    private int nearestShootPos(int ticks) {
        int best = POS_SHOOT0;
        int bestErr = Math.abs(ticks - POS_SHOOT0);

        int e1 = Math.abs(ticks - POS_SHOOT1);
        if (e1 < bestErr) { bestErr = e1; best = POS_SHOOT1; }

        int e2 = Math.abs(ticks - POS_SHOOT2);
        if (e2 < bestErr) { bestErr = e2; best = POS_SHOOT2; }

        return best;
    }

    private int nextUnshotIdxForwardOnly(int fromIdx, boolean[] shot) {
        int a = wrap3(fromIdx + 1);
        if (!shot[a]) return a;

        int b = wrap3(fromIdx + 2);
        if (!shot[b]) return b;

        return -1;
    }

    private int preferBacktrackOne(int curIdx, boolean[] shot) {
        int back = wrap3(curIdx - 1);
        if (!shot[back]) return back;

        int cur = wrap3(curIdx);
        if (!shot[cur]) return cur;

        int fwd = wrap3(curIdx + 1);
        if (!shot[fwd]) return fwd;

        return -1;
    }

    private int findOnlyRemainingUnshot(boolean[] shot) {
        int last = -1;
        for (int i = 0; i < 3; i++) if (!shot[i]) last = i;
        return last;
    }

    private void resetSequenceState() {
        seqState = SeqState.INIT;

        shotsDone = 0;
        currentIdx = 0;
        targetIdx = 0;
        targetPos = POS_SHOOT0;

        shotMask[0] = shotMask[1] = shotMask[2] = false;

        stateTimer.reset();
        moveTimer.reset();
        unknownScanTimer.reset();
        lastNonUnknownSeen = BallColor.UNKNOWN;
    }

    // =========================
    // MEMORY HELPERS (NEW)
    // =========================
    private void clearMemoryScan() {
        slotColorMem[0] = BallColor.UNKNOWN;
        slotColorMem[1] = BallColor.UNKNOWN;
        slotColorMem[2] = BallColor.UNKNOWN;
        memValid = false;

        memScanState = MemScanState.INIT;
        memScanIdx = 0;
        memLastNonUnknown = BallColor.UNKNOWN;
    }

    private void memTargetToIndex(int idx) {
        idx = wrap3(idx);
        memTargetIdx = idx;
        memTargetPos = posFromIdx(idx);
        setTargetInternal(memTargetPos);
    }

    private int chooseSlotForExpected(BallColor expected, boolean[] alreadyShot) {
        for (int i = 0; i < 3; i++) {
            if (!alreadyShot[i] && slotColorMem[i] == expected) return i;
        }
        return -1;
    }

    private int firstUnshot(boolean[] alreadyShot) {
        for (int i = 0; i < 3; i++) if (!alreadyShot[i]) return i;
        return -1;
    }

    // =========================
    // INTERNAL HELPERS (COLOR4)
    // =========================
    private BallColor readColor4() {
        float h = getHue(color4);
        if (h >= GREEN_H_MIN && h <= GREEN_H_MAX) return BallColor.GREEN;
        if (h >= PURPLE_H_MIN && h <= PURPLE_H_MAX) return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    private float getHue(NormalizedColorSensor sensor) {
        float[] hsv = new float[3];
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    private BallColor expectedForShot(Pattern p, int shotIdx) {
        if (p == Pattern.GPP) {
            return (shotIdx == 0) ? BallColor.GREEN : BallColor.PURPLE;
        } else if (p == Pattern.PGP) {
            return (shotIdx == 1) ? BallColor.GREEN : BallColor.PURPLE;
        } else {
            return (shotIdx == 2) ? BallColor.GREEN : BallColor.PURPLE;
        }
    }

    // =========================
    // INTERNAL HELPERS (TURRET)
    // =========================
    private void runTurretLogic(@NonNull TelemetryPacket packet) {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(ypr.getYaw(AngleUnit.RADIANS));

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double power = calculateTurretPID(tx);

            int pos = turretEncoderMotor.getCurrentPosition();
            if (turretLimits) {
                if (pos >= turretMaxEnc && power > 0) power = 0;
                else if (pos <= turretMinEnc && power < 0) power = 0;
            }

            turretServo.setPower(power);

            packet.put("Turret/Tx", tx);
            packet.put("Turret/HasTarget", true);
        } else {
            turretServo.setPower(0);
            packet.put("Turret/HasTarget", false);
        }
    }

    private double calculateTurretPID(double error) {
        double dt = turretPidTimer.seconds();
        if (dt <= 0) dt = 0.02;
        turretPidTimer.reset();

        if (Math.abs(error) < turretTolerance) {
            turretLastError = 0;
            return 0;
        }

        double P = turretP * error;
        double D = turretD * (error - turretLastError) / dt;
        turretLastError = error;

        double out = P + D;

        if (Math.abs(out) < turretMinPower) out = Math.signum(out) * turretMinPower;
        if (out > turretMaxPower) out = turretMaxPower;
        if (out < -turretMaxPower) out = -turretMaxPower;

        return out;
    }
}